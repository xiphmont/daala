/*Daala video codec
Copyright (c) 2006-2013 Daala project contributors.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "state.h"
#if defined(OD_X86ASM)
# include "x86/x86int.h"
#endif
#include "block_size.h"

#define DOWN(x) (OD_CLAMP255( (( (x)+(1<<4>>1) ) >>4 ) + 128))
#define UP(x) (((OD_CLAMP255(x))-128)<<4)
#define OD_CLAMPTEST(x) OD_CLAMP255(x)

/* OD_DC_RES[i] adjusts the quantization of DC for the ith plane.
   These values are based on manual tuning to optimize PSNR-HVS, while also
   attempting to keep a good visual balance between the relative resolution
   of luma, and chroma.
   FIXME: Tune this properly, see also OD_DEFAULT_QMS.*/
const od_coeff OD_DC_RES[3] = {17, 24, 17};

/* Scaling compensation for the Haar equivalent basis function. Left is
   for horizontal/vertical. Right is for diagonal. */
const od_coeff OD_DC_QM[2][OD_NBSIZES - 1][2] = {
  {{25, 30}, {21, 27}, {17, 19}},
  {{21, 25}, {18, 20}, {17, 18}}
};

static void *od_aligned_malloc(size_t _sz,size_t _align) {
  unsigned char *p;
  if (_align - 1 > UCHAR_MAX || (_align&_align-1) || _sz > ~(size_t)0-_align)
    return NULL;
  p = (unsigned char *)_ogg_malloc(_sz + _align);
  if (p != NULL) {
    int offs;
    offs = ((p-(unsigned char *)0) - 1 & _align - 1);
    p[offs] = offs;
    p += offs+1;
  }
  return p;
}

static void od_aligned_free(void *_ptr) {
  unsigned char *p;
  p = (unsigned char *)_ptr;
  if (p != NULL) {
    int offs;
    offs = *--p;
    _ogg_free(p - offs);
  }
}

/*Initializes the buffers used for reference frames.
  These buffers are padded with 16 extra pixels on each side, to allow
   (relatively) unrestricted motion vectors without special casing reading
   outside the image boundary.
  If chroma is decimated in either direction, the padding is reduced by an
   appropriate factor on the appropriate sides.*/
static int od_state_ref_imgs_init(od_state *state, int nrefs, int nio) {
  daala_info *info;
  od_img *img;
  od_img_plane *iplane;
  od_reference *ref;
  void *ref_img_data;
  size_t data_sz;
  int plane_buf_width;
  int plane_buf_height;
  int imgi;
  int pli;
  int y;
  OD_ASSERT(nrefs >= 3);
  OD_ASSERT(nrefs <= 4);
  OD_ASSERT(nio == 2);
  info = &state->info;
  data_sz = 0;
  /*TODO: Check for overflow before allocating.*/
  state->frame_buf_width = state->frame_width + (OD_UMV_PADDING << 1);
  state->frame_buf_height = state->frame_height + (OD_UMV_PADDING << 1);
  /*Reserve space for planes in nrefs upsampled reference images.*/
  for (pli = 0; pli < info->nplanes; pli++) {
    plane_buf_width = (state->frame_buf_width << 1) >> info->plane_info[pli].xdec;
    plane_buf_height = (state->frame_buf_height << 1) >> info->plane_info[pli].ydec;
    data_sz += plane_buf_width*plane_buf_height*nrefs*OD_REFERENCE_BYTES;
  }
  /*Reserve space for planes in nio input/output images.*/
  for (pli = 0; pli < info->nplanes; pli++) {
    plane_buf_width = state->frame_buf_width >> info->plane_info[pli].xdec;
    plane_buf_height = state->frame_buf_height >> info->plane_info[pli].ydec;
    data_sz += plane_buf_width*plane_buf_height*nio;
  }
  /*Reserve space for the line buffer in the up-sampler.*/
  data_sz += (state->frame_buf_width << 1)*8*OD_REFERENCE_BYTES;
#if defined(OD_DUMP_IMAGES)
  /*Reserve space for this plane in 1 upsampled visualization image.*/
  for (pli = 0; pli < info->nplanes; pli++) {
    plane_buf_width = (state->frame_buf_width << 1) >> info->plane_info[pli].xdec;
    plane_buf_height = (state->frame_buf_height << 1) >> info->plane_info[pli].ydec;
    data_sz += plane_buf_width*plane_buf_height;
  }
#endif
  state->ref_img_data = ref_img_data = od_aligned_malloc(data_sz, 32);
  if (OD_UNLIKELY(!ref_img_data)) {
    return OD_EFAULT;
  }
  /*Fill in the reference structures.  Right now all ref structures
    are upsampled by 2 in both directions.  Depending on settings in
    internal.h, they may also (temporarily) be either 8 bits deep (one
    byte) or 12 bits deep (2 bytes) with od_reftype, OD_REFERENCE_BITS
    and OD_REFERENCE_BYTES set appropriately. */
  for (imgi = 0; imgi < nrefs; imgi++) {
    ref = state->ref_imgs + imgi;
    for (pli = 0; pli < info->nplanes; pli++) {
      plane_buf_width = (state->frame_buf_width << 1) >> info->plane_info[pli].xdec;
      plane_buf_height = (state->frame_buf_height << 1) >> info->plane_info[pli].ydec;
      ref->planes[pli] = ((od_reftype *)ref_img_data)
       + ((OD_UMV_PADDING << 1) >> info->plane_info[pli].xdec)
       + plane_buf_width*((OD_UMV_PADDING << 1) >> info->plane_info[pli].ydec);
      ref_img_data = ((od_reftype *)ref_img_data) + plane_buf_width*plane_buf_height;
    }
  }
  /*Fill in the input/output image structures.  Not oversampled,
    currently 8-bit only*/
  for (imgi = 0; imgi < nio; imgi++) {
    img = state->io_imgs + imgi;
    img->nplanes = info->nplanes;
    img->width = state->frame_width;
    img->height = state->frame_height;
    for (pli = 0; pli < img->nplanes; pli++) {
      plane_buf_width = state->frame_buf_width >> info->plane_info[pli].xdec;
      plane_buf_height = state->frame_buf_height >> info->plane_info[pli].ydec;
      iplane = img->planes + pli;
      iplane->data = ((unsigned char *)ref_img_data)
       + (OD_UMV_PADDING >> info->plane_info[pli].xdec)
       + plane_buf_width*(OD_UMV_PADDING >> info->plane_info[pli].ydec);
      ref_img_data = ((unsigned char *)ref_img_data) + plane_buf_width*plane_buf_height;
      iplane->xdec = info->plane_info[pli].xdec;
      iplane->ydec = info->plane_info[pli].ydec;
      iplane->xstride = 1;
      iplane->ystride = plane_buf_width;
    }
  }
#if defined(OD_DUMP_IMAGES)
  /*Fill in the visualization image structure. Upsampled 2x, 8-bit
    depth only*/
  img = &state->vis_img;
  img->nplanes = info->nplanes;
  img->width = state->frame_buf_width << 1;
  img->height = state->frame_buf_height << 1;
  for (pli = 0; pli < img->nplanes; pli++) {
    iplane = img->planes + pli;
    plane_buf_width = img->width >> info->plane_info[pli].xdec;
    plane_buf_height = img->height >> info->plane_info[pli].ydec;
    iplane->data = ref_img_data;
    ref_img_data = ((unsigned char *)ref_img_data) + plane_buf_width*plane_buf_height;
    iplane->xdec = info->plane_info[pli].xdec;
    iplane->ydec = info->plane_info[pli].ydec;
    iplane->xstride = 1;
    iplane->ystride = plane_buf_width;
  }
#endif
  /*Fill in the line buffers.*/
  for (y = 0; y < 8; y++) {
   state->ref_line_buf[y] = ((od_reftype *)ref_img_data) + (OD_UMV_PADDING << 1);
   ref_img_data = ((od_reftype *)ref_img_data) + (state->frame_buf_width << 1);
  }
  /*Mark all of the reference image buffers available.*/
  for (imgi = 0; imgi < nrefs; imgi++) state->ref_imgi[imgi] = -1;
  return OD_SUCCESS;
}

static int od_state_mvs_init(od_state *state) {
  int nhmvbs;
  int nvmvbs;
  nhmvbs = (state->nhmbs + 1) << 2;
  nvmvbs = (state->nvmbs + 1) << 2;
  state->mv_grid = (od_mv_grid_pt **)od_calloc_2d(nvmvbs + 1, nhmvbs + 1,
   sizeof(**state->mv_grid));
  if (OD_UNLIKELY(!state->mv_grid)) {
    return OD_EFAULT;
  }
  return OD_SUCCESS;
}

static void od_restore_fpu_c(void) {}

void od_restore_fpu(od_state *state) {
  (*state->opt_vtbl.restore_fpu)();
}

void od_state_opt_vtbl_init_c(od_state *state) {
  state->opt_vtbl.mc_predict1fmv = od_mc_predict1fmv_c;
  state->opt_vtbl.mc_blend_full = od_mc_blend_full_c;
  state->opt_vtbl.mc_blend_full_split = od_mc_blend_full_split_c;
  state->opt_vtbl.restore_fpu = od_restore_fpu_c;
  OD_COPY(state->opt_vtbl.fdct_2d, OD_FDCT_2D_C, OD_NBSIZES + 1);
  OD_COPY(state->opt_vtbl.idct_2d, OD_IDCT_2D_C, OD_NBSIZES + 1);
}

static void od_state_opt_vtbl_init(od_state *state) {
#if defined(OD_X86ASM)
  od_state_opt_vtbl_init_x86(state);
#else
  od_state_opt_vtbl_init_c(state);
#endif
}

static int od_state_init_impl(od_state *state, const daala_info *info) {
  int nplanes;
  int pli;
  /*First validate the parameters.*/
  if (info == NULL) return OD_EFAULT;
  nplanes = info->nplanes;
  if (nplanes <= 0 || nplanes > OD_NPLANES_MAX) return OD_EINVAL;
  /*The first plane (the luma plane) must not be subsampled.*/
  if (info->plane_info[0].xdec || info->plane_info[0].ydec) return OD_EINVAL;
  OD_CLEAR(state, 1);
  OD_COPY(&state->info, info, 1);
  /*Frame size is a multiple of a super block.*/
  state->frame_width = (info->pic_width + (OD_SUPERBLOCK_SIZE - 1)) &
   ~(OD_SUPERBLOCK_SIZE - 1);
  state->frame_height = (info->pic_height + (OD_SUPERBLOCK_SIZE - 1)) &
   ~(OD_SUPERBLOCK_SIZE - 1);
  state->nhmbs = state->frame_width >> 4;
  state->nvmbs = state->frame_height >> 4;
  od_state_opt_vtbl_init(state);
  if (OD_UNLIKELY(od_state_ref_imgs_init(state, 4, 2))) {
    return OD_EFAULT;
  }
  if (OD_UNLIKELY(od_state_mvs_init(state))) {
    return OD_EFAULT;
  }
  state->nhsb = state->frame_width >> 5;
  state->nvsb = state->frame_height >> 5;
  for (pli = 0; pli < nplanes; pli++) {
    int xdec;
    int ydec;
    int w;
    int h;
    state->sb_dc_mem[pli] = (od_coeff*)_ogg_malloc(
     sizeof(state->sb_dc_mem[pli][0])*state->nhsb*state->nvsb);
    if (OD_UNLIKELY(!state->sb_dc_mem[pli])) {
      return OD_EFAULT;
    }
    xdec = info->plane_info[pli].xdec;
    ydec = info->plane_info[pli].ydec;
    w = state->frame_width >> xdec;
    h = state->frame_height >> ydec;
    state->ctmp[pli] = (od_coeff *)_ogg_malloc(w*h*sizeof(*state->ctmp[pli]));
    if (OD_UNLIKELY(!state->ctmp[pli])) {
      return OD_EFAULT;
    }
    state->dtmp[pli] = (od_coeff *)_ogg_malloc(w*h*sizeof(*state->dtmp[pli]));
    if (OD_UNLIKELY(!state->dtmp[pli])) {
      return OD_EFAULT;
    }
    state->mctmp[pli] = (od_coeff *)_ogg_malloc(w*h*sizeof(*state->mctmp[pli]));
    if (OD_UNLIKELY(!state->mctmp[pli])) {
      return OD_EFAULT;
    }
    state->mdtmp[pli] = (od_coeff *)_ogg_malloc(w*h*sizeof(*state->mdtmp[pli]));
    if (OD_UNLIKELY(!state->mdtmp[pli])) {
      return OD_EFAULT;
    }
    /*We predict chroma planes from the luma plane.  Since chroma can be
      subsampled, we cache subsampled versions of the luma plane in the
      frequency domain.  We can share buffers with the same subsampling.*/
    if (pli > 0) {
      int plj;
      for (plj = 1; plj < pli; plj++) {
        if (xdec == info->plane_info[plj].xdec
          && ydec == info->plane_info[plj].ydec) {
          state->ltmp[pli] = NULL;
          state->lbuf[pli] = state->ltmp[plj];
        }
      }
      if (plj >= pli) {
        state->lbuf[pli] = state->ltmp[pli] = (od_coeff *)_ogg_malloc(w*h*
          sizeof(*state->ltmp[pli]));
        if (OD_UNLIKELY(!state->lbuf[pli])) {
          return OD_EFAULT;
        }
      }
    }
    else state->lbuf[pli] = state->ltmp[pli] = NULL;
  }
  state->bsize = (unsigned char *)_ogg_malloc(
   sizeof(*state->bsize)*(state->nhsb + 2)*4*(state->nvsb + 2)*4);
  if (OD_UNLIKELY(!state->bsize)) {
    return OD_EFAULT;
  }
  state->bstride = (state->nhsb + 2)*4;
  state->bsize += 4*state->bstride + 4;
#if defined(OD_DUMP_IMAGES) || defined(OD_DUMP_RECONS)
  state->dump_tags = 0;
  state->dump_files = 0;
#endif
  return OD_SUCCESS;
}

int od_state_init(od_state *state, const daala_info *info) {
  int ret;
  ret = od_state_init_impl(state, info);
  if (OD_UNLIKELY(ret < 0)) {
    od_state_clear(state);
  }
  return ret;
}

void od_state_clear(od_state *state) {
  int pli;
#if defined(OD_DUMP_IMAGES) || defined(OD_DUMP_RECONS)
  int i;
  if (state->dump_tags > 0) {
    for (i = 0; i < state->dump_tags; i++) fclose(state->dump_files[i].fd);
    _ogg_free(state->dump_files);
    state->dump_files = 0;
    state->dump_tags = 0;
  }
#endif
  od_free_2d(state->mv_grid);
  od_aligned_free(state->ref_img_data);
  state->bsize -= 4*state->bstride + 4;
  for (pli = 0; pli < state->info.nplanes; pli++) {
    _ogg_free(state->sb_dc_mem[pli]);
    _ogg_free(state->ltmp[pli]);
    _ogg_free(state->dtmp[pli]);
    _ogg_free(state->ctmp[pli]);
    _ogg_free(state->mctmp[pli]);
    _ogg_free(state->mdtmp[pli]);
  }
  _ogg_free(state->bsize);
}

void od_adapt_ctx_reset(od_adapt_ctx *state, int is_keyframe) {
  int i;
  int ln;
  int pli;
  generic_model_init(&state->pvq_param_model[0]);
  generic_model_init(&state->pvq_param_model[1]);
  generic_model_init(&state->pvq_param_model[2]);
  for (i = 0; i < 2*OD_NBSIZES; i++) {
    state->pvq_adapt[4*i + OD_ADAPT_K_Q8] = 384;
    state->pvq_adapt[4*i + OD_ADAPT_SUM_EX_Q8] = 256;
    state->pvq_adapt[4*i + OD_ADAPT_COUNT_Q8] = 104;
    state->pvq_adapt[4*i + OD_ADAPT_COUNT_EX_Q8] = 128;
  }
  state->pvq_k1_increment = 128;
  OD_CDFS_INIT(state->pvq_k1_cdf, state->pvq_k1_increment);
  for (pli = 0; pli < OD_NPLANES_MAX; pli++) {
    for (ln = 0; ln < OD_NBSIZES; ln++)
    for (i = 0; i < PVQ_MAX_PARTITIONS; i++) {
      state->pvq_exg[pli][ln][i] = 2 << 16;
    }
  }
  for (i = 0; i < OD_NBSIZES*PVQ_MAX_PARTITIONS; i++) {
    state->pvq_ext[i] = is_keyframe ? 24576 : 2 << 16;
  }
  state->bsize_range_increment = 128;
  for (i = 0; i < 7; i++) {
    int j;
    for (j = 0; j < OD_NBSIZES; j++) {
      state->bsize_range_cdf[j][i] = range_cdf_init[i] >> 6;
    }
  }
  state->bsize16_increment = 128;
  state->bsize8_increment = 128;
  for (i = 0; i < 16; i++) {
    /* Shifting makes the initial adaptation faster. */
    state->bsize16_cdf[0][i] = split16_cdf_init[0][i]>>6;
    state->bsize16_cdf[1][i] = split16_cdf_init[1][i]>>6;
  }
  OD_SINGLE_CDF_INIT(state->bsize8_cdf, state->bsize8_increment);
  generic_model_init(&state->mv_model);
  state->skip_increment = 128;
  OD_CDFS_INIT(state->skip_cdf, state->skip_increment >> 2);
  state->mv_small_increment = 128;
  OD_CDFS_INIT_FIRST(state->mv_small_cdf, state->mv_small_increment,
   10*state->mv_small_increment);
  state->pvq_gaintheta_increment = 128;
  OD_CDFS_INIT(state->pvq_gaintheta_cdf, state->pvq_gaintheta_increment >> 2);
  state->pvq_skip_dir_increment = 128;
  OD_CDFS_INIT(state->pvq_skip_dir_cdf, state->pvq_skip_dir_increment >> 2);
  for (pli = 0; pli < OD_NPLANES_MAX; pli++) {
    generic_model_init(&state->model_dc[pli]);
    generic_model_init(&state->model_g[pli]);
    for (i = 0; i < OD_NBSIZES; i++) {
      state->ex_g[pli][i] = 8;
    }
    state->ex_sb_dc[pli] = pli > 0 ? 8 : 32768;
    for (i = 0; i < 4; i++) {
      int j;
      for (j = 0; j < 3; j++) {
        state->ex_dc[pli][i][j] = pli > 0 ? 8 : 32768;
      }
    }
  }
}

void od_state_set_mv_res(od_state *state, int mv_res) {
  int i;
  state->mv_res = mv_res;
  for (i = 0; i < 5; i++) {
    state->adapt.mv_ex[i] = state->adapt.mv_ey[i] = (24 << 16) >> mv_res;
  }
}

/*Upsamples the reconstructed image to a reference image.
  TODO: Pipeline with reconstruction.*/
void od_state_upsample(od_state *state, od_reference *dimg,
 od_coeff *simg[OD_NPLANES_MAX]) {
  int pli;
  for (pli = 0; pli < state->info.nplanes; pli++) {
    const od_coeff *src;
    od_reftype *dst;
    int xdec;
    int ydec;
    int xpad;
    int ypad;
    int w;
    int h;
    int x;
    int y;
    xdec = state->info.plane_info[pli].xdec;
    ydec = state->info.plane_info[pli].ydec;
    xpad = OD_UMV_PADDING >> xdec;
    ypad = OD_UMV_PADDING >> ydec;
    w = state->frame_width >> xdec;
    h = state->frame_height >> ydec;
    src = simg[pli];
    dst = dimg->planes[pli]
      - (ypad<<1) * ((state->frame_buf_width << 1) >> xdec);
    for (y = -ypad; y < h + ypad + 3; y++) {
      /*Horizontal filtering:*/
      if (y < h + ypad) {
        od_reftype *buf;
        buf = state->ref_line_buf[y & 7];
        /*memset(buf, src[0], ((xpad - 2) << 1) * sizeof(*buf));*/
        for (x = -xpad; x < -2; x++) {
          *(buf + (x << 1)) = DOWN(src[0]);
          *(buf + (x << 1 | 1)) = DOWN(src[0]);
        }
        *(buf - 4) = DOWN(src[0]);
        *(buf - 3) = OD_CLAMPTEST((31*DOWN(src[0]) + DOWN(src[1]) + 16) >> 5);
        *(buf - 2) = DOWN(src[0]);
        *(buf - 1) = OD_CLAMPTEST((36*DOWN(src[0]) - 5*DOWN(src[1]) + DOWN(src[1]) + 16) >> 5);
        buf[0] = DOWN(src[0]);
        buf[1] = OD_CLAMPTEST((20*(DOWN(src[0]) + DOWN(src[1]))
                             - 5*(DOWN(src[0]) + DOWN(src[2])) + DOWN(src[0]) + DOWN(src[3]) + 16) >> 5);
        buf[2] = DOWN(src[1]);
        buf[3] = OD_CLAMPTEST((20*(DOWN(src[1]) + DOWN(src[2]))
                             - 5*(DOWN(src[0]) + DOWN(src[3])) + DOWN(src[0]) + DOWN(src[4]) + 16) >> 5);
        for (x = 2; x < w - 3; x++) {
          buf[x << 1] = DOWN(src[x]);
          buf[x << 1 | 1] = OD_CLAMPTEST((20*(DOWN(src[x]) + DOWN(src[x + 1]))
                                        - 5*(DOWN(src[x - 1]) + DOWN(src[x + 2])) + DOWN(src[x - 2]) + DOWN(src[x + 3]) + 16) >> 5);
        }
        buf[x << 1] = DOWN(src[x]);
        buf[x << 1 | 1] = OD_CLAMPTEST((20*(DOWN(src[x]) + DOWN(src[x + 1]))
                                      - 5*(DOWN(src[x - 1]) + DOWN(src[x + 2])) + DOWN(src[x - 2]) + DOWN(src[x + 2]) + 16) >> 5);
        x++;
        buf[x << 1] = DOWN(src[x]);
        buf[x << 1 | 1] = OD_CLAMPTEST((20*(DOWN(src[x]) + DOWN(src[x + 1]))
                                      - 5*(DOWN(src[x - 1]) + DOWN(src[x + 1])) + DOWN(src[x - 2]) + DOWN(src[x + 1]) + 16) >> 5);
        x++;
        buf[x << 1] = DOWN(src[x]);
        buf[x << 1 | 1] = OD_CLAMPTEST((36*DOWN(src[x])
                                      - 5*DOWN(src[x - 1]) + DOWN(src[x - 2]) + 16) >> 5);
        x++;
        buf[x << 1] = DOWN(src[w - 1]);
        buf[x << 1 | 1] = OD_CLAMPTEST((31*DOWN(src[w - 1]) + DOWN(src[w - 2]) + 16) >> 5);
        /*memset(buf + (++x << 1), src[w - 1], ((xpad - 1) << 1) * sizeof(*buf));*/
        for (x++; x < w + xpad; x++) {
          buf[x << 1] = DOWN(src[w - 1]);
          buf[x << 1 | 1] = DOWN(src[w - 1]);
        }
        if (y >= 0 && y + 1 < h) src += w;
      }
      /*Vertical filtering:*/
      if (y >= -ypad + 3) {
        if (y < 1 || y > h + 3) {
          /*OD_COPY(dst - (xpad << 1),
            state->ref_line_buf[(y - 3) & 7] - (xpad << 1),
            (w + (xpad << 1)) << 1);*/
          for (x = -xpad << 1; x < (w + xpad) << 1; x++) {
            *(dst+x) = UP(*(state->ref_line_buf[(y - 3) & 7]+x));
          }
          /*fprintf(stderr, "%3i: ", (y - 3) << 1);
          for (x = -xpad << 1; x < (w + xpad) << 1; x++) {
            fprintf(stderr, "%02X", *(dst + x));
          }
          fprintf(stderr, "\n");*/
          dst += (state->frame_buf_width << 1) >> xdec;
          /*OD_COPY(dst - (xpad << 1),
                  state->ref_line_buf[(y - 3) & 7] - (xpad << 1),
                  (w + (xpad << 1)) << 1);*/
          for (x = -xpad << 1; x < (w + xpad) << 1; x++) {
            *(dst+x) = UP(*(state->ref_line_buf[(y - 3) & 7]+x));
          }
          /*fprintf(stderr, "%3i: ", (y - 3) << 1 | 1);
          for (x = -xpad << 1; x < (w + xpad) << 1; x++) {
            fprintf(stderr, "%02X", *(dst + x));
          }
          fprintf(stderr, "\n");*/
          dst += (state->frame_buf_width << 1) >> xdec;
        }
        else {
          od_reftype *buf[6];
          buf[0] = state->ref_line_buf[(y - 5) & 7];
          buf[1] = state->ref_line_buf[(y - 4) & 7];
          buf[2] = state->ref_line_buf[(y - 3) & 7];
          buf[3] = state->ref_line_buf[(y - 2) & 7];
          buf[4] = state->ref_line_buf[(y - 1) & 7];
          buf[5] = state->ref_line_buf[(y - 0) & 7];
          for (x = -xpad << 1; x < (w + xpad) << 1; x++) {
            *(dst+x) = UP(*(state->ref_line_buf[(y - 3) & 7]+x));
          }
          /*OD_COPY(dst - (xpad << 1),
                  state->ref_line_buf[(y - 3) & 7] - (xpad << 1),
                  (w + (xpad << 1)) << 1);*/
          /*fprintf(stderr, "%3i: ", (y - 3) << 1);
          for (x = -xpad << 1; x < (w + xpad) << 1; x++) {
            fprintf(stderr, "%02X", *(dst + x));
          }
          fprintf(stderr, "\n");*/
          dst += (state->frame_buf_width << 1) >> xdec;
          for (x = -xpad << 1; x < (w + xpad) << 1; x++) {
            *(dst + x) = UP(
             (20*(*(buf[2] + x) + *(buf[3] + x))
             - 5*(*(buf[1] + x) + *(buf[4] + x))
             + *(buf[0] + x) + *(buf[5] + x) + 16) >> 5);
          }
          /*fprintf(stderr, "%3i: ", (y - 3) << 1 | 1);
          for (x = -xpad << 1; x < (w + xpad) << 1; x++) {
            fprintf(stderr, "%02X", *(dst + x));
          }
          fprintf(stderr, "\n");*/
          dst += (state->frame_buf_width << 1) >> xdec;
        }
      }
    }
  }
}

/*The data used to build the following two arrays.*/
const int OD_VERT_D[22] = {
/*0  1        4  5        8  9        12  13          17  18*/
  0, 0, 1, 1, 0, 0, 1, 2, 0, 0, 2, 1, 0, -1, 1, 1, 0, -1, 0, 1, 1, -1
};

/*The vector offsets in the X direction for each vertex from the upper-left,
   indexed by [exterior corner][split state][vertex].*/
const int *const OD_VERT_SETUP_DX[4][4] = {
  {
    OD_VERT_D + 9, OD_VERT_D + 1, OD_VERT_D + 9, OD_VERT_D + 1,
  },
  {
    OD_VERT_D + 13, OD_VERT_D + 13, OD_VERT_D + 1, OD_VERT_D + 1,
  },
  {
    OD_VERT_D + 18, OD_VERT_D + 1, OD_VERT_D + 18, OD_VERT_D + 1,
  },
  {
    OD_VERT_D + 5, OD_VERT_D + 5, OD_VERT_D + 1, OD_VERT_D + 1,
  }
};

/*The vector offsets in the Y direction for each vertex from the upper-left,
   indexed by [exterior corner][split state][vertex].*/
const int *const OD_VERT_SETUP_DY[4][4] = {
  {
    OD_VERT_DY + 4, OD_VERT_DY + 4, OD_VERT_DY + 0, OD_VERT_DY + 0,
  },
  {
    OD_VERT_DY + 8, OD_VERT_DY + 0, OD_VERT_DY + 8, OD_VERT_DY + 0,
  },
  {
    OD_VERT_DY + 12, OD_VERT_DY + 12, OD_VERT_DY + 0, OD_VERT_DY + 0,
  },
  {
    OD_VERT_DY + 17, OD_VERT_DY + 0, OD_VERT_DY + 17, OD_VERT_DY + 0,
  }
};

void od_state_pred_block_from_setup(od_state *state,
 od_reftype *buf, int ystride, int ref, int pli,
 int vx, int vy, int oc, int s, int log_mvb_sz) {
  od_reftype *refp;
  int xdec;
  int ydec;
  int ref_ystride;
  od_mv_grid_pt *grid[4];
  ogg_int32_t mvx[4];
  ogg_int32_t mvy[4];
  const int *dxp;
  const int *dyp;
  int x;
  int y;
  int k;
  refp = state->ref_imgs[state->ref_imgi[ref]].planes[pli];
  xdec = state->info.plane_info[pli].xdec;
  ydec = state->info.plane_info[pli].ydec;
  ref_ystride = (state->frame_buf_width << 1) >> xdec;
  dxp = OD_VERT_SETUP_DX[oc][s];
  dyp = OD_VERT_SETUP_DY[oc][s];
  for (k = 0; k < 4; k++) {
    grid[k] = state->mv_grid[vy + (dyp[k] << log_mvb_sz)]
     + vx + (dxp[k] << log_mvb_sz);
    mvx[k] = (ogg_int32_t)grid[k]->mv[0] << (14 - xdec);
    mvy[k] = (ogg_int32_t)grid[k]->mv[1] << (14 - ydec);
  }
  x = (vx - 2) << (3 - xdec);
  y = (vy - 2) << (3 - ydec);
  od_mc_predict(state, buf, ystride, refp + y*ref_ystride + x,
   ref_ystride, mvx, mvy, oc, s,
   log_mvb_sz + 2 - xdec, log_mvb_sz + 2 - ydec);
}

void od_state_pred_block(od_state *state, od_reftype *buf, int ystride,
 int ref, int pli, int vx, int vy, int log_mvb_sz) {
  int half_mvb_sz;
  half_mvb_sz = 1 << log_mvb_sz >> 1;
  if (log_mvb_sz > 0
   && state->mv_grid[vy + half_mvb_sz][vx + half_mvb_sz].valid) {
    int half_xblk_sz;
    int half_yblk_sz;
    half_xblk_sz = 1 << (log_mvb_sz + 1 - state->info.plane_info[pli].xdec);
    half_yblk_sz = 1 << (log_mvb_sz + 1 - state->info.plane_info[pli].ydec);
    od_state_pred_block(state, buf,
     ystride, ref, pli, vx, vy, log_mvb_sz - 1);
    od_state_pred_block(state, buf + half_xblk_sz,
     ystride, ref, pli, vx + half_mvb_sz, vy, log_mvb_sz - 1);
    od_state_pred_block(state, buf + half_yblk_sz*ystride,
     ystride, ref, pli, vx, vy + half_mvb_sz, log_mvb_sz - 1);
    od_state_pred_block(state, buf + half_yblk_sz*ystride + half_xblk_sz,
     ystride, ref, pli, vx + half_mvb_sz, vy + half_mvb_sz, log_mvb_sz - 1);
  }
  else {
    int oc;
    int s;
    if (log_mvb_sz < 2) {
      int mask;
      int s1vx;
      int s1vy;
      int s3vx;
      int s3vy;
      mask = (1 << (log_mvb_sz + 1)) - 1;
      oc = !!(vx & mask);
      if (vy & mask) oc = 3 - oc;
      s1vx = vx + (OD_VERT_DX[(oc + 1) & 3] << log_mvb_sz);
      s1vy = vy + (OD_VERT_DY[(oc + 1) & 3] << log_mvb_sz);
      s3vx = vx + (OD_VERT_DX[(oc + 3) & 3] << log_mvb_sz);
      s3vy = vy + (OD_VERT_DY[(oc + 3) & 3] << log_mvb_sz);
      s = state->mv_grid[s1vy][s1vx].valid |
       state->mv_grid[s3vy][s3vx].valid << 1;
    }
    else {
      oc = 0;
      s = 3;
    }
    od_state_pred_block_from_setup(state,
     buf, ystride, ref, pli, vx, vy, oc, s, log_mvb_sz);
  }
}

int od_state_dump_yuv(od_state *state, od_img *img, const char *tag) {
  static const char *CHROMA_TAGS[4] = {
    " C420jpeg", "", " C422jpeg", " C444"
  };
  char fname[1024];
  FILE *fp;
  int pic_width;
  int pic_height;
  int y;
  int pli;
  int needs_header;
#if defined(OD_DUMP_IMAGES) || defined(OD_DUMP_RECONS)
  int i;
  needs_header = 0;
  for (i = 0; i < state->dump_tags &&
    strcmp(tag,state->dump_files[i].tag) != 0; i++);
  if(i>=state->dump_tags) {
    const char *suf;
    OD_ASSERT(strlen(tag)<16);
    state->dump_tags++;
    state->dump_files = _ogg_realloc(state->dump_files,
     state->dump_tags*sizeof(od_yuv_dumpfile));
    OD_ASSERT(state->dump_files);
    strncpy(state->dump_files[i].tag,tag,16);
#else
  {
    const char *suf;
#endif
    needs_header = 1;
    suf = getenv("OD_DUMP_IMAGES_SUFFIX");
    if (!suf) {
      suf="";
    }
    sprintf(fname, "%08i%s-%s.y4m",
     (int)daala_granule_basetime(state, state->cur_time), tag, suf);
#if defined(OD_DUMP_IMAGES) || defined(OD_DUMP_RECONS)
    state->dump_files[i].fd = fopen(fname, "wb");
  }
  fp = state->dump_files[i].fd;
#else
    fp = fopen(fname, "wb");
  }
#endif
  pic_width = state->info.pic_width;
  pic_height = state->info.pic_height;
  OD_ASSERT(img->nplanes != 2);
  if (needs_header) {
    int fps_num;
    int fps_denom;
    const char *chroma;
    fps_num = state->info.timebase_numerator;
    fps_denom = state->info.timebase_denominator*state->info.frame_duration;
    chroma = img->nplanes == 1 ? " Cmono" :
     CHROMA_TAGS[(img->planes[1].xdec == 0) + (img->planes[1].ydec == 0)*2];
    fprintf(fp, "YUV4MPEG2 W%i H%i F%i:%i Ip A%i:%i%s\n",
     pic_width, pic_height, fps_num, fps_denom,
     state->info.pixel_aspect_numerator, state->info.pixel_aspect_denominator,
     chroma);
  }
  fprintf(fp, "FRAME\n");
  for (pli = 0; pli < OD_MINI(img->nplanes, 3); pli++) {
    int xdec;
    int ydec;
    int ystride;
    xdec = img->planes[pli].xdec;
    ydec = img->planes[pli].ydec;
    ystride = img->planes[pli].ystride;
    for (y = 0; y < (pic_height + ydec) >> ydec; y++) {
      if (fwrite(img->planes[pli].data + ystride*y,
       (pic_width + xdec) >> xdec, 1, fp) < 1) {
        fprintf(stderr, "Error writing to \"%s\".\n", fname);
        return OD_EFAULT;
      }
    }
  }
  return 0;
}

#if defined(OD_DUMP_IMAGES)
# include <png.h>
# include <zlib.h>

/*Dump a PNG of the reconstructed image, or a reference frame.*/
int od_state_dump_img(od_state *state, od_img *img, const char *tag) {
  png_structp png;
  png_infop info;
  png_bytep *data;
  FILE *fp;
  char fname[1024];
  unsigned char *p_rows[3];
  unsigned char *p[3];
  int nplanes;
  int pli;
  int x;
  int y;
  char *suf;
  suf = getenv("OD_DUMP_IMAGES_SUFFIX");
  if (!suf) {
    suf="";
  }
  sprintf(fname, "%08i%s%s.png",
   (int)daala_granule_basetime(state, state->cur_time), tag, suf);
  fp = fopen(fname, "wb");
  png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if (png == NULL) {
    fclose(fp);
    return OD_EFAULT;
  }
  info = png_create_info_struct(png);
  if (info == NULL) {
    png_destroy_write_struct(&png, NULL);
    fclose(fp);
    return OD_EFAULT;
  }
  if (setjmp(png_jmpbuf(png))) {
    png_destroy_write_struct(&png, &info);
    fclose(fp);
    return OD_EFAULT;
  }
  data = (png_bytep *)od_malloc_2d(img->height, 6*img->width, sizeof(**data));
  if (img->nplanes < 3) nplanes = 1;
  else nplanes = 3;
  for (pli = 0; pli < nplanes; pli++) p_rows[pli] = img->planes[pli].data;
  /*Chroma up-sampling is just done with a box filter.
    This is very likely what will actually be used in practice on a real
     display, and also removes one more layer to search in for the source of
     artifacts.
    As an added bonus, it's dead simple.*/
  for (y = 0; y < img->height; y++) {
    int mask;
    /*LOOP VECTORIZES.*/
    for (pli = 0; pli < nplanes; pli++) p[pli] = p_rows[pli];
    for (x = 0; x < img->width; x++) {
      float yval;
      float cbval;
      float crval;
      unsigned rval;
      unsigned gval;
      unsigned bval;
      /*This is intentionally slow and very accurate.*/
      yval = (p[0][0] - 16)*(1.0F/219);
      if (nplanes >= 3) {
        cbval = (p[1][0] - 128)*(2*(1 - 0.114F)/224);
        crval = (p[2][0] - 128)*(2*(1 - 0.299F)/224);
      }
      else cbval = crval = 0;
      rval = OD_CLAMPI(0, (int)(65535*(yval + crval) + 0.5F), 65535);
      gval = OD_CLAMPI(0, (int)(65535*
       (yval - cbval*(0.114F/0.587F) - crval*(0.299F/0.587F)) + 0.5F), 65535);
      bval = OD_CLAMPI(0, (int)(65535*(yval + cbval) + 0.5F), 65535);
      data[y][6*x + 0] = (unsigned char)(rval >> 8);
      data[y][6*x + 1] = (unsigned char)(rval & 0xFF);
      data[y][6*x + 2] = (unsigned char)(gval >> 8);
      data[y][6*x + 3] = (unsigned char)(gval & 0xFF);
      data[y][6*x + 4] = (unsigned char)(bval >> 8);
      data[y][6*x + 5] = (unsigned char)(bval & 0xFF);
      for (pli = 0; pli < nplanes; pli++) {
        mask = (1 << img->planes[pli].xdec) - 1;
        p[pli] += (x & mask) == mask;
      }
    }
    for (pli = 0; pli < nplanes; pli++) {
      mask = (1 << img->planes[pli].ydec) - 1;
      p_rows[pli] += ((y & mask) == mask)*img->planes[pli].ystride;
    }
  }
  png_init_io(png, fp);
  png_set_compression_level(png, Z_DEFAULT_COMPRESSION);
  png_set_IHDR(png, info, img->width, img->height, 16, PNG_COLOR_TYPE_RGB,
   PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
  /*TODO: Define real colorspace.*/
  /*png_set_gAMA(png, info, 2.2);
  png_set_cHRM_fixed(png, info, 31006, 31616, 67000, 32000,
   21000, 71000, 14000, 8000);*/
  png_set_pHYs(png, info, state->info.pixel_aspect_numerator,
   state->info.pixel_aspect_denominator, 0);
  png_set_rows(png, info, data);
  png_write_png(png, info, PNG_TRANSFORM_IDENTITY, NULL);
  png_write_end(png, info);
  png_destroy_write_struct(&png, &info);
  od_free_2d(data);
  fclose(fp);
  return 0;
}

int od_state_dump_ref(od_state *state, od_reference *ref, const char *tag) {
  png_structp png;
  png_infop info;
  png_bytep *data;
  FILE *fp;
  char fname[1024];
  od_reftype *p_rows[3];
  od_reftype *p[3];
  int nplanes;
  int pli;
  int x;
  int y;
  int h;
  int w;
  char *suf;
  suf = getenv("OD_DUMP_IMAGES_SUFFIX");
  if (!suf) {
    suf="";
  }
  sprintf(fname, "%08i%s%s.png",
   (int)daala_granule_basetime(state, state->cur_time), tag, suf);
  fp = fopen(fname, "wb");
  png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if (png == NULL) {
    fclose(fp);
    return OD_EFAULT;
  }
  info = png_create_info_struct(png);
  if (info == NULL) {
    png_destroy_write_struct(&png, NULL);
    fclose(fp);
    return OD_EFAULT;
  }
  if (setjmp(png_jmpbuf(png))) {
    png_destroy_write_struct(&png, &info);
    fclose(fp);
    return OD_EFAULT;
  }
  w = state->frame_width << 1;
  h = state->frame_height << 1;
  data = (png_bytep *)od_malloc_2d(h, 6*w, sizeof(**data));
  if (state->info.nplanes < 3) nplanes = 1;
  else nplanes = 3;
  for (pli = 0; pli < nplanes; pli++) p_rows[pli] = ref->planes[pli];
  /*Chroma up-sampling is just done with a box filter.
    This is very likely what will actually be used in practice on a real
     display, and also removes one more layer to search in for the source of
     artifacts.
    As an added bonus, it's dead simple.*/
  for (y = 0; y < h; y++) {
    int mask;
    /*LOOP VECTORIZES.*/
    for (pli = 0; pli < nplanes; pli++) p[pli] = p_rows[pli];
    for (x = 0; x < w; x++) {
      float yval;
      float cbval;
      float crval;
      unsigned rval;
      unsigned gval;
      unsigned bval;
      int coeff_shift;
      coeff_shift = OD_REFERENCE_BITS-8;
      /*This is intentionally slow and very accurate.*/
#if OD_REFERENCE_BYTES==1
      yval = (((p[0][0] + (1 << coeff_shift >> 1)) >> coeff_shift)
       - 16)*(1.0F/219);
      if (nplanes >= 3) {
        cbval = (((p[1][0] + (1 << coeff_shift >> 1)) >> coeff_shift)
         - 128)*(2*(1 - 0.114F)/224);
        crval = (((p[2][0] + (1 << coeff_shift >> 1)) >> coeff_shift)
         - 128)*(2*(1 - 0.299F)/224);
      }
#else
      yval = (((p[0][0] + (1 << coeff_shift >> 1)) >> coeff_shift)
       + 128 - 16)*(1.0F/219);
      if (nplanes >= 3) {
        cbval = ((p[1][0] + (1 << coeff_shift >> 1)) >> coeff_shift)
         *(2*(1 - 0.114F)/224);
        crval = ((p[2][0] + (1 << coeff_shift >> 1)) >> coeff_shift)
         *(2*(1 - 0.299F)/224);
      }
#endif
      else cbval = crval = 0;
      rval = OD_CLAMPI(0, (int)(65535*(yval + crval) + 0.5F), 65535);
      gval = OD_CLAMPI(0, (int)(65535*
       (yval - cbval*(0.114F/0.587F) - crval*(0.299F/0.587F)) + 0.5F), 65535);
      bval = OD_CLAMPI(0, (int)(65535*(yval + cbval) + 0.5F), 65535);
      data[y][6*x + 0] = (unsigned char)(rval >> 8);
      data[y][6*x + 1] = (unsigned char)(rval & 0xFF);
      data[y][6*x + 2] = (unsigned char)(gval >> 8);
      data[y][6*x + 3] = (unsigned char)(gval & 0xFF);
      data[y][6*x + 4] = (unsigned char)(bval >> 8);
      data[y][6*x + 5] = (unsigned char)(bval & 0xFF);
      for (pli = 0; pli < nplanes; pli++) {
        int xdec;
        xdec = state->info.plane_info[pli].xdec;
        mask = (1 << xdec) - 1;
        p[pli] += (x & mask) == mask;
      }
    }
    for (pli = 0; pli < nplanes; pli++) {
      int xdec;
      int ydec;
      xdec = state->info.plane_info[pli].xdec;
      ydec = state->info.plane_info[pli].ydec;
      mask = (1 << ydec) - 1;
      p_rows[pli] += ((y & mask) == mask)
       *((state->frame_buf_width << 1) >> xdec);
    }
  }
  png_init_io(png, fp);
  png_set_compression_level(png, Z_DEFAULT_COMPRESSION);
  png_set_IHDR(png, info, w, h, 16, PNG_COLOR_TYPE_RGB,
   PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
  /*TODO: Define real colorspace.*/
  /*png_set_gAMA(png, info, 2.2);
  png_set_cHRM_fixed(png, info, 31006, 31616, 67000, 32000,
   21000, 71000, 14000, 8000);*/
  png_set_pHYs(png, info, state->info.pixel_aspect_numerator,
   state->info.pixel_aspect_denominator, 0);
  png_set_rows(png, info, data);
  png_write_png(png, info, PNG_TRANSFORM_IDENTITY, NULL);
  png_write_end(png, info);
  png_destroy_write_struct(&png, &info);
  od_free_2d(data);
  fclose(fp);
  return 0;
}
#endif

void od_state_mc_predict(od_state *state, int ref) {
  int nhmvbs;
  int nvmvbs;
  int pli;
  int vx;
  int vy;
  nhmvbs = (state->nhmbs + 1) << 2;
  nvmvbs = (state->nvmbs + 1) << 2;
  for (vy = 0; vy < nvmvbs; vy += 4) {
    for (vx = 0; vx < nhmvbs; vx += 4) {
      for (pli = 0; pli < state->info.nplanes; pli++) {
        daala_plane_info *iplane;
        od_coeff *dst;
        od_reftype *src;
        int blk_w;
        int blk_h;
        int blk_x;
        int blk_y;
        int w;
        int h;
        int x;
        int y;
        iplane = state->info.plane_info+pli;
        od_state_pred_block(state,
         state->mc_buf[4], OD_MCBSIZE_MAX, ref, pli, vx, vy, 2);
        /*Copy the predictor into the motion-compensation temp buffer,
          with clipping.*/
        w = state->frame_width >> iplane->xdec;
        h = state->frame_height >> iplane->ydec;
        blk_w = 16 >> iplane->xdec;
        blk_h = 16 >> iplane->ydec;
        blk_x = (vx - 2) << (2 - iplane->xdec);
        blk_y = (vy - 2) << (2 - iplane->ydec);
        src = state->mc_buf[4];
        if (blk_x < 0) {
          blk_w += blk_x;
          src -= blk_x;
          blk_x = 0;
        }
        if (blk_y < 0) {
          blk_h += blk_y;
          src -= blk_y*OD_MCBSIZE_MAX;
          blk_y = 0;
        }
        if (blk_x + blk_w > w) {
          blk_w = w - blk_x;
        }
        if (blk_y + blk_h > h) {
          blk_h = h - blk_y;
        }
        dst = state->mctmp[pli]+blk_y*w + blk_x;
        for (y = 0; y < blk_h; y++) {
          for (x = 0; x < blk_w; x++) {
            dst[x] = src[x];
          }
          src += OD_MCBSIZE_MAX;
          dst += w;
        }
      }
    }
  }
}

/*To avoiding having to special-case superblocks on the edges of the image,
   one superblock of padding is maintained on each side of the image.
  These "dummy" superblocks are notionally not subdivided.
  See the comment for the `bsize` member of `od_state` for more information
   about the data layout and meaning.*/
void od_state_init_border(od_state *state) {
  int i;
  int j;
  int nhsb;
  int nvsb;
  unsigned char *bsize;
  int bstride;
  nhsb = state->nhsb;
  nvsb = state->nvsb;
  bsize = state->bsize;
  bstride = state->bstride;
  for (i = -4; i < (nhsb+1)*4; i++) {
    for (j = -4; j < 0; j++) {
      bsize[(j*bstride) + i] = OD_LIMIT_BSIZE_MAX;
    }
    for (j = nvsb*4; j < (nvsb+1)*4; j++) {
      bsize[(j*bstride) + i] = OD_LIMIT_BSIZE_MAX;
    }
  }
  for (j = -4; j < (nvsb+1)*4; j++) {
    for (i = -4; i < 0; i++) {
      bsize[(j*bstride) + i] = OD_LIMIT_BSIZE_MAX;
    }
    for (i = nhsb*4; i < (nhsb+1)*4; i++) {
      bsize[(j*bstride) + i] = OD_LIMIT_BSIZE_MAX;
    }
  }
}

ogg_int64_t daala_granule_basetime(void *encdec, ogg_int64_t granpos) {
  od_state *state;
  state = (od_state *)encdec;
  if (granpos >= 0) {
    ogg_int64_t key_time;
    ogg_int64_t delta_time;
    key_time = granpos >> state->info.keyframe_granule_shift;
    delta_time = granpos - (key_time << state->info.keyframe_granule_shift);
    return key_time + delta_time;
  }
  return -1;
}

double daala_granule_time(void *encdec, ogg_int64_t granpos) {
  od_state *state;
  ogg_int64_t base_time;
  state = (od_state *)encdec;
  base_time = daala_granule_basetime(encdec, granpos);
  if (base_time >= 0) {
    return base_time*(double)state->info.timebase_denominator/
     state->info.timebase_numerator;
  }
  return -1;
}
