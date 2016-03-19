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

#if !defined(_encint_H)
# define _encint_H (1)

typedef struct daala_enc_ctx od_enc_ctx;
typedef struct od_params_ctx od_params_ctx;
typedef struct od_mv_est_ctx od_mv_est_ctx;
typedef struct od_enc_opt_vtbl od_enc_opt_vtbl;
typedef struct od_rollback_buffer od_rollback_buffer;
typedef struct od_input_queue od_input_queue;
typedef struct od_input_frame od_input_frame;

# include "../include/daala/daaladec.h"
# include "../include/daala/daalaenc.h"
# include "state.h"
# include "entenc.h"
# include "block_size_enc.h"

/*Constants for the packet state machine specific to the encoder.*/
/*No packet currently ready to output.*/
# define OD_PACKET_EMPTY       (0)
/*A packet ready to output.*/
# define OD_PACKET_READY       (1)
/*The number of fractional bits of precision in our \lambda values.*/
# define OD_LAMBDA_SCALE       (2)
/*The number of bits of precision to add to distortion values to match
   \lambda*R.*/
# define OD_ERROR_SCALE        (OD_LAMBDA_SCALE + OD_BITRES)

/*The complexity setting where we enable MV refinement.*/
# define OD_MC_REFINEMENT_COMPLEXITY (5)
/*The complexity setting where we enable a square pattern in basic (fullpel)
   MV refinement.*/
# define OD_MC_SQUARE_REFINEMENT_COMPLEXITY (8)
/*The complexity setting where we enable logarithmic (telescoping) MV
   refinement.*/
# define OD_MC_LOGARITHMIC_REFINEMENT_COMPLEXITY (9)
/*The complexity setting where we switch to a square pattern in subpel
   refinement.*/
# define OD_MC_SQUARE_SUBPEL_REFINEMENT_COMPLEXITY (10)

struct od_enc_opt_vtbl {
  int32_t (*mc_compute_sad_4x4)(const unsigned char *src,
   int systride, const unsigned char *ref, int dystride);
  int32_t (*mc_compute_sad_8x8)(const unsigned char *src,
   int systride, const unsigned char *ref, int dystride);
  int32_t (*mc_compute_sad_16x16)(const unsigned char *src,
   int systride, const unsigned char *ref, int dystride);
  int32_t (*mc_compute_sad_32x32)(const unsigned char *src,
   int systride, const unsigned char *ref, int dystride);
  int32_t (*mc_compute_sad_64x64)(const unsigned char *src,
   int systride, const unsigned char *ref, int dystride);
  int32_t (*mc_compute_satd_4x4)(const unsigned char *src,
   int systride, const unsigned char *ref, int dystride);
  int32_t (*mc_compute_satd_8x8)(const unsigned char *src,
   int systride, const unsigned char *ref, int dystride);
  int32_t (*mc_compute_satd_16x16)(const unsigned char *src,
   int systride, const unsigned char *ref, int dystride);
  int32_t (*mc_compute_satd_32x32)(const unsigned char *src,
   int systride, const unsigned char *ref, int dystride);
  int32_t (*mc_compute_satd_64x64)(const unsigned char *src,
   int systride, const unsigned char *ref, int dystride);
};

/*Unsanitized user parameters*/
struct od_params_ctx {
  /*Set using OD_SET_MV_LEVEL_MIN*/
  int mv_level_min;
  /*Set using OD_SET_MV_LEVEL_MAX*/
  int mv_level_max;
};

struct od_input_frame {
  daala_image *img;
  int duration;
  int type;
  int number;
  /* TODO add reference info */
};

struct od_input_queue {
  unsigned char *input_img_data;

  /* Circular queue of frame input images in display order. */
  daala_image images[OD_MAX_REORDER];
  int duration[OD_MAX_REORDER];
  int input_head;
  int input_size;

  /* Circular queue of frame indeces in encode order. */
  od_input_frame frames[OD_MAX_REORDER];
  int encode_head;
  int encode_size;

  /* Input queue parameters */
  int keyframe_rate;
  int frame_delay;

  /* Input queue state */
  int frame_number;
  int last_keyframe;
  int end_of_input;
  int closed_gop;
};

struct daala_enc_ctx{
  od_state state;
  od_enc_opt_vtbl opt_vtbl;
  oggbyte_buffer obb;
  od_ec_enc ec;
  int packet_state;
  int quality[OD_NPLANES_MAX];
  int complexity;
  int use_activity_masking;
  int use_dering;
  int use_satd;
  int qm;
  int use_haar_wavelet;
  int b_frames;
  od_mv_est_ctx *mvest;
  od_params_ctx params;
#if defined(OD_ENCODER_CHECK)
  struct daala_dec_ctx *dec;
#endif
#if defined(OD_DUMP_BSIZE_DIST)
  /* per frame */
  double bsize_dist[OD_NPLANES_MAX];
  /* per encoder lifetime */
  double bsize_dist_total[OD_NPLANES_MAX];
  FILE *bsize_dist_file;
#endif
  od_block_size_comp *bs;
  /* These buffers are for saving pixel data during block size RDO. */
  od_coeff mc_orig[OD_NBSIZES-1][OD_BSIZE_MAX*OD_BSIZE_MAX];
  od_coeff c_orig[OD_NBSIZES-1][OD_BSIZE_MAX*OD_BSIZE_MAX];
  od_coeff nosplit[OD_NBSIZES-1][OD_BSIZE_MAX*OD_BSIZE_MAX];
  od_coeff split[OD_NBSIZES-1][OD_BSIZE_MAX*OD_BSIZE_MAX];
  od_coeff block_c_orig[OD_BSIZE_MAX*OD_BSIZE_MAX];
  od_coeff block_mc_orig[OD_BSIZE_MAX*OD_BSIZE_MAX];
  od_coeff block_c_noskip[OD_BSIZE_MAX*OD_BSIZE_MAX];
  /* This structure manages reordering the input frames from display order
      to encode order.
     It currently supports in order B-frames with a periodic out of order
      P-frame specified by enc->b_frames.*/
  od_input_queue input_queue;
  /* A pointer to the currently encoding image. */
  daala_image *curr_img;
  /** Frame delay. */
  int frame_delay;
  /** Displaying order of current frame being encoded. */
  int64_t curr_display_order;
  /** Coding order of current frame being encoded. */
  int64_t curr_coding_order;
  /** Number of I or P frames encoded so far, starting from zero. */
  int64_t ip_frame_count;
#if defined(OD_DUMP_RECONS)
  od_output_queue out;
#endif
#if defined(OD_DUMP_IMAGES)
  unsigned char *dump_img_data;
  daala_image vis_img;
  daala_image tmp_vis_img;
  unsigned char *upsample_line_buf[8];
# if defined(OD_ANIMATE)
  int ani_iter;
# endif
#endif
};

/** Holds important encoder information so we can roll back decisions */
struct od_rollback_buffer {
  od_ec_enc ec;
  od_adapt_ctx adapt;
};

int od_frame_type(daala_enc_ctx *enc, int64_t coding_frame_count,
 int *is_golden, int64_t *ip_count);
void od_encode_checkpoint(const daala_enc_ctx *enc, od_rollback_buffer *rbuf);
void od_encode_rollback(daala_enc_ctx *enc, const od_rollback_buffer *rbuf);

od_mv_est_ctx *od_mv_est_alloc(od_enc_ctx *enc);
void od_mv_est_free(od_mv_est_ctx *est);
void od_mv_est(od_mv_est_ctx *est, int lambda, int num_refs);

int32_t od_mc_compute_sad8_4x4_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_sad8_8x8_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_sad8_16x16_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_sad8_32x32_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_sad8_64x64_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_sad8_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride, int w, int h);
int32_t od_mc_compute_satd8_4x4_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_satd8_8x8_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_satd8_16x16_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_satd8_32x32_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_satd8_64x64_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_sad16_4x4_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_sad16_8x8_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_sad16_16x16_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_sad16_32x32_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_sad16_64x64_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_sad16_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride, int w, int h);
int32_t od_mc_compute_satd16_4x4_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_satd16_8x8_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_satd16_16x16_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_satd16_32x32_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
int32_t od_mc_compute_satd16_64x64_c(const unsigned char *src, int systride,
 const unsigned char *ref, int dystride);
void od_enc_opt_vtbl_init_c(od_enc_ctx *enc);

# if defined(OD_DUMP_IMAGES)
void od_encode_fill_vis(daala_enc_ctx *enc);
void daala_image_draw_line(daala_image *img, int x0, int y0, int x1, int y1,
 const unsigned char ycbcr[3]);
void od_state_draw_mvs(daala_enc_ctx *enc);
# endif

# if defined(OD_X86ASM)
void od_enc_opt_vtbl_init_x86(od_enc_ctx *enc);
# endif

#endif
