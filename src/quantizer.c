/*Daala video codec
Copyright (c) 2015 Daala project contributors.  All rights reserved.

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

#include "internal.h"
#include "quantizer.h"
#include "mathops.h"

/*Daala codes 64 unique possible quantizers that are spaced out over a roughly
   logarithmic range.
  The table below maps coded quantizer values to actual quantizer values.
  After 0, which indicates lossless, quantizers are computed by
   trunc(e(((coded_quantizer)-6.235)*.10989525)*(1<<4)), which also happens to
   feature strictly equal-or-increasing interval spacing.
  That is, the interval between any two quantizers will never be smaller
   than a preceding interval.
  This gives us a log-spaced range of 9 to 8191 (representing .5625 to
   511.9375 coded in Q4), with quantizer slightly less than doubling every
   six steps.
  The starting point of 9 is not arbitrary; it represents the finest quantizer
   greater than .5 for a COEFF_SHIFT of 4, and results in a lossy encoding
   bitrate typically between between half and 3/4 the bitrate of lossless.*/
static const int OD_CODED_QUANTIZER_MAP_Q4[64]={
  /*0*/
  0x0000,
  /*1*/
  0x0009, 0x000A, 0x000B, 0x000C, 0x000D, 0x000F,
  /*7*/
  0x0011, 0x0013, 0x0015, 0x0018, 0x001B, 0x001E,
  /*13*/
  0x0021, 0x0024, 0x0029, 0x002E, 0x0034, 0x003A,
  /*19*/
  0x0041, 0x0048, 0x0051, 0x005A, 0x0064, 0x0070,
  /*25*/
  0x007D, 0x008C, 0x009C, 0x00AE, 0x00C3, 0x00D9,
  /*31*/
  0x00F3, 0x010F, 0x012F, 0x0152, 0x0179, 0x01A5,
  /*37*/
  0x01D6, 0x020D, 0x0249, 0x028E, 0x02DA, 0x032E,
  /*43*/
  0x038D, 0x03F7, 0x046D, 0x04F0, 0x0583, 0x0627,
  /*49*/
  0x06De, 0x07AA, 0x088E, 0x098D, 0x0AA9, 0x0BE6,
  /*55*/
  0x0D48, 0x0ED3, 0x108C, 0x1278, 0x149D, 0x1702,
  /*61*/
  0x19AE, 0x1CAA, 0x1FFF
};

/* Q57 log scale representation of the above values, as computed by
    od_blog64(x)-OD_Q57(4).
   The -OD_Q57(4) term is to undo the fractional shift.*/
static const int64_t OD_LOG_QUANTIZER_MAP_Q57[64]={
  /*0...
    The 'zero' value is actually a guess/hack toward having rate control
     do something sensible once we try to mix lossless/lossy in rate
     controlled streams. */
  0x060000000000000FLL,
  /*1...*/
  0x06570068E7EF5A1FLL, 0x06A4D3C25E68DC58LL,
  0x06EB3A9F01975078LL, 0x072B803473F7AD0FLL,
  0x0766A008E4788CBDLL, 0x07D053F6D2608967LL,
  0x082CC7EDF592262DLL, 0x087EF05AE409A029LL,
  0x08C8DDD448F8B845LL, 0x092B803473F7AD0FLL,
  0x0982809D5BE7072ELL, 0x09D053F6D2608967LL,
  0x0A16BAD3758EFD88LL, 0x0A570068E7EF5A1FLL,
  0x0AB7110E6CE866F3LL, 0x0B0C10500D63AA65LL,
  0x0B66A008E4788CBDLL, 0x0BB74948F5532DA5LL,
  0x0C0B73CB42E16915LL, 0x0C570068E7EF5A1FLL,
  0x0CAE00D1CFDEB43DLL, 0x0CFBD42B46583676LL,
  0x0D49A784BCD1B8B0LL, 0x0D9D5D9FD5010B37LL,
  0x0DEE7B471B3A9508LL, 0x0E4231623369E78FLL,
  0x0E92203D587039CCLL, 0x0EE2C97D694ADAB5LL,
  0x0F36F3FFB6D91624LL, 0x0F85EA0B0B27B261LL,
  0x0FD9810643D6614CLL, 0x102A0F706C06776ELL,
  0x107C814CE4823C9DLL, 0x10CD4011C8F11979LL,
  0x111DE951D9CBBA62LL, 0x116F7348BC5C617ALL,
  0x11C0C6D447C5DD36LL, 0x1212855905CA70F6LL,
  0x126274342AD0C333LL, 0x12B4CFA92466A5C3LL,
  0x130604719F24EB24LL, 0x13567817B86B02C8LL,
  0x13A801815879E9ACLL, 0x13F97AAB28BD1533LL,
  0x144AB79E4377EB64LL, 0x149B892675266F67LL,
  0x14ECCD190E4308CFLL, 0x153E058D79C21E05LL,
  0x158F376014CC5ABELL, 0x15E04FDD985E52D2LL,
  0x163184A6131C9833LL, 0x1682E075F59C669FLL,
  0x16D40C5821CE06AFLL, 0x172538F6CE5973FELL,
  0x17766F72B263DEDDLL, 0x17C79EE5E15B8445LL,
  0x1818D345842A2AD7LL, 0x1869FDC7EDEEAE76LL,
  0x18BB231D0101AB34LL, 0x190C5088789923ABLL,
  0x195D77F2334621AELL, 0x19AEADFD7851E0CELL,
  /*63*/
  0x19FFE8EA5C43CA6CLL,
};

const int OD_N_CODED_QUANTIZERS =
 sizeof(OD_CODED_QUANTIZER_MAP_Q4)/sizeof(*OD_CODED_QUANTIZER_MAP_Q4);

/*Maps coded quantizer to actual quantizer value.*/
int od_codedquantizer_to_quantizer(int cq) {
  /*The quantizers above are sensible for a COEFF_SHIFT of 4 or
     greater.
    ASSERT just in case we ever try to use them for COEFF_SHIFT < 4,
     scale to COEFF_SHIFT for COEFF_SHIFT > 4.*/
  OD_ASSERT(OD_COEFF_SHIFT >= 4);
  if (cq == 0) return 0;
  return cq < OD_N_CODED_QUANTIZERS
   ? (OD_CODED_QUANTIZER_MAP_Q4[cq]
   << OD_COEFF_SHIFT >> 4)
   : (OD_CODED_QUANTIZER_MAP_Q4[OD_N_CODED_QUANTIZERS-1]
   << OD_COEFF_SHIFT >> 4);
}

/* Used in rate control to guess bit usage.
   Does not special-case cq == 0 as we may use it for mixed control. */
int64_t od_codedquantizer_to_logquantizer(int cq) {
  OD_ASSERT(OD_COEFF_SHIFT >= 4);
  return cq < OD_N_CODED_QUANTIZERS
   ? OD_LOG_QUANTIZER_MAP_Q57[cq] + OD_Q57(OD_COEFF_SHIFT - 4)
   : OD_LOG_QUANTIZER_MAP_Q57[OD_N_CODED_QUANTIZERS-1] +
   OD_Q57(OD_COEFF_SHIFT - 4);
}

/*Maps a quantizer to the coded quantizer with a mapped value
   closest to the one passed in, except for values between 0
   (lossless) and the minimum lossy quantizer, in which case the
   minimum lossy quantizer is returned.
  In the event of a tie, we return the smaller cq.*/
int od_quantizer_to_codedquantizer(int q){
  if (q == 0) {
    return 0;
  }
  else {
    int hi;
    int lo;
    hi = OD_N_CODED_QUANTIZERS;
    lo = 1;
    /*In the event OD_COEFF_SHIFT > 4, scale the passed in quantizer
      down to Q4 from matching the shift.*/
    q = q << 4 >> OD_COEFF_SHIFT;
    while (hi > lo + 1) {
      unsigned mid;
      mid = (hi + lo) >> 1;
      if (q < OD_CODED_QUANTIZER_MAP_Q4[mid]) {
        hi = mid;
      }
      else {
        lo = mid;
      }
    }
    /*lo maps to the largest quantizer less than or equal to q.
      hi maps to either the smallest quantizer greater than q, or one past
      the end of the quantizers array.*/
    if(hi < OD_N_CODED_QUANTIZERS) {
      int lodist = q - OD_CODED_QUANTIZER_MAP_Q4[lo];
      int hidist = OD_CODED_QUANTIZER_MAP_Q4[hi] - q;
      if (lodist > hidist) return hi;
    }
    return lo;
  }
}

/*Maps a log-domain quantizer to the coded quantizer with a mapped value
   closest (in the log domain) to the one passed in.
  Unlike the linear-domain mapper, cq 0 is not special-cased as we may
   eventually want to mix it into rate controlled streams.
  In the event of a tie, we return the smaller cq.*/
int od_logquantizer_to_codedquantizer(int64_t lq){
  int hi;
  int lo;
  hi = OD_N_CODED_QUANTIZERS;
  lo = 0;
  /*In the event OD_COEFF_SHIFT > 4, scale the passed in quantizer
    down to Q4 from matching the shift.*/
  lq -= OD_Q57(OD_COEFF_SHIFT - 4);
  while (hi > lo + 1) {
    unsigned mid;
    mid = (hi + lo) >> 1;
    if (lq < OD_LOG_QUANTIZER_MAP_Q57[mid]) {
      hi = mid;
    }
    else {
      lo = mid;
    }
  }
  /*lo maps to the largest quantizer less than or equal to lq.
    hi maps to either the smallest quantizer greater than lq, or one past
    the end of the quantizers array.*/
  if(hi < OD_N_CODED_QUANTIZERS) {
    int64_t lodist = lq - OD_LOG_QUANTIZER_MAP_Q57[lo];
    int64_t hidist = OD_LOG_QUANTIZER_MAP_Q57[hi] - lq;
    if (lodist > hidist) return hi;
  }
  return lo;
}

