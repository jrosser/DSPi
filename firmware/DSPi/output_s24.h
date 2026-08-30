#pragma once

// ----------------------------------------------------------------------------
// Float -> S24 finalization of output blocks (RP2350).
//
// Two modes, chosen once per packet from the ADAT-active snapshot (both cores
// use the same snapshot via core1_eq_work.finalize_s24):
//
// ADAT active: after the last float consumer (EQ/gain, loudness, delay, peak
// metering), each output channel row of buf_out is converted IN PLACE to a
// clamped 24-bit sample held in an int32.  The slot interleave and the ADAT
// encoder both read this integer form, so the clamp/scale runs exactly once
// per sample per channel instead of once per consumer.
//
// ADAT inactive: the staging pass is skipped and each slot pair is converted
// and interleaved in one fused pass (the rows stay float), avoiding the
// second memory pass when nothing else consumes the integer form.  Both modes
// produce bit-identical slot bytes; only the number of passes differs.
//
// The rows are declared float; every integer access to them must go through
// out_s24_t (may_alias) so GCC preserves cross-type access ordering.
// ----------------------------------------------------------------------------

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

typedef int32_t out_s24_t __attribute__((may_alias));

static inline void output_block_to_s24_inplace(float *buf, uint32_t n) {
    out_s24_t *dst = (out_s24_t *)buf;
    for (uint32_t i = 0; i < n; i++) {
        // Clamp floats to +/-1.0 to avoid overflow when converting
        // scaled floats to integers
        float f = fmaxf(-1.0f, fminf(1.0f, buf[i]));

        // Use 2^N rather than (2^N)-1 to avoid scaling error
        f = f * 8388608.0f;

        // 0.0f means "an analogue value between +0.5f and -0.5f"
        f += 0.5f;

        // Convert float to signed int with forced rounding mode
        // toward negative infinity (floor rounding).
        uint32_t s;
        __asm__ volatile (
            "vcvtm.s32.f32 %[F], %[F]\n\t"
            "vmov %[S], %[F]\n\t"
            : [S] "=r" (s)
            : [F] "w" (f)
        );

        // Clamp to maximum values for a 24 bit signed integer
        if(s > 8388607) s = 8388607; if(s < -8388608) s = -8388608;

        dst[i] = s;
    }
}

// Integer interleave of two rows already finalized by
// output_block_to_s24_inplace() (ADAT-active mode).
static inline void output_pair_interleave_s24(int32_t *out_ptr,
                                              const float *l, const float *r,
                                              uint32_t n) {
    const out_s24_t *sl = (const out_s24_t *)l;
    const out_s24_t *sr = (const out_s24_t *)r;
    for (uint32_t i = 0; i < n; i++) {
        out_ptr[i*2]   = sl[i];
        out_ptr[i*2+1] = sr[i];
    }
}

// Fused convert + interleave of two float rows (ADAT-inactive mode; the rows
// are left untouched).
static inline void output_pair_convert_interleave(int32_t *out_ptr,
                                                  const float *l, const float *r,
                                                  uint32_t n) {
    for (uint32_t i = 0; i < n; i++) {
        // Clamp floats to +/-1.0 to avoid overflow when converting
        // scaled floats to integers
        float fl = fmaxf(-1.0f, fminf(1.0f, l[i]));
        float fr = fmaxf(-1.0f, fminf(1.0f, r[i]));

        // Use 2^N rather than (2^N)-1 to avoid scaling error
        fl = fl * 8388608.0f;
        fr = fr * 8388608.0f;

        // 0.0f means "an analogue value between +0.5f and -0.5f"
        fl += 0.5f;
        fr += 0.5f;

        // Convert float to signed int with forced rounding mode
        // toward negative infinity (floor rounding).
        int32_t sl, sr;
        __asm__ volatile (
            "vcvtm.s32.f32 %[FL], %[FL]\n\t"
            "vmov %[SL], %[FL]\n\t"
            "vcvtm.s32.f32 %[FR], %[FR]\n\t"
            "vmov %[SR], %[FR]\n\t"
            : [SL] "=r" (sl),
            [SR] "=r" (sr)
            : [FL] "w" (fl),
            [FR] "w" (fr)
        );

        // Clamp to maximum values for a 24 bit signed integer
        if(sl > 8388607) sl = 8388607; if(sl < -8388608) sl = -8388608;
        if(sr > 8388607) sr = 8388607; if(sr < -8388608) sr = -8388608;

        out_ptr[i*2]   = sl;
        out_ptr[i*2+1] = sr;
    }
}
