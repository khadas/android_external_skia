/*
 * Copyright 2015 Google Inc.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#ifndef SkBlitRow_opts_DEFINED
#define SkBlitRow_opts_DEFINED

#include "Sk4px.h"
#include "SkUtils.h"

namespace SK_OPTS_NS {

// Color32 uses the blend_256_round_alt algorithm from tests/BlendTest.cpp.
// It's not quite perfect, but it's never wrong in the interesting edge cases,
// and it's quite a bit faster than blend_perfect.
//
// blend_256_round_alt is our currently blessed algorithm.  Please use it or an analogous one.
static void blit_row_color32(SkPMColor* dst, const SkPMColor* src, int count, SkPMColor color) {
#if 0
    unsigned invA = 255 - SkGetPackedA32(color);
    invA += invA >> 7;
    SkASSERT(invA < 256);  // We've should have already handled alpha == 0 externally.

    Sk16h colorHighAndRound = Sk4px::DupPMColor(color).widenHi() + Sk16h(128);
    Sk16b invA_16x(invA);

    Sk4px::MapSrc(count, dst, src, [&](const Sk4px& src4) -> Sk4px {
        return (src4 * invA_16x).addNarrowHi(colorHighAndRound);
    });
#else
    if (count > 0) {
        if (0 == color) {
            if (src != dst) {
                memcpy(dst, src, count * sizeof(SkPMColor));
            }
            return;
        }
        unsigned colorA = SkGetPackedA32(color);
        if (255 == colorA) {
            sk_memset32(dst, color, count);
        } else {
            unsigned scale = 256 - SkAlpha255To256(colorA);
        #if defined(__arm__)
            /*
             * tao.zeng@amlogic.com, use NEON to optimize these funciton
             */
            // tao.zeng, alpha will not be 256, because SkAlpha255To256(colorA);
            asm volatile (
                /*
                 * regigster allocate
                 * q0   --> scale
                 * q1   --> wide color
                 * q2 - q3 --> src0 -- src8
                 */
                "vdup.32    d2, %[color]                        \n"
                "vdup.8     q0, %[scale]                        \n"     // q0 = [scale].8
                "pld        [%[src], #128]                      \n"     // preload data
                "cmp        %[count], #8                        \n"
                "vshll.u8   q1, d2, #8                          \n"     // q1 = [color 00].16
                "blt        SkBlitRow_Color32_less_8            \n"
                // main loop
            "SkBlitRow_Color32_loop_8:                          \n"
                "vldmia     %[src]!, {d4, d5, d6, d7}           \n"     // load 8 src data
                "pld        [%[src], #128]                      \n"
                "sub        %[count], %[count], #8              \n"
                "cmp        %[count], #8                        \n"
                "vmull.u8   q8, d4, d0                          \n"     // multiple scale
                "vmull.u8   q9, d5, d0                          \n"
                "vmull.u8   q10, d6, d0                         \n"
                "vmull.u8   q11, d7, d0                         \n"
                "vaddhn.i16 d4, q8, q1                          \n"     // add src color
                "vaddhn.i16 d5, q9, q1                          \n"
                "vaddhn.i16 d6, q10, q1                         \n"
                "vaddhn.i16 d7, q11, q1                         \n"
                "vstmia     %[dst]!, {d4, d5, d6, d7}           \n"
                "bge        SkBlitRow_Color32_loop_8            \n"
                "cmp        %[count], #0                        \n"
                "beq        out                                 \n"

            "SkBlitRow_Color32_less_8:                          \n"
                "cmp        %[count], #4                        \n"
                "blt        SkBlitRow_Color32_less_4            \n"

                "vldmia     %[src]!, {d4, d5}                   \n"
                "subs       %[count], %[count], #4              \n"
                "vmull.u8   q8, d4, d0                          \n"     // multiple scale
                "vmull.u8   q9, d5, d0                          \n"
                "vaddhn.i16 d4, q8, q1                          \n"     // add src color
                "vaddhn.i16 d5, q9, q1                          \n"
                "vstmia     %[dst]!, {d4, d5}                   \n"
                "beq        out                                 \n"

            "SkBlitRow_Color32_less_4:                          \n"
                "cmp        %[count], #2                        \n"
                "blt        SkBlitRow_Color32_less_2            \n"
                "vldmia     %[src]!, {d4}                       \n"
                "vmull.u8   q8, d4, d0                          \n"     // multiple scale
                "subs       %[count], %[count], #2              \n"
                "vaddhn.i16 d4, q8, q1                          \n"     // add src color
                "vstmia     %[dst]!, {d4}                       \n"
                "beq        out                                 \n"

            "SkBlitRow_Color32_less_2:                          \n"
                "vld1.32    {d4[0]}, [%[src]]!                  \n"
                "vmull.u8   q8, d4, d0                          \n"     // multiple scale
                "vaddhn.i16 d4, q8, q1                          \n"     // add src color
                "vst1.32    {d4[0]}, [%[dst]]!                  \n"
            "out:                                               \n"
                :
                : [color] "r"(color), [dst] "r" (dst), [src] "r" (src), [scale] "r" (scale), [count] "r" (count)
                : "memory"
            );
        #else
            do {
                *dst = color + SkAlphaMulQ(*src, scale);
                src += 1;
                dst += 1;
            } while (--count);
        #endif
        }
    }
#endif
}

}  // SK_OPTS_NS

#endif//SkBlitRow_opts_DEFINED
