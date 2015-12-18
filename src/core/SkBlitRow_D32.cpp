/*
 * Copyright 2011 Google Inc.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#include "SkBlitRow.h"
#include "SkBlitMask.h"
#include "SkColorPriv.h"
#include "SkUtils.h"

#define UNROLL

static void S32_Opaque_BlitRow32(SkPMColor* SK_RESTRICT dst,
                                 const SkPMColor* SK_RESTRICT src,
                                 int count, U8CPU alpha) {
#if defined(__aarch64__)
    /*
     * TODO: optimize for AARCH64
     */
    SkASSERT(255 == alpha);
    sk_memcpy32(dst, src, count);
#else
    /*
     * tao.zeng@amlogic.com, src and dst are algined by 4
     */
    asm volatile (
        "cmp        %[count], #0                    \n"
        "it         eq                              \n"
        "bxeq       lr                              \n"                 // if count == 0, return
        "pld        [%[src], #32]                   \n"
        "tst        %[src], #0x04                   \n"                 // make aligne to 8 bytes
        "ittt       ne                              \n"
        "ldrne      r12, [%[src]], #4               \n"
        "strne      r12, [%[dst]], #4               \n"
        "subne      %[count], %[count], #1          \n"
        "cmp        %[count], #16                   \n"                 //
        "blt        S32_Opaque_BlitRow32_less16     \n"                 //
        "pld        [%[src], #64]                   \n"
        "sub        %[count], #16                   \n"
    "S32_Opaque_BlitRow32_loop16:                   \n"
        "vldmia     %[src]!, {q0, q1, q2, q3}       \n"
        "pld        [%[src], #64]                   \n"
        "pld        [%[src], #96]                   \n"
        "subs       %[count], %[count], #16         \n"
        "vstmia     %[dst]!, {q0, q1, q2, q3}       \n"
        "bge        S32_Opaque_BlitRow32_loop16     \n"
        "adds       %[count], %[count], #16         \n"
        "cmp        %[count], #0                    \n"
        "it         eq                              \n"
        "bxeq       lr                              \n"

    "S32_Opaque_BlitRow32_less16:                   \n"
        "cmp        %[count], #8                    \n"
        "blt        S32_Opaque_BlitRow32_less8      \n"
        "vldmia     %[src]!, {q0, q1}               \n"
        "subs       %[count], %[count], #8          \n"
        "vstmia     %[dst]!, {q0, q1}               \n"
        "it         eq                              \n"
        "bxeq       lr                              \n"
    "S32_Opaque_BlitRow32_less8:                    \n"
        "cmp        %[count], #4                    \n"
        "blt        S32_Opaque_BlitRow32_less4      \n"
        "vldmia     %[src]!, {d0, d1}               \n"
        "subs       %[count], %[count], #4          \n"
        "vstmia     %[dst]!, {d0, d1}               \n"
        "it         eq                              \n"
        "bxeq       lr                              \n"
    "S32_Opaque_BlitRow32_less4:                    \n"
        "cmp        %[count], #2                    \n"
        "itt        ge                              \n"
        "ldmge      %[src]!, {%[alpha], r12}        \n"
        "subges     %[count], #2                    \n"
        "it         ge                              \n"
        "stmge      %[dst]!, {%[alpha], r12}        \n"
        "it         eq                              \n"
        "bxeq       lr                              \n"
        "cmp        %[count], #0                    \n"
        "it         eq                              \n"
        "bxeq       lr                              \n"
        "ldr        r12, [%[src]], #4               \n"
        "str        r12, [%[dst]], #4               \n"
        "bx         lr                              \n"
        :
        :[src] "r" (src), [dst] "r" (dst), [count] "r" (count), [alpha] "r" (alpha)
        :"cc", "memory", "r12"
    );
#endif
}

static void S32_Blend_BlitRow32(SkPMColor* SK_RESTRICT dst,
                                const SkPMColor* SK_RESTRICT src,
                                int count, U8CPU alpha) {
    SkASSERT(alpha <= 255);
    if (count > 0) {
        unsigned src_scale = SkAlpha255To256(alpha);
        unsigned dst_scale = 256 - src_scale;

#ifdef UNROLL
        if (count & 1) {
            *dst = SkAlphaMulQ(*(src++), src_scale) + SkAlphaMulQ(*dst, dst_scale);
            dst += 1;
            count -= 1;
        }

        const SkPMColor* SK_RESTRICT srcEnd = src + count;
        while (src != srcEnd) {
            *dst = SkAlphaMulQ(*(src++), src_scale) + SkAlphaMulQ(*dst, dst_scale);
            dst += 1;
            *dst = SkAlphaMulQ(*(src++), src_scale) + SkAlphaMulQ(*dst, dst_scale);
            dst += 1;
        }
#else
        do {
            *dst = SkAlphaMulQ(*src, src_scale) + SkAlphaMulQ(*dst, dst_scale);
            src += 1;
            dst += 1;
        } while (--count > 0);
#endif
    }
}

static void S32A_Opaque_BlitRow32(SkPMColor* SK_RESTRICT dst,
                                  const SkPMColor* SK_RESTRICT src,
                                  int count, U8CPU alpha) {
    SkASSERT(255 == alpha);
    if (count > 0) {
#ifdef UNROLL
        if (count & 1) {
            *dst = SkPMSrcOver(*(src++), *dst);
            dst += 1;
            count -= 1;
        }

        const SkPMColor* SK_RESTRICT srcEnd = src + count;
        while (src != srcEnd) {
            *dst = SkPMSrcOver(*(src++), *dst);
            dst += 1;
            *dst = SkPMSrcOver(*(src++), *dst);
            dst += 1;
        }
#else
        do {
            *dst = SkPMSrcOver(*src, *dst);
            src += 1;
            dst += 1;
        } while (--count > 0);
#endif
    }
}

static void S32A_Blend_BlitRow32(SkPMColor* SK_RESTRICT dst,
                                 const SkPMColor* SK_RESTRICT src,
                                 int count, U8CPU alpha) {
    SkASSERT(alpha <= 255);
    if (count > 0) {
#ifdef UNROLL
        if (count & 1) {
            *dst = SkBlendARGB32(*(src++), *dst, alpha);
            dst += 1;
            count -= 1;
        }

        const SkPMColor* SK_RESTRICT srcEnd = src + count;
        while (src != srcEnd) {
            *dst = SkBlendARGB32(*(src++), *dst, alpha);
            dst += 1;
            *dst = SkBlendARGB32(*(src++), *dst, alpha);
            dst += 1;
        }
#else
        do {
            *dst = SkBlendARGB32(*src, *dst, alpha);
            src += 1;
            dst += 1;
        } while (--count > 0);
#endif
    }
}

///////////////////////////////////////////////////////////////////////////////

static const SkBlitRow::Proc32 gDefault_Procs32[] = {
    S32_Opaque_BlitRow32,
    S32_Blend_BlitRow32,
    S32A_Opaque_BlitRow32,
    S32A_Blend_BlitRow32
};

SkBlitRow::Proc32 SkBlitRow::Factory32(unsigned flags) {
    SkASSERT(flags < SK_ARRAY_COUNT(gDefault_Procs32));
    // just so we don't crash
    flags &= kFlags32_Mask;

    SkBlitRow::Proc32 proc = PlatformProcs32(flags);
    if (NULL == proc) {
        proc = gDefault_Procs32[flags];
    }
    SkASSERT(proc);
    return proc;
}

#include "Sk4px.h"

// Color32 uses the blend_256_round_alt algorithm from tests/BlendTest.cpp.
// It's not quite perfect, but it's never wrong in the interesting edge cases,
// and it's quite a bit faster than blend_perfect.
//
// blend_256_round_alt is our currently blessed algorithm.  Please use it or an analogous one.
void SkBlitRow::Color32(SkPMColor dst[], const SkPMColor src[], int count, SkPMColor color) {
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
        #if defined(__ARM_HAVE_NEON) && !defined(__aarch64__)
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
                "vadd.u16   q8, q8, q1                          \n"     // add src color
                "vadd.u16   q9, q9, q1                          \n"
                "vadd.u16   q10, q10, q1                        \n"
                "vadd.u16   q11, q11, q1                        \n"
                "vshrn.i16  d4, q8, #8                          \n"     // narrow result
                "vshrn.i16  d5, q9, #8                          \n"
                "vshrn.i16  d6, q10, #8                         \n"
                "vshrn.i16  d7, q11, #8                         \n"
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
                "vadd.u16   q8, q8, q1                          \n"     // add src color
                "vadd.u16   q9, q9, q1                          \n"
                "vshrn.i16  d4, q8, #8                          \n"     // narrow result
                "vshrn.i16  d5, q9, #8                          \n"
                "vstmia     %[dst]!, {d4, d5}                   \n"
                "beq        out                                 \n"

            "SkBlitRow_Color32_less_4:                          \n"
                "cmp        %[count], #2                        \n"
                "blt        SkBlitRow_Color32_less_2            \n"
                "vldmia     %[src]!, {d4}                       \n"
                "vmull.u8   q8, d4, d0                          \n"     // multiple scale
                "subs       %[count], %[count], #2              \n"
                "vadd.u16   q8, q8, q1                          \n"     // add src color
                "vshrn.i16  d4, q8, #8                          \n"     // narrow result
                "vstmia     %[dst]!, {d4}                       \n"
                "beq        out                                 \n"

            "SkBlitRow_Color32_less_2:                          \n"
                "vld1.32    {d4[0]}, [%[src]]!                  \n"
                "vmull.u8   q8, d4, d0                          \n"     // multiple scale
                "vadd.u16   q8, q8, q1                          \n"     // add src color
                "vshrn.i16  d4, q8, #8                          \n"     // narrow result
                "vst1.32    {d4[0]}, [%[dst]]!                  \n"

            "out:                                               \n"
                :
                : [color] "r"(color), [dst] "r" (dst), [src] "r" (src), [scale] "r" (scale), [count] "r" (count)
                : "memory"
            );
        #else
            /*
             * TODO: optimize for AARCH64
             */
            do {
                *dst = color + SkAlphaMulQ(*src, scale);
                src += 1;
                dst += 1;
            } while (--count);
        #endif
        }
    }
}

