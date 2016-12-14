/*
 * Copyright 2015 Google Inc.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#ifndef SkBlitMask_opts_DEFINED
#define SkBlitMask_opts_DEFINED

#include "Sk4px.h"

namespace SK_OPTS_NS {

#if defined(SK_ARM_HAS_NEON)
    // The Sk4px versions below will work fine with NEON, but we have had many indications
    // that it doesn't perform as well as this NEON-specific code.  TODO(mtklein): why?
    #include "SkColor_opts_neon.h"

    template <bool isColor>
    static void D32_A8_Opaque_Color_neon(void* SK_RESTRICT dst, size_t dstRB,
                                         const void* SK_RESTRICT maskPtr, size_t maskRB,
                                         SkColor color, int width, int height) {
        SkPMColor pmc = SkPreMultiplyColor(color);
        SkPMColor* SK_RESTRICT device = (SkPMColor*)dst;
        const uint8_t* SK_RESTRICT mask = (const uint8_t*)maskPtr;
        uint8x8x4_t vpmc;

        maskRB -= width;
        dstRB -= (width << 2);

        if (width >= 8) {
            vpmc.val[NEON_A] = vdup_n_u8(SkGetPackedA32(pmc));
            vpmc.val[NEON_R] = vdup_n_u8(SkGetPackedR32(pmc));
            vpmc.val[NEON_G] = vdup_n_u8(SkGetPackedG32(pmc));
            vpmc.val[NEON_B] = vdup_n_u8(SkGetPackedB32(pmc));
        }
        do {
            int w = width;
            while (w >= 8) {
                uint8x8_t vmask = vld1_u8(mask);
                uint16x8_t vscale, vmask256 = SkAlpha255To256_neon8(vmask);
                if (isColor) {
                    vscale = vsubw_u8(vdupq_n_u16(256),
                            SkAlphaMul_neon8(vpmc.val[NEON_A], vmask256));
                } else {
                    vscale = vsubw_u8(vdupq_n_u16(256), vmask);
                }
                uint8x8x4_t vdev = vld4_u8((uint8_t*)device);

                vdev.val[NEON_A] =   SkAlphaMul_neon8(vpmc.val[NEON_A], vmask256)
                    + SkAlphaMul_neon8(vdev.val[NEON_A], vscale);
                vdev.val[NEON_R] =   SkAlphaMul_neon8(vpmc.val[NEON_R], vmask256)
                    + SkAlphaMul_neon8(vdev.val[NEON_R], vscale);
                vdev.val[NEON_G] =   SkAlphaMul_neon8(vpmc.val[NEON_G], vmask256)
                    + SkAlphaMul_neon8(vdev.val[NEON_G], vscale);
                vdev.val[NEON_B] =   SkAlphaMul_neon8(vpmc.val[NEON_B], vmask256)
                    + SkAlphaMul_neon8(vdev.val[NEON_B], vscale);

                vst4_u8((uint8_t*)device, vdev);

                mask += 8;
                device += 8;
                w -= 8;
            }

            while (w--) {
                unsigned aa = *mask++;
                if (isColor) {
                    *device = SkBlendARGB32(pmc, *device, aa);
                } else {
                    *device = SkAlphaMulQ(pmc, SkAlpha255To256(aa))
                        + SkAlphaMulQ(*device, SkAlpha255To256(255 - aa));
                }
                device += 1;
            };

            device = (uint32_t*)((char*)device + dstRB);
            mask += maskRB;

        } while (--height != 0);
    }

    static void blit_mask_d32_a8_general(SkPMColor* dst, size_t dstRB,
                                         const SkAlpha* mask, size_t maskRB,
                                         SkColor color, int w, int h) {
        D32_A8_Opaque_Color_neon<true>(dst, dstRB, mask, maskRB, color, w, h);
    }

    // As above, but made slightly simpler by requiring that color is opaque.
    static void blit_mask_d32_a8_opaque(SkPMColor* dst, size_t dstRB,
                                        const SkAlpha* maskPtr, size_t maskRB,
                                        SkColor color, int width, int height) {
    #if 0
        D32_A8_Opaque_Color_neon<false>(dst, dstRB, maskPtr, maskRB, color, width, height);
    #else
        /* optimization for aarch32, text drawing used most */
    #if defined(__arm__)
        SkPMColor pmc = SkPreMultiplyColor(color);
        uint32_t* device = (uint32_t*)dst;

        maskRB -= width;
        dstRB -= (width << 2);
        asm volatile (
            "vmov       s0,  %[pmc]                     \n"     // set ARGB of pmc
            "vdup.32    d31, %[pmc]                     \n"     // d31 = pmc.32
            "vmov.i16   q13, #0x100                     \n"
            "vmovl.u8   q0,  d0                         \n"
            "vmovl.u8   q15, d31                        \n"
            "vmov.i8    d1,  #0x01                      \n"
            :
            : [pmc] "r" (pmc)
            : "cc", "memory"
        );
        do {
        #if 0
            int w = width;
            do {
                unsigned aa = *maskPtr++;
                *device = SkAlphaMulQ(pmc, SkAlpha255To256(aa)) + SkAlphaMulQ(*device, SkAlpha255To256(255 - aa));
                device += 1;
            } while (--w != 0);
        #else
            asm volatile (
                "mov        r12, %[width]                           \n"
                "cmp        r12, #8                                 \n"     // count > 8?
                "blt        D32_Mask_Opaque_less_8                  \n"
            "D32_Mask_Opaque_loop8:                                 \n"
                "vld1.8     {d7}, [%[maskPtr]]!                     \n"     // load 8 mask
                "vld4.8     {d2, d3, d4, d5}, [%[device]]           \n"     // load 8 dst data
                "sub        r12, r12, #8                            \n"
                "vaddl.u8   q14, d7,  d1                            \n"     // 255to256(aa)
                "vsubw.u8   q8,  q13, d7                            \n"     // 255to256(255 - aa)
                "vmovl.u8   q9,  d2                                 \n"     // expend R to 16 bits
                "vmovl.u8   q10, d3                                 \n"     // expend G to 16 bits
                "vmovl.u8   q11, d4                                 \n"     // expend B to 16 bits
                "vmovl.u8   q12, d5                                 \n"     // expend A to 16 bits

                "vmul.i16   q9,  q9,  q8                            \n"     // device.R * 255to256(255 -aa)
                "vmul.i16   q10, q10, q8                            \n"     // device.G * 255to256(255 -aa)
                "vmul.i16   q11, q11, q8                            \n"     // device.B * 255to256(255 -aa)
                "vmul.i16   q12, q12, q8                            \n"     // device.A * 255to256(255 -aa)
                "vmul.i16   q1,  q14, d0[0]                         \n"     // pmc.R * 255to256(aa)
                "vmul.i16   q2,  q14, d0[1]                         \n"     // pmc.G * 255to256(aa)
                "vmul.i16   q8,  q14, d0[2]                         \n"     // pmc.B * 255to256(aa)
                "vmul.i16   q14, q14, d0[3]                         \n"     // pmc.A * 255to256(aa)
                "vshrn.i16  d18, q9,  #8                            \n"     //
                "vshrn.i16  d19, q10, #8                            \n"
                "vshrn.i16  d20, q11, #8                            \n"
                "vshrn.i16  d21, q12, #8                            \n"
                "vshrn.i16  d2,  q1,  #8                            \n"
                "vshrn.i16  d3,  q2,  #8                            \n"
                "vshrn.i16  d4,  q8,  #8                            \n"
                "vshrn.i16  d5,  q14, #8                            \n"
                "vadd.i8    q1,  q1,  q9                            \n"
                "vadd.i8    q2,  q2,  q10                           \n"
                "cmp        r12, #8                                 \n"
                "vst4.8     {d2, d3, d4, d5}, [%[device]]!          \n"
                "bge        D32_Mask_Opaque_loop8                   \n"
                "cmp        r12, #0                                 \n"
                "beq        D32_Mask_Opaque_out                     \n"

            "D32_Mask_Opaque_less_8:                                \n"
                "cmp        r12, #4                                 \n"
                "mov        r10, #0x0                               \n"
                "ldr        r11, =0x01010101                        \n"
                "vmov       d28, r10, r11                           \n"
                "ldr        r10, =0x02020202                        \n"
                "ldr        r11, =0x03030303                        \n"
                "vmov       d29, r10, r11                           \n"
                "blt        D32_Mask_Opaque_less_4                  \n"
                "vld1.32    {d7[]}, [%[maskPtr]]!                   \n"     // d7  = [aa 0..3 0..3]
                "vldmia     %[device], {d2, d3}                     \n"
                "subs       r12, r12, #4                            \n"
                "vtbl.8     d4,  {d7}, d28                          \n"     // d4  = [1 1 1 1 0 0 0 0] of aa
                "vtbl.8     d5,  {d7}, d29                          \n"     // d5  = [3 3 3 3 2 2 2 2] of aa
                "vmovl.u8   q9,  d2                                 \n"     // exprand to 16 bits
                "vmovl.u8   q10, d3                                 \n"
                "vaddl.u8   q11, d4,  d1                            \n"     // 255to256(aa)
                "vaddl.u8   q12, d5,  d1                            \n"     // 255to256(aa)
                "vsubw.u8   q3,  q13, d4                            \n"     // 255to256(255 - aa)
                "vsubw.u8   q8,  q13, d5                            \n"     // 255to256(255 - aa)
                "vmul.i16   q9,  q9,  q3                            \n"     // device * 255to256(255 -aa)
                "vmul.i16   q10, q10, q8                            \n"     // device * 255to256(255 -aa)
                "vmul.i16   q11, q15, q11                           \n"     // pmc * 255to256(aa)
                "vmul.i16   q12, q15, q12                           \n"     // pmc * 255to256(aa)
                "vshrn.i16  d2,  q9,  #8                            \n"
                "vshrn.i16  d3,  q10, #8                            \n"
                "vshrn.i16  d4,  q11, #8                            \n"
                "vshrn.i16  d5,  q12, #8                            \n"
                "vadd.i8    q1,  q1,  q2                            \n"
                "vstmia     %[device]!, {d2, d3}                    \n"
                "beq        D32_Mask_Opaque_out                     \n"

            "D32_Mask_Opaque_less_4:                                \n"
                "cmp        r12, #2                                 \n"
                "blt        D32_Mask_Opaque_less_2                  \n"
                "vld1.16    {d7[]}, [%[maskPtr]]!                   \n"
                "vldmia     %[device], {d2}                         \n"
                "subs       r12, r12, #2                            \n"
                "vtbl.8     d4,  {d7}, d28                          \n"     // d4  = [1 1 1 1 0 0 0 0] of aa
                "vmovl.u8   q9,  d2                                 \n"     // exprand to 16 bits
                "vaddl.u8   q11, d4,  d1                            \n"     // 255to256(aa)
                "vsubw.u8   q3,  q13, d4                            \n"     // 255to256(255 - aa)
                "vmul.i16   q9,  q9,  q3                            \n"     // device * 255to256(255 -aa)
                "vmul.i16   q10, q15, q11                           \n"     // pmc * 255to256(aa)
                "vshrn.i16  d2,  q9,  #8                            \n"
                "vshrn.i16  d3,  q10, #8                            \n"
                "vadd.i8    d2,  d3,  d2                            \n"
                "vstmia     %[device]!, {d2}                        \n"
                "beq        D32_Mask_Opaque_out                     \n"

            "D32_Mask_Opaque_less_2:                                \n"
                "vld1.8     {d7[]}, [%[maskPtr]]!                   \n"
                "vld1.32    {d2[0]}, [%[device]]                    \n"
                "vtbl.8     d4,  {d7}, d28                          \n"     // d4  = [1 1 1 1 0 0 0 0] of aa
                "vaddl.u8   q11, d4,  d1                            \n"     // 255to256(aa)
                "vsubw.u8   q3,  q13, d4                            \n"     // 255to256(255 - aa)
                "vmovl.u8   q9,  d2                                 \n"     // exprand to 16 bits
                "vmul.i16   q9,  q9,  q3                            \n"     // device * 255to256(255 -aa)
                "vmul.i16   q10, q15, q11                           \n"     // pmc * 255to256(aa)
                "vshrn.i16  d2,  q9,  #8                            \n"
                "vshrn.i16  d3,  q10, #8                            \n"
                "vadd.i8    d2,  d3,  d2                            \n"
                "vst1.32    {d2[0]}, [%[device]]!                   \n"

            "D32_Mask_Opaque_out:                                   \n"
                :
                : [device] "r" (device) , [maskPtr] "r" (maskPtr), [width] "r" (width)
                : "cc", "memory", "r12", "r11", "r10"
            );
        #endif
            device = (uint32_t*)((char*)device + dstRB);
            maskPtr += maskRB;
        } while (--height != 0);

      //asm volatile (
      //"D32_Mask_Opaque_table:                         \n"
      //    ".word      0x00000000                      \n"
      //    ".word      0x01010101                      \n"
      //    ".word      0x02020202                      \n"
      //    ".word      0x03030303                      \n"
      //);
    #else
        SkPMColor pmc = SkPreMultiplyColor(color);
        SkPMColor* SK_RESTRICT device = (SkPMColor*)dst;
        const uint8_t* SK_RESTRICT mask = (const uint8_t*)maskPtr;
        uint8x8x4_t vpmc;

        maskRB -= width;
        dstRB -= (width << 2);

        if (width >= 8) {
            vpmc.val[NEON_A] = vdup_n_u8(SkGetPackedA32(pmc));
            vpmc.val[NEON_R] = vdup_n_u8(SkGetPackedR32(pmc));
            vpmc.val[NEON_G] = vdup_n_u8(SkGetPackedG32(pmc));
            vpmc.val[NEON_B] = vdup_n_u8(SkGetPackedB32(pmc));
        }
        do {
            int w = width;
            while (w >= 8) {
                uint8x8_t vmask = vld1_u8(mask);
                uint16x8_t vscale, vmask256 = SkAlpha255To256_neon8(vmask);
                vscale = vsubw_u8(vdupq_n_u16(256), vmask);
                uint8x8x4_t vdev = vld4_u8((uint8_t*)device);

                vdev.val[NEON_A] =   SkAlphaMul_neon8(vpmc.val[NEON_A], vmask256)
                    + SkAlphaMul_neon8(vdev.val[NEON_A], vscale);
                vdev.val[NEON_R] =   SkAlphaMul_neon8(vpmc.val[NEON_R], vmask256)
                    + SkAlphaMul_neon8(vdev.val[NEON_R], vscale);
                vdev.val[NEON_G] =   SkAlphaMul_neon8(vpmc.val[NEON_G], vmask256)
                    + SkAlphaMul_neon8(vdev.val[NEON_G], vscale);
                vdev.val[NEON_B] =   SkAlphaMul_neon8(vpmc.val[NEON_B], vmask256)
                    + SkAlphaMul_neon8(vdev.val[NEON_B], vscale);

                vst4_u8((uint8_t*)device, vdev);

                mask += 8;
                device += 8;
                w -= 8;
            }

            while (w--) {
                unsigned aa = *mask++;
                *device = SkAlphaMulQ(pmc, SkAlpha255To256(aa))
                        + SkAlphaMulQ(*device, SkAlpha255To256(255 - aa));
                device += 1;
            };

            device = (uint32_t*)((char*)device + dstRB);
            mask += maskRB;

        } while (--height != 0);
    #endif
    #endif
    }

    // Same as _opaque, but assumes color == SK_ColorBLACK, a very common and even simpler case.
    static void blit_mask_d32_a8_black(SkPMColor* dst, size_t dstRB,
                                       const SkAlpha* maskPtr, size_t maskRB,
                                       int width, int height) {
        SkPMColor* SK_RESTRICT device = (SkPMColor*)dst;
        const uint8_t* SK_RESTRICT mask = (const uint8_t*)maskPtr;

        maskRB -= width;
        dstRB -= (width << 2);
        do {
            int w = width;
            while (w >= 8) {
                uint8x8_t vmask = vld1_u8(mask);
                uint16x8_t vscale = vsubw_u8(vdupq_n_u16(256), vmask);
                uint8x8x4_t vdevice = vld4_u8((uint8_t*)device);

                vdevice = SkAlphaMulQ_neon8(vdevice, vscale);
                vdevice.val[NEON_A] += vmask;

                vst4_u8((uint8_t*)device, vdevice);

                mask += 8;
                device += 8;
                w -= 8;
            }
            while (w-- > 0) {
                unsigned aa = *mask++;
                *device = (aa << SK_A32_SHIFT)
                            + SkAlphaMulQ(*device, SkAlpha255To256(255 - aa));
                device += 1;
            };
            device = (uint32_t*)((char*)device + dstRB);
            mask += maskRB;
        } while (--height != 0);
    }

#else
    static void blit_mask_d32_a8_general(SkPMColor* dst, size_t dstRB,
                                         const SkAlpha* mask, size_t maskRB,
                                         SkColor color, int w, int h) {
        auto s = Sk4px::DupPMColor(SkPreMultiplyColor(color));
        auto fn = [&](const Sk4px& d, const Sk4px& aa) {
            //  = (s + d(1-sa))aa + d(1-aa)
            //  = s*aa + d(1-sa*aa)
            auto left  = s.approxMulDiv255(aa),
                 right = d.approxMulDiv255(left.alphas().inv());
            return left + right;  // This does not overflow (exhaustively checked).
        };
        while (h --> 0) {
            Sk4px::MapDstAlpha(w, dst, mask, fn);
            dst  +=  dstRB / sizeof(*dst);
            mask += maskRB / sizeof(*mask);
        }
    }

    // As above, but made slightly simpler by requiring that color is opaque.
    static void blit_mask_d32_a8_opaque(SkPMColor* dst, size_t dstRB,
                                        const SkAlpha* mask, size_t maskRB,
                                        SkColor color, int w, int h) {
        SkASSERT(SkColorGetA(color) == 0xFF);
        auto s = Sk4px::DupPMColor(SkPreMultiplyColor(color));
        auto fn = [&](const Sk4px& d, const Sk4px& aa) {
            //  = (s + d(1-sa))aa + d(1-aa)
            //  = s*aa + d(1-sa*aa)
            //   ~~~>
            //  = s*aa + d(1-aa)
            return s.approxMulDiv255(aa) + d.approxMulDiv255(aa.inv());
        };
        while (h --> 0) {
            Sk4px::MapDstAlpha(w, dst, mask, fn);
            dst  +=  dstRB / sizeof(*dst);
            mask += maskRB / sizeof(*mask);
        }
    }

    // Same as _opaque, but assumes color == SK_ColorBLACK, a very common and even simpler case.
    static void blit_mask_d32_a8_black(SkPMColor* dst, size_t dstRB,
                                       const SkAlpha* mask, size_t maskRB,
                                       int w, int h) {
        auto fn = [](const Sk4px& d, const Sk4px& aa) {
            //   = (s + d(1-sa))aa + d(1-aa)
            //   = s*aa + d(1-sa*aa)
            //   ~~~>
            // a = 1*aa + d(1-1*aa) = aa + d(1-aa)
            // c = 0*aa + d(1-1*aa) =      d(1-aa)
            return aa.zeroColors() + d.approxMulDiv255(aa.inv());
        };
        while (h --> 0) {
            Sk4px::MapDstAlpha(w, dst, mask, fn);
            dst  +=  dstRB / sizeof(*dst);
            mask += maskRB / sizeof(*mask);
        }
    }
#endif

static void blit_mask_d32_a8(SkPMColor* dst, size_t dstRB,
                             const SkAlpha* mask, size_t maskRB,
                             SkColor color, int w, int h) {
    if (color == SK_ColorBLACK) {
        blit_mask_d32_a8_black(dst, dstRB, mask, maskRB, w, h);
    } else if (SkColorGetA(color) == 0xFF) {
        blit_mask_d32_a8_opaque(dst, dstRB, mask, maskRB, color, w, h);
    } else {
        blit_mask_d32_a8_general(dst, dstRB, mask, maskRB, color, w, h);
    }
}

}  // SK_OPTS_NS

#endif//SkBlitMask_opts_DEFINED
