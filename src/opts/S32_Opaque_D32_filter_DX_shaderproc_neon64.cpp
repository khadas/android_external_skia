
/*
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// S32_Opaque_D32_filter_DX_shaderproc_neon(row0, row1, fx, maxX, subY, colors, dx, count);

#include "SkFixed.h"

void S32_Opaque_D32_filter_DX_shaderproc_neon(SkFixed fx,
                                              unsigned int subY,
                                              SkFixed dx,
                                              int count,
                                              const unsigned int* image0,
                                              const unsigned int* image1,
                                              unsigned int *SK_RESTRICT colors)
{
    /*
     *
     */
    asm volatile (
        "dup            v31.8b, %w[subY]                        \n"     // v31 = [y]
        "movi           v16.8b, #16                             \n"     // v16 = [16].8
        "sub            v17.8b, v16.8b, v31.8b                  \n"     // v17 = [16 - y]
        "movi           v30.8h, #16                             \n"     // v30 = [16].16
        "cmp            %w[count], #4                            \n"
        "ldr            x14, =0x3fffc                           \n"     // x14 as mask
        "blt            S32_Opaque_D32_filter_DX_less4          \n"
        "sub            %w[count], %w[count], #4                  \n"

        "ubfx           x12, %[fx],  #12, #4                    \n"     // x12 = Xa
        "and            x7,  x14, %[fx], lsr #14                \n"
        "add            x9,  x7, %[image0]                      \n"
        "add            x7,  x7, %[image1]                      \n"
        "add            %w[fx],  %w[fx], %w[dx]                    \n"     // fx += dx
        "dup            v29.4h, w12                             \n"

    "S32_Opaque_D32_filter_DX_loop4:                            \n"
        // pixel 1 ~ 2
        "ldr            d0,  [x9]                               \n"
        "ldr            d1,  [x7]                               \n"
        "ubfx           x12, %[fx], #12, #4                     \n"     // x12 = Xb
        "umull          v4.8h, v0.8b, v17.8b                    \n"     // v4  = [a01, a00] * (16-y)
        "dup            v28.4h, w12                             \n"
        "umull          v5.8h, v1.8b, v31.8b                    \n"     // v5  = [a11, a10] * y
        "ins            v29.d[1], v28.d[0]                      \n"     // v29 = [xb, xa].16

        "and            x7,  x14, %[fx], lsr #14                \n"
        "add            x9,  x7,  %[image0]                     \n"
        "add            x7,  x7,  %[image1]                     \n"
        "ldr            d2,  [x9]                               \n"     // d2  = [b01, b00]
        "ldr            d3,  [x7]                               \n"     // d3  = [b11, b10]
        "add            %w[fx],  %w[fx],  %w[dx]                   \n"     // fx += dx
        "sub            v27.8h, v30.8h, v29.8h                  \n"     // v27 = [16-Xb, 16-Xa].16
        "umull          v6.8h, v2.8b, v17.8b                    \n"     // v6  = [b01, b00] * (16 - y)
        "umull          v7.8h, v3.8b, v31.8b                    \n"     // v7  = [b11, b00] *  y

        // pixel 3 ~ 4
        "ubfx           x12, %[fx],  #12, #4                    \n"     // x12 = Xc
        "and            x7,  x14, %[fx], lsr #14                \n"
        "dup            v26.4h, w12                             \n"     // v26 = [Xc].16
        "add            x9,  x7, %[image0]                      \n"
        "add            x7,  x7, %[image1]                      \n"
        "add            %w[fx],  %w[fx], %w[dx]                    \n"     // fx += dx
        "ins            v18.d[0], v4.d[1]                       \n"     // vswap v4.d[1], v6.d[0]
        "ins            v4.d[1],  v6.d[0]                       \n"
        "ins            v6.d[0],  v18.d[0]                      \n"     // v4  = [b00, a00].16, v5 = [b01, a01]*(16-y)
        "ins            v18.d[0], v5.d[1]                       \n"     // vswap v5.d[1], v7.d[0]
        "ins            v5.d[1],  v7.d[0]                       \n"
        "ins            v7.d[0],  v18.d[0]                      \n"     // v5  = [b10, a10].16, v7 = [b11, a11].16*y
        "ldr            d0,  [x9]                               \n"     // d0  = [c01, c00]
        "ldr            d1,  [x7]                               \n"     // d1  = [c11, c10]
        "ubfx           x12, %[fx], #12, #4                     \n"     // x12 = Xd
        "umull          v19.8h, v0.8b, v17.8b                   \n"     // v19 = [c01, c00] * (16-y)
        "dup            v25.4h, w12                             \n"     // v25 = [Xd]
        "ins            v26.d[1], v25.d[0]                      \n"     // v26 = [Xd, Xc]
        "umull          v20.8h, v1.8b, v31.8b                   \n"     // v20 = [c11, c10] * y

        "and            x7,  x14, %[fx], lsr #14                \n"
        "add            x9,  x7,  %[image0]                     \n"
        "add            x7,  x7,  %[image1]                     \n"
        "ldr            d2,  [x9]                               \n"     // d2  = [d01, d00]
        "ldr            d3,  [x7]                               \n"     // d3  = [d11, d10]
        "add            %w[fx],  %w[fx],  %w[dx]                \n"     // fx += dx
        "sub            v24.8h,  v30.8h, v26.8h                 \n"     // v24 = [16-Xd, 16-Xc].16
        "umull          v21.8h,  v2.8b, v17.8b                  \n"     // v21 = [d01, d00] * (16 - y)
        "umull          v22.8h,  v3.8b, v31.8b                  \n"     // v22 = [d11, d00] *  y

        "subs           %w[count],  %w[count], #4               \n"
        "ins            v18.d[0], v19.d[1]                      \n"     // vswap v19.d[1], v21.d[0]
        "ins            v19.d[1], v21.d[0]                      \n"
        "ins            v21.d[0], v18.d[0]                      \n"     // v19 = [d00, c00].16, v21= [d01, c01]*(16-y)
        "ins            v18.d[0], v20.d[1]                      \n"     // vswap v20.d[1], v22.d[0]
        "ins            v20.d[1], v22.d[0]                      \n"
        "ins            v22.d[0], v18.d[0]                      \n"     // v20 = [d10, c10].16, v22= [d11, c11].16*y

        "ubfx           x12, %[fx],  #12, #4                    \n"     // x12 = Xa

        "mul            v4.8h,  v4.8h,  v27.8h                  \n"     // v4  = [b00, a00]*(16-y)*[16-Xb, 16-Xa]
        "mul            v19.8h,  v19.8h,  v24.8h                \n"     // v19 = [d00, c00]*(16-y)*[16-Xd, 16-Xc]
        "and            x7,  x14, %[fx], lsr #14                \n"
        "mla            v4.8h,  v6.8h,  v29.8h                  \n"     // v4 += [b01, a01]*(16-y)*[Xb, Xa]
        "mla            v19.8h,  v21.8h,  v26.8h                \n"     // v19+= [d01, c01]*(16-y)*[Xd, Xc]
        "add            x9,  x7, %[image0]                      \n"
        "mla            v4.8h,  v5.8h,  v27.8h                  \n"     // v4 += [b10, a10]*y*[16-Xb, 16-Xa]
        "mla            v19.8h,  v20.8h,  v24.8h                \n"     // v19+= [d10, c10]*y*[16-Xd, 16-Xc]
        "add            x7,  x7, %[image1]                      \n"
        "mla            v4.8h,  v7.8h,  v29.8h                  \n"     // v4 += [b11, a11]*y*[Xb, Xa]
        "mla            v19.8h,  v22.8h, v26.8h                 \n"     // v19+= [d11, c11]*y*[Xd, Xc]
        "add            %[fx], %[fx], %[dx]                     \n"     // fx += dx
        "shrn           v23.8b,  v4.8h, #8                      \n"     // result
        "shrn2          v23.16b, v19.8h, #8                     \n"     // result
        "dup            v29.4h, w12                             \n"     // v29 = [Xa].16
        "str            q23, [%[colors]], #16                   \n"     // store data
        "bge            S32_Opaque_D32_filter_DX_loop4          \n"
        "adds           %w[count],  %w[count], #4               \n"
        "beq            out                                     \n"
        "sub            %w[fx],  %w[fx], %w[dx]                    \n"     // because fx is added once more

    "S32_Opaque_D32_filter_DX_less4:                            \n"
        "cmp            %w[count], #2                           \n"
        "blt            S32_Opaque_D32_filter_DX_less2          \n"
        // pixel 1 ~ 2
        "ubfx           x12, %[fx],  #12, #4                    \n"     // x12 = Xa
        "and            x7,  x14, %[fx], lsr #14                \n"
        "dup            v29.4h, w12                             \n"     // v29 = [Xa].16
        "add            x9,  x7, %[image0]                      \n"
        "add            %w[fx],  %w[fx], %w[dx]                    \n"     // fx += dx
        "ldr            d0,  [x9]                               \n"     // d0  = [a01, a00]
        "add            x9,  x7, %[image1]                      \n"
        "ldr            d1,  [x9]                               \n"     // d1  = [a11, a10]
        "ubfx           x12, %[fx], #12, #4                     \n"     // x12 = Xb
        "umull          v4.8h, v0.8b, v17.8b                    \n"     // v4  = [a01, a00] * (16-y)
        "dup            v28.4h, w12                             \n"
        "umull          v5.8h, v1.8b, v31.8b                    \n"     // v5  = [a11, a10] * y
        "ins            v29.d[1], v28.d[0]                      \n"     // v29 = [xb, xa].16

        "and            x7,  x14, %[fx], lsr #14                \n"
        "add            x9,  x7,  %[image0]                     \n"
        "ldr            d2,  [x9]                               \n"     // d2  = [b01, b00]
        "add            x9,  x7,  %[image1]                     \n"
        "ldr            d3,  [x9]                               \n"     // d3  = [b11, b10]
        "add            %w[fx],  %w[fx],  %w[dx]                   \n"     // fx += dx
        "sub            v27.8h, v30.8h, v29.8h                  \n"     // v27 = [16-Xb, 16-Xa].16
        "umull          v6.8h, v2.8b, v17.8b                    \n"     // v6  = [b01, b00] * (16 - y)
        "umull          v7.8h, v3.8b, v31.8b                    \n"     // v7  = [b11, b00] *  y
        "subs           %w[count], %w[count],  #2               \n"
        "ins            v18.d[0], v4.d[1]                       \n"     // vswap v4.d[1], v6.d[0]
        "ins            v4.d[1],  v6.d[0]                       \n"
        "ins            v6.d[0],  v18.d[0]                      \n"     // v4  = [b00, a00].16, v5 = [b01, a01]*(16-y)
        "ins            v18.d[0], v5.d[1]                       \n"     // vswap v5.d[1], v7.d[0]
        "ins            v5.d[1],  v7.d[0]                       \n"
        "ins            v7.d[0],  v18.d[0]                      \n"     // v5  = [b10, a10].16, v7 = [b11, a11].16*y

        "mul            v4.8h,  v4.8h,  v27.8h                  \n"     // v4  = [b00, a00]*(16-y)*[16-Xb, 16-Xa]
        "mla            v4.8h,  v6.8h,  v29.8h                  \n"     // v4 += [b01, a01]*(16-y)*[Xb, Xa]
        "mla            v4.8h,  v5.8h,  v27.8h                  \n"     // v4 += [b10, a10]*y*[16-Xb, 16-Xa]
        "mla            v4.8h,  v7.8h,  v29.8h                  \n"     // v4 += [b11, a11]*y*[Xb, Xa]
        "shrn           v3.8b,  v4.8h, #8                       \n"     // result
        "str            d3, [%[colors]], #8                     \n"     // store data
        "beq            out                                     \n"

    "S32_Opaque_D32_filter_DX_less2:                            \n"
        "ubfx           x12, %[fx],  #12, #4                    \n"     // x12 = Xa
        "and            x7,  x14, %[fx], lsr #14                \n"
        "dup            v29.4h, w12                             \n"     // v29 = [Xa].16
        "add            x9,  x7, %[image0]                      \n"
        "ldr            d0,  [x9]                               \n"     // d0  = [a01, a00]
        "add            x9,  x7, %[image1]                      \n"
        "ldr            d1,  [x9]                               \n"     // d1  = [a11, a10]
        "umull          v4.8h, v0.8b, v17.8b                    \n"     // v4  = [a01, a00] * (16-y)
        "umull          v5.8h, v1.8b, v31.8b                    \n"     // v5  = [a11, a10] * y
        "sub            v27.4h, v30.4h, v29.4h                  \n"     // v27 = [16-Xa].16
        "ins            v26.d[0], v4.d[1]                       \n"     // v26 = a01 * (16-y)
        "ins            v25.d[0], v5.d[1]                       \n"     // v25 = a11 * y

        "mul            v3.4h,  v26.4h,  v29.4h                 \n"     // v3  = a01*(16-y)*x
        "mla            v3.4h,  v25.4h,  v29.4h                 \n"     // v3 += a11*y*x
        "mla            v3.4h,  v4.4h,  v27.4h                  \n"     // v3 += a00*(16-y)*(16-x)
        "mla            v3.4h,  v5.4h,  v27.4h                  \n"     // d4 += a10*y*(16-x)
        "shrn           v3.8b,  v3.8h,  #8                      \n"
        "str            s3, [%[colors]], #4                     \n"     // store data

    "out:                                                       \n"
        :
        : [colors] "r" (colors), [image0] "r" (image0), [image1] "r" (image1), [fx] "r" (fx),
          [dx] "r" (dx), [subY] "r" (subY), [count] "r" (count)
        : "cc", "memory", "x7", "x9", "x12", "x14"
    );
}

