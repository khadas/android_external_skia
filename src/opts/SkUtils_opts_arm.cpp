/*
 * Copyright 2014 ARM Ltd.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#include "SkUtils.h"
#include "SkUtilsArm.h"

void sk_memset16_neon(uint16_t dst[], uint16_t value, int count);
void sk_memset32_neon(uint32_t dst[], uint32_t value, int count);
#if defined(SK_CPU_LENDIAN)
extern "C" void arm_memset16(uint16_t* dst, uint16_t value, int count);
extern "C" void arm_memset32(uint32_t* dst, uint32_t value, int count);
#endif

SkMemset16Proc SkMemset16GetPlatformProc() {
    // FIXME: memset.arm.S is using syntax incompatible with XCode
#if !defined(SK_CPU_LENDIAN) || defined(SK_BUILD_FOR_IOS)
    return NULL;
#else
    return arm_memset16;
#endif
}

SkMemset32Proc SkMemset32GetPlatformProc() {
    // FIXME: memset.arm.S is using syntax incompatible with XCode
#if !defined(SK_CPU_LENDIAN) || defined(SK_BUILD_FOR_IOS)
    return NULL;
#else
    return arm_memset32;
#endif
}

SkMemcpy32Proc SkMemcpy32GetPlatformProc() {
    return NULL;
}
