
#include "SkBitmapProcState_filter.h"
#include <cutils/log.h>
#include "SkBitmapProcState.h"
#include <SkUtils.h>
#include <SkBitmapProcState_utils.h>

/*
 * For clamp
 */
#define TILEX_PROCF(fx, max)    SkClampMax((fx) >> 16, max)
#define TILEY_PROCF(fy, max)    SkClampMax((fy) >> 16, max)
#define TILEX_LOW_BITS(fx, max) (((fx) >> 12) & 0xF)
#define TILEY_LOW_BITS(fy, max) (((fy) >> 12) & 0xF)
#define CHECK_FOR_DECAL


//////////////////////////////////////////////////////////////////////////////
#define DEBUG_MERGE                             1

#define CLAMPX_NOFILTER_TRANS                   0
#define REPEATX_NOFILTER_TRANS                  1
#define MIRRORX_NOFILTER_TRANS                  2
#define CLAMPX_CLAMPY_NOFILTER_SCALE_NEON       3
#define CLAMPX_CLAMPY_FILTER_SCALE_NEON         4
#define CLAMPX_CLAMPY_NOFILTER_AFFINE_NEON      5
#define CLAMPX_CLAMPY_FILTER_AFFINE_NEON        6
#define CLAMPX_CLAMPY_NOFILTER_PERSP_NEON       7
#define CLAMPX_CLAMPY_FILTER_PERSP_NEON         8
#define REPEATX_REPEATY_NOFILTER_SCALE_NEON     9
#define REPEATX_REPEATY_FILTER_SCALE_NEON       10
#define REPEATX_REPEATY_NOFILTER_AFFINE_NEON    11
#define REPEATX_REPEATY_FILTER_AFFINE_NEON      12
#define REPEATX_REPEATY_NOFILTER_PERSP_NEON     13
#define REPEATX_REPEATY_FILTER_PERSP_NEON       14

#define ShaderProc32_t(x) void (x)(const void *, int x, int y, \
                                   SkPMColor[], int count)

#define MatrixProc_t(x) void (x)(const SkBitmapProcState&,    \
                                 uint32_t bitmapXY[],         \
                                 int count,                   \
                                 int x, int y)

#define SampleProc32_t(x) void (x)(const SkBitmapProcState&,    \
                                   const uint32_t[],            \
                                   int count,                   \
                                   SkPMColor colors[])

#if 0
/*
 * extern MatrixProc declaration
 */
extern MatrixProc_t(clampx_nofilter_trans);
extern MatrixProc_t(repeatx_nofilter_trans);
extern MatrixProc_t(mirrorx_nofilter_trans);
extern MatrixProc_t(ClampX_ClampY_nofilter_scale_neon);
extern MatrixProc_t(ClampX_ClampY_filter_scale_neon);
extern MatrixProc_t(ClampX_ClampY_nofilter_affine_neon);
extern MatrixProc_t(ClampX_ClampY_filter_affine_neon);
extern MatrixProc_t(ClampX_ClampY_nofilter_persp_neon);
extern MatrixProc_t(ClampX_ClampY_filter_persp_neon);
extern MatrixProc_t(RepeatX_RepeatY_nofilter_scale_neon);
extern MatrixProc_t(RepeatX_RepeatY_filter_scale_neon);
extern MatrixProc_t(RepeatX_RepeatY_nofilter_affine_neon);
extern MatrixProc_t(RepeatX_RepeatY_filter_affine_neon);
extern MatrixProc_t(RepeatX_RepeatY_nofilter_persp_neon);
extern MatrixProc_t(RepeatX_RepeatY_filter_persp_neon);
#endif
/*
 * extern SampleProc32 declaration
 */
extern SampleProc32_t(S32_opaque_D32_filter_DX_neon);
extern SampleProc32_t(S32_opaque_D32_nofilter_DX_t_neon);
extern SampleProc32_t(S16_opaque_D32_nofilter_DX_t_neon);
extern SampleProc32_t(SI8_opaque_D32_nofilter_DX_neon);
extern SampleProc32_t(S32_alpha_D32_nofilter_DX_neon);
extern SampleProc32_t(S32_opaque_D32_nofilter_DXDY_neon);
extern SampleProc32_t(S32_alpha_D32_nofilter_DXDY_neon);
extern SampleProc32_t(S32_alpha_D32_filter_DX_neon);
extern SampleProc32_t(S32_opaque_D32_filter_DXDY_neon);
extern SampleProc32_t(S32_alpha_D32_filter_DXDY_neon);
extern SampleProc32_t(SI8_opaque_D32_filter_DX_t_neon);

/*
 * extern ShaderProc32 declaration
 */
extern ShaderProc32_t(S32_Opaque_D32_filter_DX_shaderproc);
extern ShaderProc32_t(ClampX_S16_D32_nofilter_trans_t);
extern ShaderProc32_t(Repeatx_S32_D32_nofilter_trans_t);
extern ShaderProc32_t(ClampX_S32_D32_nofilter_trans_t);

int getMatrixProcName(void *pfunc)
{
    int i = -1;

#if 0
    if (pfunc == clampx_nofilter_trans) {
        return CLAMPX_NOFILTER_TRANS;
    }
    if (pfunc == repeatx_nofilter_trans) {
        return REPEATX_NOFILTER_TRANS;
    }
    if (pfunc == mirrorx_nofilter_trans) {
        return MIRRORX_NOFILTER_TRANS;
    }
    // ClampX_ClampY ## xxx
    if (pfunc == ClampX_ClampY_nofilter_scale_neon) {
        return CLAMPX_CLAMPY_NOFILTER_SCALE_NEON;
    }
    if (pfunc == ClampX_ClampY_filter_scale_neon) {
        return CLAMPX_CLAMPY_FILTER_SCALE_NEON;
    }
    if (pfunc == ClampX_ClampY_nofilter_affine_neon) {
        return CLAMPX_CLAMPY_NOFILTER_AFFINE_NEON;
    }
    if (pfunc == ClampX_ClampY_filter_affine_neon) {
        return CLAMPX_CLAMPY_FILTER_AFFINE_NEON;
    }
    if (pfunc == ClampX_ClampY_nofilter_persp_neon) {
        return CLAMPX_CLAMPY_NOFILTER_PERSP_NEON;
    }
    if (pfunc == ClampX_ClampY_filter_persp_neon) {
        return CLAMPX_CLAMPY_FILTER_PERSP_NEON;
    }
    // RepeatX_RepeatY ## xxx
    if (pfunc == RepeatX_RepeatY_nofilter_scale_neon) {
        return REPEATX_REPEATY_NOFILTER_SCALE_NEON;
    }
    if (pfunc == RepeatX_RepeatY_filter_scale_neon) {
        return REPEATX_REPEATY_FILTER_SCALE_NEON;
    }
    if (pfunc == RepeatX_RepeatY_nofilter_affine_neon) {
        return REPEATX_REPEATY_NOFILTER_AFFINE_NEON;
    }
    if (pfunc == RepeatX_RepeatY_filter_affine_neon) {
        return REPEATX_REPEATY_FILTER_AFFINE_NEON;
    }
    if (pfunc == RepeatX_RepeatY_nofilter_persp_neon) {
        return REPEATX_REPEATY_NOFILTER_PERSP_NEON;
    }
    if (pfunc == RepeatX_RepeatY_filter_persp_neon) {
        return REPEATX_REPEATY_FILTER_PERSP_NEON;
    }
#endif
    return i;
}

/*
 * Merge operations for sampleProc and MatrixProc
 */
SkBitmapProcState::ShaderProc32 SkBitmap_find_merge_proc(SkBitmapProcState::SampleProc32 fSampleProc32,
                                                         bool clampClamp,
                                                         SkBitmapProcState::MatrixProc fMatrixProc)
{
    SkBitmapProcState::ShaderProc32 fShaderProc32 = NULL;

    int matrix_type = getMatrixProcName((void *)fMatrixProc);

    if (S32_opaque_D32_filter_DX_neon == fSampleProc32 && clampClamp) {
        fShaderProc32 = S32_Opaque_D32_filter_DX_shaderproc;
    }

#if DEBUG_MERGE
    ALOGD("fShaderProc32:%p, fSampleProc32:%p, fMatrixProc:%p, matrix_type:%d\n",
          fShaderProc32, fSampleProc32, fMatrixProc, matrix_type);
#endif
    return fShaderProc32;
}

