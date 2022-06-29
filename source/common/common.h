/*****************************************************************************
 * Copyright (C) 2013-2020 MulticoreWare, Inc
 *
 * Authors: Deepthi Nandakumar <deepthi@multicorewareinc.com>
 *          Min Chen <chenm003@163.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
 *
 * This program is also available under a commercial proprietary license.
 * For more information, contact us at license @ s265.com.
 *****************************************************************************/

#ifndef S265_COMMON_H
#define S265_COMMON_H

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <ctime>

#include <stdint.h>
#include <memory.h>
#include <assert.h>

#include "s265.h"

#if ENABLE_PPA && ENABLE_VTUNE
#error "PPA and VTUNE cannot both be enabled. Disable one of them."
#endif
#if ENABLE_PPA
#include "profile/PPA/ppa.h"
#define ProfileScopeEvent(x) PPAScopeEvent(x)
#define THREAD_NAME(n,i)
#define PROFILE_INIT()       PPA_INIT()
#define PROFILE_PAUSE()
#define PROFILE_RESUME()
#elif ENABLE_VTUNE
#include "profile/vtune/vtune.h"
#define ProfileScopeEvent(x) VTuneScopeEvent _vtuneTask(x)
#define THREAD_NAME(n,i)     vtuneSetThreadName(n, i)
#define PROFILE_INIT()       vtuneInit()
#define PROFILE_PAUSE()      __itt_pause()
#define PROFILE_RESUME()     __itt_resume()
#else
#define ProfileScopeEvent(x)
#define THREAD_NAME(n,i)
#define PROFILE_INIT()
#define PROFILE_PAUSE()
#define PROFILE_RESUME()
#endif

#define FENC_STRIDE 64
#define NUM_INTRA_MODE 35

#if defined(__GNUC__)
#define ALIGN_VAR_4(T, var)  T var __attribute__((aligned(4)))
#define ALIGN_VAR_8(T, var)  T var __attribute__((aligned(8)))
#define ALIGN_VAR_16(T, var) T var __attribute__((aligned(16)))
#define ALIGN_VAR_32(T, var) T var __attribute__((aligned(32)))
#define ALIGN_VAR_64(T, var) T var __attribute__((aligned(64)))
#if defined(__MINGW32__)
#define fseeko fseeko64
#define ftello ftello64
#endif
#elif defined(_MSC_VER)

#define ALIGN_VAR_4(T, var)  __declspec(align(4)) T var
#define ALIGN_VAR_8(T, var)  __declspec(align(8)) T var
#define ALIGN_VAR_16(T, var) __declspec(align(16)) T var
#define ALIGN_VAR_32(T, var) __declspec(align(32)) T var
#define ALIGN_VAR_64(T, var) __declspec(align(64)) T var
#define fseeko _fseeki64
#define ftello _ftelli64
#endif // if defined(__GNUC__)
#if HAVE_INT_TYPES_H
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#define S265_LL "%" PRIu64
#else
#define S265_LL "%lld"
#endif

#if _DEBUG && defined(_MSC_VER)
#define DEBUG_BREAK() __debugbreak()
#elif __APPLE_CC__
#define DEBUG_BREAK() __builtin_trap()
#else
#define DEBUG_BREAK() abort()
#endif

/* If compiled with CHECKED_BUILD perform run-time checks and log any that
 * fail, both to stderr and to a file */
#if CHECKED_BUILD || _DEBUG
namespace S265_NS { extern int g_checkFailures; }
#define S265_CHECK(expr, ...) if (!(expr)) { \
    s265_log(NULL, S265_LOG_ERROR, __VA_ARGS__); \
    FILE *fp = fopen("s265_check_failures.txt", "a"); \
    if (fp) { fprintf(fp, "%s:%d\n", __FILE__, __LINE__); fprintf(fp, __VA_ARGS__); fclose(fp); } \
    g_checkFailures++; DEBUG_BREAK(); \
}
#if _MSC_VER
#pragma warning(disable: 4127) // some checks have constant conditions
#endif
#else
#define S265_CHECK(expr, ...)
#endif

#if HIGH_BIT_DEPTH
typedef uint16_t pixel;
typedef uint32_t sum_t;
typedef uint64_t sum2_t;
typedef uint64_t pixel4;
typedef int64_t  ssum2_t;
#define SHIFT_TO_BITPLANE 9
#define HISTOGRAM_BINS 1024
#else
typedef uint8_t  pixel;
typedef uint16_t sum_t;
typedef uint32_t sum2_t;
typedef uint32_t pixel4;
typedef int32_t  ssum2_t; // Signed sum
#define SHIFT_TO_BITPLANE 7
#define HISTOGRAM_BINS 256
#endif // if HIGH_BIT_DEPTH

#if S265_DEPTH < 10
typedef uint32_t sse_t;
#else
typedef uint64_t sse_t;
#endif

#ifndef NULL
#define NULL 0
#endif

#define MAX_UINT        0xFFFFFFFFU // max. value of unsigned 32-bit integer
#define MAX_INT         2147483647  // max. value of signed 32-bit integer
#define MAX_INT64       0x7FFFFFFFFFFFFFFFLL  // max. value of signed 64-bit integer
#define MAX_DOUBLE      1.7e+308    // max. value of double-type value

#define QP_MIN          0
#define QP_MAX_SPEC     51 /* max allowed signaled QP in HEVC */
#define QP_MAX_MAX      69 /* max allowed QP to be output by rate control */

#define MIN_QPSCALE     0.21249999999999999
#define MAX_MAX_QPSCALE 615.46574234477100


template<typename T>
inline T s265_min(T a, T b) { return a < b ? a : b; }

template<typename T>
inline T s265_max(T a, T b) { return a > b ? a : b; }

template<typename T>
inline T s265_clip3(T minVal, T maxVal, T a) { return s265_min(s265_max(minVal, a), maxVal); }

template<typename T> /* clip to pixel range, 0..255 or 0..1023 */
inline pixel s265_clip(T x) { return (pixel)s265_min<T>(T((1 << S265_DEPTH) - 1), s265_max<T>(T(0), x)); }

typedef int16_t  coeff_t;      // transform coefficient

#define S265_MIN(a, b) ((a) < (b) ? (a) : (b))
#define S265_MAX(a, b) ((a) > (b) ? (a) : (b))
#define COPY1_IF_LT(x, y) {if ((y) < (x)) (x) = (y);}
#define COPY2_IF_LT(x, y, a, b) \
    if ((y) < (x)) \
    { \
        (x) = (y); \
        (a) = (b); \
    }
#define COPY3_IF_LT(x, y, a, b, c, d) \
    if ((y) < (x)) \
    { \
        (x) = (y); \
        (a) = (b); \
        (c) = (d); \
    }
#define COPY4_IF_LT(x, y, a, b, c, d, e, f) \
    if ((y) < (x)) \
    { \
        (x) = (y); \
        (a) = (b); \
        (c) = (d); \
        (e) = (f); \
    }
#define S265_MIN3(a, b, c) S265_MIN((a), S265_MIN((b), (c)))
#define S265_MAX3(a, b, c) S265_MAX((a), S265_MAX((b), (c)))
#define S265_MIN4(a, b, c, d) S265_MIN((a), S265_MIN3((b), (c), (d)))
#define S265_MAX4(a, b, c, d) S265_MAX((a), S265_MAX3((b), (c), (d)))
#define QP_BD_OFFSET (6 * (S265_DEPTH - 8))
#define MAX_CHROMA_LAMBDA_OFFSET 36

// arbitrary, but low because SATD scores are 1/4 normal
#define S265_LOOKAHEAD_QP (12 + QP_BD_OFFSET)

// Use the same size blocks as x264.  Using larger blocks seems to give artificially
// high cost estimates (intra and inter both suffer)
#define S265_LOWRES_CU_SIZE   8
#define S265_LOWRES_CU_BITS   3

#define S265_MALLOC(type, count)    (type*)s265_malloc(sizeof(type) * (count))
#define S265_FREE(ptr)              s265_free(ptr)
#define S265_FREE_ZERO(ptr)         s265_free(ptr); (ptr) = NULL
#define CHECKED_MALLOC(var, type, count) \
    { \
        var = (type*)s265_malloc(sizeof(type) * (count)); \
        if (!var) \
        { \
            s265_log(NULL, S265_LOG_ERROR, "malloc of size %d failed\n", sizeof(type) * (count)); \
            goto fail; \
        } \
    }
#define CHECKED_MALLOC_ZERO(var, type, count) \
    { \
        var = (type*)s265_malloc(sizeof(type) * (count)); \
        if (var) \
            memset((void*)var, 0, sizeof(type) * (count)); \
        else \
        { \
            s265_log(NULL, S265_LOG_ERROR, "malloc of size %d failed\n", sizeof(type) * (count)); \
            goto fail; \
        } \
    }

#if defined(_MSC_VER)
#define S265_LOG2F(x) (logf((float)(x)) * 1.44269504088896405f)
#define S265_LOG2(x) (log((double)(x)) * 1.4426950408889640513713538072172)
#else
#define S265_LOG2F(x) log2f(x)
#define S265_LOG2(x)  log2(x)
#endif

#define NUM_CU_DEPTH            4                           // maximum number of CU depths
#define NUM_FULL_DEPTH          5                           // maximum number of full depths
#define MIN_LOG2_CU_SIZE        3                           // log2(minCUSize)
#define MAX_LOG2_CU_SIZE        6                           // log2(maxCUSize) 
#define MIN_CU_SIZE             (1 << MIN_LOG2_CU_SIZE)     // minimum allowable size of CU 8x8 (8)
#define MAX_CU_SIZE             (1 << MAX_LOG2_CU_SIZE)     // maximum allowable size of CU 64x64 (64)

#define LOG2_UNIT_SIZE          2                           // log2(unitSize) (2)
#define UNIT_SIZE               (1 << LOG2_UNIT_SIZE)       // unit size of CU partition 4x4 为单位 (4)

#define LOG2_RASTER_SIZE        (MAX_LOG2_CU_SIZE - LOG2_UNIT_SIZE)// 4
#define RASTER_SIZE             (1 << LOG2_RASTER_SIZE)      // 16
#define MAX_NUM_PARTITIONS      (RASTER_SIZE * RASTER_SIZE)  // 256

#define MIN_PU_SIZE             4
#define MIN_TU_SIZE             4
#define MAX_NUM_SPU_W           (MAX_CU_SIZE / MIN_PU_SIZE) // maximum number of SPU in horizontal line 16

#define MAX_LOG2_TR_SIZE 5  // tranform size
#define MAX_LOG2_TS_SIZE 2 // TODO: RExt tranform_skip size
#define MAX_TR_SIZE (1 << MAX_LOG2_TR_SIZE)
#define MAX_TS_SIZE (1 << MAX_LOG2_TS_SIZE)

#define RDCOST_BASED_RSKIP 1
#define EDGE_BASED_RSKIP 2

#define COEF_REMAIN_BIN_REDUCTION   3 // indicates the level at which the VLC
                                      // transitions from Golomb-Rice to TU+EG(k)

#define SBH_THRESHOLD               4 // fixed sign bit hiding controlling threshold

#define C1FLAG_NUMBER               8 // maximum number of largerThan1 flag coded in one chunk:  16 in HM5
#define C2FLAG_NUMBER               1 // maximum number of largerThan2 flag coded in one chunk:  16 in HM5

#define SAO_ENCODING_RATE           0.75
#define SAO_ENCODING_RATE_CHROMA    0.5

#define MLS_GRP_NUM                 64 // Max number of coefficient groups, max(16, 64)
#define MLS_CG_SIZE                 4  // Coefficient group size of 4x4
#define MLS_CG_BLK_SIZE             (MLS_CG_SIZE * MLS_CG_SIZE)
#define MLS_CG_LOG2_SIZE            2

#define QUANT_IQUANT_SHIFT          20 // Q(QP%6) * IQ(QP%6) = 2^20
#define QUANT_SHIFT                 14 // Q(4) = 2^14
#define SCALE_BITS                  15 // Inherited from TMuC, presumably for fractional bit estimates in RDOQ
#define MAX_TR_DYNAMIC_RANGE        15 // Maximum transform dynamic range (excluding sign bit)

#define SHIFT_INV_1ST               7  // Shift after first inverse transform stage
#define SHIFT_INV_2ND               12 // Shift after second inverse transform stage

#define AMVP_DECIMATION_FACTOR      4

#define SCAN_SET_SIZE               16
#define LOG2_SCAN_SET_SIZE          4

#define ALL_IDX                     -1
#define PLANAR_IDX                  0
#define VER_IDX                     26 // index for intra VERTICAL   mode
#define HOR_IDX                     10 // index for intra HORIZONTAL mode
#define DC_IDX                      1  // index for intra DC mode
#define NUM_CHROMA_MODE             5  // total number of chroma modes
#define DM_CHROMA_IDX               36 // chroma mode index for derived from luma intra mode

#define MDCS_ANGLE_LIMIT            4 // distance from true angle that horiz or vertical scan is allowed
#define MDCS_LOG2_MAX_SIZE          3 // TUs with log2 of size greater than this can only use diagonal scan

#define MAX_NUM_REF_PICS            16 // max. number of pictures used for reference
#define MAX_NUM_REF                 16 // max. number of entries in picture reference list
#define MAX_NUM_SHORT_TERM_RPS      64 // max. number of short term reference picture set in SPS

#define REF_NOT_VALID               -1

#define AMVP_NUM_CANDS              2 // number of AMVP candidates
#define MRG_MAX_NUM_CANDS           5 // max number of final merge candidates

#define CHROMA_H_SHIFT(x) (x == S265_CSP_I420 || x == S265_CSP_I422)
#define CHROMA_V_SHIFT(x) (x == S265_CSP_I420)
#define S265_MAX_PRED_MODE_PER_CTU 85 * 2 * 8

#define MAX_NUM_TR_COEFFS           MAX_TR_SIZE * MAX_TR_SIZE // Maximum number of transform coefficients, for a 32x32 transform
#define MAX_NUM_TR_CATEGORIES       16                        // 32, 16, 8, 4 transform categories each for luma and chroma

#define PIXEL_MAX ((1 << S265_DEPTH) - 1)

#define INTEGRAL_PLANE_NUM          12 // 12 integral planes for 32x32, 32x24, 32x8, 24x32, 16x16, 16x12, 16x4, 12x16, 8x32, 8x8, 4x16 and 4x4.

#define NAL_TYPE_OVERHEAD 2
#define START_CODE_OVERHEAD 3 
#define FILLER_OVERHEAD (NAL_TYPE_OVERHEAD + START_CODE_OVERHEAD + 1)

#define MAX_NUM_DYN_REFINE          (NUM_CU_DEPTH * S265_REFINE_INTER_LEVELS)

namespace S265_NS {

enum { SAO_NUM_OFFSET = 4 };

enum SaoMergeMode
{
    SAO_MERGE_NONE,
    SAO_MERGE_LEFT,
    SAO_MERGE_UP
};

struct SaoCtuParam
{
    SaoMergeMode mergeMode;
    int  typeIdx;
    uint32_t bandPos;    // BO band position
    int  offset[SAO_NUM_OFFSET];

    void reset()
    {
        mergeMode = SAO_MERGE_NONE;
        typeIdx = -1;
        bandPos = 0;
        offset[0] = 0;
        offset[1] = 0;
        offset[2] = 0;
        offset[3] = 0;
    }
};

struct SAOParam
{
    SaoCtuParam* ctuParam[3];
    bool         bSaoFlag[2];
    int          numCuInWidth;

    SAOParam()
    {
        for (int i = 0; i < 3; i++)
            ctuParam[i] = NULL;
    }

    ~SAOParam()
    {
        delete[] ctuParam[0];
        delete[] ctuParam[1];
        delete[] ctuParam[2];
    }
};
enum TextType
{
    TEXT_LUMA     = 0,  // luma
    TEXT_CHROMA_U = 1,  // chroma U
    TEXT_CHROMA_V = 2,  // chroma V
    MAX_NUM_COMPONENT = 3
};

// coefficient scanning type used in ACS
enum ScanType
{
    SCAN_DIAG = 0,     // up-right diagonal scan
    SCAN_HOR = 1,      // horizontal first scan
    SCAN_VER = 2,      // vertical first scan
    NUM_SCAN_TYPE = 3
};

enum SignificanceMapContextType
{
    CONTEXT_TYPE_4x4 = 0,
    CONTEXT_TYPE_8x8 = 1,
    CONTEXT_TYPE_NxN = 2,
    CONTEXT_NUMBER_OF_TYPES = 3
};

/* located in pixel.cpp */
void extendPicBorder(pixel* recon, intptr_t stride, int width, int height, int marginX, int marginY);


/* located in common.cpp */
int64_t  s265_mdate(void);
#define  s265_log(param,level,psz_fmt,...) s265_internal_log(param, "s265", level, psz_fmt, ##__VA_ARGS__)
#define  s265_log_file(param, ...) general_log_file(param, "s265", __VA_ARGS__)
void     general_log(const s265_param* param, const char* caller, int level, const char* fmt, ...);
void     s265_internal_log(const s265_param* param, const char* caller, int i_level, const char *psz_fmt, ...);
void     s265_log_default(void *p_unused, int level, const char *psz_fmt,  ...);
#if _WIN32
void     general_log_file(const s265_param* param, const char* caller, int level, const char* fmt, ...);
FILE*    s265_fopen(const char* fileName, const char* mode);
int      s265_unlink(const char* fileName);
int      s265_rename(const char* oldName, const char* newName);
#else
#define  general_log_file(param, caller, level, fmt, ...) s265_internal_log(param, caller, level, fmt, ##__VA_ARGS__)
#define  s265_fopen(fileName, mode) fopen(fileName, mode)
#define  s265_unlink(fileName) unlink(fileName)
#define  s265_rename(oldName, newName) rename(oldName, newName)
#endif
int      s265_exp2fix8(double x);

double   s265_ssim2dB(double ssim);
double   s265_qScale2qp(double qScale);
double   s265_qp2qScale(double qp);
uint32_t s265_picturePlaneSize(int csp, int width, int height, int plane);

void*    s265_malloc(size_t size);
void     s265_free(void *ptr);
char*    s265_slurp_file(const char *filename);

/* located in primitives.cpp */
void     s265_setup_primitives(s265_param* param);
void     s265_report_simd(s265_param* param);
}

#include "constants.h"

#endif // ifndef S265_COMMON_H
