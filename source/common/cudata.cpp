/*****************************************************************************
 * Copyright (C) 2013-2020 MulticoreWare, Inc
 *
 * Authors: Steve Borho <steve@borho.org>
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

#include "common.h"
#include "frame.h"
#include "framedata.h"
#include "picyuv.h"
#include "mv.h"
#include "cudata.h"
#define MAX_MV 1 << 14

using namespace S265_NS;

/* for all bcast* and copy* functions, dst and src are aligned to MIN(size, 32) */

static void bcast1(uint8_t* dst, uint8_t val)  { dst[0] = val; }

static void copy4(uint8_t* dst, uint8_t* src)  { ((uint32_t*)dst)[0] = ((uint32_t*)src)[0]; }
static void bcast4(uint8_t* dst, uint8_t val)  { ((uint32_t*)dst)[0] = 0x01010101u * val; }

static void copy16(uint8_t* dst, uint8_t* src) { ((uint64_t*)dst)[0] = ((uint64_t*)src)[0]; ((uint64_t*)dst)[1] = ((uint64_t*)src)[1]; }
static void bcast16(uint8_t* dst, uint8_t val) { uint64_t bval = 0x0101010101010101ULL * val; ((uint64_t*)dst)[0] = bval; ((uint64_t*)dst)[1] = bval; }

static void copy64(uint8_t* dst, uint8_t* src) { ((uint64_t*)dst)[0] = ((uint64_t*)src)[0]; ((uint64_t*)dst)[1] = ((uint64_t*)src)[1]; 
                                                 ((uint64_t*)dst)[2] = ((uint64_t*)src)[2]; ((uint64_t*)dst)[3] = ((uint64_t*)src)[3];
                                                 ((uint64_t*)dst)[4] = ((uint64_t*)src)[4]; ((uint64_t*)dst)[5] = ((uint64_t*)src)[5];
                                                 ((uint64_t*)dst)[6] = ((uint64_t*)src)[6]; ((uint64_t*)dst)[7] = ((uint64_t*)src)[7]; }
static void bcast64(uint8_t* dst, uint8_t val) { uint64_t bval = 0x0101010101010101ULL * val;
                                                 ((uint64_t*)dst)[0] = bval; ((uint64_t*)dst)[1] = bval; ((uint64_t*)dst)[2] = bval; ((uint64_t*)dst)[3] = bval;
                                                 ((uint64_t*)dst)[4] = bval; ((uint64_t*)dst)[5] = bval; ((uint64_t*)dst)[6] = bval; ((uint64_t*)dst)[7] = bval; }

/* at 256 bytes, memset/memcpy will probably use SIMD more effectively than our uint64_t hack,
 * but hand-written assembly would beat it. */
static void copy256(uint8_t* dst, uint8_t* src) { memcpy(dst, src, 256); }
static void bcast256(uint8_t* dst, uint8_t val) { memset(dst, val, 256); }

namespace {
// file private namespace

/* Check whether 2 addresses point to the same column */
inline bool isEqualCol(int addrA, int addrB)
{
    return ((addrA ^ addrB) & (RASTER_SIZE - 1)) == 0;
}

/* Check whether 2 addresses point to the same row */
inline bool isEqualRow(int addrA, int addrB)
{
    return ((addrA ^ addrB) < RASTER_SIZE);
}

/* Check whether 2 addresses point to the same row or column */
inline bool isEqualRowOrCol(int addrA, int addrB)
{
    return isEqualCol(addrA, addrB) | isEqualRow(addrA, addrB);
}

/* Check whether one address points to the first column */
inline bool isZeroCol(int addr)
{
    return (addr & (RASTER_SIZE - 1)) == 0;
}

/* Check whether one address points to the first row */
inline bool isZeroRow(int addr)
{
    return (addr < RASTER_SIZE);
}

/* Check whether one address points to a column whose index is smaller than a given value */
inline bool lessThanCol(int addr, int val)
{
    return (addr & (RASTER_SIZE - 1)) < val;
}

/* Check whether one address points to a row whose index is smaller than a given value */
inline bool lessThanRow(int addr, int val)
{
    // addr / numUnits < val
    return (addr >> LOG2_RASTER_SIZE) < val;
}

inline MV scaleMv(MV mv, int scale)
{
    int mvx = s265_clip3(-32768, 32767, (scale * mv.x + 127 + (scale * mv.x < 0)) >> 8);
    int mvy = s265_clip3(-32768, 32767, (scale * mv.y + 127 + (scale * mv.y < 0)) >> 8);

    return MV((int32_t)mvx, (int32_t)mvy);
}

}

CUData::CUData()
{
    memset(this, 0, sizeof(*this));
}

void CUData::initialize(const CUDataMemPool& dataPool, uint32_t depth, const s265_param& param, int instance)
{
    int csp = param.internalCsp;
    m_chromaFormat  = csp;
    m_hChromaShift  = CHROMA_H_SHIFT(csp);
    m_vChromaShift  = CHROMA_V_SHIFT(csp);
    m_numPartitions = param.num4x4Partitions >> (depth * 2);// depth0对应64x64 有256个4x4  depth1对应32x32 有64个4x4

    if (!s_partSet[0])
    {
        s_numPartInCUSize = 1 << param.unitSizeDepth;////CTU中一边有多少4x4块 默认为16（64有16个4）
        switch (param.maxLog2CUSize)
        {
        case 6:
            s_partSet[0] = bcast256;
            s_partSet[1] = bcast64;
            s_partSet[2] = bcast16;
            s_partSet[3] = bcast4;
            s_partSet[4] = bcast1;
            break;
        case 5:
            s_partSet[0] = bcast64;
            s_partSet[1] = bcast16;
            s_partSet[2] = bcast4;
            s_partSet[3] = bcast1;
            s_partSet[4] = NULL;
            break;
        case 4:
            s_partSet[0] = bcast16;
            s_partSet[1] = bcast4;
            s_partSet[2] = bcast1;
            s_partSet[3] = NULL;
            s_partSet[4] = NULL;
            break;
        default:
            S265_CHECK(0, "unexpected CTU size\n");
            break;
        }
    }

    switch (m_numPartitions)
    {
    case 256: // 64x64 CU
        m_partCopy = copy256;
        m_partSet = bcast256;
        m_subPartCopy = copy64;
        m_subPartSet = bcast64;
        break;
    case 64:  // 32x32 CU
        m_partCopy = copy64;
        m_partSet = bcast64;
        m_subPartCopy = copy16;
        m_subPartSet = bcast16;
        break;
    case 16:  // 16x16 CU
        m_partCopy = copy16;
        m_partSet = bcast16;
        m_subPartCopy = copy4;
        m_subPartSet = bcast4;
        break;
    case 4:   // 8x8 CU
        m_partCopy = copy4;
        m_partSet = bcast4;
        m_subPartCopy = NULL;
        m_subPartSet = NULL;
        break;
    default:
        S265_CHECK(0, "unexpected CU partition count\n");
        break;
    }

    if (csp == S265_CSP_I400)
    {
        /* Each CU's data is layed out sequentially within the charMemBlock */
        uint8_t *charBuf = dataPool.charMemBlock + (m_numPartitions * (BytesPerPartition - 4)) * instance;

        m_qp        = (int8_t*)charBuf; charBuf += m_numPartitions;
        m_qpAnalysis = (int8_t*)charBuf; charBuf += m_numPartitions;
        m_log2CUSize         = charBuf; charBuf += m_numPartitions;
        m_lumaIntraDir       = charBuf; charBuf += m_numPartitions;
        m_tqBypass           = charBuf; charBuf += m_numPartitions;
        m_refIdx[0] = (int8_t*)charBuf; charBuf += m_numPartitions;
        m_refIdx[1] = (int8_t*)charBuf; charBuf += m_numPartitions;
        m_cuDepth            = charBuf; charBuf += m_numPartitions;
        m_predMode           = charBuf; charBuf += m_numPartitions; /* the order up to here is important in initCTU() and initSubCU() */
        m_partSize           = charBuf; charBuf += m_numPartitions;
        m_skipFlag[0]        = charBuf; charBuf += m_numPartitions;
        m_skipFlag[1]        = charBuf; charBuf += m_numPartitions;
        m_mergeFlag          = charBuf; charBuf += m_numPartitions;
        m_interDir           = charBuf; charBuf += m_numPartitions;
        m_mvpIdx[0]          = charBuf; charBuf += m_numPartitions;
        m_mvpIdx[1]          = charBuf; charBuf += m_numPartitions;
        m_tuDepth            = charBuf; charBuf += m_numPartitions;
        m_transformSkip[0]   = charBuf; charBuf += m_numPartitions;
        m_cbf[0]             = charBuf; charBuf += m_numPartitions;
        m_chromaIntraDir     = charBuf; charBuf += m_numPartitions;

        S265_CHECK(charBuf == dataPool.charMemBlock + (m_numPartitions * (BytesPerPartition - 4)) * (instance + 1), "CU data layout is broken\n"); //BytesPerPartition

        m_mv[0]  = dataPool.mvMemBlock + (instance * 4) * m_numPartitions;
        m_mv[1]  = m_mv[0] +  m_numPartitions;
        m_mvd[0] = m_mv[1] +  m_numPartitions;
        m_mvd[1] = m_mvd[0] + m_numPartitions;

        m_distortion = dataPool.distortionMemBlock + instance * m_numPartitions;

        uint32_t cuSize = param.maxCUSize >> depth;
        m_trCoeff[0] = dataPool.trCoeffMemBlock + instance * (cuSize * cuSize);
        m_trCoeff[1] = m_trCoeff[2] = 0;
        m_transformSkip[1] = m_transformSkip[2] = m_cbf[1] = m_cbf[2] = 0;
        m_fAc_den[0] = m_fDc_den[0] = 0;
    }
    else
    {
        /* Each CU's data is layed out sequentially within the charMemBlock */
        uint8_t *charBuf = dataPool.charMemBlock + (m_numPartitions * BytesPerPartition) * instance;
        // 以64x64CTU为例:共24个成员 每个成员占用256个字节，
        m_qp        = (int8_t*)charBuf; charBuf += m_numPartitions;
        m_qpAnalysis = (int8_t*)charBuf; charBuf += m_numPartitions;
        m_log2CUSize         = charBuf; charBuf += m_numPartitions;
        m_lumaIntraDir       = charBuf; charBuf += m_numPartitions;
        m_tqBypass           = charBuf; charBuf += m_numPartitions;
        m_refIdx[0] = (int8_t*)charBuf; charBuf += m_numPartitions;
        m_refIdx[1] = (int8_t*)charBuf; charBuf += m_numPartitions;
        m_cuDepth            = charBuf; charBuf += m_numPartitions;
        m_predMode           = charBuf; charBuf += m_numPartitions; /* the order up to here is important in initCTU() and initSubCU() */
        m_partSize           = charBuf; charBuf += m_numPartitions;
        m_skipFlag[0]        = charBuf; charBuf += m_numPartitions;
        m_skipFlag[1]        = charBuf; charBuf += m_numPartitions;
        m_mergeFlag          = charBuf; charBuf += m_numPartitions;
        m_interDir           = charBuf; charBuf += m_numPartitions;
        m_mvpIdx[0]          = charBuf; charBuf += m_numPartitions;
        m_mvpIdx[1]          = charBuf; charBuf += m_numPartitions;
        m_tuDepth            = charBuf; charBuf += m_numPartitions;
        m_transformSkip[0]   = charBuf; charBuf += m_numPartitions;
        m_transformSkip[1]   = charBuf; charBuf += m_numPartitions;
        m_transformSkip[2]   = charBuf; charBuf += m_numPartitions;
        m_cbf[0]             = charBuf; charBuf += m_numPartitions;
        m_cbf[1]             = charBuf; charBuf += m_numPartitions;
        m_cbf[2]             = charBuf; charBuf += m_numPartitions;
        m_chromaIntraDir     = charBuf; charBuf += m_numPartitions;

        S265_CHECK(charBuf == dataPool.charMemBlock + (m_numPartitions * BytesPerPartition) * (instance + 1), "CU data layout is broken\n");

        m_mv[0]  = dataPool.mvMemBlock + (instance * 4) * m_numPartitions;
        m_mv[1]  = m_mv[0] +  m_numPartitions;
        m_mvd[0] = m_mv[1] +  m_numPartitions;
        m_mvd[1] = m_mvd[0] + m_numPartitions;

        m_distortion = dataPool.distortionMemBlock + instance * m_numPartitions;

        uint32_t cuSize = param.maxCUSize >> depth;
        uint32_t sizeL = cuSize * cuSize;
        uint32_t sizeC = sizeL >> (m_hChromaShift + m_vChromaShift); // block chroma part
        m_trCoeff[0] = dataPool.trCoeffMemBlock + instance * (sizeL + sizeC * 2);
        m_trCoeff[1] = m_trCoeff[0] + sizeL;
        m_trCoeff[2] = m_trCoeff[0] + sizeL + sizeC;
        for (int i = 0; i < 3; i++)
            m_fAc_den[i] = m_fDc_den[i] = 0;
    }
}

void CUData::initCTU(const Frame& frame, uint32_t cuAddr, int qp, uint32_t firstRowInSlice, uint32_t lastRowInSlice, uint32_t lastCuInSlice)
{
    m_encData       = frame.m_encData;
    m_slice         = m_encData->m_slice;
    m_cuAddr        = cuAddr;
    m_cuPelX        = (cuAddr % m_slice->m_sps->numCuInWidth) << m_slice->m_param->maxLog2CUSize;
    m_cuPelY        = (cuAddr / m_slice->m_sps->numCuInWidth) << m_slice->m_param->maxLog2CUSize;
    m_absIdxInCTU   = 0;
    m_numPartitions = m_encData->m_param->num4x4Partitions;// 64x64ctu: 256
    m_bFirstRowInSlice = (uint8_t)firstRowInSlice;
    m_bLastRowInSlice  = (uint8_t)lastRowInSlice;
    m_bLastCuInSlice   = (uint8_t)lastCuInSlice;

    /* sequential memsets */
    m_partSet((uint8_t*)m_qp, (uint8_t)qp);
    m_partSet((uint8_t*)m_qpAnalysis, (uint8_t)qp);
    m_partSet(m_log2CUSize,   (uint8_t)m_slice->m_param->maxLog2CUSize);
    m_partSet(m_lumaIntraDir, (uint8_t)ALL_IDX);// 初始化为 -1 不可用
    m_partSet(m_chromaIntraDir, (uint8_t)ALL_IDX);
    m_partSet(m_tqBypass,     (uint8_t)frame.m_encData->m_param->bLossless);
    if (m_slice->m_sliceType != I_SLICE)
    {
        m_partSet((uint8_t*)m_refIdx[0], (uint8_t)REF_NOT_VALID);
        m_partSet((uint8_t*)m_refIdx[1], (uint8_t)REF_NOT_VALID);
    }

    S265_CHECK(!(frame.m_encData->m_param->bLossless && !m_slice->m_pps->bTransquantBypassEnabled), "lossless enabled without TQbypass in PPS\n");

    /* initialize the remaining CU data in one memset */
    /* 从m_cuDepth 到 m_cbf[0] 或者到  m_chromaIntraDir 全部memset 为0*/
    memset(m_cuDepth, 0, (frame.m_param->internalCsp == S265_CSP_I400 ? BytesPerPartition - 12 : BytesPerPartition - 8) * m_numPartitions);

    for (int8_t i = 0; i < NUM_TU_DEPTH; i++)
        m_refTuDepth[i] = -1;

    m_vbvAffected = false;

    uint32_t widthInCU = m_slice->m_sps->numCuInWidth;
    m_cuLeft = (m_cuAddr % widthInCU) ? m_encData->getPicCTU(m_cuAddr - 1) : NULL;
    m_cuAbove = (m_cuAddr >= widthInCU) && !m_bFirstRowInSlice ? m_encData->getPicCTU(m_cuAddr - widthInCU) : NULL;
    m_cuAboveLeft = (m_cuLeft && m_cuAbove) ? m_encData->getPicCTU(m_cuAddr - widthInCU - 1) : NULL;
    m_cuAboveRight = (m_cuAbove && ((m_cuAddr % widthInCU) < (widthInCU - 1))) ? m_encData->getPicCTU(m_cuAddr - widthInCU + 1) : NULL;
    memset(m_distortion, 0, m_numPartitions * sizeof(sse_t));
}

// initialize Sub partition
void CUData::initSubCU(const CUData& ctu, const CUGeom& cuGeom, int qp)
{
    m_absIdxInCTU   = cuGeom.absPartIdx;//subCU相对于整个CTU的4x4索引
    m_encData       = ctu.m_encData;
    m_slice         = ctu.m_slice;
    m_cuAddr        = ctu.m_cuAddr;
    m_cuPelX        = ctu.m_cuPelX + g_zscanToPelX[cuGeom.absPartIdx];
    m_cuPelY        = ctu.m_cuPelY + g_zscanToPelY[cuGeom.absPartIdx];
    m_cuLeft        = ctu.m_cuLeft;
    m_cuAbove       = ctu.m_cuAbove;
    m_cuAboveLeft   = ctu.m_cuAboveLeft;
    m_cuAboveRight  = ctu.m_cuAboveRight;
    m_bFirstRowInSlice = ctu.m_bFirstRowInSlice;
    m_bLastRowInSlice = ctu.m_bLastRowInSlice;
    m_bLastCuInSlice = ctu.m_bLastCuInSlice;
    for (int i = 0; i < 3; i++)
    {
        m_fAc_den[i] = ctu.m_fAc_den[i];
        m_fDc_den[i] = ctu.m_fDc_den[i];
    }

    S265_CHECK(m_numPartitions == cuGeom.numPartitions, "initSubCU() size mismatch\n");

    m_partSet((uint8_t*)m_qp, (uint8_t)qp);
    m_partSet((uint8_t*)m_qpAnalysis, (uint8_t)qp);

    m_partSet(m_log2CUSize,   (uint8_t)cuGeom.log2CUSize);
    m_partSet(m_lumaIntraDir, (uint8_t)ALL_IDX);//初始化
    m_partSet(m_chromaIntraDir, (uint8_t)ALL_IDX);//初始化
    m_partSet(m_tqBypass,     (uint8_t)m_encData->m_param->bLossless);//如果启用无失真编码则所有cu使用 transquantbypass
    m_partSet((uint8_t*)m_refIdx[0], (uint8_t)REF_NOT_VALID);
    m_partSet((uint8_t*)m_refIdx[1], (uint8_t)REF_NOT_VALID);
    m_partSet(m_cuDepth,      (uint8_t)cuGeom.depth);

    /* initialize the remaining CU data in one memset */
    /* 从m_predMode 到 m_cbf[0] 或者到  m_chromaIntraDir 全部memset 为0*/
    memset(m_predMode, 0, (ctu.m_chromaFormat == S265_CSP_I400 ? BytesPerPartition - 13 : BytesPerPartition - 9) * m_numPartitions);
    memset(m_distortion, 0, m_numPartitions * sizeof(sse_t));
}

/* Copy the results of a sub-part (split) CU to the parent CU */
void CUData::copyPartFrom(const CUData& subCU, const CUGeom& childGeom, uint32_t subPartIdx)
{
    S265_CHECK(subPartIdx < 4, "part unit should be less than 4\n");

    uint32_t offset = childGeom.numPartitions * subPartIdx;

    m_bFirstRowInSlice = subCU.m_bFirstRowInSlice;
    m_bLastCuInSlice = subCU.m_bLastCuInSlice;

    m_subPartCopy((uint8_t*)m_qp + offset, (uint8_t*)subCU.m_qp);
    m_subPartCopy((uint8_t*)m_qpAnalysis + offset, (uint8_t*)subCU.m_qpAnalysis);
    m_subPartCopy(m_log2CUSize + offset, subCU.m_log2CUSize);
    m_subPartCopy(m_lumaIntraDir + offset, subCU.m_lumaIntraDir);
    m_subPartCopy(m_tqBypass + offset, subCU.m_tqBypass);
    m_subPartCopy((uint8_t*)m_refIdx[0] + offset, (uint8_t*)subCU.m_refIdx[0]);
    m_subPartCopy((uint8_t*)m_refIdx[1] + offset, (uint8_t*)subCU.m_refIdx[1]);
    m_subPartCopy(m_cuDepth + offset, subCU.m_cuDepth);
    m_subPartCopy(m_predMode + offset, subCU.m_predMode);
    m_subPartCopy(m_partSize + offset, subCU.m_partSize);
    m_subPartCopy(m_mergeFlag + offset, subCU.m_mergeFlag);
    m_subPartCopy(m_interDir + offset, subCU.m_interDir);
    m_subPartCopy(m_mvpIdx[0] + offset, subCU.m_mvpIdx[0]);
    m_subPartCopy(m_mvpIdx[1] + offset, subCU.m_mvpIdx[1]);
    m_subPartCopy(m_tuDepth + offset, subCU.m_tuDepth);

    m_subPartCopy(m_transformSkip[0] + offset, subCU.m_transformSkip[0]);
    m_subPartCopy(m_cbf[0] + offset, subCU.m_cbf[0]);

    memcpy(m_mv[0] + offset, subCU.m_mv[0], childGeom.numPartitions * sizeof(MV));
    memcpy(m_mv[1] + offset, subCU.m_mv[1], childGeom.numPartitions * sizeof(MV));
    memcpy(m_mvd[0] + offset, subCU.m_mvd[0], childGeom.numPartitions * sizeof(MV));
    memcpy(m_mvd[1] + offset, subCU.m_mvd[1], childGeom.numPartitions * sizeof(MV));

    memcpy(m_distortion + offset, subCU.m_distortion, childGeom.numPartitions * sizeof(sse_t));

    uint32_t tmp = 1 << ((m_slice->m_param->maxLog2CUSize - childGeom.depth) * 2);
    uint32_t tmp2 = subPartIdx * tmp;
    memcpy(m_trCoeff[0] + tmp2, subCU.m_trCoeff[0], sizeof(coeff_t)* tmp);

    if (subCU.m_chromaFormat != S265_CSP_I400)
    {
        m_subPartCopy(m_transformSkip[1] + offset, subCU.m_transformSkip[1]);
        m_subPartCopy(m_transformSkip[2] + offset, subCU.m_transformSkip[2]);
        m_subPartCopy(m_cbf[1] + offset, subCU.m_cbf[1]);
        m_subPartCopy(m_cbf[2] + offset, subCU.m_cbf[2]);
        m_subPartCopy(m_chromaIntraDir + offset, subCU.m_chromaIntraDir);

        uint32_t tmpC = tmp >> (m_hChromaShift + m_vChromaShift);
        uint32_t tmpC2 = tmp2 >> (m_hChromaShift + m_vChromaShift);
        memcpy(m_trCoeff[1] + tmpC2, subCU.m_trCoeff[1], sizeof(coeff_t) * tmpC);
        memcpy(m_trCoeff[2] + tmpC2, subCU.m_trCoeff[2], sizeof(coeff_t) * tmpC);
    }
}

/* If a sub-CU part is not present (off the edge of the picture) its depth and
 * log2size should still be configured */
void CUData::setEmptyPart(const CUGeom& childGeom, uint32_t subPartIdx)
{
    uint32_t offset = childGeom.numPartitions * subPartIdx;
    m_subPartSet(m_cuDepth + offset, (uint8_t)childGeom.depth);
    m_subPartSet(m_log2CUSize + offset, (uint8_t)childGeom.log2CUSize);
}

/* Copy all CU data from one instance to the next, except set lossless flag
 * This will only get used when --cu-lossless is enabled but --lossless is not. */
void CUData::initLosslessCU(const CUData& cu, const CUGeom& cuGeom)
{
    /* Start by making an exact copy */
    m_encData      = cu.m_encData;
    m_slice        = cu.m_slice;
    m_cuAddr       = cu.m_cuAddr;
    m_cuPelX       = cu.m_cuPelX;
    m_cuPelY       = cu.m_cuPelY;
    m_cuLeft       = cu.m_cuLeft;
    m_cuAbove      = cu.m_cuAbove;
    m_cuAboveLeft  = cu.m_cuAboveLeft;
    m_cuAboveRight = cu.m_cuAboveRight;
    m_absIdxInCTU  = cuGeom.absPartIdx;
    m_numPartitions = cuGeom.numPartitions;
    memcpy(m_qp, cu.m_qp, BytesPerPartition * m_numPartitions);
    memcpy(m_mv[0],  cu.m_mv[0],  m_numPartitions * sizeof(MV));
    memcpy(m_mv[1],  cu.m_mv[1],  m_numPartitions * sizeof(MV));
    memcpy(m_mvd[0], cu.m_mvd[0], m_numPartitions * sizeof(MV));
    memcpy(m_mvd[1], cu.m_mvd[1], m_numPartitions * sizeof(MV));
    memcpy(m_distortion, cu.m_distortion, m_numPartitions * sizeof(sse_t));

    /* force TQBypass to true */
    m_partSet(m_tqBypass, true);// lossless 下 跳过trans quant

    /* clear residual coding flags */
    m_partSet(m_predMode, cu.m_predMode[0] & (MODE_INTRA | MODE_INTER));
    m_partSet(m_tuDepth, 0);
    m_partSet(m_cbf[0], 0);
    m_partSet(m_transformSkip[0], 0);

    if (cu.m_chromaFormat != S265_CSP_I400)
    {
        m_partSet(m_chromaIntraDir, (uint8_t)ALL_IDX);
        m_partSet(m_cbf[1], 0);
        m_partSet(m_cbf[2], 0);
        m_partSet(m_transformSkip[1], 0);
        m_partSet(m_transformSkip[2], 0);
    }
}

/* Copy completed predicted CU to CTU in picture */
void CUData::copyToPic(uint32_t depth) const
{
    CUData& ctu = *m_encData->getPicCTU(m_cuAddr);

    m_partCopy((uint8_t*)ctu.m_qp + m_absIdxInCTU, (uint8_t*)m_qp);
    m_partCopy((uint8_t*)ctu.m_qpAnalysis + m_absIdxInCTU, (uint8_t*)m_qpAnalysis);
    m_partCopy(ctu.m_log2CUSize + m_absIdxInCTU, m_log2CUSize);
    m_partCopy(ctu.m_lumaIntraDir + m_absIdxInCTU, m_lumaIntraDir);
    m_partCopy(ctu.m_tqBypass + m_absIdxInCTU, m_tqBypass);
    m_partCopy((uint8_t*)ctu.m_refIdx[0] + m_absIdxInCTU, (uint8_t*)m_refIdx[0]);
    m_partCopy((uint8_t*)ctu.m_refIdx[1] + m_absIdxInCTU, (uint8_t*)m_refIdx[1]);
    m_partCopy(ctu.m_cuDepth + m_absIdxInCTU, m_cuDepth);
    m_partCopy(ctu.m_predMode + m_absIdxInCTU, m_predMode);
    m_partCopy(ctu.m_partSize + m_absIdxInCTU, m_partSize);
    m_partCopy(ctu.m_mergeFlag + m_absIdxInCTU, m_mergeFlag);
    m_partCopy(ctu.m_interDir + m_absIdxInCTU, m_interDir);
    m_partCopy(ctu.m_mvpIdx[0] + m_absIdxInCTU, m_mvpIdx[0]);
    m_partCopy(ctu.m_mvpIdx[1] + m_absIdxInCTU, m_mvpIdx[1]);
    m_partCopy(ctu.m_tuDepth + m_absIdxInCTU, m_tuDepth);
    m_partCopy(ctu.m_transformSkip[0] + m_absIdxInCTU, m_transformSkip[0]);
    m_partCopy(ctu.m_cbf[0] + m_absIdxInCTU, m_cbf[0]);

    memcpy(ctu.m_mv[0] + m_absIdxInCTU, m_mv[0], m_numPartitions * sizeof(MV));
    memcpy(ctu.m_mv[1] + m_absIdxInCTU, m_mv[1], m_numPartitions * sizeof(MV));
    memcpy(ctu.m_mvd[0] + m_absIdxInCTU, m_mvd[0], m_numPartitions * sizeof(MV));
    memcpy(ctu.m_mvd[1] + m_absIdxInCTU, m_mvd[1], m_numPartitions * sizeof(MV));

    memcpy(ctu.m_distortion + m_absIdxInCTU, m_distortion, m_numPartitions * sizeof(sse_t));

    uint32_t tmpY = 1 << ((m_slice->m_param->maxLog2CUSize - depth) * 2);
    uint32_t tmpY2 = m_absIdxInCTU << (LOG2_UNIT_SIZE * 2);
    memcpy(ctu.m_trCoeff[0] + tmpY2, m_trCoeff[0], sizeof(coeff_t)* tmpY);

    if (ctu.m_chromaFormat != S265_CSP_I400)
    {
        m_partCopy(ctu.m_transformSkip[1] + m_absIdxInCTU, m_transformSkip[1]);
        m_partCopy(ctu.m_transformSkip[2] + m_absIdxInCTU, m_transformSkip[2]);
        m_partCopy(ctu.m_cbf[1] + m_absIdxInCTU, m_cbf[1]);
        m_partCopy(ctu.m_cbf[2] + m_absIdxInCTU, m_cbf[2]);
        m_partCopy(ctu.m_chromaIntraDir + m_absIdxInCTU, m_chromaIntraDir);

        uint32_t tmpC = tmpY >> (m_hChromaShift + m_vChromaShift);
        uint32_t tmpC2 = tmpY2 >> (m_hChromaShift + m_vChromaShift);
        memcpy(ctu.m_trCoeff[1] + tmpC2, m_trCoeff[1], sizeof(coeff_t) * tmpC);
        memcpy(ctu.m_trCoeff[2] + tmpC2, m_trCoeff[2], sizeof(coeff_t) * tmpC);
    }
}

/* The reverse of copyToPic, called only by encodeResidue */
void CUData::copyFromPic(const CUData& ctu, const CUGeom& cuGeom, int csp, bool copyQp)
{
    m_encData       = ctu.m_encData;
    m_slice         = ctu.m_slice;
    m_cuAddr        = ctu.m_cuAddr;
    m_cuPelX        = ctu.m_cuPelX + g_zscanToPelX[cuGeom.absPartIdx];
    m_cuPelY        = ctu.m_cuPelY + g_zscanToPelY[cuGeom.absPartIdx];
    m_absIdxInCTU   = cuGeom.absPartIdx;
    m_numPartitions = cuGeom.numPartitions;

    /* copy out all prediction info for this part */
    if (copyQp)
    {
        m_partCopy((uint8_t*)m_qp, (uint8_t*)ctu.m_qp + m_absIdxInCTU);
        m_partCopy((uint8_t*)m_qpAnalysis, (uint8_t*)ctu.m_qpAnalysis + m_absIdxInCTU);
    }

    m_partCopy(m_log2CUSize,   ctu.m_log2CUSize + m_absIdxInCTU);
    m_partCopy(m_lumaIntraDir, ctu.m_lumaIntraDir + m_absIdxInCTU);
    m_partCopy(m_tqBypass,     ctu.m_tqBypass + m_absIdxInCTU);
    m_partCopy((uint8_t*)m_refIdx[0], (uint8_t*)ctu.m_refIdx[0] + m_absIdxInCTU);
    m_partCopy((uint8_t*)m_refIdx[1], (uint8_t*)ctu.m_refIdx[1] + m_absIdxInCTU);
    m_partCopy(m_cuDepth,      ctu.m_cuDepth + m_absIdxInCTU);
    m_partSet(m_predMode, ctu.m_predMode[m_absIdxInCTU] & (MODE_INTRA | MODE_INTER)); /* clear skip flag */
    m_partCopy(m_partSize,     ctu.m_partSize + m_absIdxInCTU);
    m_partCopy(m_mergeFlag,    ctu.m_mergeFlag + m_absIdxInCTU);
    m_partCopy(m_interDir,     ctu.m_interDir + m_absIdxInCTU);
    m_partCopy(m_mvpIdx[0],    ctu.m_mvpIdx[0] + m_absIdxInCTU);
    m_partCopy(m_mvpIdx[1],    ctu.m_mvpIdx[1] + m_absIdxInCTU);
    m_partCopy(m_chromaIntraDir, ctu.m_chromaIntraDir + m_absIdxInCTU);

    memcpy(m_mv[0], ctu.m_mv[0] + m_absIdxInCTU, m_numPartitions * sizeof(MV));
    memcpy(m_mv[1], ctu.m_mv[1] + m_absIdxInCTU, m_numPartitions * sizeof(MV));
    memcpy(m_mvd[0], ctu.m_mvd[0] + m_absIdxInCTU, m_numPartitions * sizeof(MV));
    memcpy(m_mvd[1], ctu.m_mvd[1] + m_absIdxInCTU, m_numPartitions * sizeof(MV));

    memcpy(m_distortion, ctu.m_distortion + m_absIdxInCTU, m_numPartitions * sizeof(sse_t));

    /* clear residual coding flags */
    m_partSet(m_tuDepth, 0);
    m_partSet(m_transformSkip[0], 0);
    m_partSet(m_cbf[0], 0);

    if (csp != S265_CSP_I400)
    {        
        m_partSet(m_transformSkip[1], 0);
        m_partSet(m_transformSkip[2], 0);
        m_partSet(m_cbf[1], 0);
        m_partSet(m_cbf[2], 0);
    }
}

/* Only called by encodeResidue, these fields can be modified during inter/intra coding */
void CUData::updatePic(uint32_t depth, int picCsp) const
{
    CUData& ctu = *m_encData->getPicCTU(m_cuAddr);

    m_partCopy((uint8_t*)ctu.m_qp + m_absIdxInCTU, (uint8_t*)m_qp);
    m_partCopy((uint8_t*)ctu.m_qpAnalysis + m_absIdxInCTU, (uint8_t*)m_qpAnalysis);
    m_partCopy(ctu.m_transformSkip[0] + m_absIdxInCTU, m_transformSkip[0]);
    m_partCopy(ctu.m_predMode + m_absIdxInCTU, m_predMode);
    m_partCopy(ctu.m_tuDepth + m_absIdxInCTU, m_tuDepth);
    m_partCopy(ctu.m_cbf[0] + m_absIdxInCTU, m_cbf[0]);

    uint32_t tmpY = 1 << ((m_slice->m_param->maxLog2CUSize - depth) * 2);
    uint32_t tmpY2 = m_absIdxInCTU << (LOG2_UNIT_SIZE * 2);
    memcpy(ctu.m_trCoeff[0] + tmpY2, m_trCoeff[0], sizeof(coeff_t)* tmpY);

    if (ctu.m_chromaFormat != S265_CSP_I400 && picCsp != S265_CSP_I400)
    {
        m_partCopy(ctu.m_transformSkip[1] + m_absIdxInCTU, m_transformSkip[1]);
        m_partCopy(ctu.m_transformSkip[2] + m_absIdxInCTU, m_transformSkip[2]);

        m_partCopy(ctu.m_cbf[1] + m_absIdxInCTU, m_cbf[1]);
        m_partCopy(ctu.m_cbf[2] + m_absIdxInCTU, m_cbf[2]);
        m_partCopy(ctu.m_chromaIntraDir + m_absIdxInCTU, m_chromaIntraDir);

        tmpY  >>= m_hChromaShift + m_vChromaShift;
        tmpY2 >>= m_hChromaShift + m_vChromaShift;
        memcpy(ctu.m_trCoeff[1] + tmpY2, m_trCoeff[1], sizeof(coeff_t) * tmpY);
        memcpy(ctu.m_trCoeff[2] + tmpY2, m_trCoeff[2], sizeof(coeff_t) * tmpY);
    }
}
// 用于找到 当前pu的左边的neighbor的位置（包含两部分，一部分是 neighbor 所在的ctu/cu，另一部分是 该neighbor4x4 在该ctu/cu中的4x4 偏移alPartUnitIdx）
//如果在 同一个cu 内 则返回this （cu）+ cu 内的偏移地址
//如果不在同一个cu 内 则返回对应的CTU + CTU 内的偏移地址
const CUData* CUData::getPULeft(uint32_t& lPartUnitIdx, uint32_t curPartUnitIdx) const
{
    uint32_t absPartIdx = g_zscanToRaster[curPartUnitIdx];//当前pu的4x4的位置

    if (!isZeroCol(absPartIdx))//如果不是ctu的0列
    {
        uint32_t absZorderCUIdx   = g_zscanToRaster[m_absIdxInCTU];//获取当前pu所在cu的首个4x4位置
        lPartUnitIdx = g_rasterToZscan[absPartIdx - 1];//当前pu首个4x4位置左移一个4x4
        if (isEqualCol(absPartIdx, absZorderCUIdx))//当前pu首个4x4的位置和其所在cu的首个4x4的位置为同一列
            return m_encData->getPicCTU(m_cuAddr);////这时他的Left pu 所在的cu 不是本身cu 但是属于同一个ctu,所以返回当前ctu 以及ctu内的偏移lPartUnitIdx
        else// 否则，如果不属于同一列，则当前4x4的leftpu 所在的cu和本身的cu属于同一cu，调整ctu内的偏移为当前cu内的偏移，然后返回当前cu的地址就好
        {
            lPartUnitIdx -= m_absIdxInCTU;
            // 这里lPartUnitIdx 开始表示的是相对CTU的索引 需要改为相对 this cu的索引所以需要减去this cu 在CTU中索引
            return this;
        }
    }
    //如果是ctu的第0列，调整ctu内的偏移地址为当前4x4地址所在ctu的最后一行，返回当前ctu的leftctu
    lPartUnitIdx = g_rasterToZscan[absPartIdx + s_numPartInCUSize - 1];
    return m_cuLeft;
}
// 用于找到 当前pu的上边的neighbor的位置（包含两部分，一部分是 neighbor 所在的ctu/cu，另一部分是 该neighbor4x4 在该ctu/cu中的4x4 偏移alPartUnitIdx）
const CUData* CUData::getPUAbove(uint32_t& aPartUnitIdx, uint32_t curPartUnitIdx) const
{
    uint32_t absPartIdx = g_zscanToRaster[curPartUnitIdx];//当前4x4的位置

    if (!isZeroRow(absPartIdx))//如果不是ctu的0行
    {
        uint32_t absZorderCUIdx = g_zscanToRaster[m_absIdxInCTU];//获取当前pu所在cu的首个4x4位置
        aPartUnitIdx = g_rasterToZscan[absPartIdx - RASTER_SIZE];//当前4x4位置上移一行
        if (isEqualRow(absPartIdx, absZorderCUIdx))//当前pu4x4的位置和其所在cu的首个4x4的位置为同一行
            return m_encData->getPicCTU(m_cuAddr);//这时他的above 所在的cu 不是本身cu 但是属于同一个ctu,所以返回当前ctu 以及ctu内的偏移
        else// 否则，如果不属于同一行，则当前4x4的above cu 和本身的cu属于同一cu，调整ctu内的偏移为当前cu内的偏移，然后返回当前cu的地址就好
            aPartUnitIdx -= m_absIdxInCTU;
        return this;
    }
    //如果是ctu的第0行，调整ctu内的偏移地址为当前4x4地址所在ctu的最后一行，返回当前ctu的above ctu
    aPartUnitIdx = g_rasterToZscan[absPartIdx + ((s_numPartInCUSize - 1) << LOG2_RASTER_SIZE)];
    return m_cuAbove;
}


// 用于找到 当前pu的左上角的neighbor的位置（包含两部分，一部分是 neighbor 所在的ctu，另一部分是 该neighbor4x4 在该ctu中的4x4 偏移alPartUnitIdx）
const CUData* CUData::getPUAboveLeft(uint32_t& alPartUnitIdx, uint32_t curPartUnitIdx) const
{
    uint32_t absPartIdx = g_zscanToRaster[curPartUnitIdx];//当前pu的 abspartIdx

    if (!isZeroCol(absPartIdx))//非ctu第0列4x4
    {
        if (!isZeroRow(absPartIdx))//非ctu第0行4x4
        {
            uint32_t absZorderCUIdx  = g_zscanToRaster[m_absIdxInCTU];//获取当前curPartUnitIdx 所在cu的首个4x4 abspartIdx
            alPartUnitIdx = g_rasterToZscan[absPartIdx - RASTER_SIZE - 1];//neighboer左上角4x4 位置的索引 为当前pu位置上移一行 左移一列
            if (isEqualRowOrCol(absPartIdx, absZorderCUIdx))// 如果当前pu的首个4x4位置和当前其所在cu的首个4x4是同一行或者同一列
                return m_encData->getPicCTU(m_cuAddr);//当前ctu 就为当前pu的 aboveleft ctu
            else
            {
                //当前cu 就为当前pu的 aboveleft cu, alPartUnitIdx 可以直接使用当前cu内的偏移就好,无需ctu内的偏移
                alPartUnitIdx -= m_absIdxInCTU;//ctu 偏移地址 改为当前cu内的偏移地址
                return this;// //当前cu 就为当前pu的 aboveleft ctu, 所以返回this 就好
            }
        }
        //  否则，如果当前pu的4x4 idx 位于该ctu的第0行但非第0列,当前pu的 aboveleft ctu 为当前ctu above ctu,
        //  设置 alPartUnitIdx 为 在当前pu的首个4x4 位置往下移动 15行，再往左移动一个4x4 位置
        alPartUnitIdx = g_rasterToZscan[absPartIdx + ((s_numPartInCUSize - 1) << LOG2_RASTER_SIZE) - 1];// 左下脚位置寻址
        return m_cuAbove; 
    }

    if (!isZeroRow(absPartIdx))
    {   // 如果 该pu的首个4x4位置 是该ctu的第0列 但非ctu的第0行位置,则当前pu的 aboveleft ctu 为当前ctu的left ctu, 并设置 alPartUnitIdx
        // 这里 absPartIdx - RASTER_SIZE + s_numPartInCUSize - 1 表示 上移一行 在往右移到最后一列位置的4x4 
        alPartUnitIdx = g_rasterToZscan[absPartIdx - RASTER_SIZE + s_numPartInCUSize - 1];
        return m_cuLeft;
    }

    //否则 该pu的首个4x4位置 是该ctu的第0列同时也是第0行位置,返回alpartUnitIdx为 255位置的 idx（ctu的最后一个4x4位置）
    alPartUnitIdx = m_encData->m_param->num4x4Partitions - 1;
    return m_cuAboveLeft;// 返回  则当前pu的 aboveleft ctu 为当前ctu的above left ctu
}
// 用于找到 当前pu的右上角的neighbor的位置（包含两部分，一部分是 neighbor 所在的ctu，另一部分是 该neighbor4x4 在该ctu中的4x4 偏移alPartUnitIdx）
const CUData* CUData::getPUAboveRight(uint32_t& arPartUnitIdx, uint32_t curPartUnitIdx) const
{
    if ((m_encData->getPicCTU(m_cuAddr)->m_cuPelX + g_zscanToPelX[curPartUnitIdx] + UNIT_SIZE) >= m_slice->m_sps->picWidthInLumaSamples)
        return NULL;//右边的4x4超出图片边界了

    uint32_t absPartIdxRT = g_zscanToRaster[curPartUnitIdx];//当前pu的 abspartIdx

    if (lessThanCol(absPartIdxRT, s_numPartInCUSize - 1))//非最右侧ctu中的一列4x4
    {
        if (!isZeroRow(absPartIdxRT))//不为当前ctu中的第0行4x4
        {   // 这里在zigzag顺序上, 当前pu的4x4 位置的顺序要大于右上角位置的zigzag顺序, 则表示右上角可用
            if (curPartUnitIdx > g_rasterToZscan[absPartIdxRT - RASTER_SIZE + 1])
            {
                //get当前cu的内部的右上角4x4的索引
                uint32_t absZorderCUIdx = g_zscanToRaster[m_absIdxInCTU] + (1 << (m_log2CUSize[0] - LOG2_UNIT_SIZE)) - 1;
                arPartUnitIdx = g_rasterToZscan[absPartIdxRT - RASTER_SIZE + 1];// 设置返回值 为上移一行 右移一列
                if (isEqualRowOrCol(absPartIdxRT, absZorderCUIdx))//
                    return m_encData->getPicCTU(m_cuAddr);
                else
                {
                    arPartUnitIdx -= m_absIdxInCTU;
                    return this;
                }
            }// 否则上顺序上右上角滞后，不可用
            return NULL;
        }
        arPartUnitIdx = g_rasterToZscan[absPartIdxRT + ((s_numPartInCUSize - 1) << LOG2_RASTER_SIZE) + 1];
        return m_cuAbove;
    }

    if (!isZeroRow(absPartIdxRT))
        return NULL;

    arPartUnitIdx = g_rasterToZscan[(s_numPartInCUSize - 1) << LOG2_RASTER_SIZE];
    return m_cuAboveRight;
}

const CUData* CUData::getPUBelowLeft(uint32_t& blPartUnitIdx, uint32_t curPartUnitIdx) const
{
    if ((m_encData->getPicCTU(m_cuAddr)->m_cuPelY + g_zscanToPelY[curPartUnitIdx] + UNIT_SIZE) >= m_slice->m_sps->picHeightInLumaSamples)
        return NULL;

    uint32_t absPartIdxLB = g_zscanToRaster[curPartUnitIdx];

    if (lessThanRow(absPartIdxLB, s_numPartInCUSize - 1))
    {
        if (!isZeroCol(absPartIdxLB))
        {
            if (curPartUnitIdx > g_rasterToZscan[absPartIdxLB + RASTER_SIZE - 1])
            {
                uint32_t absZorderCUIdxLB = g_zscanToRaster[m_absIdxInCTU] + (((1 << (m_log2CUSize[0] - LOG2_UNIT_SIZE)) - 1) << LOG2_RASTER_SIZE);
                blPartUnitIdx = g_rasterToZscan[absPartIdxLB + RASTER_SIZE - 1];
                if (isEqualRowOrCol(absPartIdxLB, absZorderCUIdxLB))
                    return m_encData->getPicCTU(m_cuAddr);
                else
                {
                    blPartUnitIdx -= m_absIdxInCTU;
                    return this;
                }
            }
            return NULL;
        }
        blPartUnitIdx = g_rasterToZscan[absPartIdxLB + RASTER_SIZE + s_numPartInCUSize - 1];
        return m_cuLeft;
    }

    return NULL;
}

const CUData* CUData::getPUBelowLeftAdi(uint32_t& blPartUnitIdx,  uint32_t curPartUnitIdx, uint32_t partUnitOffset) const
{
    if ((m_encData->getPicCTU(m_cuAddr)->m_cuPelY + g_zscanToPelY[curPartUnitIdx] + (partUnitOffset << LOG2_UNIT_SIZE)) >= m_slice->m_sps->picHeightInLumaSamples)
        return NULL;

    uint32_t absPartIdxLB = g_zscanToRaster[curPartUnitIdx];

    if (lessThanRow(absPartIdxLB, s_numPartInCUSize - partUnitOffset))
    {
        if (!isZeroCol(absPartIdxLB))
        {
            if (curPartUnitIdx > g_rasterToZscan[absPartIdxLB + (partUnitOffset << LOG2_RASTER_SIZE) - 1])
            {
                uint32_t absZorderCUIdxLB = g_zscanToRaster[m_absIdxInCTU] + (((1 << (m_log2CUSize[0] - LOG2_UNIT_SIZE)) - 1) << LOG2_RASTER_SIZE);
                blPartUnitIdx = g_rasterToZscan[absPartIdxLB + (partUnitOffset << LOG2_RASTER_SIZE) - 1];
                if (isEqualRowOrCol(absPartIdxLB, absZorderCUIdxLB))
                    return m_encData->getPicCTU(m_cuAddr);
                else
                {
                    blPartUnitIdx -= m_absIdxInCTU;
                    return this;
                }
            }
            return NULL;
        }
        blPartUnitIdx = g_rasterToZscan[absPartIdxLB + (partUnitOffset << LOG2_RASTER_SIZE) + s_numPartInCUSize - 1];
        return m_cuLeft;
    }

    return NULL;
}
// 用于找到 当前pu的右上角4x4的above right neighbors的cus（包含两部分，一部分是 neighbor 所在的ctu/cu，另一部分是 该neighbor4x4 在该ctu/cu中的4x4 偏移alPartUnitIdx）
const CUData* CUData::getPUAboveRightAdi(uint32_t& arPartUnitIdx, uint32_t curPartUnitIdx, uint32_t partUnitOffset) const
{
    if ((m_encData->getPicCTU(m_cuAddr)->m_cuPelX + g_zscanToPelX[curPartUnitIdx] + (partUnitOffset << LOG2_UNIT_SIZE)) >= m_slice->m_sps->picWidthInLumaSamples)
        return NULL;//右移了partUnitOffset右的4x4超出图片边界了

    uint32_t absPartIdxRT = g_zscanToRaster[curPartUnitIdx];//当前pu的自身右上角的4x4的abspartIdx

    if (lessThanCol(absPartIdxRT, s_numPartInCUSize - partUnitOffset))// 如果 absPartIdxRT+partUnitOffset < s_numPartInCUSize(16）则aboveright没有位于右侧的ctu
    {
        if (!isZeroRow(absPartIdxRT))//如果不是第0行
        {   //如果 在顺序上当前4x4 （curPartUnitIdx) 的索引顺序比右上角的第partUnitOffset个4x4的顺序要大，则该右上角存在
            if (curPartUnitIdx > g_rasterToZscan[absPartIdxRT - RASTER_SIZE + partUnitOffset])
            {   //调整返回的右上角4x4的位置偏移为当前ctu内该当前cu里面该pu内右上角位置4x4的位置
                uint32_t absZorderCUIdx = g_zscanToRaster[m_absIdxInCTU] + (1 << (m_log2CUSize[0] - LOG2_UNIT_SIZE)) - 1;// 获取当前pu所在cu内的右上角位置的4x4 索引
                arPartUnitIdx = g_rasterToZscan[absPartIdxRT - RASTER_SIZE + partUnitOffset];//上移一行 右移partUnitOffset
                //？？？ 这里似乎 只需要判断 isEqualRow 就可以
                if (isEqualRowOrCol(absPartIdxRT, absZorderCUIdx))//如果当前pu的右上角4x4和其所在cu的右上角4x4 属于同一行or列
                    return m_encData->getPicCTU(m_cuAddr); // 返回当前ctu 和对应右上角4x4的偏移地址
                else
                {
                    arPartUnitIdx -= m_absIdxInCTU;//否则位于同一个cu内，ctu内偏移地址转换为cu内的偏移地址 返回this 指针
                    return this;
                }
            }
            //否则，顺序上小于右上角4x4的顺讯，该右上角不可用
            return NULL;
        }
        //如果是第0行，则当前4x4的右上角偏移了offset的4x4位于当前ctu的abovectu中的最后一行相同clo 在偏移partUnitOffset 的位置
        arPartUnitIdx = g_rasterToZscan[absPartIdxRT + ((s_numPartInCUSize - 1) << LOG2_RASTER_SIZE) + partUnitOffset];
        return m_cuAbove;
    }
    //否则当前右上角4x4位于右侧的ctu
    if (!isZeroRow(absPartIdxRT))
        return NULL;//如果当前4x4不是当前ctu的第0行，则其右上角的4x4 位于右侧的ctu 还没有重构/被编码 不可用
    // 否则当前pu的右上角4x4为当前ctu的第0行,则其右上角的第offset个4x4位于已经编码了的右上角ctu里面的最后一行偏移了partUnitOffset的位置
    arPartUnitIdx = g_rasterToZscan[((s_numPartInCUSize - 1) << LOG2_RASTER_SIZE) + partUnitOffset - 1];
    return m_cuAboveRight;
}

/* Get left QpMinCu */
const CUData* CUData::getQpMinCuLeft(uint32_t& lPartUnitIdx, uint32_t curAbsIdxInCTU) const
{
    uint32_t absZorderQpMinCUIdx = curAbsIdxInCTU & (0xFF << (m_encData->m_param->unitSizeDepth - m_slice->m_pps->maxCuDQPDepth) * 2);
    uint32_t absRorderQpMinCUIdx = g_zscanToRaster[absZorderQpMinCUIdx];

    // check for left CTU boundary
    if (isZeroCol(absRorderQpMinCUIdx))
        return NULL;

    // get index of left-CU relative to top-left corner of current quantization group
    lPartUnitIdx = g_rasterToZscan[absRorderQpMinCUIdx - 1];

    // return pointer to current CTU
    return m_encData->getPicCTU(m_cuAddr);
}

/* Get above QpMinCu */
const CUData* CUData::getQpMinCuAbove(uint32_t& aPartUnitIdx, uint32_t curAbsIdxInCTU) const
{
    uint32_t absZorderQpMinCUIdx = curAbsIdxInCTU & (0xFF << (m_encData->m_param->unitSizeDepth - m_slice->m_pps->maxCuDQPDepth) * 2);
    uint32_t absRorderQpMinCUIdx = g_zscanToRaster[absZorderQpMinCUIdx];

    // check for top CTU boundary
    if (isZeroRow(absRorderQpMinCUIdx))// ctu 上边界时，above qp 不可用
        return NULL;

    // get index of top-CU relative to top-left corner of current quantization group
    aPartUnitIdx = g_rasterToZscan[absRorderQpMinCUIdx - RASTER_SIZE];

    // return pointer to current CTU
    return m_encData->getPicCTU(m_cuAddr);
}

/* Get reference QP from left QpMinCu or latest coded QP */
/*
curAbsIdxInCTU 表示 cu 相对于 ctu根节点的相对位置 以4x4 为单位
m_absIdxInCTU 
 */
int8_t CUData::getRefQP(uint32_t curAbsIdxInCTU) const
{
    uint32_t lPartIdx = 0, aPartIdx = 0;
    const CUData* cULeft = getQpMinCuLeft(lPartIdx, m_absIdxInCTU + curAbsIdxInCTU);
    const CUData* cUAbove = getQpMinCuAbove(aPartIdx, m_absIdxInCTU + curAbsIdxInCTU);

    return ((cULeft ? cULeft->m_qp[lPartIdx] : getLastCodedQP(curAbsIdxInCTU)) + (cUAbove ? cUAbove->m_qp[aPartIdx] : getLastCodedQP(curAbsIdxInCTU)) + 1) >> 1;
}

int CUData::getLastValidPartIdx(int absPartIdx) const
{
    int lastValidPartIdx = absPartIdx - 1;

    while (lastValidPartIdx >= 0 && m_predMode[lastValidPartIdx] == MODE_NONE)
    {
        uint32_t depth = m_cuDepth[lastValidPartIdx];
        lastValidPartIdx -= m_numPartitions >> (depth << 1);
    }

    return lastValidPartIdx;
}

int8_t CUData::getLastCodedQP(uint32_t absPartIdx) const
{
    uint32_t quPartIdxMask = 0xFF << (m_encData->m_param->unitSizeDepth - m_slice->m_pps->maxCuDQPDepth) * 2;
    int lastValidPartIdx = getLastValidPartIdx(absPartIdx & quPartIdxMask);

    if (lastValidPartIdx >= 0)
        return m_qp[lastValidPartIdx];
    else
    {
        if (m_absIdxInCTU)
            return m_encData->getPicCTU(m_cuAddr)->getLastCodedQP(m_absIdxInCTU);
        else if (m_cuAddr > 0 && !(m_slice->m_pps->bEntropyCodingSyncEnabled && !(m_cuAddr % m_slice->m_sps->numCuInWidth)))
            return m_encData->getPicCTU(m_cuAddr - 1)->getLastCodedQP(m_encData->m_param->num4x4Partitions);
        else
            return (int8_t)m_slice->m_sliceQp;
    }
}

/* Get allowed chroma intra modes */
void CUData::getAllowedChromaDir(uint32_t absPartIdx, uint32_t* modeList) const
{
    modeList[0] = PLANAR_IDX;
    modeList[1] = VER_IDX;
    modeList[2] = HOR_IDX;
    modeList[3] = DC_IDX;
    modeList[4] = DM_CHROMA_IDX;

    uint32_t lumaMode = m_lumaIntraDir[absPartIdx];

    for (int i = 0; i < NUM_CHROMA_MODE - 1; i++)
    {
        if (lumaMode == modeList[i])
        {
            modeList[i] = 34; // VER+8 mode
            break;
        }
    }
}

/* Get most probable intra modes */
int CUData::getIntraDirLumaPredictor(uint32_t absPartIdx, uint32_t* intraDirPred) const
{
    const CUData* tempCU;
    uint32_t tempPartIdx;
    uint32_t leftIntraDir, aboveIntraDir;

    // Get intra direction of left PU
    tempCU = getPULeft(tempPartIdx, m_absIdxInCTU + absPartIdx);

    leftIntraDir = (tempCU && tempCU->isIntra(tempPartIdx)) ? tempCU->m_lumaIntraDir[tempPartIdx] : DC_IDX;

    // Get intra direction of above PU
    tempCU = g_zscanToPelY[m_absIdxInCTU + absPartIdx] > 0 ? getPUAbove(tempPartIdx, m_absIdxInCTU + absPartIdx) : NULL;

    aboveIntraDir = (tempCU && tempCU->isIntra(tempPartIdx)) ? tempCU->m_lumaIntraDir[tempPartIdx] : DC_IDX;

    if (leftIntraDir == aboveIntraDir)
    {
        if (leftIntraDir >= 2) // angular modes
        {
            intraDirPred[0] = leftIntraDir;
            intraDirPred[1] = ((leftIntraDir - 2 + 31) & 31) + 2;
            intraDirPred[2] = ((leftIntraDir - 2 +  1) & 31) + 2;
        }
        else //non-angular
        {
            intraDirPred[0] = PLANAR_IDX;
            intraDirPred[1] = DC_IDX;
            intraDirPred[2] = VER_IDX;
        }
        return 1;
    }
    else
    {
        intraDirPred[0] = leftIntraDir;
        intraDirPred[1] = aboveIntraDir;

        if (leftIntraDir && aboveIntraDir) //both modes are non-planar
            intraDirPred[2] = PLANAR_IDX;
        else
            intraDirPred[2] =  (leftIntraDir + aboveIntraDir) < 2 ? VER_IDX : DC_IDX;
        return 2;
    }
}

uint32_t CUData::getCtxSplitFlag(uint32_t absPartIdx, uint32_t depth) const
{
    const CUData* tempCU;
    uint32_t    tempPartIdx;
    uint32_t    ctx;

    // Get left split flag
    tempCU = getPULeft(tempPartIdx, m_absIdxInCTU + absPartIdx);
    ctx  = (tempCU) ? ((tempCU->m_cuDepth[tempPartIdx] > depth) ? 1 : 0) : 0;

    // Get above split flag
    tempCU = getPUAbove(tempPartIdx, m_absIdxInCTU + absPartIdx);
    ctx += (tempCU) ? ((tempCU->m_cuDepth[tempPartIdx] > depth) ? 1 : 0) : 0;

    return ctx;
}
// intra 下 tu 深度控制
// 注意只有在 intra 8x8 cu 下 当 partSize 为 SIZE_NxN 时，tu 被隐式分割成 4x4
void CUData::getIntraTUQtDepthRange(uint32_t tuDepthRange[2], uint32_t absPartIdx) const
{
    uint32_t log2CUSize = m_log2CUSize[absPartIdx];//当前cu size
    uint32_t splitFlag = m_partSize[absPartIdx] != SIZE_2Nx2N; //只有在intra cu 为最小的cu时(通过8x8)时，才可以不等于SIZE_2Nx2N,此时splitFlag为1 tu 被隐式分割 ）

    tuDepthRange[0] = m_slice->m_sps->quadtreeTULog2MinSize;// minTu size 2 4x4
    tuDepthRange[1] = m_slice->m_sps->quadtreeTULog2MaxSize;// MaxTu size default 5 32x32
    // 通过TU深度控制参数 设置 minTu size为 为当前cusize在 PartSize 划分的情况下的最小tu size 
    // slower 下参数书为 3
    tuDepthRange[0] = s265_clip3(tuDepthRange[0], tuDepthRange[1], log2CUSize - (m_slice->m_sps->quadtreeTUMaxDepthIntra - 1 + splitFlag));
}
//inter 下  tu 深度控制
void CUData::getInterTUQtDepthRange(uint32_t tuDepthRange[2], uint32_t absPartIdx) const
{
    uint32_t log2CUSize = m_log2CUSize[absPartIdx];//当前cu size
    uint32_t quadtreeTUMaxDepth = m_slice->m_sps->quadtreeTUMaxDepthInter;//参数控制的inter tu的最大深度
    uint32_t splitFlag = quadtreeTUMaxDepth == 1 && m_partSize[absPartIdx] != SIZE_2Nx2N;// 对于inter cu 如果不允许tu划分 则只在SIZE_2Nx2N 下菜不允许,否则允许划分一次

    tuDepthRange[0] = m_slice->m_sps->quadtreeTULog2MinSize;
    tuDepthRange[1] = m_slice->m_sps->quadtreeTULog2MaxSize;

    tuDepthRange[0] = s265_clip3(tuDepthRange[0], tuDepthRange[1], log2CUSize - (quadtreeTUMaxDepth - 1 + splitFlag));
}

uint32_t CUData::getCtxSkipFlag(uint32_t absPartIdx) const
{
    const CUData* tempCU;
    uint32_t tempPartIdx;
    uint32_t ctx;

    // Get BCBP of left PU
    tempCU = getPULeft(tempPartIdx, m_absIdxInCTU + absPartIdx);
    ctx    = tempCU ? tempCU->isSkipped(tempPartIdx) : 0;

    // Get BCBP of above PU
    tempCU = getPUAbove(tempPartIdx, m_absIdxInCTU + absPartIdx);
    ctx   += tempCU ? tempCU->isSkipped(tempPartIdx) : 0;

    return ctx;
}

bool CUData::setQPSubCUs(int8_t qp, uint32_t absPartIdx, uint32_t depth)
{
    uint32_t curPartNumb = m_encData->m_param->num4x4Partitions >> (depth << 1);
    uint32_t curPartNumQ = curPartNumb >> 2;

    if (m_cuDepth[absPartIdx] > depth)
    {
        for (uint32_t subPartIdx = 0; subPartIdx < 4; subPartIdx++)
            if (setQPSubCUs(qp, absPartIdx + subPartIdx * curPartNumQ, depth + 1))
                return true;
    }
    else
    {
        if (getQtRootCbf(absPartIdx))
            return true;
        else
            setQPSubParts(qp, absPartIdx, depth);
    }

    return false;
}

void CUData::setPUInterDir(uint8_t dir, uint32_t absPartIdx, uint32_t puIdx)
{
    uint32_t curPartNumQ = m_numPartitions >> 2;
    S265_CHECK(puIdx < 2, "unexpected part unit index\n");

    switch (m_partSize[absPartIdx])
    {
    case SIZE_2Nx2N:
        memset(m_interDir + absPartIdx, dir, 4 * curPartNumQ);
        break;
    case SIZE_2NxN:
        memset(m_interDir + absPartIdx, dir, 2 * curPartNumQ);
        break;
    case SIZE_Nx2N:
        memset(m_interDir + absPartIdx, dir, curPartNumQ);
        memset(m_interDir + absPartIdx + 2 * curPartNumQ, dir, curPartNumQ);
        break;
    case SIZE_NxN:
        memset(m_interDir + absPartIdx, dir, curPartNumQ);
        break;
    case SIZE_2NxnU:
        if (!puIdx)
        {
            memset(m_interDir + absPartIdx, dir, (curPartNumQ >> 1));
            memset(m_interDir + absPartIdx + curPartNumQ, dir, (curPartNumQ >> 1));
        }
        else
        {
            memset(m_interDir + absPartIdx, dir, (curPartNumQ >> 1));
            memset(m_interDir + absPartIdx + curPartNumQ, dir, ((curPartNumQ >> 1) + (curPartNumQ << 1)));
        }
        break;
    case SIZE_2NxnD:
        if (!puIdx)
        {
            memset(m_interDir + absPartIdx, dir, ((curPartNumQ << 1) + (curPartNumQ >> 1)));
            memset(m_interDir + absPartIdx + (curPartNumQ << 1) + curPartNumQ, dir, (curPartNumQ >> 1));
        }
        else
        {
            memset(m_interDir + absPartIdx, dir, (curPartNumQ >> 1));
            memset(m_interDir + absPartIdx + curPartNumQ, dir, (curPartNumQ >> 1));
        }
        break;
    case SIZE_nLx2N:
        if (!puIdx)
        {
            memset(m_interDir + absPartIdx, dir, (curPartNumQ >> 2));
            memset(m_interDir + absPartIdx + (curPartNumQ >> 1), dir, (curPartNumQ >> 2));
            memset(m_interDir + absPartIdx + (curPartNumQ << 1), dir, (curPartNumQ >> 2));
            memset(m_interDir + absPartIdx + (curPartNumQ << 1) + (curPartNumQ >> 1), dir, (curPartNumQ >> 2));
        }
        else
        {
            memset(m_interDir + absPartIdx, dir, (curPartNumQ >> 2));
            memset(m_interDir + absPartIdx + (curPartNumQ >> 1), dir, (curPartNumQ + (curPartNumQ >> 2)));
            memset(m_interDir + absPartIdx + (curPartNumQ << 1), dir, (curPartNumQ >> 2));
            memset(m_interDir + absPartIdx + (curPartNumQ << 1) + (curPartNumQ >> 1), dir, (curPartNumQ + (curPartNumQ >> 2)));
        }
        break;
    case SIZE_nRx2N:
        if (!puIdx)
        {
            memset(m_interDir + absPartIdx, dir, (curPartNumQ + (curPartNumQ >> 2)));
            memset(m_interDir + absPartIdx + curPartNumQ + (curPartNumQ >> 1), dir, (curPartNumQ >> 2));
            memset(m_interDir + absPartIdx + (curPartNumQ << 1), dir, (curPartNumQ + (curPartNumQ >> 2)));
            memset(m_interDir + absPartIdx + (curPartNumQ << 1) + curPartNumQ + (curPartNumQ >> 1), dir, (curPartNumQ >> 2));
        }
        else
        {
            memset(m_interDir + absPartIdx, dir, (curPartNumQ >> 2));
            memset(m_interDir + absPartIdx + (curPartNumQ >> 1), dir, (curPartNumQ >> 2));
            memset(m_interDir + absPartIdx + (curPartNumQ << 1), dir, (curPartNumQ >> 2));
            memset(m_interDir + absPartIdx + (curPartNumQ << 1) + (curPartNumQ >> 1), dir, (curPartNumQ >> 2));
        }
        break;
    default:
        S265_CHECK(0, "unexpected part type\n");
        break;
    }
}

template<typename T>
void CUData::setAllPU(T* p, const T& val, int absPartIdx, int puIdx)
{
    int i;

    p += absPartIdx;
    int numElements = m_numPartitions;

    switch (m_partSize[absPartIdx])
    {
    case SIZE_2Nx2N:
        for (i = 0; i < numElements; i++)
            p[i] = val;
        break;

    case SIZE_2NxN:
        numElements >>= 1;
        for (i = 0; i < numElements; i++)
            p[i] = val;
        break;

    case SIZE_Nx2N:
        numElements >>= 2;
        for (i = 0; i < numElements; i++)
        {
            p[i] = val;
            p[i + 2 * numElements] = val;
        }
        break;

    case SIZE_2NxnU:
    {
        int curPartNumQ = numElements >> 2;
        if (!puIdx)
        {
            T *pT  = p;
            T *pT2 = p + curPartNumQ;
            for (i = 0; i < (curPartNumQ >> 1); i++)
            {
                pT[i] = val;
                pT2[i] = val;
            }
        }
        else
        {
            T *pT  = p;
            for (i = 0; i < (curPartNumQ >> 1); i++)
                pT[i] = val;

            pT = p + curPartNumQ;
            for (i = 0; i < ((curPartNumQ >> 1) + (curPartNumQ << 1)); i++)
                pT[i] = val;
        }
        break;
    }

    case SIZE_2NxnD:
    {
        int curPartNumQ = numElements >> 2;
        if (!puIdx)
        {
            T *pT  = p;
            for (i = 0; i < ((curPartNumQ >> 1) + (curPartNumQ << 1)); i++)
                pT[i] = val;

            pT = p + (numElements - curPartNumQ);
            for (i = 0; i < (curPartNumQ >> 1); i++)
                pT[i] = val;
        }
        else
        {
            T *pT  = p;
            T *pT2 = p + curPartNumQ;
            for (i = 0; i < (curPartNumQ >> 1); i++)
            {
                pT[i] = val;
                pT2[i] = val;
            }
        }
        break;
    }

    case SIZE_nLx2N:
    {
        int curPartNumQ = numElements >> 2;
        if (!puIdx)
        {
            T *pT  = p;
            T *pT2 = p + (curPartNumQ << 1);
            T *pT3 = p + (curPartNumQ >> 1);
            T *pT4 = p + (curPartNumQ << 1) + (curPartNumQ >> 1);

            for (i = 0; i < (curPartNumQ >> 2); i++)
            {
                pT[i] = val;
                pT2[i] = val;
                pT3[i] = val;
                pT4[i] = val;
            }
        }
        else
        {
            T *pT  = p;
            T *pT2 = p + (curPartNumQ << 1);
            for (i = 0; i < (curPartNumQ >> 2); i++)
            {
                pT[i] = val;
                pT2[i] = val;
            }

            pT  = p + (curPartNumQ >> 1);
            pT2 = p + (curPartNumQ << 1) + (curPartNumQ >> 1);
            for (i = 0; i < ((curPartNumQ >> 2) + curPartNumQ); i++)
            {
                pT[i] = val;
                pT2[i] = val;
            }
        }
        break;
    }

    case SIZE_nRx2N:
    {
        int curPartNumQ = numElements >> 2;
        if (!puIdx)
        {
            T *pT  = p;
            T *pT2 = p + (curPartNumQ << 1);
            for (i = 0; i < ((curPartNumQ >> 2) + curPartNumQ); i++)
            {
                pT[i] = val;
                pT2[i] = val;
            }

            pT  = p + curPartNumQ + (curPartNumQ >> 1);
            pT2 = p + numElements - curPartNumQ + (curPartNumQ >> 1);
            for (i = 0; i < (curPartNumQ >> 2); i++)
            {
                pT[i] = val;
                pT2[i] = val;
            }
        }
        else
        {
            T *pT  = p;
            T *pT2 = p + (curPartNumQ >> 1);
            T *pT3 = p + (curPartNumQ << 1);
            T *pT4 = p + (curPartNumQ << 1) + (curPartNumQ >> 1);
            for (i = 0; i < (curPartNumQ >> 2); i++)
            {
                pT[i] = val;
                pT2[i] = val;
                pT3[i] = val;
                pT4[i] = val;
            }
        }
        break;
    }

    case SIZE_NxN:
    default:
        S265_CHECK(0, "unknown partition type\n");
        break;
    }
}

void CUData::setPUMv(int list, const MV& mv, int absPartIdx, int puIdx)
{
    setAllPU(m_mv[list], mv, absPartIdx, puIdx);
}

void CUData::setPURefIdx(int list, int8_t refIdx, int absPartIdx, int puIdx)
{
    setAllPU(m_refIdx[list], refIdx, absPartIdx, puIdx);
}

void CUData::getPartIndexAndSize(uint32_t partIdx, uint32_t& outPartAddr, int& outWidth, int& outHeight) const
{
    int cuSize = 1 << m_log2CUSize[0];// cuSize pixl 单位
    int partType = m_partSize[0];

    int tmp = partTable[partType][partIdx][0];// 该partType 下第 partIdx [0]表示size 高4bit为x_size 低4bit为y_size 
    outWidth = ((tmp >> 4) * cuSize) >> 2;// pu_width pixl 单位
    outHeight = ((tmp & 0xF) * cuSize) >> 2;// pu_height tmp 的低4bit * 1/4的cuSzie pixl 单位
    //将一个cu 的 num_partions 分成了16份,根据partitionType 计算第 partIdx 个 pu 的地址（相对当前cu 的 4x4 offset)
    outPartAddr = (partAddrTable[partType][partIdx] * m_numPartitions) >> 4;
}

void CUData::getMvField(const CUData* cu, uint32_t absPartIdx, int picList, MVField& outMvField) const
{
    if (cu)
    {
        outMvField.mv = cu->m_mv[picList][absPartIdx];
        outMvField.refIdx = cu->m_refIdx[picList][absPartIdx];
    }
    else
    {
        // OUT OF BOUNDARY
        outMvField.mv = 0;
        outMvField.refIdx = REF_NOT_VALID;
    }
}

void CUData::deriveLeftRightTopIdx(uint32_t partIdx, uint32_t& partIdxLT, uint32_t& partIdxRT) const
{
    partIdxLT = m_absIdxInCTU;
    partIdxRT = g_rasterToZscan[g_zscanToRaster[partIdxLT] + (1 << (m_log2CUSize[0] - LOG2_UNIT_SIZE)) - 1];

    switch (m_partSize[0])
    {
    case SIZE_2Nx2N: break;
    case SIZE_2NxN:
        partIdxLT += (partIdx == 0) ? 0 : m_numPartitions >> 1;
        partIdxRT += (partIdx == 0) ? 0 : m_numPartitions >> 1;
        break;
    case SIZE_Nx2N:
        partIdxLT += (partIdx == 0) ? 0 : m_numPartitions >> 2;
        partIdxRT -= (partIdx == 1) ? 0 : m_numPartitions >> 2;
        break;
    case SIZE_NxN:
        partIdxLT += (m_numPartitions >> 2) * partIdx;
        partIdxRT +=  (m_numPartitions >> 2) * (partIdx - 1);
        break;
    case SIZE_2NxnU:
        partIdxLT += (partIdx == 0) ? 0 : m_numPartitions >> 3;
        partIdxRT += (partIdx == 0) ? 0 : m_numPartitions >> 3;
        break;
    case SIZE_2NxnD:
        partIdxLT += (partIdx == 0) ? 0 : (m_numPartitions >> 1) + (m_numPartitions >> 3);
        partIdxRT += (partIdx == 0) ? 0 : (m_numPartitions >> 1) + (m_numPartitions >> 3);
        break;
    case SIZE_nLx2N:
        partIdxLT += (partIdx == 0) ? 0 : m_numPartitions >> 4;
        partIdxRT -= (partIdx == 1) ? 0 : (m_numPartitions >> 2) + (m_numPartitions >> 4);
        break;
    case SIZE_nRx2N:
        partIdxLT += (partIdx == 0) ? 0 : (m_numPartitions >> 2) + (m_numPartitions >> 4);
        partIdxRT -= (partIdx == 1) ? 0 : m_numPartitions >> 4;
        break;
    default:
        S265_CHECK(0, "unexpected part index\n");
        break;
    }
}
// 找到当前cu中第puIdx 个pu内部的左下角的4x4的idx 在当前cTU中 的索引
uint32_t CUData::deriveLeftBottomIdx(uint32_t puIdx) const
{
    uint32_t outPartIdxLB; 
    //对于8x8 cu: base 偏移 0*16 （注意 16 为一个ctu内4x4的 raster stride 即: 1 << LOG2_RASTER_SIZE）--> outPartIdxLB = 0
    //对于16x16 cu: base 编译 1*16 --> outPartIdxLB = 2
    //对于32x32 cu: base 偏移 3*16 --> outPartIdxLB = 10
    //对于64x64 cu: base 偏移 7*16  == ( (1<< m_log2CUSize[0])/4/2 - 1)*16 --> outPartIdxLB = 42
    // 计算在当前 cu起始地址m_absIdxInCTU 上的base上的偏移 （在raster 顺序上偏移几个stride）然后在转为 zscan

    outPartIdxLB = g_rasterToZscan[g_zscanToRaster[m_absIdxInCTU] + (((1 << (m_log2CUSize[0] - LOG2_UNIT_SIZE - 1)) - 1) << LOG2_RASTER_SIZE)];

    switch (m_partSize[0])// 在cu 内部偏移
    {
    case SIZE_2Nx2N:
        outPartIdxLB += m_numPartitions >> 1;// for 16x16： + 8，32x32: +32， 64x64: +128 
        break;
    case SIZE_2NxN:
        outPartIdxLB += puIdx ? m_numPartitions >> 1 : 0; //for puIdx 0 + 0;  puIdx 1 for 16x16： + 2*16，32x32: + 4*16， 64x64: + 8*16 
        break;
    case SIZE_Nx2N:
        // 第0个pu +对应cu一半的m_numPartitions  第1个pu +  3/4 的m_numPartitions
        outPartIdxLB += puIdx ? (m_numPartitions >> 2) * 3 : m_numPartitions >> 1;
        break;
    case SIZE_NxN:
         // 第puIdx个pu +对应cu的m_numPartitions的 1/4 * puIdx 
        outPartIdxLB += (m_numPartitions >> 2) * puIdx;
        break;
    case SIZE_2NxnU:
        //第0个pu 需要回退到1/8 个 m_numPartitions 4x4
        //第1个pu   + 对应cu一半的m_numPartitions
        outPartIdxLB += puIdx ? m_numPartitions >> 1 : -((int)m_numPartitions >> 3);
        break;
    case SIZE_2NxnD:
        outPartIdxLB += puIdx ? m_numPartitions >> 1 : (m_numPartitions >> 2) + (m_numPartitions >> 3);
        break;
    case SIZE_nLx2N:
        outPartIdxLB += puIdx ? (m_numPartitions >> 1) + (m_numPartitions >> 4) : m_numPartitions >> 1;
        break;
    case SIZE_nRx2N:
        outPartIdxLB += puIdx ? (m_numPartitions >> 1) + (m_numPartitions >> 2) + (m_numPartitions >> 4) : m_numPartitions >> 1;
        break;
    default:
        S265_CHECK(0, "unexpected part index\n");
        break;
    }
    return outPartIdxLB;
}

/* Derives the partition index of neighboring bottom right block */
uint32_t CUData::deriveRightBottomIdx(uint32_t puIdx) const
{
    uint32_t outPartIdxRB;
    outPartIdxRB = g_rasterToZscan[g_zscanToRaster[m_absIdxInCTU] +
                                   (((1 << (m_log2CUSize[0] - LOG2_UNIT_SIZE - 1)) - 1) << LOG2_RASTER_SIZE) +
                                   (1 << (m_log2CUSize[0] - LOG2_UNIT_SIZE)) - 1];

    switch (m_partSize[0])
    {
    case SIZE_2Nx2N:
        outPartIdxRB += m_numPartitions >> 1;
        break;
    case SIZE_2NxN:
        outPartIdxRB += puIdx ? m_numPartitions >> 1 : 0;
        break;
    case SIZE_Nx2N:
        outPartIdxRB += puIdx ? m_numPartitions >> 1 : m_numPartitions >> 2;
        break;
    case SIZE_NxN:
        outPartIdxRB += (m_numPartitions >> 2) * (puIdx - 1);
        break;
    case SIZE_2NxnU:
        outPartIdxRB += puIdx ? m_numPartitions >> 1 : -((int)m_numPartitions >> 3);
        break;
    case SIZE_2NxnD:
        outPartIdxRB += puIdx ? m_numPartitions >> 1 : (m_numPartitions >> 2) + (m_numPartitions >> 3);
        break;
    case SIZE_nLx2N:
        outPartIdxRB += puIdx ? m_numPartitions >> 1 : (m_numPartitions >> 3) + (m_numPartitions >> 4);
        break;
    case SIZE_nRx2N:
        outPartIdxRB += puIdx ? m_numPartitions >> 1 : (m_numPartitions >> 2) + (m_numPartitions >> 3) + (m_numPartitions >> 4);
        break;
    default:
        S265_CHECK(0, "unexpected part index\n");
        break;
    }
    return outPartIdxRB;
}

bool CUData::hasEqualMotion(uint32_t absPartIdx, const CUData& candCU, uint32_t candAbsPartIdx) const
{   
    // dir 要相同
    if (m_interDir[absPartIdx] != candCU.m_interDir[candAbsPartIdx])
        return false;

    for (uint32_t refListIdx = 0; refListIdx < 2; refListIdx++)
    {
        if (m_interDir[absPartIdx] & (1 << refListIdx))//有refListIdx 上的 mv 信息
        {
            if (m_mv[refListIdx][absPartIdx] != candCU.m_mv[refListIdx][candAbsPartIdx] ||
                m_refIdx[refListIdx][absPartIdx] != candCU.m_refIdx[refListIdx][candAbsPartIdx])
                // mv 不等 或者 refIdx 不等
                return false;
        }
    }

    return true;
}

/* Construct list of merging candidates, returns count */
uint32_t CUData::getInterMergeCandidates(uint32_t absPartIdx, uint32_t puIdx, MVField(*candMvField)[2], uint8_t* candDir) const
{
    uint32_t absPartAddr = m_absIdxInCTU + absPartIdx;// cu 在ctu内的地址 + pu在cu内的偏移
    const bool isInterB = m_slice->isInterB();

    const uint32_t maxNumMergeCand = m_slice->m_maxNumMergeCand;

    for (uint32_t i = 0; i < maxNumMergeCand; ++i)
    {
        candMvField[i][0].mv = 0;
        candMvField[i][1].mv = 0;
        candMvField[i][0].refIdx = REF_NOT_VALID;
        candMvField[i][1].refIdx = REF_NOT_VALID;
    }
//  这部分带没有实际用处了
//    /* calculate the location of upper-left corner pixel and size of the current PU */
//    int xP, yP, nPSW, nPSH;
//
//    int cuSize = 1 << m_log2CUSize[0];// 当前cu的size 大小
//    int partMode = m_partSize[0];//当前cu的partMode
//
//    int tmp = partTable[partMode][puIdx][0];// 该partMode下第 puIdx个pu 的size （[0]表示size)  高4bit为x_size 低4bit为y_size
//    nPSW = ((tmp >> 4) * cuSize) >> 2;// 当前pu以pixl 为单位的w
//    nPSH = ((tmp & 0xF) * cuSize) >> 2; //当前pu以pixl 为单位的h
//
//    tmp = partTable[partMode][puIdx][1];// 该partMode下第 puIdx个pu 的 offset （[1]表示offset) 高4bit为x_offset 低4bit为y_offset
//    xP = ((tmp >> 4) * cuSize) >> 2;// 当前pu以pixl 为单位相对于所在cu的 x_offset
//    yP = ((tmp & 0xF) * cuSize) >> 2; // 当前pu以pixl 为单位相对于所在cu的 y_offset
//
    uint32_t count = 0;

    uint32_t partIdxLT, partIdxRT, partIdxLB = deriveLeftBottomIdx(puIdx);// 找到当前cu中第puIdx 个pu内部的左下角的4x4的idx 在当前cTU中 的索引
    PartSize curPS = (PartSize)m_partSize[absPartIdx];// 当前cu的partMode 用partMode 表示更好
    
    // left
    uint32_t leftPartIdx = 0;
    //找到当前pu的最左下角的4x4位置的左侧的cu 赋给cuLeft 并返回 leftParIdx表示器位于cuLeft中位置
    const CUData* cuLeft = getPULeft(leftPartIdx, partIdxLB);//
    // 左侧cu 有效 并且 不是当前cu划分为pu时的右侧pu 并且左侧是 帧间预测
    bool isAvailableA1 = cuLeft &&
        !(puIdx == 1 && (curPS == SIZE_Nx2N || curPS == SIZE_nLx2N || curPS == SIZE_nRx2N)) &&
        cuLeft->isInter(leftPartIdx);
    if (isAvailableA1)
    {
        // get Inter Dir
        candDir[count] = cuLeft->m_interDir[leftPartIdx];
        // get Mv from Left
        cuLeft->getMvField(cuLeft, leftPartIdx, 0, candMvField[count][0]);// list0的 mv
        if (isInterB)
            cuLeft->getMvField(cuLeft, leftPartIdx, 1, candMvField[count][1]);// list1 的mv

        if (++count == maxNumMergeCand)//已经找到所允许的的个数的mergecandidates了 直接返回
            return maxNumMergeCand;
    }
   // 找到当前cu中第puIdx 个pu内部的右上角的4x4的idx 在当前cTU中 的索引 赋给 partIdxRT
   // 找到当前cu中第puIdx 个pu内部的左上角的4x4的idx 在当前cTU中 的索引 赋给 partIdxLT
    deriveLeftRightTopIdx(puIdx, partIdxLT, partIdxRT);

    // above
    uint32_t abovePartIdx = 0;
    const CUData* cuAbove = getPUAbove(abovePartIdx, partIdxRT);
    // 上侧cu 有效 并且 不是当前cu划分为pu时的下侧pu 并且上侧cu是 帧间预测
    bool isAvailableB1 = cuAbove &&
        !(puIdx == 1 && (curPS == SIZE_2NxN || curPS == SIZE_2NxnU || curPS == SIZE_2NxnD)) &&
        cuAbove->isInter(abovePartIdx);
    // 如果 B1 有效  并且（ A1 无效 or B1的 mv信息与A1 不相等 ）  
    if (isAvailableB1 && (!isAvailableA1 || !cuLeft->hasEqualMotion(leftPartIdx, *cuAbove, abovePartIdx)))
    {
        // get Inter Dir
        candDir[count] = cuAbove->m_interDir[abovePartIdx];
        // get Mv from Left
        cuAbove->getMvField(cuAbove, abovePartIdx, 0, candMvField[count][0]);
        if (isInterB)
            cuAbove->getMvField(cuAbove, abovePartIdx, 1, candMvField[count][1]);

        if (++count == maxNumMergeCand)
            return maxNumMergeCand;
    }

    // above right
    uint32_t aboveRightPartIdx = 0;
    const CUData* cuAboveRight = getPUAboveRight(aboveRightPartIdx, partIdxRT);
    bool isAvailableB0 = cuAboveRight &&
        cuAboveRight->isInter(aboveRightPartIdx);
    if (isAvailableB0 && (!isAvailableB1 || !cuAbove->hasEqualMotion(abovePartIdx, *cuAboveRight, aboveRightPartIdx)))
    {
        // get Inter Dir
        candDir[count] = cuAboveRight->m_interDir[aboveRightPartIdx];
        // get Mv from Left
        cuAboveRight->getMvField(cuAboveRight, aboveRightPartIdx, 0, candMvField[count][0]);
        if (isInterB)
            cuAboveRight->getMvField(cuAboveRight, aboveRightPartIdx, 1, candMvField[count][1]);

        if (++count == maxNumMergeCand)
            return maxNumMergeCand;
    }

    // left bottom
    uint32_t leftBottomPartIdx = 0;
    const CUData* cuLeftBottom = this->getPUBelowLeft(leftBottomPartIdx, partIdxLB);
    bool isAvailableA0 = cuLeftBottom &&
        cuLeftBottom->isInter(leftBottomPartIdx);
    if (isAvailableA0 && (!isAvailableA1 || !cuLeft->hasEqualMotion(leftPartIdx, *cuLeftBottom, leftBottomPartIdx)))
    {
        // get Inter Dir
        candDir[count] = cuLeftBottom->m_interDir[leftBottomPartIdx];
        // get Mv from Left
        cuLeftBottom->getMvField(cuLeftBottom, leftBottomPartIdx, 0, candMvField[count][0]);
        if (isInterB)
            cuLeftBottom->getMvField(cuLeftBottom, leftBottomPartIdx, 1, candMvField[count][1]);

        if (++count == maxNumMergeCand)
            return maxNumMergeCand;
    }

    // above left
    if (count < 4)
    {
        uint32_t aboveLeftPartIdx = 0;
        const CUData* cuAboveLeft = getPUAboveLeft(aboveLeftPartIdx, absPartAddr);
        bool isAvailableB2 = cuAboveLeft &&
            cuAboveLeft->isInter(aboveLeftPartIdx);
        if (isAvailableB2 && (!isAvailableA1 || !cuLeft->hasEqualMotion(leftPartIdx, *cuAboveLeft, aboveLeftPartIdx))
            && (!isAvailableB1 || !cuAbove->hasEqualMotion(abovePartIdx, *cuAboveLeft, aboveLeftPartIdx)))
        {
            // get Inter Dir
            candDir[count] = cuAboveLeft->m_interDir[aboveLeftPartIdx];
            // get Mv from Left
            cuAboveLeft->getMvField(cuAboveLeft, aboveLeftPartIdx, 0, candMvField[count][0]);
            if (isInterB)
                cuAboveLeft->getMvField(cuAboveLeft, aboveLeftPartIdx, 1, candMvField[count][1]);

            if (++count == maxNumMergeCand)
                return maxNumMergeCand;
        }
    }
    if (m_slice->m_sps->bTemporalMVPEnabled)
    {
        uint32_t partIdxRB = deriveRightBottomIdx(puIdx);
        MV colmv;
        int ctuIdx = -1;

        // image boundary check
        if (m_encData->getPicCTU(m_cuAddr)->m_cuPelX + g_zscanToPelX[partIdxRB] + UNIT_SIZE < m_slice->m_sps->picWidthInLumaSamples &&
            m_encData->getPicCTU(m_cuAddr)->m_cuPelY + g_zscanToPelY[partIdxRB] + UNIT_SIZE < m_slice->m_sps->picHeightInLumaSamples)
        {
            uint32_t absPartIdxRB = g_zscanToRaster[partIdxRB];
            uint32_t numUnits = s_numPartInCUSize;//CTU中一边有多少4x4块 默认为16（64有16个4）
            bool bNotLastCol = lessThanCol(absPartIdxRB, numUnits - 1); // is not at the last column of CTU
            bool bNotLastRow = lessThanRow(absPartIdxRB, numUnits - 1); // is not at the last row    of CTU

            if (bNotLastCol && bNotLastRow)
            {
                absPartAddr = g_rasterToZscan[absPartIdxRB + RASTER_SIZE + 1];
                ctuIdx = m_cuAddr;
            }
            else if (bNotLastCol)
                absPartAddr = g_rasterToZscan[(absPartIdxRB + 1) & (numUnits - 1)];
            else if (bNotLastRow)
            {
                absPartAddr = g_rasterToZscan[absPartIdxRB + RASTER_SIZE - numUnits + 1];
                ctuIdx = m_cuAddr + 1;
            }
            else // is the right bottom corner of CTU
                absPartAddr = 0;
        }

        int maxList = isInterB ? 2 : 1;
        int dir = 0, refIdx = 0;
        for (int list = 0; list < maxList; list++)
        {
            bool bExistMV = ctuIdx >= 0 && getColMVP(colmv, refIdx, list, ctuIdx, absPartAddr);
            if (!bExistMV)
            {
                uint32_t partIdxCenter = deriveCenterIdx(puIdx);
                bExistMV = getColMVP(colmv, refIdx, list, m_cuAddr, partIdxCenter);
            }
            if (bExistMV)
            {
                dir |= (1 << list);
                candMvField[count][list].mv = colmv;
                candMvField[count][list].refIdx = refIdx;
            }
        }

        if (dir != 0)
        {
            candDir[count] = (uint8_t)dir;

            if (++count == maxNumMergeCand)
                return maxNumMergeCand;
        }
    }

    if (isInterB)
    {
        const uint32_t cutoff = count * (count - 1);
        uint32_t priorityList0 = 0xEDC984; // { 0, 1, 0, 2, 1, 2, 0, 3, 1, 3, 2, 3 }
        uint32_t priorityList1 = 0xB73621; // { 1, 0, 2, 0, 2, 1, 3, 0, 3, 1, 3, 2 }

        for (uint32_t idx = 0; idx < cutoff; idx++, priorityList0 >>= 2, priorityList1 >>= 2)
        {
            int i = priorityList0 & 3;
            int j = priorityList1 & 3;

            if ((candDir[i] & 0x1) && (candDir[j] & 0x2))
            {
                // get Mv from cand[i] and cand[j]
                int refIdxL0 = candMvField[i][0].refIdx;
                int refIdxL1 = candMvField[j][1].refIdx;
                int refPOCL0 = m_slice->m_refPOCList[0][refIdxL0];
                int refPOCL1 = m_slice->m_refPOCList[1][refIdxL1];
                if (!(refPOCL0 == refPOCL1 && candMvField[i][0].mv == candMvField[j][1].mv))
                {
                    candMvField[count][0].mv = candMvField[i][0].mv;
                    candMvField[count][0].refIdx = refIdxL0;
                    candMvField[count][1].mv = candMvField[j][1].mv;
                    candMvField[count][1].refIdx = refIdxL1;
                    candDir[count] = 3;

                    if (++count == maxNumMergeCand)
                        return maxNumMergeCand;
                }
            }
        }
    }
    int numRefIdx = (isInterB) ? S265_MIN(m_slice->m_numRefIdx[0], m_slice->m_numRefIdx[1]) : m_slice->m_numRefIdx[0];
    int r = 0;
    int refcnt = 0;
    while (count < maxNumMergeCand)
    {
        candDir[count] = 1;
        candMvField[count][0].mv.word = 0;
        candMvField[count][0].refIdx = r;

        if (isInterB)
        {
            candDir[count] = 3;
            candMvField[count][1].mv.word = 0;
            candMvField[count][1].refIdx = r;
        }

        count++;

        if (refcnt == numRefIdx - 1)
            r = 0;
        else
        {
            ++r;
            ++refcnt;
        }
    }

    return count;
}

// Create the PMV list. Called for each reference index.
int CUData::getPMV(InterNeighbourMV *neighbours, uint32_t picList, uint32_t refIdx, MV* amvpCand, MV* pmv) const
{
    MV directMV[MD_ABOVE_LEFT + 1];
    MV indirectMV[MD_ABOVE_LEFT + 1];
    bool validDirect[MD_ABOVE_LEFT + 1];
    bool validIndirect[MD_ABOVE_LEFT + 1];

    // Left candidate.
    validDirect[MD_BELOW_LEFT]  = getDirectPMV(directMV[MD_BELOW_LEFT], neighbours + MD_BELOW_LEFT, picList, refIdx);
    validDirect[MD_LEFT]        = getDirectPMV(directMV[MD_LEFT], neighbours + MD_LEFT, picList, refIdx);
    // Top candidate.
    validDirect[MD_ABOVE_RIGHT] = getDirectPMV(directMV[MD_ABOVE_RIGHT], neighbours + MD_ABOVE_RIGHT, picList, refIdx);
    validDirect[MD_ABOVE]       = getDirectPMV(directMV[MD_ABOVE], neighbours + MD_ABOVE, picList, refIdx);
    validDirect[MD_ABOVE_LEFT]  = getDirectPMV(directMV[MD_ABOVE_LEFT], neighbours + MD_ABOVE_LEFT, picList, refIdx);

    // Left candidate.
    validIndirect[MD_BELOW_LEFT]  = getIndirectPMV(indirectMV[MD_BELOW_LEFT], neighbours + MD_BELOW_LEFT, picList, refIdx);
    validIndirect[MD_LEFT]        = getIndirectPMV(indirectMV[MD_LEFT], neighbours + MD_LEFT, picList, refIdx);
    // Top candidate.
    validIndirect[MD_ABOVE_RIGHT] = getIndirectPMV(indirectMV[MD_ABOVE_RIGHT], neighbours + MD_ABOVE_RIGHT, picList, refIdx);
    validIndirect[MD_ABOVE]       = getIndirectPMV(indirectMV[MD_ABOVE], neighbours + MD_ABOVE, picList, refIdx);
    validIndirect[MD_ABOVE_LEFT]  = getIndirectPMV(indirectMV[MD_ABOVE_LEFT], neighbours + MD_ABOVE_LEFT, picList, refIdx);

    int num = 0;
    // Left predictor search
    if (validDirect[MD_BELOW_LEFT])
        amvpCand[num++] = directMV[MD_BELOW_LEFT];
    else if (validDirect[MD_LEFT])
        amvpCand[num++] = directMV[MD_LEFT];
    else if (validIndirect[MD_BELOW_LEFT])
        amvpCand[num++] = indirectMV[MD_BELOW_LEFT];
    else if (validIndirect[MD_LEFT])
        amvpCand[num++] = indirectMV[MD_LEFT];

    bool bAddedSmvp = num > 0;

    // Above predictor search
    if (validDirect[MD_ABOVE_RIGHT])
        amvpCand[num++] = directMV[MD_ABOVE_RIGHT];
    else if (validDirect[MD_ABOVE])
        amvpCand[num++] = directMV[MD_ABOVE];
    else if (validDirect[MD_ABOVE_LEFT])
        amvpCand[num++] = directMV[MD_ABOVE_LEFT];

    if (!bAddedSmvp)
    {
        if (validIndirect[MD_ABOVE_RIGHT])
            amvpCand[num++] = indirectMV[MD_ABOVE_RIGHT];
        else if (validIndirect[MD_ABOVE])
            amvpCand[num++] = indirectMV[MD_ABOVE];
        else if (validIndirect[MD_ABOVE_LEFT])
            amvpCand[num++] = indirectMV[MD_ABOVE_LEFT];
    }

    int numMvc = 0;
    for (int dir = MD_LEFT; dir <= MD_ABOVE_LEFT; dir++)
    {
        if (validDirect[dir] && directMV[dir].notZero())
            pmv[numMvc++] = directMV[dir];

        if (validIndirect[dir] && indirectMV[dir].notZero())
            pmv[numMvc++] = indirectMV[dir];
    }

    if (num == 2)
        num -= amvpCand[0] == amvpCand[1];

    // Get the collocated candidate. At this step, either the first candidate
    // was found or its value is 0.
    if (m_slice->m_sps->bTemporalMVPEnabled && num < 2)
    {
        int tempRefIdx = neighbours[MD_COLLOCATED].refIdx[picList];
        if (tempRefIdx != -1)
        {
            uint32_t cuAddr = neighbours[MD_COLLOCATED].cuAddr[picList];
            const Frame* colPic = m_slice->m_refFrameList[m_slice->isInterB() && !m_slice->m_colFromL0Flag][m_slice->m_colRefIdx];
            const CUData* colCU = colPic->m_encData->getPicCTU(cuAddr);

            // Scale the vector
            int colRefPOC = colCU->m_slice->m_refPOCList[tempRefIdx >> 4][tempRefIdx & 0xf];
            int colPOC = colCU->m_slice->m_poc;

            int curRefPOC = m_slice->m_refPOCList[picList][refIdx];
            int curPOC = m_slice->m_poc;
            pmv[numMvc++] = amvpCand[num++] = scaleMvByPOCDist(neighbours[MD_COLLOCATED].mv[picList], curPOC, curRefPOC, colPOC, colRefPOC);
        }
    }

    while (num < AMVP_NUM_CANDS)
        amvpCand[num++] = 0;

    return numMvc;
}

/* Constructs a list of candidates for AMVP, and a larger list of motion candidates */
void CUData::getNeighbourMV(uint32_t puIdx, uint32_t absPartIdx, InterNeighbourMV* neighbours) const
{
    // Set the temporal neighbour to unavailable by default.
    neighbours[MD_COLLOCATED].unifiedRef = -1;

    uint32_t partIdxLT, partIdxRT, partIdxLB = deriveLeftBottomIdx(puIdx);
    deriveLeftRightTopIdx(puIdx, partIdxLT, partIdxRT);

    // Load the spatial MVs.
    getInterNeighbourMV(neighbours + MD_BELOW_LEFT, partIdxLB, MD_BELOW_LEFT);
    getInterNeighbourMV(neighbours + MD_LEFT,       partIdxLB, MD_LEFT);
    getInterNeighbourMV(neighbours + MD_ABOVE_RIGHT,partIdxRT, MD_ABOVE_RIGHT);
    getInterNeighbourMV(neighbours + MD_ABOVE,      partIdxRT, MD_ABOVE);
    getInterNeighbourMV(neighbours + MD_ABOVE_LEFT, partIdxLT, MD_ABOVE_LEFT);

    if (m_slice->m_sps->bTemporalMVPEnabled)
    {
        uint32_t absPartAddr = m_absIdxInCTU + absPartIdx;
        uint32_t partIdxRB = deriveRightBottomIdx(puIdx);

        // co-located RightBottom temporal predictor (H)
        int ctuIdx = -1;

        // image boundary check
        if (m_encData->getPicCTU(m_cuAddr)->m_cuPelX + g_zscanToPelX[partIdxRB] + UNIT_SIZE < m_slice->m_sps->picWidthInLumaSamples &&
            m_encData->getPicCTU(m_cuAddr)->m_cuPelY + g_zscanToPelY[partIdxRB] + UNIT_SIZE < m_slice->m_sps->picHeightInLumaSamples)
        {
            uint32_t absPartIdxRB = g_zscanToRaster[partIdxRB];
            uint32_t numUnits = s_numPartInCUSize;//CTU中一边有多少4x4块 默认为16（64有16个4）
            bool bNotLastCol = lessThanCol(absPartIdxRB, numUnits - 1); // is not at the last column of CTU
            bool bNotLastRow = lessThanRow(absPartIdxRB, numUnits - 1); // is not at the last row    of CTU

            if (bNotLastCol && bNotLastRow)
            {
                absPartAddr = g_rasterToZscan[absPartIdxRB + RASTER_SIZE + 1];
                ctuIdx = m_cuAddr;
            }
            else if (bNotLastCol)
                absPartAddr = g_rasterToZscan[(absPartIdxRB + 1) & (numUnits - 1)];
            else if (bNotLastRow)
            {
                absPartAddr = g_rasterToZscan[absPartIdxRB + RASTER_SIZE - numUnits + 1];
                ctuIdx = m_cuAddr + 1;
            }
            else // is the right bottom corner of CTU
                absPartAddr = 0;
        }

        if (!(ctuIdx >= 0 && getCollocatedMV(ctuIdx, absPartAddr, neighbours + MD_COLLOCATED)))
        {
            uint32_t partIdxCenter =  deriveCenterIdx(puIdx);
            uint32_t curCTUIdx = m_cuAddr;
            getCollocatedMV(curCTUIdx, partIdxCenter, neighbours + MD_COLLOCATED);
        }
    }
}

void CUData::getInterNeighbourMV(InterNeighbourMV *neighbour, uint32_t partUnitIdx, MVP_DIR dir) const
{
    const CUData* tmpCU = NULL;
    uint32_t idx = 0;

    switch (dir)
    {
    case MD_LEFT:
        tmpCU = getPULeft(idx, partUnitIdx);
        break;
    case MD_ABOVE:
        tmpCU = getPUAbove(idx, partUnitIdx);
        break;
    case MD_ABOVE_RIGHT:
        tmpCU = getPUAboveRight(idx, partUnitIdx);
        break;
    case MD_BELOW_LEFT:
        tmpCU = getPUBelowLeft(idx, partUnitIdx);
        break;
    case MD_ABOVE_LEFT:
        tmpCU = getPUAboveLeft(idx, partUnitIdx);
        break;
    default:
        break;
    }

    if (!tmpCU)
    {
        // Mark the PMV as unavailable.
        for (int i = 0; i < 2; i++)
            neighbour->refIdx[i] = -1;
        return;
    }

    for (int i = 0; i < 2; i++)
    {
        // Get the MV.
        neighbour->mv[i] = tmpCU->m_mv[i][idx];

        // Get the reference idx.
        neighbour->refIdx[i] = tmpCU->m_refIdx[i][idx];
    }
}

/* Clip motion vector to within slightly padded boundary of picture (the
 * MV may reference a block that is completely within the padded area).
 * Note this function is unaware of how much of this picture is actually
 * available for use (re: frame parallelism) */
void CUData::clipMv(MV& outMV) const
{
    const uint32_t mvshift = 2;
    uint32_t offset = 8;

    int32_t xmax = (int32_t)((m_slice->m_sps->picWidthInLumaSamples + offset - m_cuPelX - 1) << mvshift);
    int32_t xmin = -(int32_t)((m_encData->m_param->maxCUSize + offset + m_cuPelX - 1) << mvshift);

    int32_t ymax = (int32_t)((m_slice->m_sps->picHeightInLumaSamples + offset - m_cuPelY - 1) << mvshift);
    int32_t ymin = -(int32_t)((m_encData->m_param->maxCUSize + offset + m_cuPelY - 1) << mvshift);

    outMV.x = S265_MIN(xmax, S265_MAX(xmin, outMV.x));
    outMV.y = S265_MIN(ymax, S265_MAX(ymin, outMV.y));
}

// Load direct spatial MV if available.
bool CUData::getDirectPMV(MV& pmv, InterNeighbourMV *neighbours, uint32_t picList, uint32_t refIdx) const
{
    int curRefPOC = m_slice->m_refPOCList[picList][refIdx];
    for (int i = 0; i < 2; i++, picList = !picList)
    {
        int partRefIdx = neighbours->refIdx[picList];
        if (partRefIdx >= 0 && curRefPOC == m_slice->m_refPOCList[picList][partRefIdx])
        {
            pmv = neighbours->mv[picList];
            return true;
        }
    }
    return false;
}

// Load indirect spatial MV if available. An indirect MV has to be scaled.
bool CUData::getIndirectPMV(MV& outMV, InterNeighbourMV *neighbours, uint32_t picList, uint32_t refIdx) const
{
    int curPOC = m_slice->m_poc;
    int neibPOC = curPOC;
    int curRefPOC = m_slice->m_refPOCList[picList][refIdx];

    for (int i = 0; i < 2; i++, picList = !picList)
    {
        int partRefIdx = neighbours->refIdx[picList];
        if (partRefIdx >= 0)
        {
            int neibRefPOC = m_slice->m_refPOCList[picList][partRefIdx];
            MV mvp = neighbours->mv[picList];

            outMV = scaleMvByPOCDist(mvp, curPOC, curRefPOC, neibPOC, neibRefPOC);
            return true;
        }
    }
    return false;
}

bool CUData::getColMVP(MV& outMV, int& outRefIdx, int picList, int cuAddr, int partUnitIdx) const
{
    const Frame* colPic = m_slice->m_refFrameList[m_slice->isInterB() && !m_slice->m_colFromL0Flag][m_slice->m_colRefIdx];
    const CUData* colCU = colPic->m_encData->getPicCTU(cuAddr);

    uint32_t absPartAddr = partUnitIdx & TMVP_UNIT_MASK;
    if (colCU->m_predMode[partUnitIdx] == MODE_NONE || colCU->isIntra(absPartAddr))
        return false;

    int colRefPicList = m_slice->m_bCheckLDC ? picList : m_slice->m_colFromL0Flag;

    int colRefIdx = colCU->m_refIdx[colRefPicList][absPartAddr];

    if (colRefIdx < 0)
    {
        colRefPicList = !colRefPicList;
        colRefIdx = colCU->m_refIdx[colRefPicList][absPartAddr];

        if (colRefIdx < 0)
            return false;
    }

    // Scale the vector
    int colRefPOC = colCU->m_slice->m_refPOCList[colRefPicList][colRefIdx];
    int colPOC = colCU->m_slice->m_poc;
    MV colmv = colCU->m_mv[colRefPicList][absPartAddr];

    int curRefPOC = m_slice->m_refPOCList[picList][outRefIdx];
    int curPOC = m_slice->m_poc;

    outMV = scaleMvByPOCDist(colmv, curPOC, curRefPOC, colPOC, colRefPOC);
    return true;
}

// Cache the collocated MV.
bool CUData::getCollocatedMV(int cuAddr, int partUnitIdx, InterNeighbourMV *neighbour) const
{
    const Frame* colPic = m_slice->m_refFrameList[m_slice->isInterB() && !m_slice->m_colFromL0Flag][m_slice->m_colRefIdx];
    const CUData* colCU = colPic->m_encData->getPicCTU(cuAddr);

    uint32_t absPartAddr = partUnitIdx & TMVP_UNIT_MASK;
    if (colCU->m_predMode[partUnitIdx] == MODE_NONE || colCU->isIntra(absPartAddr))
        return false;

    for (int list = 0; list < 2; list++)
    {
        neighbour->cuAddr[list] = cuAddr;
        int colRefPicList = m_slice->m_bCheckLDC ? list : m_slice->m_colFromL0Flag;
        int colRefIdx = colCU->m_refIdx[colRefPicList][absPartAddr];

        if (colRefIdx < 0)
            colRefPicList = !colRefPicList;

        neighbour->refIdx[list] = colCU->m_refIdx[colRefPicList][absPartAddr];
        neighbour->refIdx[list] |= colRefPicList << 4;

        neighbour->mv[list] = colCU->m_mv[colRefPicList][absPartAddr];
    }

    return neighbour->unifiedRef != -1;
}

MV CUData::scaleMvByPOCDist(const MV& inMV, int curPOC, int curRefPOC, int colPOC, int colRefPOC) const
{
    int diffPocD = colPOC - colRefPOC;
    int diffPocB = curPOC - curRefPOC;

    if (diffPocD == diffPocB)
        return inMV;
    else
    {
        int tdb   = s265_clip3(-128, 127, diffPocB);
        int tdd   = s265_clip3(-128, 127, diffPocD);
        int x     = (0x4000 + abs(tdd / 2)) / tdd;
        int scale = s265_clip3(-4096, 4095, (tdb * x + 32) >> 6);
        return scaleMv(inMV, scale);
    }
}

uint32_t CUData::deriveCenterIdx(uint32_t puIdx) const
{
    uint32_t absPartIdx;
    int puWidth, puHeight;

    getPartIndexAndSize(puIdx, absPartIdx, puWidth, puHeight);

    return g_rasterToZscan[g_zscanToRaster[m_absIdxInCTU + absPartIdx]
                           + ((puHeight >> (LOG2_UNIT_SIZE + 1)) << LOG2_RASTER_SIZE)
                           + (puWidth  >> (LOG2_UNIT_SIZE + 1))];
}

void CUData::getTUEntropyCodingParameters(TUEntropyCodingParameters &result, uint32_t absPartIdx, uint32_t log2TrSize, bool bIsLuma) const
{
    bool bIsIntra = isIntra(absPartIdx);

    // set the group layout
    const uint32_t log2TrSizeCG = log2TrSize - 2;

    // set the scan orders
    if (bIsIntra)
    {
        uint32_t dirMode;

        if (bIsLuma)
            dirMode = m_lumaIntraDir[absPartIdx];
        else
        {
            dirMode = m_chromaIntraDir[absPartIdx];
            if (dirMode == DM_CHROMA_IDX)
            {
                dirMode = m_lumaIntraDir[(m_chromaFormat == S265_CSP_I444) ? absPartIdx : absPartIdx & 0xFC];
                dirMode = (m_chromaFormat == S265_CSP_I422) ? g_chroma422IntraAngleMappingTable[dirMode] : dirMode;
            }
        }

        if (log2TrSize <= (MDCS_LOG2_MAX_SIZE - m_hChromaShift) || (bIsLuma && log2TrSize == MDCS_LOG2_MAX_SIZE))
            result.scanType = dirMode >= 22 && dirMode <= 30 ? SCAN_HOR : dirMode >= 6 && dirMode <= 14 ? SCAN_VER : SCAN_DIAG;
        else
            result.scanType = SCAN_DIAG;
    }
    else
        result.scanType = SCAN_DIAG;

    result.scan     = g_scanOrder[result.scanType][log2TrSize - 2];
    result.scanCG   = g_scanOrderCG[result.scanType][log2TrSizeCG];

    if (log2TrSize == 2)
        result.firstSignificanceMapContext = 0;
    else if (log2TrSize == 3)
        result.firstSignificanceMapContext = (result.scanType != SCAN_DIAG && bIsLuma) ? 15 : 9;
    else
        result.firstSignificanceMapContext = bIsLuma ? 21 : 12;
}

#define CU_SET_FLAG(bitfield, flag, value) (bitfield) = ((bitfield) & (~(flag))) | ((~((value) - 1)) & (flag))

void CUData::calcCTUGeoms(uint32_t ctuWidth, uint32_t ctuHeight, uint32_t maxCUSize, uint32_t minCUSize, CUGeom cuDataArray[CUGeom::MAX_GEOMS])
{
    uint32_t num4x4Partition = (1U << ((g_log2Size[maxCUSize] - LOG2_UNIT_SIZE) << 1));// 256 for maxCUSize 64

    // Initialize the coding blocks inside the CTB
    for (uint32_t log2CUSize = g_log2Size[maxCUSize], rangeCUIdx = 0; log2CUSize >= g_log2Size[minCUSize]; log2CUSize--)
    {// 6 5 4 3
        uint32_t blockSize = 1 << log2CUSize;// 64 32 16 8
        uint32_t sbWidth   = 1 << (g_log2Size[maxCUSize] - log2CUSize);//在一个CTU 的宽度上有 分别有 1/2/4/8 个 64/32/16/8 
        int32_t lastLevelFlag = log2CUSize == g_log2Size[minCUSize];// 0 0 0 1 //标记对于（最小的cu 通常8x8）的CUGeom的flag 的标记 

        for (uint32_t sbY = 0; sbY < sbWidth; sbY++)
        {
            for (uint32_t sbX = 0; sbX < sbWidth; sbX++)
            {
                uint32_t depthIdx = g_depthScanIdx[sbY][sbX];// 注意这里 depthIdx 实际上用sCanIdx表示更合适
//g_depthScanIdx：
//64x64:  totol cu 1
//    {   0, 
//32x32:  total cu 4
//    {   0,   1,
//    {   2,   3,

//16x16:  total cu 16
//    {   0,   1,   4,   5,
//    {   2,   3,   6,   7, 
//    {   8,   9,  12,  13, 
//    {  10,  11,  14,  15, 

//8x8: total cu 64
//    {   0,   1,   4,   5,  16,  17,  20,  21,  },
//    {   2,   3,   6,   7,  18,  19,  22,  23,  },
//    {   8,   9,  12,  13,  24,  25,  28,  29,  },
//    {  10,  11,  14,  15,  26,  27,  30,  31,  },
//    {  32,  33,  36,  37,  48,  49,  52,  53,  },
//    {  34,  35,  38,  39,  50,  51,  54,  55,  },
//    {  40,  41,  44,  45,  56,  57,  60,  61,  },
//    {  42,  43,  46,  47,  58,  59,  62,  63,  }

//rangeCUIdx
// 64x64:0  32x32:1  16x16:5  8x8:21
                uint32_t cuIdx = rangeCUIdx + depthIdx;
// cuIdx: total 1 + 4 + 16 + 64 共85个cu
// 64x64: 0           --> childIdx =     1
// 32x32: 1 + 0~3     --> childIdx =     1     +       2*2         + (0～3)*4     5/9/13/17
// 16x16: 5 + 0～15   --> childIdx =     5     +       4*4         + (0～15)*4    21/25/29/~/81
// 8x8:   21 + 0~63  --> childIdx =     21     +       8*8         + (0～63)*4    85/89/73/~/  注意 大于85的cu的childIdx 无效
                uint32_t childIdx = rangeCUIdx + sbWidth * sbWidth + (depthIdx << 2);
                uint32_t px = sbX * blockSize; // pix偏移
                uint32_t py = sbY * blockSize;
                //起始坐标点位于图片内
                int32_t presentFlag = px < ctuWidth && py < ctuHeight;
                //需要被强制分割
                int32_t splitMandatoryFlag = presentFlag && !lastLevelFlag && (px + blockSize > ctuWidth || py + blockSize > ctuHeight);

                /* Offset of the luma CU in the X, Y direction in terms of pixels from the CTU origin */
                uint32_t xOffset = (sbX * blockSize) >> 3; //按照8x8cu为单位 看当前cu的偏移
                uint32_t yOffset = (sbY * blockSize) >> 3;
                S265_CHECK(cuIdx < CUGeom::MAX_GEOMS, "CU geom index bug\n");

                CUGeom *cu = cuDataArray + cuIdx; // current cu 对应的CUGeom 指针位置 cuIdx 取值 0～84
                cu->log2CUSize = log2CUSize;// 当前cu的log2 size
                cu->childOffset = childIdx - cuIdx; 
//for64x64 childIdx =1 cuIdx=0 cu->childOffset = 1;
//for 32x32 childIdx = 1 + 4 + (0/1/2/3)*4   cuIdx = 1 + 0/1/2/3   childOffset = 4(1-->5) /7(2-->9)/10(3-->13)/13(4-->17) 
// 四叉树结构,当cuIdx为 0 childIdx 为4 刚好是 childOffset =4
// 当 cuIdx为 1 时，childIdx 需要往前挪动4 其childoffset 相对位置 需要+3 =7
// 同理 cuidx 为 2时 childIdx 需要往前挪动8 其childoffset相对位置  需要+6 = 10
// 同理 cuidx 为 3时 childIdx 需要往前挪动12 其childoffset相对位置  需要+9 = 13
                cu->absPartIdx = g_depthScanIdx[yOffset][xOffset] * 4; // 这里乘以4 因为_depthScanIdx[yOffset][xOffset] 表示的 8x8 而每个8x8 有 4个4x4

//64x64:
//0
//32x32
//0   64
//128 192
//16x16
//0   16  64  80
//32  48  96  112
//128 144 192  208
//160 176 224  240

//8x8
//0  4   16   20｜ 64
//8  12  24   28｜   
//32 36  48   52｜
//40 44  56   60｜
//--------------------------------
// 128          ｜192  196  208  212
//              ｜200  204  216  220
//              ｜224  228  240  244
//              ｜232  236  248  252
                // 64x64:256 32x32:64  16x16: 16  8x8: 4
                //当前cu有多少个4x4
                cu->numPartitions = (num4x4Partition >> ((g_log2Size[maxCUSize] - cu->log2CUSize) * 2));
                //当前cu的深度信息
                cu->depth = g_log2Size[maxCUSize] - log2CUSize;
                cu->geomRecurId = cuIdx;//每个cu 在当前ctu中的索引 （0~84)

                cu->flags = 0;
                CU_SET_FLAG(cu->flags, CUGeom::PRESENT, presentFlag);
                CU_SET_FLAG(cu->flags, CUGeom::SPLIT_MANDATORY | CUGeom::SPLIT, splitMandatoryFlag);
                CU_SET_FLAG(cu->flags, CUGeom::LEAF, lastLevelFlag);
            }
        }
        rangeCUIdx += sbWidth * sbWidth;
    }
}
