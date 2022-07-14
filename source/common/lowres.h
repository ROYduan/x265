/*****************************************************************************
 * Copyright (C) 2013-2020 MulticoreWare, Inc
 *
 * Authors: Gopu Govindaswamy <gopu@multicorewareinc.com>
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

#ifndef S265_LOWRES_H
#define S265_LOWRES_H

#include "primitives.h"
#include "common.h"
#include "picyuv.h"
#include "mv.h"

namespace S265_NS {
// private namespace

struct ReferencePlanes
{
    ReferencePlanes() { memset(this, 0, sizeof(ReferencePlanes)); }

    pixel*   fpelPlane[3];
    pixel*   lowresPlane[4];
    PicYuv*  reconPic;

    /* 1/16th resolution : Level-0 HME planes */
    pixel*   fpelLowerResPlane[3];
    pixel*   lowerResPlane[4];

    bool     isWeighted;
    bool     isLowres;
    bool     isHMELowres;

    intptr_t lumaStride;
    intptr_t chromaStride;

    struct {
        int      weight;
        int      offset;
        int      shift;
        int      round;
    } w[3];

    pixel* getLumaAddr(uint32_t ctuAddr, uint32_t absPartIdx) { return fpelPlane[0] + reconPic->m_cuOffsetY[ctuAddr] + reconPic->m_buOffsetY[absPartIdx]; }
    pixel* getCbAddr(uint32_t ctuAddr, uint32_t absPartIdx)   { return fpelPlane[1] + reconPic->m_cuOffsetC[ctuAddr] + reconPic->m_buOffsetC[absPartIdx]; }
    pixel* getCrAddr(uint32_t ctuAddr, uint32_t absPartIdx)   { return fpelPlane[2] + reconPic->m_cuOffsetC[ctuAddr] + reconPic->m_buOffsetC[absPartIdx]; }

    /* lowres motion compensation, you must provide a buffer and stride for QPEL averaged pixels
     * in case QPEL is required.  Else it returns a pointer to the HPEL pixels */
    inline pixel *lowresMC(intptr_t blockOffset, const MV& qmv, pixel *buf, intptr_t& outstride, bool hme)
    {
        intptr_t YStride = hme ? lumaStride / 2 : lumaStride;
        pixel *plane[4];
        for (int i = 0; i < 4; i++)// 设置数据指针为 1/2*1/2 小图的4个plane (0 h v hv ) 还是 1/4*1/4 小小图的4个plane 
        {
            plane[i] = hme ? lowerResPlane[i] : lowresPlane[i];
        }
        if ((qmv.x | qmv.y) & 1)//有1/4分像素 需要差值
        {
            int hpelA = (qmv.y & 2) | ((qmv.x & 2) >> 1);
            pixel *frefA = plane[hpelA] + blockOffset + (qmv.x >> 2) + (qmv.y >> 2) * YStride;
            int qmvx = qmv.x + (qmv.x & 1);
            int qmvy = qmv.y + (qmv.y & 1);
            int hpelB = (qmvy & 2) | ((qmvx & 2) >> 1);
            pixel *frefB = plane[hpelB] + blockOffset + (qmvx >> 2) + (qmvy >> 2) * YStride;
            primitives.pu[LUMA_8x8].pixelavg_pp[(outstride % 64 == 0) && (YStride % 64 == 0)](buf, outstride, frefA, YStride, frefB, YStride, 32);
            return buf;//返回传进来的buf空间的地址
        }
        else //只有 整像素 或者已经差值好的 1/2 像素 时
        {
            outstride = YStride;
            int hpel = (qmv.y & 2) | ((qmv.x & 2) >> 1);//找到对应的 0 h v hv的plane
            return plane[hpel] + blockOffset + (qmv.x >> 2) + (qmv.y >> 2) * YStride;// 返回对应plane的偏移后的地址
        }
    }

    inline int lowresQPelCost(pixel *fenc, intptr_t blockOffset, const MV& qmv, pixelcmp_t comp, bool hme)
    {
        intptr_t YStride = hme ? lumaStride / 2 : lumaStride;
        pixel *plane[4];
        for (int i = 0; i < 4; i++)
        {
            plane[i] = hme ? lowerResPlane[i] : lowresPlane[i];
        }
        if ((qmv.x | qmv.y) & 1)
        {
            ALIGN_VAR_16(pixel, subpelbuf[8 * 8]);
            int hpelA = (qmv.y & 2) | ((qmv.x & 2) >> 1);
            pixel *frefA = plane[hpelA] + blockOffset + (qmv.x >> 2) + (qmv.y >> 2) * YStride;
            int qmvx = qmv.x + (qmv.x & 1);
            int qmvy = qmv.y + (qmv.y & 1);
            int hpelB = (qmvy & 2) | ((qmvx & 2) >> 1);
            pixel *frefB = plane[hpelB] + blockOffset + (qmvx >> 2) + (qmvy >> 2) * YStride;
            primitives.pu[LUMA_8x8].pixelavg_pp[NONALIGNED](subpelbuf, 8, frefA, YStride, frefB, YStride, 32);
            return comp(fenc, FENC_STRIDE, subpelbuf, 8);
        }
        else
        {
            int hpel = (qmv.y & 2) | ((qmv.x & 2) >> 1);
            pixel *fref = plane[hpel] + blockOffset + (qmv.x >> 2) + (qmv.y >> 2) * YStride;
            return comp(fenc, FENC_STRIDE, fref, YStride);
        }
    }
};
// [64/32/16] // [64x64/32x32/16x16/8x8] //
static const uint32_t aqLayerDepth[3][4][4] = {
    {  // ctu size 64
        { 1, 0, 1, 0 },//qgSize:64x64
        { 1, 1, 1, 0 },//qgSize:32x32
        { 1, 1, 1, 0 },//qgSize:16x16
        { 1, 1, 1, 1 } // qgSize:8x8
    },
    {  // ctu size 32
        { 1, 1, 0, 0 },//32x32
        { 1, 1, 0, 0 },//16x16
        { 1, 1, 1, 0 },//8x8
        { 0, 0, 0, 0 },
    },
    {  // ctu size 16
        { 1, 0, 0, 0 },//16x16
        { 1, 1, 0, 0 },//8x8
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 }
    }
};

// min aq size for ctu size 64, 32 and 16
static const uint32_t minAQSize[3] = { 3, 2, 1 };

struct PicQPAdaptationLayer
{
    uint32_t aqPartWidth;
    uint32_t aqPartHeight;
    uint32_t numAQPartInWidth;
    uint32_t numAQPartInHeight;
    uint32_t minAQDepth;
    double*  dActivity;
    double*  dQpOffset;

    double*  dCuTreeOffset;
    double*  dCuTreeOffset8x8;
    double   dAvgActivity;
    bool     bQpSize;

    bool  create(uint32_t width, uint32_t height, uint32_t aqPartWidth, uint32_t aqPartHeight, uint32_t numAQPartInWidthExt, uint32_t numAQPartInHeightExt);
    void  destroy();
};

/* lowres buffers, sizes and strides */
struct Lowres : public ReferencePlanes
{
    pixel *buffer[4];
    pixel *lowerResBuffer[4]; // Level-0 buffer

    int    frameNum;         // Presentation frame number
    int    sliceType;        // Slice type decided by lookahead
    int    width;            // width of lowres frame in pixels
    int    lines;            // height of lowres frame in pixel lines
    int    leadingBframes;   // number of leading B frames for P or I

    bool   bScenecut;        // Set to false if the frame cannot possibly be part of a real scenecut.
    bool   bKeyframe;
    bool   bLastMiniGopBFrame; //一个nimigop内的最后一个B帧也是编码顺序上的最后一帧
    bool   bIsFadeEnd;

    double ipCostRatio;

    /* lookahead output data */
    int64_t   costEst[S265_BFRAME_MAX + 2][S265_BFRAME_MAX + 2];
    int64_t   costEstAq[S265_BFRAME_MAX + 2][S265_BFRAME_MAX + 2];
    int32_t*  rowSatds[S265_BFRAME_MAX + 2][S265_BFRAME_MAX + 2];
    int       intraMbs[S265_BFRAME_MAX + 2];
    int32_t*  intraCost;
    uint8_t*  intraMode;
    int64_t   satdCost;
    uint16_t* lowresCostForRc;
    uint16_t* lowresCosts[S265_BFRAME_MAX + 2][S265_BFRAME_MAX + 2];
    int32_t*  lowresMvCosts[2][S265_BFRAME_MAX + 2];
    MV*       lowresMvs[2][S265_BFRAME_MAX + 2];
    int       largeMvs[S265_BFRAME_MAX + 2];
    int       veryLargeMvs[S265_BFRAME_MAX + 2];
    int       hasSmallMvs[S265_BFRAME_MAX + 2];
    uint32_t  maxBlocksInRow;
    uint32_t  maxBlocksInCol;
    uint32_t  maxBlocksInRowFullRes;
    uint32_t  maxBlocksInColFullRes;

    /* Hierarchical Motion Estimation */
    bool      bEnableHME;
    int32_t*  lowerResMvCosts[2][S265_BFRAME_MAX + 2];
    MV*       lowerResMvs[2][S265_BFRAME_MAX + 2];

    /* used for vbvLookahead */
    int       plannedType[S265_LOOKAHEAD_MAX + 1];
    int64_t   plannedSatd[S265_LOOKAHEAD_MAX + 1];
    int64_t   plannedSatdIntra[S265_LOOKAHEAD_MAX + 1];
    int       indB;
    int       bframes;

    /* rate control / adaptive quant data */
    double*   qpAqOffset;      // AQ QP offset values for each 16x16 CU
    double*   qpCuTreeOffset;  // cuTree QP offset + AQ Qp offset  values for each 16x16 CU
    double*   qpAqMotionOffset;
    int*      invQscaleFactor; // qScale values for qp Aq Offsets
    int*      invQscaleFactor8x8; // temporary buffer for qg-size 8
    uint32_t* blockVariance;
    uint64_t  wp_ssd[3];       // This is different than SSDY, this is sum(pixel^2) - sum(pixel)^2 for entire frame
    uint64_t  wp_sum[3];
    double    frameVariance;
    int* edgeInclined;


    /* cutree intermediate data */
    PicQPAdaptationLayer* pAQLayer;
    uint32_t maxAQDepth;
    uint32_t widthFullRes;
    uint32_t heightFullRes;
    uint32_t m_maxCUSize;
    uint32_t m_qgSize;
    
    uint16_t* propagateCost;
    double    weightedCostDelta[S265_BFRAME_MAX + 2];
    ReferencePlanes weightedRef[S265_BFRAME_MAX + 2];
    /* For hist-based scenecut */
    bool   m_bIsMaxThres;
    double interPCostPercDiff;
    double intraCostPercDiff;
    bool   m_bIsHardScenecut;

    /*hierarchy prediction info*/
    int     i_level;
    int     i_gop_size;
    int     i_bref;
    int     i_max_depth;
    int     i_gop_id;
    int     i_temporal_id;
    int     i_ref_value;


    bool create(s265_param* param, PicYuv *origPic, uint32_t qgSize);
    void destroy();
    void init(PicYuv *origPic, int poc);
};
}

#endif // ifndef S265_LOWRES_H
