/*****************************************************************************
 * Copyright (C) 2013-2020 MulticoreWare, Inc
 *
 * Authors: Gopu Govindaswamy <gopu@multicorewareinc.com>
 *          Steve Borho <steve@borho.org>
 *          Ashok Kumar Mishra <ashok@multicorewareinc.com>
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
#include "primitives.h"
#include "lowres.h"
#include "mv.h"

#include "slicetype.h"
#include "motion.h"
#include "ratecontrol.h"

#if DETAILED_CU_STATS
#define ProfileLookaheadTime(elapsed, count) ScopedElapsedTime _scope(elapsed); count++
#else
#define ProfileLookaheadTime(elapsed, count)
#endif

using namespace S265_NS;

namespace {

/* Compute variance to derive AC energy of each block */
inline uint32_t acEnergyVar(Frame *curFrame, uint64_t sum_ssd, int shift, int plane)
{
    uint32_t sum = (uint32_t)sum_ssd;
    uint32_t ssd = (uint32_t)(sum_ssd >> 32);

    curFrame->m_lowres.wp_sum[plane] += sum;
    curFrame->m_lowres.wp_ssd[plane] += ssd;
    return ssd - ((uint64_t)sum * sum >> shift);
}

/* Find the energy of each block in Y/Cb/Cr plane */
inline uint32_t acEnergyPlane(Frame *curFrame, pixel* src, intptr_t srcStride, int plane, int colorFormat, uint32_t qgSize)
{
    if ((colorFormat != S265_CSP_I444) && plane)
    {
        if (qgSize == 8)
        {
            ALIGN_VAR_4(pixel, pix[4 * 4]);
            primitives.cu[BLOCK_4x4].copy_pp(pix, 4, src, srcStride);
            return acEnergyVar(curFrame, primitives.cu[BLOCK_4x4].var(pix, 4), 4, plane);
        }
        else
        {
            ALIGN_VAR_8(pixel, pix[8 * 8]);
            primitives.cu[BLOCK_8x8].copy_pp(pix, 8, src, srcStride);
            return acEnergyVar(curFrame, primitives.cu[BLOCK_8x8].var(pix, 8), 6, plane);
        }
    }
    else
    {
        if (qgSize == 8)
            return acEnergyVar(curFrame, primitives.cu[BLOCK_8x8].var(src, srcStride), 6, plane);
        else
            return acEnergyVar(curFrame, primitives.cu[BLOCK_16x16].var(src, srcStride), 8, plane);
    }
}

} // end anonymous namespace

namespace S265_NS {

bool computeEdge(pixel* edgePic, pixel* refPic, pixel* edgeTheta, intptr_t stride, int height, int width, bool bcalcTheta, pixel whitePixel)
{
    intptr_t rowOne = 0, rowTwo = 0, rowThree = 0, colOne = 0, colTwo = 0, colThree = 0;
    intptr_t middle = 0, topLeft = 0, topRight = 0, bottomLeft = 0, bottomRight = 0;

    const int startIndex = 1;

    if (!edgePic || !refPic || (!edgeTheta && bcalcTheta))
    {
        return false;
    }
    else
    {
        float gradientH = 0, gradientV = 0, radians = 0, theta = 0;
        float gradientMagnitude = 0;
        pixel blackPixel = 0;

        //Applying Sobel filter expect for border pixels
        height = height - startIndex;
        width = width - startIndex;
        for (int rowNum = startIndex; rowNum < height; rowNum++)
        {
            rowTwo = rowNum * stride;
            rowOne = rowTwo - stride;
            rowThree = rowTwo + stride;

            for (int colNum = startIndex; colNum < width; colNum++)
            {

                 /*  Horizontal and vertical gradients
                     [ -3   0   3 ]        [-3   -10  -3 ]
                 gH =[ -10  0   10]   gV = [ 0    0    0 ]
                     [ -3   0   3 ]        [ 3    10   3 ] */

                colTwo = colNum;
                colOne = colTwo - startIndex;
                colThree = colTwo + startIndex;
                middle = rowTwo + colTwo;
                topLeft = rowOne + colOne;
                topRight = rowOne + colThree;
                bottomLeft = rowThree + colOne;
                bottomRight = rowThree + colThree;
                gradientH = (float)(-3 * refPic[topLeft] + 3 * refPic[topRight] - 10 * refPic[rowTwo + colOne] + 10 * refPic[rowTwo + colThree] - 3 * refPic[bottomLeft] + 3 * refPic[bottomRight]);
                gradientV = (float)(-3 * refPic[topLeft] - 10 * refPic[rowOne + colTwo] - 3 * refPic[topRight] + 3 * refPic[bottomLeft] + 10 * refPic[rowThree + colTwo] + 3 * refPic[bottomRight]);
                gradientMagnitude = sqrtf(gradientH * gradientH + gradientV * gradientV);
                if(bcalcTheta) 
                {
                    edgeTheta[middle] = 0;
                    radians = atan2(gradientV, gradientH);
                    theta = (float)((radians * 180) / PI);
                    if (theta < 0)
                       theta = 180 + theta;
                    edgeTheta[middle] = (pixel)theta;
                }
                edgePic[middle] = (pixel)(gradientMagnitude >= EDGE_THRESHOLD ? whitePixel : blackPixel);
            }
        }
        return true;
    }
}

void edgeFilter(Frame *curFrame, s265_param* param)
{
    int height = curFrame->m_fencPic->m_picHeight;
    int width = curFrame->m_fencPic->m_picWidth;
    intptr_t stride = curFrame->m_fencPic->m_stride;
    uint32_t numCuInHeight = (height + param->maxCUSize - 1) / param->maxCUSize;
    int maxHeight = numCuInHeight * param->maxCUSize;

    memset(curFrame->m_edgePic, 0, stride * (maxHeight + (curFrame->m_fencPic->m_lumaMarginY * 2)) * sizeof(pixel));
    memset(curFrame->m_gaussianPic, 0, stride * (maxHeight + (curFrame->m_fencPic->m_lumaMarginY * 2)) * sizeof(pixel));
    memset(curFrame->m_thetaPic, 0, stride * (maxHeight + (curFrame->m_fencPic->m_lumaMarginY * 2)) * sizeof(pixel));

    pixel *src = (pixel*)curFrame->m_fencPic->m_picOrg[0];
    pixel *edgePic = curFrame->m_edgePic + curFrame->m_fencPic->m_lumaMarginY * stride + curFrame->m_fencPic->m_lumaMarginX;
    pixel *refPic = curFrame->m_gaussianPic + curFrame->m_fencPic->m_lumaMarginY * stride + curFrame->m_fencPic->m_lumaMarginX;
    pixel *edgeTheta = curFrame->m_thetaPic + curFrame->m_fencPic->m_lumaMarginY * stride + curFrame->m_fencPic->m_lumaMarginX;

    for (int i = 0; i < height; i++)
    {
        memcpy(edgePic, src, width * sizeof(pixel));
        memcpy(refPic, src, width * sizeof(pixel));
        src += stride;
        edgePic += stride;
        refPic += stride;
    }

    //Applying Gaussian filter on the picture
    src = (pixel*)curFrame->m_fencPic->m_picOrg[0];
    refPic = curFrame->m_gaussianPic + curFrame->m_fencPic->m_lumaMarginY * stride + curFrame->m_fencPic->m_lumaMarginX;
    edgePic = curFrame->m_edgePic + curFrame->m_fencPic->m_lumaMarginY * stride + curFrame->m_fencPic->m_lumaMarginX;
    pixel pixelValue = 0;

    for (int rowNum = 0; rowNum < height; rowNum++)
    {
        for (int colNum = 0; colNum < width; colNum++)
        {
            if ((rowNum >= 2) && (colNum >= 2) && (rowNum != height - 2) && (colNum != width - 2)) //Ignoring the border pixels of the picture
            {
                /*  5x5 Gaussian filter
                    [2   4   5   4   2]
                 1  [4   9   12  9   4]
                --- [5   12  15  12  5]
                159 [4   9   12  9   4]
                    [2   4   5   4   2]*/

                const intptr_t rowOne = (rowNum - 2)*stride, colOne = colNum - 2;
                const intptr_t rowTwo = (rowNum - 1)*stride, colTwo = colNum - 1;
                const intptr_t rowThree = rowNum * stride, colThree = colNum;
                const intptr_t rowFour = (rowNum + 1)*stride, colFour = colNum + 1;
                const intptr_t rowFive = (rowNum + 2)*stride, colFive = colNum + 2;
                const intptr_t index = (rowNum*stride) + colNum;

                pixelValue = ((2 * src[rowOne + colOne] + 4 * src[rowOne + colTwo] + 5 * src[rowOne + colThree] + 4 * src[rowOne + colFour] + 2 * src[rowOne + colFive] +
                    4 * src[rowTwo + colOne] + 9 * src[rowTwo + colTwo] + 12 * src[rowTwo + colThree] + 9 * src[rowTwo + colFour] + 4 * src[rowTwo + colFive] +
                    5 * src[rowThree + colOne] + 12 * src[rowThree + colTwo] + 15 * src[rowThree + colThree] + 12 * src[rowThree + colFour] + 5 * src[rowThree + colFive] +
                    4 * src[rowFour + colOne] + 9 * src[rowFour + colTwo] + 12 * src[rowFour + colThree] + 9 * src[rowFour + colFour] + 4 * src[rowFour + colFive] +
                    2 * src[rowFive + colOne] + 4 * src[rowFive + colTwo] + 5 * src[rowFive + colThree] + 4 * src[rowFive + colFour] + 2 * src[rowFive + colFive]) / 159);
                refPic[index] = pixelValue;
            }
        }
    }

    if(!computeEdge(edgePic, refPic, edgeTheta, stride, height, width, true))
        s265_log(NULL, S265_LOG_ERROR, "Failed edge computation!");
}

//Find the angle of a block by averaging the pixel angles 
inline void findAvgAngle(const pixel* block, intptr_t stride, uint32_t size, uint32_t &angle)
{
    int sum = 0;
    for (uint32_t y = 0; y < size; y++)
    {
        for (uint32_t x = 0; x < size; x++)
        {
            sum += block[x];
        }
        block += stride;
    }
    angle = sum / (size*size);
}

uint32_t LookaheadTLD::edgeDensityCu(Frame* curFrame, uint32_t &avgAngle, uint32_t blockX, uint32_t blockY, uint32_t qgSize)
{
    pixel *edgeImage = curFrame->m_edgePic + curFrame->m_fencPic->m_lumaMarginY * curFrame->m_fencPic->m_stride + curFrame->m_fencPic->m_lumaMarginX;
    pixel *edgeTheta = curFrame->m_thetaPic + curFrame->m_fencPic->m_lumaMarginY * curFrame->m_fencPic->m_stride + curFrame->m_fencPic->m_lumaMarginX;
    intptr_t srcStride = curFrame->m_fencPic->m_stride;
    intptr_t blockOffsetLuma = blockX + (blockY * srcStride);
    int plane = 0; // Sobel filter is applied only on Y component
    uint32_t var;

    if (qgSize == 8)
    {
        findAvgAngle(edgeTheta + blockOffsetLuma, srcStride, qgSize, avgAngle);
        var = acEnergyVar(curFrame, primitives.cu[BLOCK_8x8].var(edgeImage + blockOffsetLuma, srcStride), 6, plane);
    }
    else
    {
        findAvgAngle(edgeTheta + blockOffsetLuma, srcStride, 16, avgAngle);
        var = acEnergyVar(curFrame, primitives.cu[BLOCK_16x16].var(edgeImage + blockOffsetLuma, srcStride), 8, plane);
    }
    s265_emms();
    return var;
}

/* Find the total AC energy of each block in all planes */
uint32_t LookaheadTLD::acEnergyCu(Frame* curFrame, uint32_t blockX, uint32_t blockY, int csp, uint32_t qgSize)
{
    intptr_t stride = curFrame->m_fencPic->m_stride;
    intptr_t cStride = curFrame->m_fencPic->m_strideC;
    intptr_t blockOffsetLuma = blockX + (blockY * stride);
    int hShift = CHROMA_H_SHIFT(csp);
    int vShift = CHROMA_V_SHIFT(csp);
    intptr_t blockOffsetChroma = (blockX >> hShift) + ((blockY >> vShift) * cStride);

    uint32_t var;

    var  = acEnergyPlane(curFrame, curFrame->m_fencPic->m_picOrg[0] + blockOffsetLuma, stride, 0, csp, qgSize);
    if (csp != S265_CSP_I400 && curFrame->m_fencPic->m_picCsp != S265_CSP_I400)
    {
        var += acEnergyPlane(curFrame, curFrame->m_fencPic->m_picOrg[1] + blockOffsetChroma, cStride, 1, csp, qgSize);
        var += acEnergyPlane(curFrame, curFrame->m_fencPic->m_picOrg[2] + blockOffsetChroma, cStride, 2, csp, qgSize);
    }
    s265_emms();
    return var;
}

/* Find the sum of pixels of each block for luma plane */
uint32_t LookaheadTLD::lumaSumCu(Frame* curFrame, uint32_t blockX, uint32_t blockY, uint32_t qgSize)
{
    intptr_t stride = curFrame->m_fencPic->m_stride;
    intptr_t blockOffsetLuma = blockX + (blockY * stride);
    uint64_t sum_ssd;

    if (qgSize == 8)
        sum_ssd = primitives.cu[BLOCK_8x8].var(curFrame->m_fencPic->m_picOrg[0] + blockOffsetLuma, stride);
    else
        sum_ssd = primitives.cu[BLOCK_16x16].var(curFrame->m_fencPic->m_picOrg[0] + blockOffsetLuma, stride);

    s265_emms();
    return (uint32_t)sum_ssd;
}

void LookaheadTLD::xPreanalyzeQp(Frame* curFrame)
{
    const uint32_t width = curFrame->m_fencPic->m_picWidth;
    const uint32_t height = curFrame->m_fencPic->m_picHeight;

    for (uint32_t d = 0; d < 4; d++)
    {
        int ctuSizeIdx = 6 - g_log2Size[curFrame->m_param->maxCUSize];
        int aqDepth = g_log2Size[curFrame->m_param->maxCUSize] - g_log2Size[curFrame->m_param->rc.qgSize];
        if (!aqLayerDepth[ctuSizeIdx][aqDepth][d])
            continue;

        PicQPAdaptationLayer* pcAQLayer = &curFrame->m_lowres.pAQLayer[d];
        const uint32_t aqPartWidth = pcAQLayer->aqPartWidth;
        const uint32_t aqPartHeight = pcAQLayer->aqPartHeight;
        double* pcAQU = pcAQLayer->dActivity;
        double* pcQP = pcAQLayer->dQpOffset;
        double* pcCuTree = pcAQLayer->dCuTreeOffset;

        for (uint32_t y = 0; y < height; y += aqPartHeight)
        {
            for (uint32_t x = 0; x < width; x += aqPartWidth, pcAQU++, pcQP++, pcCuTree++)
            {
                double dMaxQScale = pow(2.0, curFrame->m_param->rc.qpAdaptationRange / 6.0);
                double dCUAct = *pcAQU;
                double dAvgAct = pcAQLayer->dAvgActivity;

                double dNormAct = (dMaxQScale*dCUAct + dAvgAct) / (dCUAct + dMaxQScale*dAvgAct);
                double dQpOffset = (S265_LOG2(dNormAct) / S265_LOG2(2.0)) * 6.0;
                *pcQP = dQpOffset;
                *pcCuTree = dQpOffset;
            }
        }
    }
}

void LookaheadTLD::xPreanalyze(Frame* curFrame)
{
    const uint32_t width = curFrame->m_fencPic->m_picWidth;
    const uint32_t height = curFrame->m_fencPic->m_picHeight;
    const intptr_t stride = curFrame->m_fencPic->m_stride;

    for (uint32_t d = 0; d < 4; d++)
    {
        int ctuSizeIdx = 6 - g_log2Size[curFrame->m_param->maxCUSize];
        int aqDepth = g_log2Size[curFrame->m_param->maxCUSize] - g_log2Size[curFrame->m_param->rc.qgSize];
        if (!aqLayerDepth[ctuSizeIdx][aqDepth][d])
            continue;

        const pixel* src = curFrame->m_fencPic->m_picOrg[0];;
        PicQPAdaptationLayer* pQPLayer = &curFrame->m_lowres.pAQLayer[d];
        const uint32_t aqPartWidth = pQPLayer->aqPartWidth;
        const uint32_t aqPartHeight = pQPLayer->aqPartHeight;
        double* pcAQU = pQPLayer->dActivity;

        double dSumAct = 0.0;
        for (uint32_t y = 0; y < height; y += aqPartHeight)
        {
            const uint32_t currAQPartHeight = S265_MIN(aqPartHeight, height - y);
            for (uint32_t x = 0; x < width; x += aqPartWidth, pcAQU++)
            {
                const uint32_t currAQPartWidth = S265_MIN(aqPartWidth, width - x);
                const pixel* pBlkY = &src[x];
                uint64_t sum[4] = { 0, 0, 0, 0 };
                uint64_t sumSq[4] = { 0, 0, 0, 0 };
                uint32_t by = 0;
                for (; by < currAQPartHeight >> 1; by++)
                {
                    uint32_t bx = 0;
                    for (; bx < currAQPartWidth >> 1; bx++)
                    {
                        sum[0] += pBlkY[bx];
                        sumSq[0] += pBlkY[bx] * pBlkY[bx];
                    }
                    for (; bx < currAQPartWidth; bx++)
                    {
                        sum[1] += pBlkY[bx];
                        sumSq[1] += pBlkY[bx] * pBlkY[bx];
                    }
                    pBlkY += stride;
                }
                for (; by < currAQPartHeight; by++)
                {
                    uint32_t bx = 0;
                    for (; bx < currAQPartWidth >> 1; bx++)
                    {
                        sum[2] += pBlkY[bx];
                        sumSq[2] += pBlkY[bx] * pBlkY[bx];
                    }
                    for (; bx < currAQPartWidth; bx++)
                    {
                        sum[3] += pBlkY[bx];
                        sumSq[3] += pBlkY[bx] * pBlkY[bx];
                    }
                    pBlkY += stride;
                }

                assert((currAQPartWidth & 1) == 0);
                assert((currAQPartHeight & 1) == 0);
                const uint32_t pixelWidthOfQuadrants = currAQPartWidth >> 1;
                const uint32_t pixelHeightOfQuadrants = currAQPartHeight >> 1;
                const uint32_t numPixInAQPart = pixelWidthOfQuadrants * pixelHeightOfQuadrants;

                double dMinVar = MAX_DOUBLE;
                if (numPixInAQPart != 0)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        const double dAverage = double(sum[i]) / numPixInAQPart;
                        const double dVariance = double(sumSq[i]) / numPixInAQPart - dAverage * dAverage;
                        dMinVar = S265_MIN(dMinVar, dVariance);
                    }
                }
                else
                {
                    dMinVar = 0.0;
                }
                double dActivity = 1.0 + dMinVar;
                *pcAQU = dActivity;
                dSumAct += dActivity;
            }
            src += stride * currAQPartHeight;
        }

        const double dAvgAct = dSumAct / (pQPLayer->numAQPartInWidth * pQPLayer->numAQPartInHeight);
        pQPLayer->dAvgActivity = dAvgAct;
    }

    xPreanalyzeQp(curFrame);

    int minAQDepth = curFrame->m_lowres.pAQLayer->minAQDepth;

    PicQPAdaptationLayer* pQPLayer = &curFrame->m_lowres.pAQLayer[minAQDepth];
    const uint32_t aqPartWidth = pQPLayer->aqPartWidth;
    const uint32_t aqPartHeight = pQPLayer->aqPartHeight;
    double* pcQP = pQPLayer->dQpOffset;

    // Use new qp offset values for qpAqOffset, qpCuTreeOffset and invQscaleFactor buffer
    int blockXY = 0;
    for (uint32_t y = 0; y < height; y += aqPartHeight)
    {
        for (uint32_t x = 0; x < width; x += aqPartWidth, pcQP++)
        {
            curFrame->m_lowres.invQscaleFactor[blockXY] = s265_exp2fix8(*pcQP);
            blockXY++;

            acEnergyCu(curFrame, x, y, curFrame->m_param->internalCsp, curFrame->m_param->rc.qgSize);
        }
    }
}

void LookaheadTLD::calcAdaptiveQuantFrame(Frame *curFrame, s265_param* param)
{
    /* Actual adaptive quantization */
    int maxCol = curFrame->m_fencPic->m_picWidth;
    int maxRow = curFrame->m_fencPic->m_picHeight;
    int blockCount, loopIncr;
    float modeOneConst, modeTwoConst;
    if (param->rc.qgSize == 8)
    {   // 相当于对应lowres 中 4x4 的个数了
        blockCount = curFrame->m_lowres.maxBlocksInRowFullRes * curFrame->m_lowres.maxBlocksInColFullRes;
        modeOneConst = 11.427f;
        modeTwoConst = 8.f;
        loopIncr = 8;
    }
    else
    {  // 相当于对应lowres 中 8x8 的个数了
        blockCount = widthInCU * heightInCU;
        modeOneConst = 14.427f;
        modeTwoConst = 11.f;
        loopIncr = 16;
    }

    float* quantOffsets = curFrame->m_quantOffsets;
    for (int y = 0; y < 3; y++)
    {
        curFrame->m_lowres.wp_ssd[y] = 0;// accumulate init
        curFrame->m_lowres.wp_sum[y] = 0;
    }

    if (!(param->rc.bStatRead && param->rc.cuTree && IS_REFERENCED(curFrame)))
    {
        /* Calculate Qp offset for each 16x16 or 8x8 block in the frame */
        if (param->rc.aqMode == S265_AQ_NONE || param->rc.aqStrength == 0)
        {
            if (param->rc.aqMode && param->rc.aqStrength == 0)
            {
                if (quantOffsets)
                {
                    for (int cuxy = 0; cuxy < blockCount; cuxy++)
                    {
                        curFrame->m_lowres.qpCuTreeOffset[cuxy] = curFrame->m_lowres.qpAqOffset[cuxy] = quantOffsets[cuxy];
                        curFrame->m_lowres.invQscaleFactor[cuxy] = s265_exp2fix8(curFrame->m_lowres.qpCuTreeOffset[cuxy]);
                    }
                }
                else
                {
                    memset(curFrame->m_lowres.qpCuTreeOffset, 0, blockCount * sizeof(double));
                    memset(curFrame->m_lowres.qpAqOffset, 0, blockCount * sizeof(double));
                    for (int cuxy = 0; cuxy < blockCount; cuxy++)
                        curFrame->m_lowres.invQscaleFactor[cuxy] = 256;
                }
            }

            /* Need variance data for weighted prediction and dynamic refinement*/
            if (param->bEnableWeightedPred || param->bEnableWeightedBiPred)
            {
                for (int blockY = 0; blockY < maxRow; blockY += loopIncr)
                    for (int blockX = 0; blockX < maxCol; blockX += loopIncr)
                        acEnergyCu(curFrame, blockX, blockY, param->internalCsp, param->rc.qgSize);
            }
        }
        else
        {
            if (param->rc.hevcAq)
            {
                // New method for calculating variance and qp offset
                xPreanalyze(curFrame);
            }
            else
            {
                int blockXY = 0, inclinedEdge = 0;
                double avg_adj_pow2 = 0, avg_adj = 0, qp_adj = 0;
                double bias_strength = 0.f;
                double strength = 0.f;

                if (param->rc.aqMode == S265_AQ_EDGE)
                    edgeFilter(curFrame, param);

                if (param->rc.aqMode == S265_AQ_EDGE && !param->bHistBasedSceneCut && param->recursionSkipMode == EDGE_BASED_RSKIP)
                {
                    pixel* src = curFrame->m_edgePic + curFrame->m_fencPic->m_lumaMarginY * curFrame->m_fencPic->m_stride + curFrame->m_fencPic->m_lumaMarginX;
                    primitives.planecopy_pp_shr(src, curFrame->m_fencPic->m_stride, curFrame->m_edgeBitPic,
                        curFrame->m_fencPic->m_stride, curFrame->m_fencPic->m_picWidth, curFrame->m_fencPic->m_picHeight, SHIFT_TO_BITPLANE);
                }

                if (param->rc.aqMode == S265_AQ_AUTO_VARIANCE || param->rc.aqMode == S265_AQ_AUTO_VARIANCE_BIASED || param->rc.aqMode == S265_AQ_EDGE)
                {
                    double bit_depth_correction = 1.f / (1 << (2 * (S265_DEPTH - 8)));
                    for (int blockY = 0; blockY < maxRow; blockY += loopIncr)
                    {
                        for (int blockX = 0; blockX < maxCol; blockX += loopIncr)
                        {
                            uint32_t energy, edgeDensity, avgAngle;
                            energy = acEnergyCu(curFrame, blockX, blockY, param->internalCsp, param->rc.qgSize);
                            if (param->rc.aqMode == S265_AQ_EDGE)
                            {
                                edgeDensity = edgeDensityCu(curFrame, avgAngle, blockX, blockY, param->rc.qgSize);
                                if (edgeDensity)
                                {
                                    qp_adj = pow(edgeDensity * bit_depth_correction + 1, 0.1);
                                    //Increasing the QP of a block if its edge orientation lies around the multiples of 45 degree
                                    if ((avgAngle >= EDGE_INCLINATION - 15 && avgAngle <= EDGE_INCLINATION + 15) || (avgAngle >= EDGE_INCLINATION + 75 && avgAngle <= EDGE_INCLINATION + 105))
                                        curFrame->m_lowres.edgeInclined[blockXY] = 1;
                                    else
                                        curFrame->m_lowres.edgeInclined[blockXY] = 0;
                                }
                                else
                                {
                                    qp_adj = pow(energy * bit_depth_correction + 1, 0.1);
                                    curFrame->m_lowres.edgeInclined[blockXY] = 0;
                                }
                            }
                            else
                                qp_adj = pow(energy * bit_depth_correction + 1, 0.1);
                            curFrame->m_lowres.qpCuTreeOffset[blockXY] = qp_adj;
                            avg_adj += qp_adj;
                            avg_adj_pow2 += qp_adj * qp_adj;
                            blockXY++;
                        }
                    }
                    avg_adj /= blockCount;
                    avg_adj_pow2 /= blockCount;
                    strength = param->rc.aqStrength * avg_adj;
                    avg_adj = avg_adj - 0.5f * (avg_adj_pow2 - modeTwoConst) / avg_adj;
                    bias_strength = param->rc.aqStrength;
                }
                else
                    strength = param->rc.aqStrength * 1.0397f;

                blockXY = 0;
                for (int blockY = 0; blockY < maxRow; blockY += loopIncr)
                {
                    for (int blockX = 0; blockX < maxCol; blockX += loopIncr)
                    {
                        if (param->rc.aqMode == S265_AQ_AUTO_VARIANCE_BIASED)
                        {
                            qp_adj = curFrame->m_lowres.qpCuTreeOffset[blockXY];
                            qp_adj = strength * (qp_adj - avg_adj) + bias_strength * (1.f - modeTwoConst / (qp_adj * qp_adj));
                        }
                        else if (param->rc.aqMode == S265_AQ_AUTO_VARIANCE)
                        {
                            qp_adj = curFrame->m_lowres.qpCuTreeOffset[blockXY];
                            qp_adj = strength * (qp_adj - avg_adj);
                        }
                        else if (param->rc.aqMode == S265_AQ_EDGE)
                        {
                            inclinedEdge = curFrame->m_lowres.edgeInclined[blockXY];
                            qp_adj = curFrame->m_lowres.qpCuTreeOffset[blockXY];
                            if(inclinedEdge && (qp_adj - avg_adj > 0))
                                qp_adj = ((strength + AQ_EDGE_BIAS) * (qp_adj - avg_adj));
                            else
                                qp_adj = strength * (qp_adj - avg_adj);
                        }
                        else
                        {
                            uint32_t energy = acEnergyCu(curFrame, blockX, blockY, param->internalCsp, param->rc.qgSize);
                            qp_adj = strength * (S265_LOG2(S265_MAX(energy, 1)) - (modeOneConst + 2 * (S265_DEPTH - 8)));
                        }

                        if (param->bHDR10Opt)
                        {
                            uint32_t sum = lumaSumCu(curFrame, blockX, blockY, param->rc.qgSize);
                            uint32_t lumaAvg = sum / (loopIncr * loopIncr);
                            if (lumaAvg < 301)
                                qp_adj += 3;
                            else if (lumaAvg >= 301 && lumaAvg < 367)
                                qp_adj += 2;
                            else if (lumaAvg >= 367 && lumaAvg < 434)
                                qp_adj += 1;
                            else if (lumaAvg >= 501 && lumaAvg < 567)
                                qp_adj -= 1;
                            else if (lumaAvg >= 567 && lumaAvg < 634)
                                qp_adj -= 2;
                            else if (lumaAvg >= 634 && lumaAvg < 701)
                                qp_adj -= 3;
                            else if (lumaAvg >= 701 && lumaAvg < 767)
                                qp_adj -= 4;
                            else if (lumaAvg >= 767 && lumaAvg < 834)
                                qp_adj -= 5;
                            else if (lumaAvg >= 834)
                                qp_adj -= 6;
                        }
                        if (quantOffsets != NULL)
                            qp_adj += quantOffsets[blockXY];
                        curFrame->m_lowres.qpAqOffset[blockXY] = qp_adj;
                        curFrame->m_lowres.qpCuTreeOffset[blockXY] = qp_adj;
                        curFrame->m_lowres.invQscaleFactor[blockXY] = s265_exp2fix8(qp_adj);
                        blockXY++;
                    }
                }
            }
        }

        if (param->rc.qgSize == 8)
        {
            // 一个 8x8的cu 对应4个x4 的block
            for (int cuY = 0; cuY < heightInCU; cuY++)
            {
                for (int cuX = 0; cuX < widthInCU; cuX++)
                {
                    const int cuXY = cuX + cuY * widthInCU;
                    curFrame->m_lowres.invQscaleFactor8x8[cuXY] = (curFrame->m_lowres.invQscaleFactor[cuX * 2 + cuY * widthInCU * 4] +
                        curFrame->m_lowres.invQscaleFactor[cuX * 2 + cuY * widthInCU * 4 + 1] +
                        curFrame->m_lowres.invQscaleFactor[cuX * 2 + cuY * widthInCU * 4 + curFrame->m_lowres.maxBlocksInRowFullRes] +
                        curFrame->m_lowres.invQscaleFactor[cuX * 2 + cuY * widthInCU * 4 + curFrame->m_lowres.maxBlocksInRowFullRes + 1]) / 4;
                }
            }
        }
    }

    if (param->bEnableWeightedPred || param->bEnableWeightedBiPred)
    {
        if (param->rc.bStatRead && param->rc.cuTree && IS_REFERENCED(curFrame))
        {
            for (int blockY = 0; blockY < maxRow; blockY += loopIncr)
                for (int blockX = 0; blockX < maxCol; blockX += loopIncr)
                    acEnergyCu(curFrame, blockX, blockY, param->internalCsp, param->rc.qgSize);
        }

        int hShift = CHROMA_H_SHIFT(param->internalCsp);
        int vShift = CHROMA_V_SHIFT(param->internalCsp);
        maxCol = ((maxCol + 8) >> 4) << 4;
        maxRow = ((maxRow + 8) >> 4) << 4;
        int width[3]  = { maxCol, maxCol >> hShift, maxCol >> hShift };
        int height[3] = { maxRow, maxRow >> vShift, maxRow >> vShift };

        for (int i = 0; i < 3; i++)
        {
            uint64_t sum, ssd;
            sum = curFrame->m_lowres.wp_sum[i];
            ssd = curFrame->m_lowres.wp_ssd[i];
            curFrame->m_lowres.wp_ssd[i] = ssd - (sum * sum + (width[i] * height[i]) / 2) / (width[i] * height[i]);
        }
    }

    if (param->bDynamicRefine || param->bEnableFades)
    {
        uint64_t blockXY = 0, rowVariance = 0;
        curFrame->m_lowres.frameVariance = 0;
        for (int blockY = 0; blockY < maxRow; blockY += loopIncr)
        {
            for (int blockX = 0; blockX < maxCol; blockX += loopIncr)
            {
                curFrame->m_lowres.blockVariance[blockXY] = acEnergyCu(curFrame, blockX, blockY, param->internalCsp, param->rc.qgSize);
                rowVariance += curFrame->m_lowres.blockVariance[blockXY];
                blockXY++;
            }
            curFrame->m_lowres.frameVariance += (rowVariance / maxCol);
        }
        curFrame->m_lowres.frameVariance /= maxRow;
    }
}

void LookaheadTLD::lowresIntraEstimate(Lowres& fenc, uint32_t qgSize)
{
    ALIGN_VAR_32(pixel, prediction[S265_LOWRES_CU_SIZE * S265_LOWRES_CU_SIZE]);
    pixel fencIntra[S265_LOWRES_CU_SIZE * S265_LOWRES_CU_SIZE];
    pixel neighbours[2][S265_LOWRES_CU_SIZE * 4 + 1];
    pixel* samples = neighbours[0], *filtered = neighbours[1];

    const int lookAheadLambda = (int)s265_lambda_tab[S265_LOOKAHEAD_QP];
    const int intraPenalty = 5 * lookAheadLambda;
    const int lowresPenalty = 4; /* fixed CU cost overhead */

    const int cuSize  = S265_LOWRES_CU_SIZE;
    const int cuSize2 = cuSize << 1;
    const int sizeIdx = S265_LOWRES_CU_BITS - 2; // 8x8 对应的index 为 1

    pixelcmp_t satd = primitives.pu[sizeIdx].satd;
    int planar = !!(cuSize >= 8);// 如果 lowre 的cu size >=8 则采用planar 预测 否则不用 这里始终为 1

    int costEst = 0, costEstAq = 0;

    for (int cuY = 0; cuY < heightInCU; cuY++)
    {
        fenc.rowSatds[0][0][cuY] = 0;// 每一行的累加器初始化

        for (int cuX = 0; cuX < widthInCU; cuX++)
        {
            const int cuXY = cuX + cuY * widthInCU;
            const intptr_t pelOffset = cuSize * cuX + cuSize * cuY * fenc.lumaStride;
            pixel *pixCur = fenc.lowresPlane[0] + pelOffset;// 寻址

            /* copy fenc pixels *///拷贝 8x8的orig data 到栈区存储区间
            primitives.cu[sizeIdx].copy_pp(fencIntra, cuSize, pixCur, fenc.lumaStride);

            /* collect reference sample pixels */
            pixCur -= fenc.lumaStride + 1;//指针移动到左上角处
            memcpy(samples, pixCur, (2 * cuSize + 1) * sizeof(pixel)); /* top */ //top上边一行参考像素 1+8+8
            for (int i = 1; i <= 2 * cuSize; i++)
                samples[cuSize2 + i] = pixCur[i * fenc.lumaStride];    /* left */ // 左列参考像素 8 + 8

            primitives.cu[sizeIdx].intra_filter(samples, filtered);// 参考像素滤波 应该可以不用的

            int cost, icost = me.COST_MAX;
            uint32_t ilowmode = 0;

            /* DC and planar */
            primitives.cu[sizeIdx].intra_pred[DC_IDX](prediction, cuSize, samples, 0, cuSize <= 16);
            cost = satd(fencIntra, cuSize, prediction, cuSize);
            COPY2_IF_LT(icost, cost, ilowmode, DC_IDX);

            primitives.cu[sizeIdx].intra_pred[PLANAR_IDX](prediction, cuSize, neighbours[planar], 0, 0);
            cost = satd(fencIntra, cuSize, prediction, cuSize);
            COPY2_IF_LT(icost, cost, ilowmode, PLANAR_IDX);

            /* scan angular predictions */
            int filter, acost = me.COST_MAX;
            uint32_t mode, alowmode = 4;
            for (mode = 5; mode < 35; mode += 5)// 6个大方向
            {
                filter = !!(g_intraFilterFlags[mode] & cuSize);
                primitives.cu[sizeIdx].intra_pred[mode](prediction, cuSize, neighbours[filter], mode, cuSize <= 16);
                cost = satd(fencIntra, cuSize, prediction, cuSize);
                COPY2_IF_LT(acost, cost, alowmode, mode);
                // fast 算法
            }
            for (uint32_t dist = 2; dist >= 1; dist--)//在6中大方向中的某个方向上继续做精细化pred，完了之后再 进一步缩小步长再做一次
            {
                int minusmode = alowmode - dist;
                int plusmode = alowmode + dist;

                mode = minusmode;// 负方向
                filter = !!(g_intraFilterFlags[mode] & cuSize);
                primitives.cu[sizeIdx].intra_pred[mode](prediction, cuSize, neighbours[filter], mode, cuSize <= 16);
                cost = satd(fencIntra, cuSize, prediction, cuSize);
                COPY2_IF_LT(acost, cost, alowmode, mode);

                mode = plusmode;//正方向
                filter = !!(g_intraFilterFlags[mode] & cuSize);
                primitives.cu[sizeIdx].intra_pred[mode](prediction, cuSize, neighbours[filter], mode, cuSize <= 16);
                cost = satd(fencIntra, cuSize, prediction, cuSize);
                COPY2_IF_LT(acost, cost, alowmode, mode);// 一次循环后 决策处 负/正方向上的最优者
                // fast 算法
            }
            COPY2_IF_LT(icost, acost, ilowmode, alowmode);// 最优的cost 与最优的ilowmode 

            icost += intraPenalty + lowresPenalty; /* estimate intra signal cost */ //intra惩罚与低分辨率惩罚

            fenc.lowresCosts[0][0][cuXY] = (uint16_t)(S265_MIN(icost, LOWRES_COST_MASK) | (0 << LOWRES_COST_SHIFT));// 0表示 不用list0/1 作参考
            fenc.intraCost[cuXY] = icost;
            fenc.intraMode[cuXY] = (uint8_t)ilowmode;
            /* do not include edge blocks in the 
            frame cost estimates, they are not very accurate */
            const bool bFrameScoreCU = (cuX > 0 && cuX < widthInCU - 1 &&
                                        cuY > 0 && cuY < heightInCU - 1) || widthInCU <= 2 || heightInCU <= 2;
            int icostAq;
            if (qgSize == 8)
                icostAq = (bFrameScoreCU && fenc.invQscaleFactor) ? ((icost * fenc.invQscaleFactor8x8[cuXY] + 128) >> 8) : icost;
            else
                icostAq = (bFrameScoreCU && fenc.invQscaleFactor) ? ((icost * fenc.invQscaleFactor[cuXY] +128) >> 8) : icost;

            if (bFrameScoreCU)
            {
                costEst += icost;
                costEstAq += icostAq;
            }

            fenc.rowSatds[0][0][cuY] += icostAq;
        }
    }

    fenc.costEst[0][0] = costEst;
    fenc.costEstAq[0][0] = costEstAq;
}

uint32_t LookaheadTLD::weightCostLuma(Lowres& fenc, Lowres& ref, WeightParam& wp)
{
    pixel *src = ref.fpelPlane[0];
    intptr_t stride = fenc.lumaStride;

    if (wp.wtPresent)
    {
        int offset = wp.inputOffset << (S265_DEPTH - 8);
        int scale = wp.inputWeight;
        int denom = wp.log2WeightDenom;
        int round = denom ? 1 << (denom - 1) : 0;
        int correction = IF_INTERNAL_PREC - S265_DEPTH; // intermediate interpolation depth
        int widthHeight = (int)stride;

        primitives.weight_pp(ref.buffer[0], wbuffer[0], stride, widthHeight, paddedLines,
            scale, round << correction, denom + correction, offset);
        src = fenc.weightedRef[fenc.frameNum - ref.frameNum].fpelPlane[0];
    }

    uint32_t cost = 0;
    intptr_t pixoff = 0;
    int mb = 0;

    for (int y = 0; y < fenc.lines; y += 8, pixoff = y * stride)
    {
        for (int x = 0; x < fenc.width; x += 8, mb++, pixoff += 8)
        {
            int satd = primitives.pu[LUMA_8x8].satd(src + pixoff, stride, fenc.fpelPlane[0] + pixoff, stride);
            cost += S265_MIN(satd, fenc.intraCost[mb]);
        }
    }

    return cost;
}

bool LookaheadTLD::allocWeightedRef(Lowres& fenc)
{
    intptr_t planesize = fenc.buffer[1] - fenc.buffer[0];
    paddedLines = (int)(planesize / fenc.lumaStride);

    wbuffer[0] = S265_MALLOC(pixel, 4 * planesize);
    if (wbuffer[0])
    {
        wbuffer[1] = wbuffer[0] + planesize;
        wbuffer[2] = wbuffer[1] + planesize;
        wbuffer[3] = wbuffer[2] + planesize;
    }
    else
        return false;

    return true;
}

void LookaheadTLD::weightsAnalyse(Lowres& fenc, Lowres& ref)
{
    static const float epsilon = 1.f / 128.f;
    int deltaIndex = fenc.frameNum - ref.frameNum;

    WeightParam wp;
    wp.wtPresent = 0;

    if (!wbuffer[0])
    {
        if (!allocWeightedRef(fenc))
            return;
    }

    ReferencePlanes& weightedRef = fenc.weightedRef[deltaIndex];
    intptr_t padoffset = fenc.lowresPlane[0] - fenc.buffer[0];
    for (int i = 0; i < 4; i++)
        weightedRef.lowresPlane[i] = wbuffer[i] + padoffset;

    weightedRef.fpelPlane[0] = weightedRef.lowresPlane[0];
    weightedRef.lumaStride = fenc.lumaStride;
    weightedRef.isLowres = true;
    weightedRef.isWeighted = false;
    weightedRef.isHMELowres = ref.bEnableHME;

    /* epsilon is chosen to require at least a numerator of 127 (with denominator = 128) */
    float guessScale, fencMean, refMean;
    s265_emms();
    if (fenc.wp_ssd[0] && ref.wp_ssd[0])
        guessScale = sqrtf((float)fenc.wp_ssd[0] / ref.wp_ssd[0]);
    else
        guessScale = 1.0f;
    fencMean = (float)fenc.wp_sum[0] / (fenc.lines * fenc.width) / (1 << (S265_DEPTH - 8));
    refMean = (float)ref.wp_sum[0] / (fenc.lines * fenc.width) / (1 << (S265_DEPTH - 8));

    /* Early termination */
    if (fabsf(refMean - fencMean) < 0.5f && fabsf(1.f - guessScale) < epsilon)
        return;

    int minoff = 0, minscale, mindenom;
    unsigned int minscore = 0, origscore = 1;
    int found = 0;

    wp.setFromWeightAndOffset((int)(guessScale * 128 + 0.5f), 0, 7, true);
    mindenom = wp.log2WeightDenom;
    minscale = wp.inputWeight;

    origscore = minscore = weightCostLuma(fenc, ref, wp);

    if (!minscore)
        return;

    unsigned int s = 0;
    int curScale = minscale;
    int curOffset = (int)(fencMean - refMean * curScale / (1 << mindenom) + 0.5f);
    if (curOffset < -128 || curOffset > 127)
    {
        /* Rescale considering the constraints on curOffset. We do it in this order
        * because scale has a much wider range than offset (because of denom), so
        * it should almost never need to be clamped. */
        curOffset = s265_clip3(-128, 127, curOffset);
        curScale = (int)((1 << mindenom) * (fencMean - curOffset) / refMean + 0.5f);
        curScale = s265_clip3(0, 127, curScale);
    }
    SET_WEIGHT(wp, true, curScale, mindenom, curOffset);
    s = weightCostLuma(fenc, ref, wp);
    COPY4_IF_LT(minscore, s, minscale, curScale, minoff, curOffset, found, 1);

    /* Use a smaller denominator if possible */
    if (mindenom > 0 && !(minscale & 1))
    {
        unsigned long idx;
        CTZ(idx, minscale);
        int shift = S265_MIN((int)idx, mindenom);
        mindenom -= shift;
        minscale >>= shift;
    }

    if (!found || (minscale == 1 << mindenom && minoff == 0) || (float)minscore / origscore > 0.998f)
        return;
    else
    {
        SET_WEIGHT(wp, true, minscale, mindenom, minoff);

        // set weighted delta cost
        fenc.weightedCostDelta[deltaIndex] = minscore / origscore;

        int offset = wp.inputOffset << (S265_DEPTH - 8);
        int scale = wp.inputWeight;
        int denom = wp.log2WeightDenom;
        int round = denom ? 1 << (denom - 1) : 0;
        int correction = IF_INTERNAL_PREC - S265_DEPTH; // intermediate interpolation depth
        intptr_t stride = ref.lumaStride;
        int widthHeight = (int)stride;

        for (int i = 0; i < 4; i++)
            primitives.weight_pp(ref.buffer[i], wbuffer[i], stride, widthHeight, paddedLines,
            scale, round << correction, denom + correction, offset);

        weightedRef.isWeighted = true;
    }
}

#define ADAPTIVEGOP_RES_THRES (416 * 240)

Lookahead::Lookahead(s265_param *param, ThreadPool* pool)
{
    m_param = param;
    m_pool  = pool;

    m_lastNonB = NULL;
    m_isSceneTransition = false;
    m_scratch  = NULL;
    m_tld      = NULL;
    m_filled   = false;
    m_outputSignalRequired = false;
    m_isActive = true;
    m_inputCount = 0;
    m_extendGopBoundary = false;
    m_8x8Height = ((m_param->sourceHeight / 2) + S265_LOWRES_CU_SIZE - 1) >> S265_LOWRES_CU_BITS;
    m_8x8Width = ((m_param->sourceWidth / 2) + S265_LOWRES_CU_SIZE - 1) >> S265_LOWRES_CU_BITS;
    m_4x4Height = ((m_param->sourceHeight / 4) + S265_LOWRES_CU_SIZE - 1) >> S265_LOWRES_CU_BITS;
    m_4x4Width = ((m_param->sourceWidth / 4) + S265_LOWRES_CU_SIZE - 1) >> S265_LOWRES_CU_BITS;
    m_cuCount = m_8x8Width * m_8x8Height;
    m_8x8Blocks = m_8x8Width > 2 && m_8x8Height > 2 ? (m_cuCount + 4 - 2 * (m_8x8Width + m_8x8Height)) : m_cuCount;
    m_isFadeIn = false;
    m_fadeCount = 0;
    m_fadeStart = -1;

    /* thres init */
    int32_t adapt_gop_cost_ratio_thres[] = { 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2 };
    double resolutionbase = sqrt( (double)(param->sourceHeight >> 1) * (param->sourceWidth >> 1) / ADAPTIVEGOP_RES_THRES );
    i_large_mv_thres = 16 * resolutionbase; // 416,240 => 8; 1280x720 => 24; 1920x1080 =>  36; 3840x2160 => 68;
    i_large_mv_thres2 =
        16 * ( 1 + adapt_gop_cost_ratio_thres[param->subpelRefine] + ((param->sourceHeight + param->sourceWidth) > 2500 ? 1 : 0) ) / 10.0 *
        ( (param->sourceHeight / 2 + param->sourceWidth / 2) / (416.0 + 240.0) );
    i_large_mv_thres3 =
        16 * ( 1 + (adapt_gop_cost_ratio_thres[param->subpelRefine] + ((param->sourceHeight + param->sourceWidth) > 2500 ? 1 : 0)) / 10.0 ) *
        ( (param->sourceHeight / 2 + param->sourceWidth / 2) / (416.0 + 240.0) );

    /* Allow the strength to be adjusted via qcompress, since the two concepts
     * are very similar. */
    m_cuTreeStrength = (m_param->rc.hevcAq ? 6.0 : 5.0) * (1.0 - m_param->rc.qCompress);

    m_lastKeyframe = -m_param->keyframeMax;
    m_sliceTypeBusy = false;
    m_fullQueueSize = S265_MAX(1, m_param->lookaheadDepth);
    m_bAdaptiveQuant = m_param->rc.aqMode ||
                       m_param->bEnableWeightedPred ||
                       m_param->bEnableWeightedBiPred ||
                       m_param->bAQMotion ||
                       m_param->rc.hevcAq;

    /* If we have a thread pool and are using --b-adapt 2, it is generally
     * preferable to perform all motion searches for each lowres frame in large
     * batched; this will create one job per --bframe per lowres frame, and
     * these jobs are performed by workers bonded to the thread running
     * slicetypeDecide() */
    m_bBatchMotionSearch = m_pool && m_param->bFrameAdaptive == S265_B_ADAPT_TRELLIS;

    /* It is also beneficial to pre-calculate all possible frame cost estimates
     * using worker threads bonded to the worker thread running
     * slicetypeDecide(). This creates bframes * bframes jobs which take less
     * time than the motion search batches but there are many of them. This may
     * do much unnecessary work, some frame cost estimates are not needed, so if
     * the thread pool is small we disable this feature after the initial burst
     * of work */
    m_bBatchFrameCosts = m_bBatchMotionSearch;

    if (m_param->lookaheadSlices && !m_pool)
    {
        s265_log(param, S265_LOG_WARNING, "No pools found; disabling lookahead-slices\n");
        m_param->lookaheadSlices = 0;
    }

    if (m_param->lookaheadSlices && (m_param->sourceHeight < 720))
    {
        s265_log(param, S265_LOG_WARNING, "Source height < 720p; disabling lookahead-slices\n");
        m_param->lookaheadSlices = 0;
    }

    if (m_param->lookaheadSlices > 1)
    {
        m_numRowsPerSlice = m_8x8Height / m_param->lookaheadSlices;//每个slice 占有多少行8x8
        m_numRowsPerSlice = S265_MAX(m_numRowsPerSlice, 10);            // at least 10 rows per slice
        m_numRowsPerSlice = S265_MIN(m_numRowsPerSlice, m_8x8Height);   // but no more than the full picture
        m_numCoopSlices = m_8x8Height / m_numRowsPerSlice;
        m_param->lookaheadSlices = m_numCoopSlices; //实际的条带数         // report actual final slice count
    }
    else
    {
        m_numRowsPerSlice = m_8x8Height;
        m_numCoopSlices = 1;
    }
    if (param->gopLookahead && (param->gopLookahead > (param->lookaheadDepth - param->bframes - 2)))
    {
        param->gopLookahead = S265_MAX(0, param->lookaheadDepth - param->bframes - 2);
        s265_log(param, S265_LOG_WARNING, "Gop-lookahead cannot be greater than (rc-lookahead - length of the mini-gop); Clipping gop-lookahead to %d\n", param->gopLookahead);
    }
#if DETAILED_CU_STATS
    m_slicetypeDecideElapsedTime = 0;
    m_preLookaheadElapsedTime = 0;
    m_countSlicetypeDecide = 0;
    m_countPreLookahead = 0;
#endif

    memset(m_histogram, 0, sizeof(m_histogram));
}

#if DETAILED_CU_STATS
void Lookahead::getWorkerStats(int64_t& batchElapsedTime, uint64_t& batchCount, int64_t& coopSliceElapsedTime, uint64_t& coopSliceCount)
{
    batchElapsedTime = coopSliceElapsedTime = 0;
    coopSliceCount = batchCount = 0;
    int tldCount = m_pool ? m_pool->m_numWorkers : 1;
    for (int i = 0; i < tldCount; i++)
    {
        batchElapsedTime += m_tld[i].batchElapsedTime;
        coopSliceElapsedTime += m_tld[i].coopSliceElapsedTime;
        batchCount += m_tld[i].countBatches;
        coopSliceCount += m_tld[i].countCoopSlices;
    }
}
#endif

bool Lookahead::create()
{
    int numTLD = 1 + (m_pool ? m_pool->m_numWorkers : 0);// 注意threadlocaldata 在使用了线程池下，比线程池里面的线程个数要多1
    m_tld = new LookaheadTLD[numTLD];
    for (int i = 0; i < numTLD; i++)
    {
        m_tld[i].init(m_8x8Width, m_8x8Height, m_8x8Blocks);
        m_tld[i].setUsePskip(m_param->rc.costCalPskip);
    }
    m_scratch = S265_MALLOC(int, m_tld[0].widthInCU);

    return m_tld && m_scratch;
}

void Lookahead::stopJobs()
{
    if (m_pool && !m_inputQueue.empty())
    {
        m_inputLock.acquire();
        m_isActive = false;
        bool wait = m_outputSignalRequired = m_sliceTypeBusy;
        m_inputLock.release();

        if (wait)
            m_outputSignal.wait();
    }
    if (m_pool && m_param->lookaheadThreads > 0)
    {
        for (int i = 0; i < m_numPools; i++)
            m_pool[i].stopWorkers();
    }
}
void Lookahead::destroy()
{
    // these two queues will be empty unless the encode was aborted
    while (!m_inputQueue.empty())
    {
        Frame* curFrame = m_inputQueue.popFront();
        curFrame->destroy();
        delete curFrame;
    }

    while (!m_outputQueue.empty())
    {
        Frame* curFrame = m_outputQueue.popFront();
        curFrame->destroy();
        delete curFrame;
    }

    S265_FREE(m_scratch);
    delete [] m_tld;
    if (m_param->lookaheadThreads > 0)
        delete [] m_pool;
}
/* The synchronization of slicetypeDecide is managed here.  The findJob() method
 * polls the occupancy of the input queue. If the queue is
 * full, it will run slicetypeDecide() and output a mini-gop of frames to the
 * output queue. If the flush() method has been called (implying no new pictures
 * will be received) then the input queue is considered full if it has even one
 * picture left. getDecidedPicture() removes pictures from the output queue and
 * only blocks as a last resort. It does not start removing pictures until
 * m_filled is true, which occurs after *more than* the lookahead depth of
 * pictures have been input so slicetypeDecide() should have started prior to
 * output pictures being withdrawn. The first slicetypeDecide() will obviously
 * still require a blocking wait, but after this slicetypeDecide() will maintain
 * its lead over the encoder (because one picture is added to the input queue
 * each time one is removed from the output) and decides slice types of pictures
 * just ahead of when the encoder needs them */

/* Called by API thread */
void Lookahead::addPicture(Frame& curFrame, int sliceType)
{
    // 首先检查inputQueue 如果满足帧数要求了会触发slicetypedecision
    checkLookaheadQueue(m_inputCount); // 这里应该放到 addPicture(curFrame);后,添加完后立马检查并触发
    curFrame.m_lowres.sliceType = sliceType;
    addPicture(curFrame);//否则添加到 m_inputQueue
}

void Lookahead::addPicture(Frame& curFrame)
{
    m_inputLock.acquire();
    m_inputQueue.pushBack(curFrame);
    m_inputLock.release();
    m_inputCount++;
}

void Lookahead::checkLookaheadQueue(int &frameCnt)
{
    /* determine if the lookahead is (over) filled enough for frames to begin to
     * be consumed by frame encoders */
    if (!m_filled)
    {
        if (!m_param->bframes & !m_param->lookaheadDepth)
            m_filled = true; /* zero-latency */
        else if (frameCnt >= m_param->lookaheadDepth + 2 + m_param->bframes)
            m_filled = true; /* full capacity plus mini-gop lag */
    }

    // 以下代码仅仅 提前 启动lookahead线程池里面的worker进入slicetype 分析决策
    // 不需要下面的代码 也可以的

    m_inputLock.acquire();
   // 首次满足lookahead分析需要的帧数时,将从这里唤醒lookahead线程池里面的worker线程触发（WorkerThread::threadMain()函数中的m_wakeEvent.wait()）
   // 从而进入wihle循环 执行 m_curJobProvider->findJob(m_id);(通过虚函数执行 Lookahead::findJob)
   // 但是Lookahead::findJob 中需要首先获取 m_inputLock.acquire();
   // 而 显然 该m_inputLock已经被当前PassEncoder线程 占有
   // 故而Lookahead::findJob会继续阻塞
    if (m_pool && m_inputQueue.size() >= m_fullQueueSize)
        tryWakeOne();//--> Lookahead::findJob
    m_inputLock.release();
}

/* Called by API thread */
void Lookahead::flush()
{
    /* force slicetypeDecide to run until the input queue is empty */
    m_fullQueueSize = 1;
    m_filled = true;
}

void Lookahead::setLookaheadQueue()
{
    m_filled = false;
    m_fullQueueSize = S265_MAX(1, m_param->lookaheadDepth);
}

void Lookahead::findJob(int /*workerThreadID*/)
{
    bool doDecide;

    m_inputLock.acquire();
    if (m_inputQueue.size() >= m_fullQueueSize && !m_sliceTypeBusy && m_isActive)
        doDecide = m_sliceTypeBusy = true;
    else
        doDecide = m_helpWanted = false;
    m_inputLock.release();

    if (!doDecide)
        return;

    ProfileLookaheadTime(m_slicetypeDecideElapsedTime, m_countSlicetypeDecide);
    ProfileScopeEvent(slicetypeDecideEV);

    slicetypeDecide();

    m_inputLock.acquire();
    if (m_outputSignalRequired)
    {
        m_outputSignal.trigger();
        m_outputSignalRequired = false;
    }
    m_sliceTypeBusy = false;
    m_inputLock.release();
}

/* Called by API thread */
Frame* Lookahead::getDecidedPicture()
{
    if (m_filled)
    {// lookahead 中有足够的帧了有帧才尝试取
        m_outputLock.acquire();
        Frame *out = m_outputQueue.popFront();
        m_outputLock.release();
        //outputQueue 中还有 直接拿走
        if (out)
        {
            m_inputCount--;
            return out;
        }
        //output 队列中没有了,需要调用帧类型决策来讲决策好的帧推到 outputQeue
        findJob(-1); /* run slicetypeDecide() if necessary */

        m_inputLock.acquire();
        bool wait = m_outputSignalRequired = m_sliceTypeBusy;
        m_inputLock.release();

        if (wait)
            m_outputSignal.wait();

        out = m_outputQueue.popFront();
        if (out)
            m_inputCount--;
        return out;
    }
    else
        return NULL;
}

/* Called by rate-control to calculate the estimated SATD cost for a given
 * picture.  It assumes dpb->prepareEncode() has already been called for the
 * picture and all the references are established */
void Lookahead::getEstimatedPictureCost(Frame *curFrame)
{
    Lowres *frames[S265_LOOKAHEAD_MAX];

    // POC distances to each reference
    Slice *slice = curFrame->m_encData->m_slice;
    int p0 = 0, p1, b;
    int poc = slice->m_poc;
    int l0poc = slice->m_rps.numberOfNegativePictures ? slice->m_refPOCList[0][0] : -1;
    int l1poc = slice->m_refPOCList[1][0];

    switch (slice->m_sliceType)
    {
    case I_SLICE:
        frames[p0] = &curFrame->m_lowres;
        b = p1 = 0;
        break;

    case P_SLICE:
        b = p1 = poc - l0poc;
        frames[p0] = &slice->m_refFrameList[0][0]->m_lowres;
        frames[b] = &curFrame->m_lowres;
        break;

    case B_SLICE:
        if (l0poc >= 0)
        {
            b = poc - l0poc;
            p1 = b + l1poc - poc;
            frames[p0] = &slice->m_refFrameList[0][0]->m_lowres;
            frames[b] = &curFrame->m_lowres;
            frames[p1] = &slice->m_refFrameList[1][0]->m_lowres;
        }
        else 
        {
            p0 = b = 0;
            p1 = b + l1poc - poc;
            frames[p0] = frames[b] = &curFrame->m_lowres;
            frames[p1] = &slice->m_refFrameList[1][0]->m_lowres;
        }
        
        break;

    default:
        return;
    }
    // 注意 assuming 需要的cost 在lookahead 阶段已经计算过了
    S265_CHECK(curFrame->m_lowres.costEst[b - p0][p1 - b] > 0, "Slice cost not estimated\n")

    if (m_param->rc.cuTree && !m_param->rc.bStatRead)
        /* update row satds based on cutree offsets */
        curFrame->m_lowres.satdCost = frameCostRecalculate(frames, p0, p1, b);
    else
    {
        if (m_param->rc.aqMode)
            curFrame->m_lowres.satdCost = curFrame->m_lowres.costEstAq[b - p0][p1 - b];
        else
            curFrame->m_lowres.satdCost = curFrame->m_lowres.costEst[b - p0][p1 - b];
    }
    // VBV 时需要计算每个 ctu 行的cost
    if (m_param->rc.vbvBufferSize && m_param->rc.vbvMaxBitrate)
    {
        /* aggregate lowres row satds to CTU resolution */
        curFrame->m_lowres.lowresCostForRc = curFrame->m_lowres.lowresCosts[b - p0][p1 - b];
        uint32_t lowresRow = 0, lowresCol = 0, lowresCuIdx = 0, sum = 0, intraSum = 0;
        uint32_t scale = m_param->maxCUSize / (2 * S265_LOWRES_CU_SIZE);// 每个 64x64 的ctu 覆盖4个 16x16（对应lowre上8x8的单位）的高度与宽短
        uint32_t numCuInHeight = (m_param->sourceHeight + m_param->maxCUSize - 1) / m_param->maxCUSize;
        uint32_t widthInLowresCu = (uint32_t)m_8x8Width, heightInLowresCu = (uint32_t)m_8x8Height;
        double *qp_offset = 0;
        /* Factor in qpoffsets based on Aq/Cutree in CU costs */
        if (m_param->rc.aqMode || m_param->bAQMotion)
            qp_offset = (frames[b]->sliceType == S265_TYPE_B || !m_param->rc.cuTree) ? frames[b]->qpAqOffset : frames[b]->qpCuTreeOffset;

        for (uint32_t row = 0; row < numCuInHeight; row++) // ctu 高度
        {
            lowresRow = row * scale; // ctu row index 乘以4 转换为 lowresRow index : 0   4   8  12 
            for (uint32_t cnt = 0; cnt < scale && lowresRow < heightInLowresCu; lowresRow++, cnt++)// 每个ctu高度4个lowrescu行
            {
                sum = 0; intraSum = 0;
                int diff = 0;
                lowresCuIdx = lowresRow * widthInLowresCu;// lowresRow index对应ROw 行的首个lowrescu 的index
                for (lowresCol = 0; lowresCol < widthInLowresCu; lowresCol++, lowresCuIdx++)//每个lowrescu 行有widthInLowresCu 列
                {
                    // 取出每个lowresCuindx 的 cost
                    uint16_t lowresCuCost = curFrame->m_lowres.lowresCostForRc[lowresCuIdx] & LOWRES_COST_MASK;
                    if (qp_offset)
                    {
                        double qpOffset;
                        if (m_param->rc.qgSize == 8)
                            qpOffset = (qp_offset[lowresCol * 2 + lowresRow * widthInLowresCu * 4] +
                            qp_offset[lowresCol * 2 + lowresRow * widthInLowresCu * 4 + 1] +
                            qp_offset[lowresCol * 2 + lowresRow * widthInLowresCu * 4 + curFrame->m_lowres.maxBlocksInRowFullRes] +
                            qp_offset[lowresCol * 2 + lowresRow * widthInLowresCu * 4 + curFrame->m_lowres.maxBlocksInRowFullRes + 1]) / 4;
                        else
                            qpOffset = qp_offset[lowresCuIdx];
                        lowresCuCost = (uint16_t)((lowresCuCost * s265_exp2fix8(qpOffset) + 128) >> 8);//cost 考率qpdelta 的缩放影响
                        int32_t intraCuCost = curFrame->m_lowres.intraCost[lowresCuIdx];
                        curFrame->m_lowres.intraCost[lowresCuIdx] = (intraCuCost * s265_exp2fix8(qpOffset) + 128) >> 8;// intra cost 考虑qpdelta 的缩放影响
                    }
                    if (m_param->bIntraRefresh && slice->m_sliceType == S265_TYPE_P)
                        for (uint32_t x = curFrame->m_encData->m_pir.pirStartCol; x <= curFrame->m_encData->m_pir.pirEndCol; x++)
                            diff += curFrame->m_lowres.intraCost[lowresCuIdx] - lowresCuCost;
                    curFrame->m_lowres.lowresCostForRc[lowresCuIdx] = lowresCuCost;// 覆盖掉原有的lowresCosts
                    sum += lowresCuCost;// 一个lowrescu行 cost 累加
                    intraSum += curFrame->m_lowres.intraCost[lowresCuIdx];
                }
                curFrame->m_encData->m_rowStat[row].satdForVbv += sum;
                curFrame->m_encData->m_rowStat[row].satdForVbv += diff;
                curFrame->m_encData->m_rowStat[row].intraSatdForVbv += intraSum;
            }
        }
    }
}

void PreLookaheadGroup::processTasks(int workerThreadID)
{
    if (workerThreadID < 0)
        workerThreadID = m_lookahead.m_pool ? m_lookahead.m_pool->m_numWorkers : 0;
    // 当 workerthreadID 为 -1 时，表示在pool 模式下使用额外的lookaheadTLD进行preLookahead 分析
    LookaheadTLD& tld = m_lookahead.m_tld[workerThreadID];

    m_lock.acquire();
    while (m_jobAcquired < m_jobTotal)
    {
        Frame* preFrame = m_preframes[m_jobAcquired++];// 一帧一帧 /一个一个任务取出来
        ProfileLookaheadTime(m_lookahead.m_preLookaheadElapsedTime, m_lookahead.m_countPreLookahead);
        ProfileScopeEvent(prelookahead);
        m_lock.release();
        preFrame->m_lowres.init(preFrame->m_fencPic, preFrame->m_poc);// 做一些初始化的工作
        if (m_lookahead.m_bAdaptiveQuant)
            tld.calcAdaptiveQuantFrame(preFrame, m_lookahead.m_param);
        tld.lowresIntraEstimate(preFrame->m_lowres, m_lookahead.m_param->rc.qgSize);
        preFrame->m_lowresInit = true;

        m_lock.acquire();
    }
    m_lock.release();
}


static void set_gop_info_internal(  Lowres *cur_frame, int32_t *gop_id, int32_t gop_size,
    int32_t depth )
{
    static const int32_t max_depth[] = { 0, 1, 1, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4 };// gop_size max 17
    //int32_t max_depth[] = { 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4 };// gop_size max 17  todo: BD_rate_test
    //assert( gop_size <= 17 );
    cur_frame->i_gop_size = gop_size;
    cur_frame->i_max_depth = max_depth[gop_size];
    // if( s264_is_key_frame( h, cur_frame ) || (cur_frame->b_scenecut && IS_S264_TYPE_I( cur_frame->i_type )) )
    // {
    //     cur_frame->i_gop_id = 0;
    //     cur_frame->i_level = 0;
    //     cur_frame->i_bref = 1;
    //     cur_frame->i_temporal_id = 0;
    //     if( cur_frame->i_type != S264_TYPE_I && cur_frame->i_type != S264_TYPE_IDR )
    //        cur_frame->i_type = h->param.b_open_gop ? S264_TYPE_I : S264_TYPE_IDR;

    //ZY TODO:目前先看一般情况,应该还有其他的情况
    //此时应该是没有AUTO，但是谨慎起见，判断一下
     if(!IS_S265_TYPE_B(cur_frame->sliceType) && cur_frame->sliceType != S265_TYPE_AUTO)
     {
         cur_frame->i_bref = 1;
         cur_frame->i_temporal_id = 0;
         cur_frame->i_gop_id = *gop_id;
         cur_frame->i_level = 0;
     }
     else
     {
        cur_frame->i_level = depth;
        cur_frame->i_bref = (depth < cur_frame->i_max_depth);// 小于max_depth 时 都用作 ref
        cur_frame->i_gop_id = *gop_id;
//      cur_frame->i_temporal_id = S264_MAX(0, depth - 1);// note: this seems to be also ok    todo: BD_rate_test
        // 当当前minigop B帧个数> 3时，最中间的Bref 的 i_temporal_id 设置为0(与i/p 帧属 同一个参考重要程度)
        // 否则 当当前的minigop 较小,(B帧个数<=3时) 最中间的Bref 的 i_temporal_id 从 1开始计算
        cur_frame->i_temporal_id = cur_frame->i_gop_size > 4 ? S265_MAX(0, depth - 1): S265_MAX(0, depth );
//      cur_frame->i_temporal_id = S264_MAX(0, depth);//tested   todo:BD_rate_test
        cur_frame->sliceType = cur_frame->i_bref ? S265_TYPE_BREF : S265_TYPE_B;
        // if( cur_frame->i_level == 0 )
        // {
        //     cur_frame->i_type = S264_TYPE_P;
        // }
    }
    ( *gop_id )++;
}

static void set_gop_info_random_access( Lowres **frames, int32_t *gop_id, int32_t left,
    int32_t right, int32_t gop_size, int32_t depth )
{
    if( left > right )
    {
        return;
    }

    if( depth < 5 )
    {
        int mid = (left + right) / 2;
        Lowres *cur_frame = frames[mid];
        set_gop_info_internal( cur_frame, gop_id, gop_size, depth);
        set_gop_info_random_access( frames, gop_id, left, mid - 1, gop_size, depth + 1);
        set_gop_info_random_access( frames, gop_id, mid + 1, right, gop_size, depth + 1);
    }
    else // this code will never be hit
    {
        for( int32_t idx = left; idx <= right; idx++ )
        {
            Lowres *cur_frame = frames[idx];
            set_gop_info_internal( cur_frame, gop_id, gop_size, depth );
        }
    }
}

/* called by API thread or worker thread with inputQueueLock acquired */
void Lookahead::slicetypeDecide()
{
    PreLookaheadGroup pre(*this);
    Lowres* frames[S265_LOOKAHEAD_MAX + S265_BFRAME_MAX + 4];
    //Frame*  list[S265_BFRAME_MAX + 4];
    Frame*  list[S265_LOOKAHEAD_MAX + S265_BFRAME_MAX + 4];
    memset(frames, 0, sizeof(frames));
    memset(list, 0, sizeof(list));
    int maxSearch = S265_MIN(m_param->lookaheadDepth, S265_LOOKAHEAD_MAX);
    maxSearch = S265_MAX(1, maxSearch);

    {
        ScopedLock lock(m_inputLock);

        Frame *curFrame = m_inputQueue.first();
        int j;
        for (j = 0; j < maxSearch; j++)
        //for (j = 0; j < m_param->bframes + 2; j++)
        {
            if (!curFrame) break;
            list[j] = curFrame;
            curFrame = curFrame->m_next;
        }

        curFrame = m_inputQueue.first();
        frames[0] = m_lastNonB;
        //构建frames数组 将需要分析的帧放到frames[1...maxSearch]
        for (j = 0; j < maxSearch; j++)//最多分析这么多帧
        {
            if (!curFrame) break;//遇到null了，后面没有帧了
            frames[j + 1] = &curFrame->m_lowres;

            if (!curFrame->m_lowresInit)//统计需要进行 pre-analysis的帧数量
                pre.m_preframes[pre.m_jobTotal++] = curFrame;

            curFrame = curFrame->m_next;
        }

        maxSearch = j;
    }

    /* perform pre-analysis on frames which need it, using a bonded task group */
    if (pre.m_jobTotal)
    {
        /* 如果有线程池，这里唤醒可用的线程去执行m_jobTotal个任务，在可用的线程
           小于m_jobTotal时，会有一些首先完成分配给他的任务的线程再次取得任务进一步执行
        */
        if (m_pool)
            pre.tryBondPeers(*m_pool, pre.m_jobTotal);//唤醒最多不超过m_jobTotal 个sleep 线程执行processTasks() 启动pre-analysis 含有 lowres_init 和aq and intra_cost_estimate
        pre.processTasks(-1);// 有线程池时, 可以不需要这里再调用processTasks了（当然调用了也没有问题，如果任务已经被全部领走了，这里会直接返回）
        pre.waitForExit();// 等待所有的线程完成各自的processTasks
    }

    if(m_param->bEnableFades)
    {
        int j, endIndex = 0, length = S265_BFRAME_MAX + 4;
        for (j = 0; j < length; j++)
            m_frameVariance[j] = -1;
        for (j = 0; list[j] != NULL; j++)
            m_frameVariance[list[j]->m_poc % length] = list[j]->m_lowres.frameVariance;
        for (int k = list[0]->m_poc % length; k <= list[j - 1]->m_poc % length; k++)
        {
            if (m_frameVariance[k]  == -1)
                break;
            if((k > 0 && m_frameVariance[k] >= m_frameVariance[k - 1]) || 
                (k == 0 && m_frameVariance[k] >= m_frameVariance[length - 1]))
            {
                m_isFadeIn = true;
                if (m_fadeCount == 0 && m_fadeStart == -1)
                {
                    for(int temp = list[0]->m_poc; temp <= list[j - 1]->m_poc; temp++)
                        if (k == temp % length) {
                            m_fadeStart = temp ? temp - 1 : 0;
                            break;
                        }
                }
                m_fadeCount = list[endIndex]->m_poc > m_fadeStart ? list[endIndex]->m_poc - m_fadeStart : 0;
                endIndex++;
            }
            else
            {
                if (m_isFadeIn && m_fadeCount >= m_param->fpsNum / m_param->fpsDenom)
                {
                    for (int temp = 0; list[temp] != NULL; temp++)
                    {
                        if (list[temp]->m_poc == m_fadeStart + (int)m_fadeCount)
                        {
                            list[temp]->m_lowres.bIsFadeEnd = true;
                            break;
                        }
                    }
                }
                m_isFadeIn = false;
                m_fadeCount = 0;
                m_fadeStart = -1;
            }
            if (k == length - 1)
                k = -1;
        }
    }

    if (m_lastNonB &&
        ((m_param->bFrameAdaptive && m_param->bframes) ||
         m_param->rc.cuTree || m_param->scenecutThreshold || m_param->bHistBasedSceneCut ||
         (m_param->lookaheadDepth && m_param->rc.vbvBufferSize)))
    {
        if(!m_param->rc.bStatRead)
            slicetypeAnalyse(frames, false);//真正的帧类型分析
        if (m_param->bliveVBV2pass)
        {
            int numFrames;
            for (numFrames = 0; numFrames < maxSearch; numFrames++)
            {
                Lowres *fenc = frames[numFrames + 1];
                if (!fenc)
                    break;
            }
            vbvLookahead(frames, numFrames, false);
        }
    }

    int bframes, brefs;
    bool isClosedGopRadl = m_param->radl && (m_param->keyframeMax != m_param->keyframeMin);
    for (bframes = 0, brefs = 0;; bframes++)
    {
        Lowres& frm = list[bframes]->m_lowres;

        if (frm.sliceType == S265_TYPE_BREF && !m_param->bBPyramid && brefs == m_param->bBPyramid)
        {
            frm.sliceType = S265_TYPE_B;
            s265_log(m_param, S265_LOG_WARNING, "B-ref at frame %d incompatible with B-pyramid\n",
                frm.frameNum);
        }

        /* pyramid with multiple B-refs needs a big enough dpb that the preceding P-frame stays available.
            * smaller dpb could be supported by smart enough use of mmco, but it's easier just to forbid it. */
        else if (frm.sliceType == S265_TYPE_BREF && m_param->bBPyramid == S265_B_PYRAMID_STRICT && brefs &&
            m_param->maxNumReferences <= (brefs + 3))
        {
            frm.sliceType = S265_TYPE_B;
            s265_log(m_param, S265_LOG_WARNING, "B-ref at frame %d incompatible with B-pyramid and %d reference frames\n",
                frm.sliceType, m_param->maxNumReferences);
        }
        if (((!m_param->bIntraRefresh || frm.frameNum == 0) && frm.frameNum - m_lastKeyframe >= m_param->keyframeMax &&
            (!m_extendGopBoundary || frm.frameNum - m_lastKeyframe >= m_param->keyframeMax + m_param->gopLookahead)) ||
            (frm.frameNum == (m_param->chunkStart - 1)) || (frm.frameNum == m_param->chunkEnd))
        {
            if (frm.sliceType == S265_TYPE_AUTO || frm.sliceType == S265_TYPE_I)
                frm.sliceType = m_param->bOpenGOP && m_lastKeyframe >= 0 ? S265_TYPE_I : S265_TYPE_IDR;
            bool warn = frm.sliceType != S265_TYPE_IDR;
            if (warn && m_param->bOpenGOP)
                warn &= frm.sliceType != S265_TYPE_I;
            if (warn)
            {
                s265_log(m_param, S265_LOG_WARNING, "specified frame type (%d) at %d is not compatible with keyframe interval\n",
                    frm.sliceType, frm.frameNum);
                frm.sliceType = m_param->bOpenGOP && m_lastKeyframe >= 0 ? S265_TYPE_I : S265_TYPE_IDR;
            }
        }
        if (frm.bIsFadeEnd){
            frm.sliceType = m_param->bOpenGOP && m_lastKeyframe >= 0 ? S265_TYPE_I : S265_TYPE_IDR;
        }
        if (m_param->bResetZoneConfig)
        {
            for (int i = 0; i < m_param->rc.zonefileCount; i++)
            {
                int curZoneStart = m_param->rc.zones[i].startFrame;
                curZoneStart += curZoneStart ? m_param->rc.zones[i].zoneParam->radl : 0;
                if (curZoneStart == frm.frameNum)
                    frm.sliceType = S265_TYPE_IDR;
            }
        }
        if ((frm.sliceType == S265_TYPE_I && frm.frameNum - m_lastKeyframe >= m_param->keyframeMin) || (frm.frameNum == (m_param->chunkStart - 1)) || (frm.frameNum == m_param->chunkEnd))
        {
            if (m_param->bOpenGOP)
            {
                m_lastKeyframe = frm.frameNum;
                frm.bKeyframe = true;
            }
            else
                frm.sliceType = S265_TYPE_IDR;
        }
        if (frm.sliceType == S265_TYPE_IDR && frm.bScenecut && isClosedGopRadl)
        {
            if (!m_param->bHistBasedSceneCut || (m_param->bHistBasedSceneCut && frm.m_bIsHardScenecut))
            {
                for (int i = bframes; i < bframes + m_param->radl; i++)
                    list[i]->m_lowres.sliceType = S265_TYPE_B;
                list[(bframes + m_param->radl)]->m_lowres.sliceType = S265_TYPE_IDR;
            }
        }
        if (frm.sliceType == S265_TYPE_IDR)
        {
            /* Closed GOP */
            m_lastKeyframe = frm.frameNum;
            frm.bKeyframe = true;
            int zoneRadl = 0;
            if (m_param->bResetZoneConfig)
            {
                for (int i = 0; i < m_param->rc.zonefileCount; i++)
                {
                    int zoneStart = m_param->rc.zones[i].startFrame;
                    zoneStart += zoneStart ? m_param->rc.zones[i].zoneParam->radl : 0;
                    if (zoneStart == frm.frameNum)
                    {
                        zoneRadl = m_param->rc.zones[i].zoneParam->radl;
                        m_param->radl = 0;
                        m_param->rc.zones->zoneParam->radl = i < m_param->rc.zonefileCount - 1 ? m_param->rc.zones[i + 1].zoneParam->radl : 0;
                        break;
                    }
                }
            }
            if (bframes > 0 && !m_param->radl && !zoneRadl)
            {
                list[bframes - 1]->m_lowres.sliceType = S265_TYPE_P;
                bframes--;
            }
        }
        if (bframes == m_param->bframes || !list[bframes + 1])
        {
            if (IS_S265_TYPE_B(frm.sliceType))
                s265_log(m_param, S265_LOG_WARNING, "specified frame type is not compatible with max B-frames\n");
            if (frm.sliceType == S265_TYPE_AUTO || IS_S265_TYPE_B(frm.sliceType))
                frm.sliceType = S265_TYPE_P;
        }
        if (frm.sliceType == S265_TYPE_BREF)
            brefs++;
        if (frm.sliceType == S265_TYPE_AUTO)
            frm.sliceType = S265_TYPE_B;
        else if (!IS_S265_TYPE_B(frm.sliceType))
        {
            //如果是非S265_TYPE_B，自然也可以做参考帧，我们在这里标记一下i_bref。
            //另：我们需要i_bref这个变量吗？或许可以去掉
            frm.i_bref = 1;
            break;
        }
            
    }

    if (bframes)
        list[bframes - 1]->m_lowres.bLastMiniGopBFrame = true;
    list[bframes]->m_lowres.leadingBframes = bframes;
    m_lastNonB = &list[bframes]->m_lowres;
    m_histogram[bframes]++;

    if (m_param->bBPyramid && bframes > 1 && !brefs)
    {
        list[bframes / 2]->m_lowres.sliceType = S265_TYPE_BREF;
        brefs++;
    }

    /*ZY TODO:参考关系变了cost计算可能也要变*/
    /* calculate the frame costs ahead of time for estimateFrameCost while we still have lowres */
    if (m_param->rc.rateControlMode != S265_RC_CQP)
    {
        int p0, p1, b;
        /* For zero latency tuning, calculate frame cost to be used later in RC */
        if (!maxSearch)
        {
            for (int i = 0; i <= bframes; i++)
               frames[i + 1] = &list[i]->m_lowres;
        }

        /* estimate new non-B cost */
        p1 = b = bframes + 1;
        p0 = (IS_S265_TYPE_I(frames[bframes + 1]->sliceType)) ? b : 0;

        CostEstimateGroup estGroup(*this, frames);

        estGroup.singleCost(p0, p1, b);

        if (bframes)
        {
            if (m_param->bBPyramid == S265_B_PYRAMID_HIER)
            {
                p0 = 0; // last nonb
                bool isp0available = frames[bframes + 1]->sliceType == S265_TYPE_IDR ? false : true;

                for (b = 1; b <= bframes; b++)
                {
                    for (p1 = b + 1; p1 <= bframes + 1 && frames[p1]->i_level >= frames[b]->i_level; p1++)
                        ; // find new bref or p level lower than cur b
                    for (p0 = b - 1; p0 >= 0 && frames[p0]->i_level >= frames[b]->i_level; p0--)
                        ; // find new bref or p level lower than cur b
                    if (!isp0available && p0 == 0)
                        p0 = b;

                    estGroup.singleCost(p0, p1, b);
                }
            }
            else
            {
                p0 = 0; // last nonb
                bool isp0available = frames[bframes + 1]->sliceType == S265_TYPE_IDR ? false : true;

                for (b = 1; b <= bframes; b++)
                {
                    if (!isp0available)
                        p0 = b;

                    if (frames[b]->sliceType == S265_TYPE_B)
                        for (p1 = b; frames[p1]->sliceType == S265_TYPE_B; p1++)
                            ; // find new nonb or bref
                    else
                        p1 = bframes + 1;

                    estGroup.singleCost(p0, p1, b);

                    if (frames[b]->sliceType == S265_TYPE_BREF)
                    {
                        p0 = b;
                        isp0available = true;
                    }
                }
            }
        }
    }

    /* 实现时域滤波*/
    if (m_param->mctf.enable)
    {
        Lowres** newframes = frames+1; //no last nonb
        
        for( int b = 0; b <= bframes ; b++ )
        {
            int range = 2;
            if (m_param->mctf.method)
                range = 4;
            //TODO：全P帧情况之后再讨论
            //if (((bframes + 1 >= m_param->mctf.gopsize) && newframes[b]->sliceType != S265_TYPE_B) && newframes[b]->i_temporal_id < 1)
            if (((bframes + 1 >= m_param->mctf.gopsize) && newframes[b]->sliceType != S265_TYPE_B) && newframes[b]->i_temporal_id < 1)
            {
                if(newframes[b]->sliceType == S265_TYPE_BREF && bframes<=range*2)
                    continue;
                if(newframes[b]->sliceType == S265_TYPE_P && bframes<range)
                    continue;
                if (b + range > maxSearch)
                    continue;
                // TODO: 根据参数设置
                // printf("bframes num %d\n", bframes);
                float estQp = m_param->mctf.qp;
                // float est_qp = 17;
                filterInput(list, newframes, b, estQp, bframes);
            }
        }
    }

    m_inputLock.acquire();
    /* dequeue all frames from inputQueue that are about to be enqueued
     * in the output queue. The order is important because Frame can
     * only be in one list at a time */
    int64_t pts[S265_BFRAME_MAX + 1];
    for (int i = 0; i <= bframes; i++)
    {
        Frame *curFrame;
        curFrame = m_inputQueue.popFront();
        pts[i] = curFrame->m_pts;
        maxSearch--;
    }
    m_inputLock.release();

    m_outputLock.acquire();
    /* add non-B to output queue */
    int idx = 0;
    list[bframes]->m_reorderedPts = pts[idx++];
    m_outputQueue.pushBack(*list[bframes]);

    if(m_param->bBPyramid == S265_B_PYRAMID_HIER)
    {
        
        int  put_flag[S265_BFRAME_MAX + 2] = {0};
        for (int i = 0; i < bframes; i++)
        {
            int minimumNum = 10 + bframes;
            int minimumId = 0;
            for(int j = 0; j<bframes; j++)
            {
                if(put_flag[j])
                    continue;
                if(minimumNum > list[j]->m_lowres.i_gop_id)
                {
                    minimumNum = list[j]->m_lowres.i_gop_id;
                    minimumId = j;
                }
            }
            put_flag[minimumId] = 1;
            list[minimumId]->m_reorderedPts = pts[idx++];
            m_outputQueue.pushBack(*list[minimumId]);
        }
    }
    else
    {

    
        /* Add B-ref frame next to P frame in output queue, the B-ref encode before non B-ref frame */
        if (brefs)
        {
            for (int i = 0; i < bframes; i++)
            {
                if (list[i]->m_lowres.sliceType == S265_TYPE_BREF)
                {
                    list[i]->m_reorderedPts = pts[idx++];
                    m_outputQueue.pushBack(*list[i]);
                }
            }
        }

        /* add B frames to output queue */
        for (int i = 0; i < bframes; i++)
        {
            /* push all the B frames into output queue except B-ref, which already pushed into output queue */
            if (list[i]->m_lowres.sliceType != S265_TYPE_BREF)
            {
                list[i]->m_reorderedPts = pts[idx++];
                m_outputQueue.pushBack(*list[i]);
            }
        }
    }

    bool isKeyFrameAnalyse = (m_param->rc.cuTree || (m_param->rc.vbvBufferSize && m_param->lookaheadDepth));
    if (isKeyFrameAnalyse && IS_S265_TYPE_I(m_lastNonB->sliceType))
    {
        m_inputLock.acquire();
        Frame *curFrame = m_inputQueue.first();
        frames[0] = m_lastNonB;
        int j;
        for (j = 0; j < maxSearch; j++)
        {
            frames[j + 1] = &curFrame->m_lowres;
            curFrame = curFrame->m_next;
        }
        m_inputLock.release();

        frames[j + 1] = NULL;
        if (!m_param->rc.bStatRead)
            slicetypeAnalyse(frames, true);
        if (m_param->bliveVBV2pass)
        {
            int numFrames;
            for (numFrames = 0; numFrames < maxSearch; numFrames++)
            {
                Lowres *fenc = frames[numFrames + 1];
                if (!fenc)
                    break;
            }
            vbvLookahead(frames, numFrames, true);
        }
    }
    m_outputLock.release();
}

void Lookahead::vbvLookahead(Lowres **frames, int numFrames, int keyframe)
{
    int prevNonB = 0, curNonB = 1, idx = 0;
    while (curNonB < numFrames && IS_S265_TYPE_B(frames[curNonB]->sliceType))
        curNonB++;
    int nextNonB = keyframe ? prevNonB : curNonB;
    int nextB = prevNonB + 1;
    int nextBRef = 0, curBRef = 0;
    if (m_param->bBPyramid && curNonB - prevNonB > 1)
        curBRef = (prevNonB + curNonB + 1) / 2;
    int miniGopEnd = keyframe ? prevNonB : curNonB;
    while (curNonB <= numFrames)
    {
        /* P/I cost: This shouldn't include the cost of nextNonB */
        if (nextNonB != curNonB)
        {
            int p0 = IS_S265_TYPE_I(frames[curNonB]->sliceType) ? curNonB : prevNonB;
            frames[nextNonB]->plannedSatd[idx] = vbvFrameCost(frames, p0, curNonB, curNonB);
            frames[nextNonB]->plannedType[idx] = frames[curNonB]->sliceType;

            /* Save the nextNonB Cost in each B frame of the current miniGop */
            if (curNonB > miniGopEnd)
            {
                for (int j = nextB; j < miniGopEnd; j++)
                {
                    frames[j]->plannedSatd[frames[j]->indB] = frames[nextNonB]->plannedSatd[idx];
                    frames[j]->plannedType[frames[j]->indB++] = frames[nextNonB]->plannedType[idx];
                }
            }
            idx++;
        }

        /* Handle the B-frames: coded order */
        if (m_param->bBPyramid && curNonB - prevNonB > 1)
            nextBRef = (prevNonB + curNonB + 1) / 2;

        for (int i = prevNonB + 1; i < curNonB; i++, idx++)
        {
            int64_t satdCost = 0;
            int type = S265_TYPE_B;
            if (nextBRef)
            {
                if (i == nextBRef)
                {
                    satdCost = vbvFrameCost(frames, prevNonB, curNonB, nextBRef);
                    type = S265_TYPE_BREF;
                }
                else if (i < nextBRef)
                    satdCost = vbvFrameCost(frames, prevNonB, nextBRef, i);
                else
                    satdCost = vbvFrameCost(frames, nextBRef, curNonB, i);
            }
            else
                satdCost = vbvFrameCost(frames, prevNonB, curNonB, i);
            frames[nextNonB]->plannedSatd[idx] = satdCost;
            frames[nextNonB]->plannedType[idx] = type;
            /* Save the nextB Cost in each B frame of the current miniGop */

            for (int j = nextB; j < miniGopEnd; j++)
            {
                if (curBRef && curBRef == i)
                    break;
                if (j >= i && j !=nextBRef)
                    continue;
                frames[j]->plannedSatd[frames[j]->indB] = satdCost;
                frames[j]->plannedType[frames[j]->indB++] = type;
            }
        }
        prevNonB = curNonB;
        curNonB++;
        while (curNonB <= numFrames && IS_S265_TYPE_B(frames[curNonB]->sliceType))
            curNonB++;
    }

    frames[nextNonB]->plannedType[idx] = S265_TYPE_AUTO;
}


int64_t Lookahead::vbvFrameCost(Lowres **frames, int p0, int p1, int b)
{
    CostEstimateGroup estGroup(*this, frames);
    int64_t cost = estGroup.singleCost(p0, p1, b);

    if (m_param->rc.aqMode || m_param->bAQMotion)
    {
        if (m_param->rc.cuTree)
            return frameCostRecalculate(frames, p0, p1, b);
        else
            return frames[b]->costEstAq[b - p0][p1 - b];
    }

    return cost;
}

#define IS_S265_TYPE_AUTO_OR_B(x) ((x)==S265_TYPE_AUTO || IS_S265_TYPE_B(x))
int Lookahead::check_gop8( Lowres **frames, int32_t gop_start )
{
    static const int32_t adaptive_gop4_ratio[] = { 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85 };
    static const int32_t adaptive_gop8_ratio[] = { 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80 };
    int32_t gop_end = gop_start + 8;
    CostEstimateGroup estGroup(*this, frames);
    estGroup.singleCost(gop_start, gop_end, gop_end);
    double adapt_gop_cost = 1.0 * (frames[gop_end]->largeMvs[gop_end - gop_start]) / m_8x8Blocks;
    double large_mv_thres8 = 5 / 10.0;
    if( adapt_gop_cost >= large_mv_thres8 )
    {
        double costRatio =
            1.0 * frames[gop_end]->costEst[gop_end - gop_start][0] / frames[gop_end]->costEst[0][0];
        double ratioGop4 = adaptive_gop4_ratio[m_param->subpelRefine] / 100.0;
        double ratioGop8 = adaptive_gop8_ratio[m_param->subpelRefine] / 100.0;

        int cut_gop =( costRatio < 0.98 );

        if( cut_gop )
        { // not a scene cut
            if( (costRatio > ratioGop4) ||
                (costRatio > ratioGop8 && m_preGopSize == 4) )
            { // if exceed maximum, must be gop=4
                return 0;
            }
        }
    }
    return 1;
}

int Lookahead::check_gop16( Lowres **frames, int32_t gop_start )
{
    //int32_t large_mv_abr_thresh16[] = { 68, 68, 73, 73, 73, 73, 73, 73, 78, 78, 78, 78 };     // --large-mvabr-thresh16
    //int32_t cost_ratio_abr_thresh16[] =  { 57, 57, 62, 62, 62, 62, 66, 66, 66, 66, 70, 70 };  // --cost-ratioabr-thresh16
    //int32_t intra_ratio_abr_thresh16[] = { 60, 60, 62, 62, 62, 62, 62, 62, 72, 72, 78, 78 };  // --cost-ratioabr-thresh16
    int32_t large_mv_abr_thresh16[] = { 75, 75, 75, 75, 75, 75, 75, 75, 80, 80, 80, 80 };     // --large-mvabr-thresh16
    int32_t cost_ratio_abr_thresh16[] = { 70, 70, 70, 70, 70, 70, 70, 80, 80, 80, 80, 80 };  // --cost-ratioabr-thresh16
    int32_t intra_ratio_abr_thresh16[] = { 70, 70, 70, 70, 75, 75, 75, 75, 80, 80, 80, 80 };  // --cost-ratioabr-thresh16

    double large_mv_thres16 =
        large_mv_abr_thresh16[m_param->subpelRefine] / 100.0;
    double intra_ratio_thresh16 =
        intra_ratio_abr_thresh16[m_param->subpelRefine] / 100.0;

    int32_t gop_end = gop_start + 16;
    CostEstimateGroup estGroup(*this, frames);
    estGroup.singleCost(gop_start, gop_end, gop_end);
    double large_mv_frac = 1.0 * ( frames[gop_end]->veryLargeMvs[gop_end - gop_start] ) / m_8x8Blocks;
    double intra_frac = 1.0 * ( frames[gop_end]->intraMbs[gop_end - gop_start] ) / m_8x8Blocks;

    if( intra_frac <= intra_ratio_thresh16 && large_mv_frac <= large_mv_thres16 )
    {
        double ratio_gop16 = cost_ratio_abr_thresh16[m_param->subpelRefine] / 100.0;
        double cost_ratio16 =
            1.0 * frames[gop_end]->costEst[gop_end - gop_start][0] / frames[gop_end]->costEst[0][0];
        double similarity = 1.0 * frames[gop_start]->costEst[0][0] / frames[gop_end]->costEst[0][0];

        if( cost_ratio16 < ratio_gop16 && cost_ratio16 >= 0.05 && similarity >= 0.3333 &&
            similarity <= 3 )
        {
            return 1;
        }
    }
    return 0;
}

//#if ( S264_TEMPORAL_FILTERING && HAVE_AVX2 )// temproal-filtering

static const double  s_chroma_factor = 0.55;
static const int32_t s_sigma_multiplier = 9;
static const int32_t s_sigma_zero_point = 10;
static const int32_t s_interpolation_filter[4][8] = {
    {0, 0, 0, 64, 0, 0, 0, 0},  // 0
    {0, 2, -9, 57, 19, -7, 2, 0},  // 4
    {0, 1, -7, 38, 38, -7, 1, 0},  // 8
    {0, 2, -7, 19, 57, -9, 2, 0}  // 12
};
static const double s_ref_strengths[3][2] = {
    // abs(POC offset)
    //  1,    2
    {0.85, 0.60},  // s_range * 2
    {1.20, 1.00},  // s_range
    {0.30, 0.30}  // otherwise
};

static const double s_ref_strengths2[3][4] =
{ // abs(POC offset)
  //  1,    2     3     4
  {0.85, 0.57, 0.41, 0.33},  // m_range * 2
  {1.13, 0.97, 0.81, 0.57},  // m_range
  {0.30, 0.30, 0.30, 0.30}   // otherwise
};

static const uint32_t permute_left_table[9][8] = { {0, 1, 2, 3, 4, 5, 6, 7}, {0, 0, 1, 2, 3, 4, 5, 6}, {0, 0, 0, 1, 2, 3, 4, 5}, {0, 0, 0, 0, 1, 2, 3, 4}, {0, 0, 0, 0, 0, 1, 2, 3},
    {0, 0, 0, 0, 0, 0, 1, 2}, {0, 0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0} };

static const uint32_t permute_right_table[8][8] = { {0, 1, 2, 3, 4, 5, 6, 7}, {0, 1, 2, 3, 4, 5, 6, 6}, {0, 1, 2, 3, 4, 5, 5, 5}, {0, 1, 2, 3, 4, 4, 4, 4}, {0, 1, 2, 3, 3, 3, 3, 3},
    {0, 1, 2, 2, 2, 2, 2, 2}, {0, 1, 1, 1, 1, 1, 1, 1}, {0, 0, 0, 0, 0, 0, 0, 0} };

#ifdef USE_TEMPORAL_FILTER_AVX
#include <immintrin.h>
#include <emmintrin.h>
#include <smmintrin.h>
static void temporalFilterRowAvx( const int32_t height, const int32_t width, const int32_t y, const int32_t x, const int32_t stride, const int32_t blockSizeX, const int32_t blockSizeY,
    const int32_t yInt, const int32_t xInt, pixel *srcImage, int32_t tempArray[23][16], const int32_t *xFilter )
{
    const int32_t numFilterTaps = 7;
    const int32_t centreTapOffset = 3;
    const int     stepX = 8;
    for( int32_t by = 1; by < blockSizeY + numFilterTaps; by++ )
    {
        const int32_t  yOffset = S265_MAX(0, S265_MIN(height - 1, y + by + yInt - centreTapOffset));
        const pixel *sourceRow = srcImage + yOffset * stride;
        for( int32_t bx = 0; bx < blockSizeX; bx += stepX )
        {
            int32_t        base = S265_MAX(-1, S265_MIN(width - 7, x + bx + xInt - centreTapOffset));
            const pixel *rowStart = (sourceRow + base);
            // AVX2
            __m256i accum_a = _mm256_setzero_si256();
#define PROCESS_FILTER( rowStart1,filter )  {\
            __m256i tempArray_avx2_a = _mm256_setzero_si256();\
            __m256i filter_a = _mm256_set1_epi32(*(filter));\
            tempArray_avx2_a = _mm256_cvtepu8_epi32(_mm_loadu_si128((const __m128i *)(rowStart1)));\
            accum_a = _mm256_add_epi32(accum_a, _mm256_mullo_epi32(tempArray_avx2_a, filter_a));}

            PROCESS_FILTER( rowStart + 1, xFilter + 1 );
            PROCESS_FILTER( rowStart + 2, xFilter + 2 );
            PROCESS_FILTER( rowStart + 3, xFilter + 3 );
            PROCESS_FILTER( rowStart + 4, xFilter + 4 );
            PROCESS_FILTER( rowStart + 5, xFilter + 5 );
            PROCESS_FILTER( rowStart + 6, xFilter + 6 );
#undef PROCESS_FILTER

            // padding left
            if( base < 0 )
            {
                int32_t padNum = S265_MIN( stepX, abs(x + bx + xInt - centreTapOffset + 1) );
                __m256i table = _mm256_lddqu_si256( (__m256i *)permute_left_table + padNum );
                accum_a = _mm256_permutevar8x32_epi32( accum_a, table );
            }

            // padding right
            if( width - 7 < (x + bx + xInt - centreTapOffset + 8) )
            {
                int     padNum = S265_MIN( stepX, x + bx + xInt - centreTapOffset + 8 - (width - 7) );
                __m256i table = _mm256_lddqu_si256( (__m256i *)permute_right_table + padNum - 1) ;
                accum_a = _mm256_permutevar8x32_epi32( accum_a, table );
            }
            // store
            _mm256_storeu_si256( (__m256i *)(tempArray[by] + bx), accum_a );
        }
    }
}

static void temporalFilterColAvx( const int32_t x, const int32_t dstStride, const int32_t blockSizeX, const int32_t blockSizeY, pixel *dstRow, const int32_t tempArray[23][16], const int32_t *yFilter )
{
    const pixel max_value = 255;
    const int     stepX = 8;
    const __m256i mm_min = _mm256_set1_epi32(0);
    const __m256i mm_max = _mm256_set1_epi32((int32_t)max_value);

    for( int32_t by = 0; by < blockSizeY; by++, dstRow += dstStride )
    {
        pixel *dstPel = dstRow + x;
        for( int32_t bx = 0; bx < blockSizeX; bx += stepX, dstPel += stepX )
        {
            // AVX2
            __m256i accum_a = _mm256_setzero_si256();

#define PROCESS_FILTER( tempArray1,filter ) {\
            __m256i tempArray_avx2_a;\
            tempArray_avx2_a = _mm256_loadu_si256( (const __m256i *)(tempArray1) );\
            __m256i filter_a = _mm256_set1_epi32(*(filter));\
            tempArray_avx2_a = _mm256_mullo_epi32( tempArray_avx2_a, filter_a );\
            accum_a = _mm256_add_epi32( accum_a, tempArray_avx2_a );}

            PROCESS_FILTER( tempArray[by + 1] + bx, yFilter + 1 );
            PROCESS_FILTER( tempArray[by + 2] + bx, yFilter + 2 );
            PROCESS_FILTER( tempArray[by + 3] + bx, yFilter + 3 );
            PROCESS_FILTER( tempArray[by + 4] + bx, yFilter + 4 );
            PROCESS_FILTER( tempArray[by + 5] + bx, yFilter + 5 );
            PROCESS_FILTER( tempArray[by + 6] + bx, yFilter + 6 );
#undef PROCESS_FILTER

            const __m256i shift_11 = _mm256_set1_epi32(1 << 11);
            accum_a = _mm256_add_epi32(accum_a, shift_11);
            accum_a = _mm256_srai_epi32(accum_a, 12);
            accum_a = _mm256_min_epi32(mm_max, _mm256_max_epi32(accum_a, mm_min));
            __m256i pack_s32_16 = _mm256_packs_epi32(accum_a, accum_a);
            __m256i permute_16 = _mm256_permute4x64_epi64(pack_s32_16, 0x88);
            __m256i pack_s16_8 = _mm256_packus_epi16(permute_16, permute_16);
            _mm256_storeu_si256((__m256i *)(dstPel), pack_s16_8);
        }
    }
}
#endif

static void temporalFilterRowC( const int32_t height, const int32_t width, const int32_t y, const int32_t x, const int32_t stride, const int32_t blockSizeX, const int32_t blockSizeY,
    const int32_t yInt, const int32_t xInt, pixel *srcImage, int32_t tempArray[23][16], const int32_t *xFilter )
{
    const int32_t numFilterTaps = 7;
    const int32_t centreTapOffset = 3;
    //const int     stepX = 8;
    for( int32_t by = 1; by < blockSizeY + numFilterTaps; by++ )
    {
        const int32_t  yOffset = S265_MAX(0, S265_MIN(height - 1, y + by + yInt - centreTapOffset));
        const pixel *sourceRow = srcImage + yOffset * stride;
        for( int32_t bx = 0; bx < blockSizeX; bx++ )
        {
            int32_t        base = S265_MAX(-1, S265_MIN(width - 7, x + bx + xInt - centreTapOffset));
            const pixel *rowStart = (sourceRow + base);

            int iSum = 0;
            iSum += xFilter[1] * rowStart[1];
            iSum += xFilter[2] * rowStart[2];
            iSum += xFilter[3] * rowStart[3];
            iSum += xFilter[4] * rowStart[4];
            iSum += xFilter[5] * rowStart[5];
            iSum += xFilter[6] * rowStart[6];

            tempArray[by][bx] = iSum;
        }
    }
}

static void temporalFilterColC( const int32_t x, const int32_t dstStride, const int32_t blockSizeX, const int32_t blockSizeY, pixel *dstRow, const int32_t tempArray[23][16], const int32_t *yFilter )
{
    const pixel max_value = 255;
    //const int     stepX = 8;

    for( int32_t by = 0; by < blockSizeY; by++, dstRow += dstStride )
    {
        pixel *dstPel = dstRow + x;
        for (int32_t bx = 0; bx < blockSizeX; bx++, dstPel++)
        {
            // AVX2
            int iSum = 0;

            iSum += yFilter[1] * tempArray[by + 1][bx];
            iSum += yFilter[2] * tempArray[by + 2][bx];
            iSum += yFilter[3] * tempArray[by + 3][bx];
            iSum += yFilter[4] * tempArray[by + 4][bx];
            iSum += yFilter[5] * tempArray[by + 5][bx];
            iSum += yFilter[6] * tempArray[by + 6][bx];

            iSum = (iSum + (1 << 11)) >> 12;
            iSum = iSum < 0 ? 0 : (iSum > max_value ? max_value : iSum);
            *dstPel = iSum;
        }
    }
}

void Lookahead::applyMotion( MV *lowresMv, Frame *refFrame, Frame *curFrame, pixel *dst[3], int *inter_cost, int32_t *intra_cost )
{
    pixel **src = refFrame->m_fencPic->m_picOrg;
    pixel **cur = curFrame->m_fencPic->m_picOrg;
    uint32_t sourceHeight = refFrame->m_fencPic->m_picHeight;
    uint32_t sourceWidth = refFrame->m_fencPic->m_picWidth;
    intptr_t s[3] = {refFrame->m_fencPic->m_stride, refFrame->m_fencPic->m_strideC,refFrame->m_fencPic->m_strideC};
    uint32_t mbStride = refFrame->m_lowres.maxBlocksInRow;

    //static const int32_t lumaBlockSize = 16;

    for( int32_t c = 0; c < 3; c++ )
    {
        const int32_t csx = c > 0 ? 1 : 0;
        const int32_t csy = csx;
        const int32_t blockSizeX = 16 >> csx;
        const int32_t blockSizeY = 16 >> csy;
        pixel *     dstImage = dst[c];
        pixel *     srcImage = src[c];
        pixel *     curImage = cur[c];
        const int32_t stride = s[c];
        const int32_t height = sourceHeight >> csy;
        const int32_t width = sourceWidth >> csx;
        int32_t       blockIndex = 0;
        int32_t       shift = 1 << (4 - csy);
        //zy for test
        int refIndex = refFrame->m_poc;
        int curIndex = curFrame->m_poc;
        int32_t dir = curIndex > refIndex ? 1 : 0;
        //int32_t dist = curIndex > refIndex ? curIndex - refIndex - 1 : refIndex - curIndex - 1;
        int32_t dist = curIndex > refIndex ? curIndex - refIndex : refIndex - curIndex;
        MV *newLowresMv = refFrame->m_lowres.lowresMvs[dir][dist];

        for( int32_t y = 0; y < height; y += blockSizeY )
        {
            for( int32_t x = 0; x < width; x += blockSizeX )
            {
                blockIndex = (x / shift) + (y / shift) * mbStride;
                //assert(blockIndex < h->mb.i_mb_count);  
                const int16_t mvX = lowresMv[blockIndex].x << 3; // 1 for int 1/4 pel, 2 for 1/16 pel
                const int16_t mvY = lowresMv[blockIndex].y << 3; // 1 for int 1/4 pel, 2 for 1/16 pel
                const int32_t dx = mvX >> csx;
                const int32_t dy = mvY >> csy;
                const int32_t *xFilter = s_interpolation_filter[(dx & 0xf) >> 2];
                const int32_t *yFilter = s_interpolation_filter[(dy & 0xf) >> 2]; // will add 6 bit.
                const int32_t xInt = mvX >> (4 + csx);
                const int32_t yInt = mvY >> (4 + csy);
                int32_t targetX = s265_clip3(0, (int32_t)sourceWidth, x + xInt);
                int32_t targetY = s265_clip3(0, (int32_t)sourceHeight, y + yInt);
                int32_t newBlockIndex = (targetX / shift) + (targetY / shift) * mbStride;
                const int16_t newMvX = newLowresMv[newBlockIndex].x << 3; // 1 for int 1/4 pel, 2 for 1/16 pel
                const int16_t newMvY = newLowresMv[newBlockIndex].y << 3; // 1 for int 1/4 pel, 2 for 1/16 pel

                int mvNotMatch = 0;
                int mvNotAvalible = 0;
                if (m_param->mctf.mvMatch)
                {
                    if (lowresMv[blockIndex].x == 0x7FFF || lowresMv[blockIndex].y == 0x7FFF)
                    {
                        mvNotAvalible = 1;
                    }
                    if (newMvX + mvX != 0 || newMvY + mvY != 0)
                    {
                        mvNotMatch = 1;
                    }
                }

                if (mvNotMatch || mvNotAvalible || ( m_param->mctf.thres[0]
                    && (intra_cost[blockIndex] * m_param->mctf.thres[0] <= inter_cost[blockIndex] * 10 || inter_cost[blockIndex] > 64 * m_param->mctf.thres[1] / 2) ))
                { // the larger m_iTemporalFilterThres means the less possibility to unfilter
                    for (int32_t by = 0; by < blockSizeY; by++)
                    {
                        if (y + by >= height)
                            break;
                        memcpy((dstImage + (y + by) * stride + x), (curImage + (y + by) * stride + x), blockSizeX * sizeof(curImage[0]));
                    }
                    continue;
                }

                int32_t tempArray[23][16];// 16+7
#ifdef USE_TEMPORAL_FILTER_AVX
                if ((m_param->cpuid & S265_CPU_AVX) && (m_param->cpuid & S265_CPU_AVX2))
                    temporalFilterRowAvx(height, width, y, x, stride, blockSizeX, blockSizeY, yInt, xInt, srcImage, tempArray, xFilter);
                else
                    temporalFilterRowC(height, width, y, x, stride, blockSizeX, blockSizeY, yInt, xInt, srcImage, tempArray, xFilter);
#else
                temporalFilterRowC(height, width, y, x, stride, blockSizeX, blockSizeY, yInt, xInt, srcImage, tempArray, xFilter);
#endif
                pixel *dstRow = dstImage + y * stride;
#ifdef USE_TEMPORAL_FILTER_AVX
                if ((m_param->cpuid & S265_CPU_AVX) && (m_param->cpuid & S265_CPU_AVX2))
                    temporalFilterColAvx(x, stride, blockSizeX, blockSizeY, dstRow, tempArray, yFilter);
                else
                    temporalFilterColC(x, stride, blockSizeX, blockSizeY, dstRow, tempArray, yFilter);

#else
                temporalFilterColC(x, stride, blockSizeX, blockSizeY, dstRow, tempArray, yFilter);
#endif
            }
        }
    }
}

#define SHIFTBIT 16
#define SHIFTVALUE (1 << SHIFTBIT)  // 2^16

#ifdef USE_TEMPORAL_FILTER_AVX
static void bilateralFilterCoreAvx( const int32_t c, const int32_t height, const int32_t width, const int32_t numRefs, pixel *correctedPics[10][3], const pixel *srcPelRow, const int32_t srcStride,
    pixel *dstPelRow, const int32_t dstStride, const int32_t expValue[2][1024], const int32_t offsetIndex[10] )
{
    const pixel maxSampleValue = 255;
    const int32_t stepX = 8;

    for( int32_t y = 0; y < height; y++, srcPelRow += srcStride, dstPelRow += dstStride )
    {
        const pixel *srcPel = srcPelRow;
        pixel       *dstPel = dstPelRow;
        for( int32_t x = 0; x < width; x += stepX, srcPel += stepX, dstPel += stepX )
        {
            // const int32_t orgVal = (pixel_t)*srcPel;
            __m256i org_val_avx2 = _mm256_cvtepu8_epi32(_mm_loadu_si128((const __m128i *)(srcPel)));
            __m256i temporal_weight_sum_avx2 = _mm256_set1_epi32((int32_t)(SHIFTVALUE));
            __m256i new_val_avx2 = _mm256_slli_epi32(org_val_avx2, 16);
            for( int32_t i = 0; i < numRefs; i++ )
            {
                const pixel *corrected_pel_ptr = correctedPics[i][c] + (y * srcStride + x);
                const int32_t  index = S265_MIN(1, abs(offsetIndex[i]) - 1);
                __m256i        ref_val_avx2 = _mm256_cvtepu8_epi32(_mm_loadu_si128((const __m128i *)(corrected_pel_ptr)));
                __m256i        diff_avx2 = _mm256_abs_epi32(_mm256_sub_epi32(ref_val_avx2, org_val_avx2));
                __m256i        weight_avx2 = _mm256_i32gather_epi32(expValue[index], diff_avx2, 4);
                new_val_avx2 = _mm256_add_epi32(new_val_avx2, _mm256_mullo_epi32(weight_avx2, ref_val_avx2));
                temporal_weight_sum_avx2 = _mm256_add_epi32(temporal_weight_sum_avx2, weight_avx2);
            }
            int32_t temporal_weight_sum_array[8];
            int32_t new_val_array[8];
            _mm256_storeu_si256((__m256i *)temporal_weight_sum_array, temporal_weight_sum_avx2);
            _mm256_storeu_si256((__m256i *)new_val_array, new_val_avx2);
            for( int i = 0; i < stepX; ++i )
            {
                new_val_array[i] = (new_val_array[i] + temporal_weight_sum_array[i] / 2) / temporal_weight_sum_array[i];
                pixel sample_val = (pixel)round(new_val_array[i]);
                sample_val = (sample_val < 0 ? 0 : (sample_val > maxSampleValue ? maxSampleValue : sample_val));
                *(dstPel + i) = sample_val;
            }
        }
    }
}
#endif

void Lookahead::bilateralFilterCoreC( const int32_t c, const int32_t height, const int32_t width, const int32_t numRefs, pixel *correctedPics[10][3], const pixel *srcPelRow, const int32_t srcStride,
    pixel *dstPelRow, const int32_t dstStride, const double expValue[4][4][1024], const int32_t offsetIndex[10], int s_range )
{
    const pixel maxSampleValue = 255;
    const int32_t stepX = 1; //ZY TODO:先逐个进行 调好后考虑汇编

    int refStrengthRow = 2;
    //int s_range = 2;
    if (numRefs == s_range * 2)
    {
        refStrengthRow = 0;
    }
    else if (numRefs == s_range)
    {
        refStrengthRow = 1;
    }

    static const int lumaBlockSize=8;    
    const int blkSizeX = lumaBlockSize;
    const int blkSizeY = lumaBlockSize;   
    int blockNumWidth = width / blkSizeX;
    int blockNumHeight = height / blkSizeY;
    int totalBlkNum = blockNumWidth * blockNumHeight;

    int *errorList[10] = {0};
    int *noiseList[10] = {0};
    // if(numRefs<=0)
    //     return;
    for(int i=0; i< numRefs; i++)
    {
        errorList[i] = (int*)s265_malloc(totalBlkNum * sizeof(int));
        noiseList[i] = (int*)s265_malloc(totalBlkNum * sizeof(int));
    }

    for( int32_t y = 0; y < height; y++, srcPelRow += srcStride, dstPelRow += dstStride )
    {
        const pixel *srcPel = srcPelRow;
        pixel       *dstPel = dstPelRow;
        for( int32_t x = 0; x < width; x += stepX, srcPel += stepX, dstPel += stepX )
        {
            const int orgVal = (int) *srcPel;
            double temporalWeightSum = 1.0;
            double newVal = (double) orgVal;

            double minError = 9999999;
            int blkX = x / blkSizeX;
            int blkY = y / blkSizeY;
            int blkIndex = blkX + blkY * blockNumWidth;

            if (m_param->mctf.method)
            {
                if ((y % blkSizeY == 0) && (x % blkSizeX == 0))
                {
                    for (int32_t i = 0; i < numRefs; i++)
                    {
                        double variance = 0, diffsum = 0; 
                        int ssd = 0;
                        for (int32_t y1 = 0; y1 < blkSizeY - 1; y1++)
                        {
                            for (int32_t x1 = 0; x1 < blkSizeX - 1; x1++)
                            {
                                pixel pix = *(srcPel + x1);
                                pixel pixR = *(srcPel + x1 + 1);
                                pixel pixD = *(srcPel + x1 + srcStride);
                                pixel ref = *(correctedPics[i][c] + ((y + y1) * srcStride + x + x1));
                                pixel refR = *(correctedPics[i][c] + ((y + y1) * srcStride + x + x1 + 1));
                                pixel refD = *(correctedPics[i][c] + ((y + y1 + 1) * srcStride + x + x1));

                                int diff = pix - ref;
                                int diffR = pixR - refR;
                                int diffD = pixD - refD;
                                if ((x1 < blkSizeX - 1) && (y1 < blkSizeY - 1))
                                {
                                    variance += diff * diff;
                                    diffsum += (diffR - diff) * (diffR - diff);
                                    diffsum += (diffD - diff) * (diffD - diff);
                                }
                                ssd += diff * diff;
                            }
                        }
                        errorList[i][blkIndex] = ssd;
                        noiseList[i][blkIndex] = (int)round((300 * variance + 50) / (10 * diffsum + 50));
                    }
                }

                for (int i = 0; i < numRefs; i++)
                {
                    minError = S265_MIN(minError, (double)errorList[i][blkIndex]);
                }
            }

            for( int32_t i = 0; i < numRefs; i++ )
            {
                const pixel *pCorrectedPelPtr = correctedPics[i][c] + (y * srcStride + x);
                int refVal = (int) *pCorrectedPelPtr;
                
                double diff = (double)(refVal - orgVal);
                int diffInt = abs(refVal - orgVal);
                diff *= 4;
                double weight;

                if (m_param->mctf.method)
                {
                    const int error = errorList[i][blkIndex];
                    const int noise = noiseList[i][blkIndex];
                    const int index = S265_MIN(3, abs(offsetIndex[i]) - 1);
                    double ww = 1;
                    ww *= (noise < 25) ? 1 : 1.2;
                    ww *= (error < 50) ? 1.2 : ((error > 100) ? 0.8 : 1);
                    ww *= ((minError + 1) / (error + 1));
                    int sw_index = (noise < 25) ? 1 : 0;
                    sw_index = (sw_index << 1) + ((error < 50) ? 1 : 0);
                    weight = expValue[sw_index][index][diffInt] * ww;
                }
                else
                {
                    const int32_t index = S265_MIN(1, abs(offsetIndex[i]) - 1);
                    weight = expValue[0][index][diffInt];
                }
                newVal += weight * refVal;
                temporalWeightSum += weight;
            }
            newVal /= temporalWeightSum;
            pixel sampleVal = (pixel)round(newVal);
            sampleVal=(sampleVal<0?0 : (sampleVal>maxSampleValue ? maxSampleValue : sampleVal));
            *dstPel = sampleVal;
        }
    }
    for(int i=0; i< numRefs; i++)
    {
        s265_free(errorList[i]);
        s265_free(noiseList[i]);
    }
}

void Lookahead::bilateralFilter( pixel *correctedPics[10][3], Frame *curFrame, double overallStrength,  int32_t numRefs,
    const int32_t sRange, int32_t mQp, int32_t offsetIndex[10], bool replace )
{
    PicYuv* dst_pic = replace ? curFrame->m_fencPic : curFrame->m_filteredPic;
    int32_t sourceHeight = curFrame->m_fencPic->m_picHeight;
    int32_t sourceWidth = curFrame->m_fencPic->m_picWidth;
    int32_t refStrengthRow = 2;
    if( numRefs == sRange * 2 )
    {
        refStrengthRow = 0;
    }
    else if( numRefs == sRange )
    {
        refStrengthRow = 1;
    }

    const int32_t lumaSigmaSq = (mQp - s_sigma_zero_point) * (mQp - s_sigma_zero_point) * s_sigma_multiplier;
    const int32_t chromaSigmaSq = 30 * 30;
    double        filteringDouble[4];
    int32_t       expValue[2][1024];
    double        expValueFloat[4][4][1024];

    for( int32_t c = 0; c < 3; c++ )
    {
        int32_t           shift = c ? 1 : 0;
        const int32_t     height = sourceHeight >> shift;
        const int32_t     width = sourceWidth >> shift;
        const pixel       *srcPelRow = curFrame->m_originalPic->m_picOrg[c];
        const int32_t     srcStride = c ? curFrame->m_fencPic->m_strideC : curFrame->m_fencPic->m_stride;
        pixel             *dstPelRow = dst_pic->m_picOrg[c];
        const int32_t     dstStride = c ? dst_pic->m_strideC : dst_pic->m_stride;
        const int32_t     sigmaSq = c ? chromaSigmaSq : lumaSigmaSq;
        const double      weightScaling = overallStrength * (c ? s_chroma_factor : 0.4);
        for( int m = 0; m < 2; m++ )
        {
            filteringDouble[m] = weightScaling * s_ref_strengths[refStrengthRow][m];
        }
#ifdef USE_TEMPORAL_FILTER_AVX
        //simd method not support mctf.method 1
        if ((m_param->cpuid & S265_CPU_AVX) && (m_param->cpuid & S265_CPU_AVX2) && (!m_param->mctf.method))
        {
            const pixel maxSampleValue = 255;
            const int32_t bitDepthDiffWeighting = 4;
            if (c < 2)
            { // table calulate only for Y and U, V just reuse from U
                for (int i = 0; i <= maxSampleValue; i++)
                {
                    for (int m = 0; m < 2; m++)
                    {
                        expValue[m][i] = (int32_t)(exp(-i * i * bitDepthDiffWeighting * bitDepthDiffWeighting / (2 * sigmaSq * 1.0)) * filteringDouble[m] * SHIFTVALUE + 0.5);
                    }
                }
            }
            bilateralFilterCoreAvx(c, height, width, numRefs, correctedPics, srcPelRow, srcStride, dstPelRow, dstStride, expValue, offsetIndex);
        }
        else
#endif
        {
            const pixel maxSampleValue = 255;
            const int32_t bitDepthDiffWeighting = 4;
            //sw *= (noise < 25) ? 1.3 : 0.8;
            //sw *= (error < 50) ? 1.3 : 1;
            double sw_list[4]={0.8, 1.04, 1.3, 1.69};
            if (c < 2)
            { // table calulate only for Y and U, V just reuse from U
                if (m_param->mctf.method)
                {
                    for (int m = 0; m < 4; m++)
                    {
                        filteringDouble[m] = weightScaling * s_ref_strengths2[refStrengthRow][m];
                    }
                    for (int i = 0; i <= maxSampleValue; i++)
                    {
                        for (int sw = 0; sw < 4; sw++)
                        {
                            for (int m = 0; m < 4; m++)
                                expValueFloat[sw][m][i] = filteringDouble[m] * exp(-i * i * bitDepthDiffWeighting * bitDepthDiffWeighting / (2 * sw_list[sw] * sigmaSq * 1.0));
                        }
                    }
                }
                else
                {
                    for (int i = 0; i <= maxSampleValue; i++)
                    {
                        for (int m = 0; m < 2; m++)
                            expValueFloat[0][m][i] = exp(-i * i * bitDepthDiffWeighting * bitDepthDiffWeighting / (2 * sigmaSq * 1.0)) * filteringDouble[m];
                    }
                }
            }
            //weight = weightScaling * s_ref_strengths2[refStrengthRow][index] * ww * exp(-diffSq / (2 * sw * sigmaSq));
            //weight = weightScaling * s_ref_strengths[refStrengthRow][index] * exp(-diffSq / (2 * sigmaSq));
            bilateralFilterCoreC(c, height, width, numRefs, correctedPics, srcPelRow, srcStride, dstPelRow, dstStride, expValueFloat, offsetIndex, sRange);
        }
    }
}

int Lookahead::temporalFilter( Frame **frames, Lowres **lowresFrames, int32_t b, const int32_t sRange, int32_t qp, bool replace)
{
    int32_t firstFrame = b - sRange;
    int32_t lastFrame = b + sRange;

    //determine motion vectors
    int32_t   numRef = 0;
    pixel     *temp[10][3]={{0}};
    int32_t   origOffset = -sRange;
    int       origOffsetIndex[10];
    CostEstimateGroup estGroup(*this, lowresFrames);


    for( int32_t idx = firstFrame; idx <= lastFrame; idx++ )
    {
        if( idx < 0 || frames[idx] == NULL )
        {
            origOffset++;
            continue;  // frame not available
        }
        else if( idx == b )
        {  // hop over frame that will be filtered
            origOffset++;
            continue;
        }

        if( idx < b )
        {
            estGroup.singleCost(idx, b, b);
            if(m_param->mctf.mvMatch)
            {
                estGroup.singleCost(idx, b, idx);//add for test
            }
        }
        else
        {
            estGroup.singleCost(b, idx, b);
            if(m_param->mctf.mvMatch)
            {
                estGroup.singleCost(b, idx, idx);//add for test
            }
        }

        int32_t  dir  = idx > b ? 1 : 0;
        //int32_t  dist = idx > b ? idx - b - 1 : b - idx - 1;
        int32_t  dist = idx > b ? idx - b : b - idx;
        MV *lowresMv = lowresFrames[b]->lowresMvs[dir][dist];
        int *interCost = lowresFrames[b]->lowresMvCosts[dir][dist];
        int32_t *intra_cost = lowresFrames[b]->intraCost;

        temp[numRef][0] = (pixel*)s265_malloc( (frames[b]->m_fencPic->m_picHeight + 2*frames[b]->m_fencPic->m_lumaMarginY) * frames[b]->m_fencPic->m_stride);
        temp[numRef][1] = (pixel*)s265_malloc( ((frames[b]->m_fencPic->m_picHeight >> 1) + 2*frames[b]->m_fencPic->m_chromaMarginY) * frames[b]->m_fencPic->m_strideC);
        temp[numRef][2] = (pixel*)s265_malloc( ((frames[b]->m_fencPic->m_picHeight >> 1) + 2*frames[b]->m_fencPic->m_chromaMarginY) * frames[b]->m_fencPic->m_strideC);
        if(!temp[numRef][0] || !temp[numRef][1] || !temp[numRef][2]) continue;
        origOffsetIndex[numRef] = origOffset;
        //printf("Tempoal filter deal frame poc: %d, frame_type:%d, ref poc %d\n", frames[b]->m_poc, lowresFrames[b]->sliceType, frames[idx]->m_poc);
        applyMotion( lowresMv, frames[idx], frames[b], temp[numRef], interCost, intra_cost );
            
        numRef++;
        origOffset++;
    }
    // filter
    double overallStrength = lowresFrames[b]->sliceType == S265_TYPE_BREF ? m_param->mctf.strength[2] : lowresFrames[b]->sliceType == S265_TYPE_P ? m_param->mctf.strength[1] : m_param->mctf.strength[0];

    if( numRef == 0 )
    {
        return 0;
    }

    bilateralFilter(temp, frames[b], overallStrength, numRef, sRange, qp, origOffsetIndex, replace);

    for( int32_t i = 0; i < numRef; i++ )
    {
        if( temp[i][0] )
            s265_free(temp[i][0]);
        if( temp[i][1] )
            s265_free(temp[i][1]);
        if( temp[i][2] )
            s265_free(temp[i][2]);
    }

    return 1;
}


void Lookahead::filterInput( Frame **frames, Lowres **lowresFrames, int32_t b,float estQp, int bframe )
{
    int32_t filterRange = 2;
    if(m_param->mctf.method)
    {
        filterRange = 4;
    }
    if (m_param->mctf.range)
    {
        int frameRange = (bframe + 2) / 4;
        filterRange = S265_MIN(filterRange, frameRange);
    }
    int qp = s265_clip3( 17,40, (int)( estQp + 0.5 ) );

    if (m_param->bEnablePsnr || m_param->bEnableSsim) {
        if (temporalFilter( frames, lowresFrames, b, filterRange, qp, false)) {
            //filter pixl writed into m_filteredPic, so swap pointers
            frames[b]->m_filteredPic->copyParam(frames[b]->m_fencPic);
            // pixel *temp = frames[b]->m_filteredPic->m_picOrg[0];
            // frames[b]->m_filteredPic->m_picOrg[0] =  frames[b]->m_fencPic->m_picOrg[0];
            // frames[b]->m_fencPic->m_picOrg[0] = temp;

            // temp = frames[b]->m_filteredPic->m_picOrg[1];
            // frames[b]->m_filteredPic->m_picOrg[1] =  frames[b]->m_fencPic->m_picOrg[1];
            // frames[b]->m_fencPic->m_picOrg[1] = temp;

            // temp = frames[b]->m_filteredPic->m_picOrg[2];
            // frames[b]->m_filteredPic->m_picOrg[2] =  frames[b]->m_fencPic->m_picOrg[2];
            // frames[b]->m_fencPic->m_picOrg[2] = temp;
            // frames[b]->m_originalPic = frames[b]->m_filteredPic;

            PicYuv* temp = frames[b]->m_filteredPic;
            frames[b]->m_filteredPic = frames[b]->m_fencPic;
            frames[b]->m_fencPic = temp;
        }
    }
    else
    {  //directly write filtered pixl into m_fencPic
        temporalFilter( frames, lowresFrames, b, filterRange, qp, true);
    }
}

void Lookahead::slicetypeAnalyse(Lowres **frames, bool bKeyframe)
{
    int numFrames, origNumFrames, keyintLimit, framecnt;
    int maxSearch = S265_MIN(m_param->lookaheadDepth, S265_LOOKAHEAD_MAX);
    int cuCount = m_8x8Blocks;
    int resetStart;
    bool bIsVbvLookahead = m_param->rc.vbvBufferSize && m_param->lookaheadDepth;

    /* count undecided frames */
    for (framecnt = 0; framecnt < maxSearch; framecnt++)
    {
        Lowres *fenc = frames[framecnt + 1];
        if (!fenc || fenc->sliceType != S265_TYPE_AUTO)
            break;
    }

    if (!framecnt)
    {
        if (m_param->rc.cuTree)
        {
            if(m_param->rc.cuTreeType)
            {
                cuTree2(frames, 0, bKeyframe);
            }
            else
            {
                cuTree(frames, 0, bKeyframe);
            }
        }
            
        return;
    }
    frames[framecnt + 1] = NULL;//结尾标志

    if (m_param->bResetZoneConfig)
    {
        for (int i = 0; i < m_param->rc.zonefileCount; i++)
        {
            int curZoneStart = m_param->rc.zones[i].startFrame, nextZoneStart = 0;
            curZoneStart += curZoneStart ? m_param->rc.zones[i].zoneParam->radl : 0;
            nextZoneStart += (i + 1 < m_param->rc.zonefileCount) ? m_param->rc.zones[i + 1].startFrame + m_param->rc.zones[i + 1].zoneParam->radl : m_param->totalFrames;
            if (curZoneStart <= frames[0]->frameNum && nextZoneStart > frames[0]->frameNum)
                m_param->keyframeMax = nextZoneStart - curZoneStart;
        }
    }
    int keylimit = m_param->keyframeMax;
    if (frames[0]->frameNum < m_param->chunkEnd)
    {
        int chunkStart = (m_param->chunkStart - m_lastKeyframe - 1);
        int chunkEnd = (m_param->chunkEnd - m_lastKeyframe);
        if ((chunkStart > 0) && (chunkStart < m_param->keyframeMax))
            keylimit = chunkStart;
        else if ((chunkEnd > 0) && (chunkEnd < m_param->keyframeMax))
            keylimit = chunkEnd;
    }

    int keyFrameLimit = keylimit + m_lastKeyframe - frames[0]->frameNum - 1;
    if (m_param->gopLookahead && keyFrameLimit <= m_param->bframes + 1)
        keyintLimit = keyFrameLimit + m_param->gopLookahead;
    else
        keyintLimit = keyFrameLimit;

    origNumFrames = numFrames = m_param->bIntraRefresh ? framecnt : S265_MIN(framecnt, keyintLimit);
    if (bIsVbvLookahead)
        numFrames = framecnt;
    else if (m_param->bOpenGOP && numFrames < framecnt)
        numFrames++;
    else if (numFrames == 0)
    {
        frames[1]->sliceType = S265_TYPE_I;
        return;
    }

    if (m_bBatchMotionSearch)
    {
        /* pre-calculate all motion searches, using many worker threads */
        CostEstimateGroup estGroup(*this, frames);
        for (int b = 2; b < numFrames; b++)
        {
            for (int i = 1; i <= m_param->bframes + 1; i++)
            {
                int p0 = b - i;
                if (p0 < 0)
                    continue;

                /* Skip search if already done */
                if (frames[b]->lowresMvs[0][i][0].x != 0x7FFF)
                    continue;

                /* perform search to p1 at same distance, if possible */
                int p1 = b + i;
                if (p1 >= numFrames || frames[b]->lowresMvs[1][i][0].x != 0x7FFF)
                    p1 = b;

                estGroup.add(p0, p1, b);
            }
        }
        /* auto-disable after the first batch if pool is small */
        m_bBatchMotionSearch &= m_pool->m_numWorkers >= 4;
        estGroup.finishBatch();

        if (m_bBatchFrameCosts)
        {
            /* pre-calculate all frame cost estimates, using many worker threads */
            for (int b = 2; b < numFrames; b++)
            {
                for (int i = 1; i <= m_param->bframes + 1; i++)
                {
                    if (b < i)
                        continue;

                    /* only measure frame cost in this pass if motion searches
                     * are already done */
                    if (frames[b]->lowresMvs[0][i][0].x == 0x7FFF)
                        continue;

                    int p0 = b - i;

                    for (int j = 0; j <= m_param->bframes; j++)
                    {
                        int p1 = b + j;
                        if (p1 >= numFrames)
                            break;

                        /* ensure P1 search is done */
                        if (j && frames[b]->lowresMvs[1][j][0].x == 0x7FFF)
                            continue;

                        /* ensure frame cost is not done */
                        if (frames[b]->costEst[i][j] >= 0)
                            continue;

                        estGroup.add(p0, p1, b);
                    }
                }
            }

            /* auto-disable after the first batch if the pool is not large */
            m_bBatchFrameCosts &= m_pool->m_numWorkers > 12;
            estGroup.finishBatch();
        }
    }

    int numBFrames = 0;
    int numAnalyzed = numFrames;
    bool isScenecut = false;

    /* Temporal computations for scenecut detection */
    if (m_param->bHistBasedSceneCut && m_param->bEnableTradScdInHscd)
    {
        for (int i = numFrames - 1; i > 0; i--)
        {
            if (frames[i]->interPCostPercDiff > 0.0)
                continue;
            int64_t interCost = frames[i]->costEst[1][0];
            int64_t intraCost = frames[i]->costEst[0][0];
            if (interCost < 0 || intraCost < 0)
                continue;
            int times = 0;
            double averagePcost = 0.0, averageIcost = 0.0;
            for (int j = i - 1; j >= 0 && times < 5; j--, times++)
            {
                if (frames[j]->costEst[0][0] > 0 && frames[j]->costEst[1][0] > 0)
                {
                    averageIcost += frames[j]->costEst[0][0];
                    averagePcost += frames[j]->costEst[1][0];
                }
                else
                    times--;
            }
            if (times)
            {
                averageIcost = averageIcost / times;
                averagePcost = averagePcost / times;
                frames[i]->interPCostPercDiff = abs(interCost - averagePcost) / S265_MIN(interCost, averagePcost) * 100;
                frames[i]->intraCostPercDiff = abs(intraCost - averageIcost) / S265_MIN(intraCost, averageIcost) * 100;
            }
        }
    }

    /* When scenecut threshold is set, use scenecut detection for I frame placements */
    if (!m_param->bHistBasedSceneCut || (m_param->bHistBasedSceneCut && m_param->bEnableTradScdInHscd && frames[1]->bScenecut))
        isScenecut = scenecut(frames, 0, 1, true, origNumFrames);
    else if (m_param->bHistBasedSceneCut && frames[1]->bScenecut)
        isScenecut = true;

    if (isScenecut && (m_param->bHistBasedSceneCut || m_param->scenecutThreshold))
    {
        frames[1]->sliceType = S265_TYPE_I;
        return;
    }
    if (m_param->gopLookahead && (keyFrameLimit >= 0) && (keyFrameLimit <= m_param->bframes + 1))
    {
        bool sceneTransition = m_isSceneTransition;
        m_extendGopBoundary = false;
        for (int i = m_param->bframes + 1; i < origNumFrames; i += m_param->bframes + 1)
        {
            if (!m_param->bHistBasedSceneCut || (m_param->bHistBasedSceneCut && m_param->bEnableTradScdInHscd && frames[i + 1]->bScenecut))
                scenecut(frames, i, i + 1, true, origNumFrames);

            for (int j = i + 1; j <= S265_MIN(i + m_param->bframes + 1, origNumFrames); j++)
            {
                if (frames[j]->bScenecut)
                {
                    if (m_param->bEnableTradScdInHscd)
                        m_extendGopBoundary = scenecutInternal(frames, j - 1, j, true);
                    else
                        m_extendGopBoundary = true;
                    break;
                }
            }
            if (m_extendGopBoundary)
                break;
        }
        m_isSceneTransition = sceneTransition;
    }
    if (m_param->bframes)
    {
        if (m_param->bFrameAdaptive == S265_B_ADAPT_OPTIMAL_FAST)
        {
            int32_t gop_start = 0;
            int32_t search_len = S265_MIN(m_param->bframes + 1, origNumFrames);
            int32_t gop_size = 0;
            if( m_param->bBPyramid == S265_B_PYRAMID_HIER )
            {
                for( int j = 1; j < search_len; j++ )//first minigop scenecut check
                {
                    if( scenecut( frames, j, j + 1, 0, origNumFrames ) )
                    {
                        gop_size = j;
                        break;
                    }
                }
                if( gop_size > 0 )
                {
                    if( gop_size >= 8 )
                    {
                        int firstgop4 =( frames[0]->frameNum - m_lastKeyframe < 4 );
                        int is_gop8_good = firstgop4 ? 0 : check_gop8( frames, gop_start );
                        gop_size = is_gop8_good ? 8 : 4;
                    }
                    m_preGopSize = gop_size;
                    if(IS_S265_TYPE_AUTO_OR_B(frames[gop_start+gop_size]->sliceType))
                        frames[gop_start+gop_size]->sliceType = S265_TYPE_P;
                    //process_gop( h, frames, gop_start, gop_size, 1 );
                    gop_start += gop_size;
                }
            }
            while( gop_start < numFrames )
            {
                for( gop_size = 0; (gop_size < m_param->bframes + 1) && (gop_start + gop_size + 1 <= numFrames); gop_size++ )
                {
                    if( !IS_S265_TYPE_AUTO_OR_B(frames[gop_start + gop_size + 1]->sliceType) )
                    {
                        gop_size++;
                        break;
                    }
                }
                if( gop_size >= 8 )
                {
                    int is_gop16_good = 0;
                    if( gop_size >= 16 )
                    {
                        is_gop16_good = check_gop16( frames, gop_start );
                        gop_size=16;
                    }
                    if( !is_gop16_good )
                    {
                        int is_gop8_good = check_gop8( frames, gop_start );
                        gop_size = is_gop8_good ? 8 : S265_MIN( gop_size, 4 );
                    }
                }
                m_preGopSize = gop_size;
                if(IS_S265_TYPE_AUTO_OR_B(frames[gop_start+gop_size]->sliceType))
                    frames[gop_start+gop_size]->sliceType = S265_TYPE_P;
                
                //process_gop( h, frames, gop_start, gop_size, 0 );
                gop_start += gop_size;
            }

            for (int i = 1; i < numFrames; i++)
            {
                if (frames[i]->sliceType == S265_TYPE_AUTO)
                    frames[i]->sliceType = S265_TYPE_B;
            }
            numBFrames = 0;
            while (numBFrames < numFrames && frames[numBFrames + 1]->sliceType == S265_TYPE_B)
                numBFrames++;
        }
        else if (m_param->bFrameAdaptive == S265_B_ADAPT_TRELLIS)
        {
            if (numFrames > 1)
            {
                char best_paths[S265_BFRAME_MAX + 1][S265_LOOKAHEAD_MAX + 1] = { "", "P" };
                int best_path_index = numFrames % (S265_BFRAME_MAX + 1);

                /* Perform the frame type analysis. */
                for (int j = 2; j <= numFrames; j++)
                    slicetypePath(frames, j, best_paths);

                numBFrames = (int)strspn(best_paths[best_path_index], "B");

                /* Load the results of the analysis into the frame types. */
                for (int j = 1; j < numFrames; j++)
                    frames[j]->sliceType = best_paths[best_path_index][j - 1] == 'B' ? S265_TYPE_B : S265_TYPE_P;
            }
            frames[numFrames]->sliceType = S265_TYPE_P;
        }
        else if (m_param->bFrameAdaptive == S265_B_ADAPT_FAST)
        {
            CostEstimateGroup estGroup(*this, frames);

            int64_t cost1p0, cost2p0, cost1b1, cost2p1;

            for (int i = 0; i <= numFrames - 2; )
            {
                cost2p1 = estGroup.singleCost(i + 0, i + 2, i + 2, true);
                if (frames[i + 2]->intraMbs[2] > cuCount / 2)
                {
                    frames[i + 1]->sliceType = S265_TYPE_P;
                    frames[i + 2]->sliceType = S265_TYPE_P;
                    i += 2;
                    continue;
                }

                cost1b1 = estGroup.singleCost(i + 0, i + 2, i + 1);
                cost1p0 = estGroup.singleCost(i + 0, i + 1, i + 1);
                cost2p0 = estGroup.singleCost(i + 1, i + 2, i + 2);

                if (cost1p0 + cost2p0 < cost1b1 + cost2p1)
                {
                    frames[i + 1]->sliceType = S265_TYPE_P;
                    i += 1;
                    continue;
                }

// arbitrary and untuned
#define INTER_THRESH 300
#define P_SENS_BIAS (50 - m_param->bFrameBias)
                frames[i + 1]->sliceType = S265_TYPE_B;

                int j;
                for (j = i + 2; j <= S265_MIN(i + m_param->bframes, numFrames - 1); j++)
                {
                    int64_t pthresh = S265_MAX(INTER_THRESH - P_SENS_BIAS * (j - i - 1), INTER_THRESH / 10);
                    int64_t pcost = estGroup.singleCost(i + 0, j + 1, j + 1, true);
                    if (pcost > pthresh * cuCount || frames[j + 1]->intraMbs[j - i + 1] > cuCount / 3)
                        break;
                    frames[j]->sliceType = S265_TYPE_B;
                }

                frames[j]->sliceType = S265_TYPE_P;
                i = j;
            }
            frames[numFrames]->sliceType = S265_TYPE_P;
            numBFrames = 0;
            while (numBFrames < numFrames && frames[numBFrames + 1]->sliceType == S265_TYPE_B)
                numBFrames++;
        }
        else
        {
            //将BREF的决定更换一个位置，此处先注释掉
            // if(m_param->bBPyramid == S265_B_PYRAMID_HIER)
            // {
                
            //     numBFrames = S265_MIN(numFrames - 1, m_param->bframes);
            //     for (int j = 1; j < numFrames; j = j + numBFrames + 1)
            //     {
            //         int32_t gop_id = 1;
            //         int32_t numBFramesReal = S265_MIN(numBFrames, numFrames-j);
            //         set_gop_info_internal( frames[j+ numBFramesReal], &gop_id, numBFramesReal+2, 0);
            //         set_gop_info_random_access( frames, &gop_id, j, j+ numBFramesReal-1, numBFramesReal+2, 1);
            //         frames[j+ numBFramesReal]->sliceType = S265_TYPE_P;
            //     }
                    
            // }
            // else
            {
                numBFrames = S265_MIN(numFrames - 1, m_param->bframes);
                for (int j = 1; j < numFrames; j++)
                    frames[j]->sliceType = (j % (numBFrames + 1)) ? S265_TYPE_B : S265_TYPE_P;

                frames[numFrames]->sliceType = S265_TYPE_P;
            }
        }



        int zoneRadl = m_param->rc.zonefileCount && m_param->bResetZoneConfig ? m_param->rc.zones->zoneParam->radl : 0;
        bool bForceRADL = zoneRadl || (m_param->radl && (m_param->keyframeMax == m_param->keyframeMin));
        bool bLastMiniGop = (framecnt >= m_param->bframes + 1) ? false : true;
        int radl = m_param->radl ? m_param->radl : zoneRadl;
        int preRADL = m_lastKeyframe + m_param->keyframeMax - radl - 1; /*Frame preceeding RADL in POC order*/
        if (bForceRADL && (frames[0]->frameNum == preRADL) && !bLastMiniGop)
        {
            int j = 1;
            numBFrames = m_param->radl ? m_param->radl : zoneRadl;
            for (; j <= numBFrames; j++)
                frames[j]->sliceType = S265_TYPE_B;
            frames[j]->sliceType = S265_TYPE_I;
        }
        else /* Check scenecut and RADL on the first minigop. */
        {
            for (int j = 1; j < numBFrames + 1; j++)
            {
                bool isNextScenecut = false;
                if (!m_param->bHistBasedSceneCut || (m_param->bHistBasedSceneCut && frames[j + 1]->bScenecut))
                    isNextScenecut = scenecut(frames, j, j + 1, false, origNumFrames);
                if (isNextScenecut || (bForceRADL && frames[j]->frameNum == preRADL))
                {
                    frames[j]->sliceType = S265_TYPE_P;
                    numAnalyzed = j;
                    break;
                }
            }
        }
        resetStart = bKeyframe ? 1 : S265_MIN(numBFrames + 2, numAnalyzed + 1);
    }
    else
    {
        for (int j = 1; j <= numFrames; j++)
            frames[j]->sliceType = S265_TYPE_P;

        resetStart = bKeyframe ? 1 : 2;
    }

    /*这里帧类型不会再改变了，确定B帧的层级*/
    if (m_param->bBPyramid == S265_B_PYRAMID_HIER)
    {
        int bframes = 0;
        int curNonB = 0;
        for (int j = 1; j <= numFrames; j++)
        {
            assert(frames[j]->sliceType != S265_TYPE_AUTO);
            //这里应该没有auto的情况了，但是先写上以免有其他问题
            if (IS_S265_TYPE_B(frames[j]->sliceType) || frames[j]->sliceType == S265_TYPE_AUTO)
                bframes++;
            else
            {
                //if (frames[j]->sliceType == S265_TYPE_P)
                {
                    if (bframes > 0)
                    {
                        int32_t gop_id = 1;
                        set_gop_info_internal(frames[j], &gop_id, bframes + 1, 0);
                        set_gop_info_random_access(frames, &gop_id, curNonB + 1, j - 1, bframes + 1, 1);
                    }
                    else
                    {
                        int32_t gop_id = 1;
                        set_gop_info_internal(frames[j], &gop_id, 1, 0);
                    }
                }
                curNonB = j;
                bframes = 0;
            }
        }
    }

    if (m_param->bAQMotion)
        aqMotion(frames, bKeyframe);//在低分辨率上做过搜索后，每个8x8 should have an lowres mv，这里计算基于运动适量大小的aq

    if (m_param->rc.cuTree)
    {
        if (m_param->rc.cuTreeType)
        {
            cuTree2(frames, S265_MIN(numFrames, m_param->keyframeMax), bKeyframe);
        }
        else
        {
            cuTree(frames, S265_MIN(numFrames, m_param->keyframeMax), bKeyframe);
        }
    }

    if (m_param->gopLookahead && (keyFrameLimit >= 0) && (keyFrameLimit <= m_param->bframes + 1) && !m_extendGopBoundary)
        keyintLimit = keyFrameLimit;

    if (!m_param->bIntraRefresh)
        for (int j = keyintLimit + 1; j <= numFrames; j += m_param->keyframeMax)
        {
            frames[j]->sliceType = S265_TYPE_I;
            frames[j]->i_level = 0;
            frames[j]->i_gop_size = 1;
            frames[j]->i_bref = 1;
            frames[j]->i_max_depth = 0;
            frames[j]->i_gop_id = 1;
            frames[j]->i_temporal_id = 0;
            resetStart = S265_MIN(resetStart, j + 1);
        }

    if (bIsVbvLookahead)
        vbvLookahead(frames, numFrames, bKeyframe);
    int maxp1 = S265_MIN(m_param->bframes + 1, origNumFrames);

    /* Restore frame types for all frames that haven't actually been decided yet. */
    for (int j = resetStart; j <= numFrames; j++)
    {
        frames[j]->sliceType = S265_TYPE_AUTO;
        frames[j]->i_level = 0;
        frames[j]->i_gop_size = 1;
        frames[j]->i_bref = 0;
        frames[j]->i_max_depth = 0;
        frames[j]->i_gop_id = 1;
        frames[j]->i_temporal_id = 0;

        /* If any frame marked as scenecut is being restarted for sliceDecision, 
         * undo scene Transition flag */
        if (j <= maxp1 && frames[j]->bScenecut && m_isSceneTransition)
            m_isSceneTransition = false;
    }
}

bool Lookahead::scenecut(Lowres **frames, int p0, int p1, bool bRealScenecut, int numFrames)
{
    /* Only do analysis during a normal scenecut check. */
    if (bRealScenecut && m_param->bframes)
    {
        int origmaxp1 = p0 + 1;
        /* Look ahead to avoid coding short flashes as scenecuts. */
        origmaxp1 += m_param->bframes;
        int maxp1 = S265_MIN(origmaxp1, numFrames);
        bool fluctuate = false;
        bool noScenecuts = false;
        int64_t avgSatdCost = 0;
        if (frames[p0]->costEst[p1 - p0][0] > -1)
            avgSatdCost = frames[p0]->costEst[p1 - p0][0];
        int cnt = 1;
        /* Where A and B are scenes: AAAAAABBBAAAAAA
         * If BBB is shorter than (maxp1-p0), it is detected as a flash
         * and not considered a scenecut. */
        for (int cp1 = p1; cp1 <= maxp1; cp1++)
        {
            if (!scenecutInternal(frames, p0, cp1, false) && !m_param->bHistBasedSceneCut)
            {
                /* Any frame in between p0 and cur_p1 cannot be a real scenecut. */
                for (int i = cp1; i > p0; i--)
                {
                    frames[i]->bScenecut = false;
                    noScenecuts = false;
                }
            }
            else if ((m_param->bHistBasedSceneCut && frames[cp1]->m_bIsMaxThres) || scenecutInternal(frames, cp1 - 1, cp1, false))
            {
                /* If current frame is a Scenecut from p0 frame as well as Scenecut from
                 * preceeding frame, mark it as a Scenecut */
                frames[cp1]->bScenecut = true;
                noScenecuts = true;
            }

            /* compute average satdcost of all the frames in the mini-gop to confirm 
             * whether there is any great fluctuation among them to rule out false positives */
            S265_CHECK(frames[cp1]->costEst[cp1 - p0][0]!= -1, "costEst is not done \n");
            avgSatdCost += frames[cp1]->costEst[cp1 - p0][0];
            cnt++;
        }

        /* Identify possible scene fluctuations by comparing the satd cost of the frames.
         * This could denote the beginning or ending of scene transitions.
         * During a scene transition(fade in/fade outs), if fluctuate remains false,
         * then the scene had completed its transition or stabilized */
        if (noScenecuts)
        {
            fluctuate = false;
            avgSatdCost /= cnt;
            for (int i = p1; i <= maxp1; i++)
            {
                int64_t curCost  = frames[i]->costEst[i - p0][0];
                int64_t prevCost = frames[i - 1]->costEst[i - 1 - p0][0];
                if (fabs((double)(curCost - avgSatdCost)) > 0.1 * avgSatdCost || 
                    fabs((double)(curCost - prevCost)) > 0.1 * prevCost)
                {
                    fluctuate = true;
                    if (!m_isSceneTransition && frames[i]->bScenecut)
                    {
                        m_isSceneTransition = true;
                        /* just mark the first scenechange in the scene transition as a scenecut. */
                        for (int j = i + 1; j <= maxp1; j++)
                            frames[j]->bScenecut = false;
                        break;
                    }
                }
                frames[i]->bScenecut = false;
            }
        }
        if (!fluctuate && !noScenecuts)
            m_isSceneTransition = false; /* Signal end of scene transitioning */
    }

    if (m_param->csvLogLevel >= 2)
    {
        int64_t icost = frames[p1]->costEst[0][0];
        int64_t pcost = frames[p1]->costEst[p1 - p0][0];
        frames[p1]->ipCostRatio = (double)icost / pcost;
    }

    /* A frame is always analysed with bRealScenecut = true first, and then bRealScenecut = false,
       the former for I decisions and the latter for P/B decisions. It's possible that the first 
       analysis detected scenecuts which were later nulled due to scene transitioning, in which 
       case do not return a true scenecut for this frame */

    if (!frames[p1]->bScenecut)
        return false;
    /* Check only scene transitions if max threshold */
    if (m_param->bHistBasedSceneCut && frames[p1]->m_bIsMaxThres)
        return frames[p1]->bScenecut;

    return scenecutInternal(frames, p0, p1, bRealScenecut);
}

bool Lookahead::scenecutInternal(Lowres **frames, int p0, int p1, bool bRealScenecut)
{
    Lowres *frame = frames[p1];

    CostEstimateGroup estGroup(*this, frames);
    estGroup.singleCost(p0, p1, p1);
    int64_t icost = frame->costEst[0][0];
    int64_t pcost = frame->costEst[p1 - p0][0];
    int gopSize = (frame->frameNum - m_lastKeyframe) % m_param->keyframeMax;
    float threshMax = (float)(m_param->scenecutThreshold / 100.0);
    /* magic numbers pulled out of thin air */
    float threshMin = (float)(threshMax * 0.25);
    double bias = m_param->scenecutBias;
    if (m_param->bHistBasedSceneCut)
    {
        double minT = TEMPORAL_SCENECUT_THRESHOLD * (1 + m_param->edgeTransitionThreshold);
        if (frame->interPCostPercDiff > minT || frame->intraCostPercDiff > minT)
        {
            if (bRealScenecut && frame->bScenecut)
                s265_log(m_param, S265_LOG_DEBUG, "scene cut at %d \n", frame->frameNum);
            return frame->bScenecut;
        }
        else
            return false;
    }
    else if (bRealScenecut)
    {
        if (m_param->keyframeMin == m_param->keyframeMax)
            threshMin = threshMax;
        if (gopSize <= m_param->keyframeMin / 4 || m_param->bIntraRefresh)
            bias = threshMin / 4;
        else if (gopSize <= m_param->keyframeMin)
            bias = threshMin * gopSize / m_param->keyframeMin;
        else
        {
            bias = threshMin
                + (threshMax - threshMin)
                * (gopSize - m_param->keyframeMin)
                / (m_param->keyframeMax - m_param->keyframeMin);
        }
    }
    bool res = pcost >= (1.0 - bias) * icost;
    if (res && bRealScenecut)
    {
        int imb = frame->intraMbs[p1 - p0];
        int pmb = m_8x8Blocks - imb;
        s265_log(m_param, S265_LOG_DEBUG, "scene cut at %d Icost:%d Pcost:%d ratio:%.4f bias:%.4f gop:%d (imb:%d pmb:%d)\n",
                 frame->frameNum, icost, pcost, 1. - (double)pcost / icost, bias, gopSize, imb, pmb);
    }
    return res;
}

void Lookahead::slicetypePath(Lowres **frames, int length, char(*best_paths)[S265_LOOKAHEAD_MAX + 1])
{
    char paths[2][S265_LOOKAHEAD_MAX + 1];
    int num_paths = S265_MIN(m_param->bframes + 1, length);
    int64_t best_cost = 1LL << 62;
    int idx = 0;

    /* Iterate over all currently possible paths */
    for (int path = 0; path < num_paths; path++)
    {
        /* Add suffixes to the current path */
        int len = length - (path + 1);
        memcpy(paths[idx], best_paths[len % (S265_BFRAME_MAX + 1)], len);
        memset(paths[idx] + len, 'B', path);
        strcpy(paths[idx] + len + path, "P");

        /* Calculate the actual cost of the current path */
        int64_t cost = slicetypePathCost(frames, paths[idx], best_cost);
        if (cost < best_cost)
        {
            best_cost = cost;
            idx ^= 1;
        }
    }

    //printf("length: %d, cost: %d, path: %s \n", length, best_cost, paths[idx ^ 1]);
    /* Store the best path. */
    memcpy(best_paths[length % (S265_BFRAME_MAX + 1)], paths[idx ^ 1], length);
}

static void addcost(CostEstimateGroup *estGroup, Lowres **frames, int left_ref, int right_ref, int64_t threshold, int64_t *cost)
{
    if (right_ref - left_ref > 1)
	{
        int middle = left_ref + ( right_ref - left_ref ) / 2;
        *cost += estGroup->singleCost(left_ref, right_ref, middle);
        if (middle > left_ref && *cost < threshold)
            addcost(estGroup, frames, left_ref, middle, threshold, cost);
        if (right_ref > middle && *cost < threshold)
            addcost(estGroup, frames, middle, right_ref, threshold, cost);
    }
}

int64_t Lookahead::slicetypePathCost(Lowres **frames, char *path, int64_t threshold)
{
    int64_t cost = 0;
    int loc = 1;
    int cur_p = 0;

    CostEstimateGroup estGroup(*this, frames);

    path--; /* Since the 1st path element is really the second frame */
    while (path[loc])
    {
        int next_p = loc;
        /* Find the location of the next P-frame. */
        while (path[next_p] != 'P')
            next_p++;

        /* Add the cost of the P-frame found above */
        cost += estGroup.singleCost(cur_p, next_p, next_p);

        /* Early terminate if the cost we have found is larger than the best path cost so far */
        if (cost > threshold)
            break;

        if (m_param->bBPyramid && next_p - cur_p > 2)
        {
            if( m_param->bBPyramid == S265_B_PYRAMID_HIER )
            {
                addcost( &estGroup, frames, cur_p, next_p, threshold, &cost);
            }
            else
            {
            int middle = cur_p + (next_p - cur_p) / 2;
            cost += estGroup.singleCost(cur_p, next_p, middle);

            for (int next_b = loc; next_b < middle && cost < threshold; next_b++)
                cost += estGroup.singleCost(cur_p, middle, next_b);

            for (int next_b = middle + 1; next_b < next_p && cost < threshold; next_b++)
                cost += estGroup.singleCost(middle, next_p, next_b);
            }

        }
        else
        {
            for (int next_b = loc; next_b < next_p && cost < threshold; next_b++)
                cost += estGroup.singleCost(cur_p, next_p, next_b);
        }

        loc = next_p + 1;
        cur_p = next_p;
    }

    return cost;
}

void Lookahead::aqMotion(Lowres **frames, bool bIntra)
{
    if (!bIntra)
    {
        int curnonb = 0, lastnonb = 1;
        int bframes = 0, i = 1;
        while (frames[lastnonb]->sliceType != S265_TYPE_P)
            lastnonb++;
        bframes = lastnonb - 1;
        if (m_param->bBPyramid && bframes > 1)
        {
            int middle = (bframes + 1) / 2;
            for (i = 1; i < lastnonb; i++)
            {
                int p0 = i > middle ? middle : curnonb;
                int p1 = i < middle ? middle : lastnonb;
                if (i != middle)
                    calcMotionAdaptiveQuantFrame(frames, p0, p1, i);
            }
            calcMotionAdaptiveQuantFrame(frames, curnonb, lastnonb, middle);
        }
        else
            for (i = 1; i < lastnonb; i++)
                calcMotionAdaptiveQuantFrame(frames, curnonb, lastnonb, i);
        calcMotionAdaptiveQuantFrame(frames, curnonb, lastnonb, lastnonb);
    }
}

void Lookahead::calcMotionAdaptiveQuantFrame(Lowres **frames, int p0, int p1, int b)
{
    int listDist[2] = { b - p0, p1 - b };
    int32_t strideInCU = m_8x8Width;
    double qp_adj = 0, avg_adj = 0, avg_adj_pow2 = 0, sd;
    for (uint16_t blocky = 0; blocky < m_8x8Height; blocky++)
    {
        int cuIndex = blocky * strideInCU;
        for (uint16_t blockx = 0; blockx < m_8x8Width; blockx++, cuIndex++)
        {
            int32_t lists_used = frames[b]->lowresCosts[b - p0][p1 - b][cuIndex] >> LOWRES_COST_SHIFT;
            double displacement = 0;
            for (uint16_t list = 0; list < 2; list++)
            {
                if ((lists_used >> list) & 1)
                {
                    MV *mvs = frames[b]->lowresMvs[list][listDist[list]];
                    int32_t x = mvs[cuIndex].x;
                    int32_t y = mvs[cuIndex].y;
                    // NOTE: the dynamic range of abs(x) and abs(y) is 15-bits
                    displacement += sqrt((double)(abs(x) * abs(x)) + (double)(abs(y) * abs(y)));
                }
                else
                    displacement += 0.0;
            }
            if (lists_used == 3)
                displacement = displacement / 2;
            qp_adj = pow(displacement, 0.1);
            frames[b]->qpAqMotionOffset[cuIndex] = qp_adj;
            avg_adj += qp_adj;
            avg_adj_pow2 += qp_adj * qp_adj;
        }
    }
    avg_adj /= m_cuCount;
    avg_adj_pow2 /= m_cuCount;
    sd = sqrt((avg_adj_pow2 - (avg_adj * avg_adj)));
    if (sd > 0)
    {
        for (uint16_t blocky = 0; blocky < m_8x8Height; blocky++)
        {
            int cuIndex = blocky * strideInCU;
            for (uint16_t blockx = 0; blockx < m_8x8Width; blockx++, cuIndex++)
            {
                qp_adj = frames[b]->qpAqMotionOffset[cuIndex];
                qp_adj = (qp_adj - avg_adj) / sd;
                if (qp_adj > 1)
                {
                    frames[b]->qpAqOffset[cuIndex] += qp_adj;
                    frames[b]->qpCuTreeOffset[cuIndex] += qp_adj;
                    frames[b]->invQscaleFactor[cuIndex] += s265_exp2fix8(qp_adj);
                }
            }
        }
    }
}
//// b_intra 表明了mb_tree 传递cost时 是在frames[1] 终止(此时frames[0] 已经被编码)还是在frames[0] 终止(此时frames[0]是一个keyframe 还没有被编码)
void Lookahead::cuTree(Lowres **frames, int numframes, bool bIntra)
{
    int idx = !bIntra;
    int lastnonb, curnonb = 1;
    int bframes = 0;

    s265_emms();
    double totalDuration = 0.0;
    for (int j = 0; j <= numframes; j++)
        totalDuration += (double)m_param->fpsDenom / m_param->fpsNum;

    double averageDuration = totalDuration / (numframes + 1);

    int i = numframes;//本次前向可分析帧的数量

    while (i > 0 && frames[i]->sliceType == S265_TYPE_B)
        i--;

    lastnonb = i; //从后往前找，找到第一个非B帧 (寻找最后一个非B帧)

    /* Lookaheadless MB-tree is not a theoretically distinct case; the same extrapolation could
     * be applied to the end of a lookahead buffer of any size.  However, it's most needed when
     * lookahead=0, so that's what's currently implemented. */
    if (!m_param->lookaheadDepth) //如果lookahead前向分析帧数量为0
    {
        if (bIntra)// //当frames[0]是intra时，由于没有lookahead，i_propagate_cost 为 0
        {
            memset(frames[0]->propagateCost, 0, m_cuCount * sizeof(uint16_t));
            //直接应用aq结果到f_qp_offset，此时mbtree没有效果
            if (m_param->rc.qgSize == 8)
                memcpy(frames[0]->qpCuTreeOffset, frames[0]->qpAqOffset, m_cuCount * 4 * sizeof(double));
            else
                memcpy(frames[0]->qpCuTreeOffset, frames[0]->qpAqOffset, m_cuCount * sizeof(double));
            return;
        }
        //非intra情况下：??? 
        std::swap(frames[lastnonb]->propagateCost, frames[0]->propagateCost);//指针交换
        memset(frames[0]->propagateCost, 0, m_cuCount * sizeof(uint16_t));
    }
    else
    {
        if (lastnonb < idx)
            return;
        // 在最远处的第一个非B帧位置开始/截断（最后被编码的一帧） propagate_cost 进行 frames[last_nonb] 的清空
        memset(frames[lastnonb]->propagateCost, 0, m_cuCount * sizeof(uint16_t));
    }

    CostEstimateGroup estGroup(*this, frames);// 定义一个对象

    if( m_param->bBPyramid == S265_B_PYRAMID_HIER )
    {
        // 首个被编码的非B帧
        int first_nonb = bIntra ? 0 : frames[1]->i_gop_size;
        for( int32_t t = first_nonb + 1; t <= i; t++ )
        {
            if( frames[t]->sliceType == S265_TYPE_BREF ) // 所有的BREf 都首先清空
                memset( frames[t]->propagateCost, 0, m_cuCount * sizeof(uint16_t) );
        }
    }

    while (i-- > idx)
    {
        curnonb = i;
        // while (frames[curnonb]->sliceType == S265_TYPE_B && curnonb > 0)
        //     curnonb--;
        // //从last_nonb位置的前一个位置(最后一个非B帧位置的之前一个位置) 往前寻找 离last_nonb最近的一个非B帧
        while (IS_S265_TYPE_B(frames[curnonb]->sliceType) && curnonb > 0)
            curnonb--;

        if (curnonb < idx) //idx 为首个未编码帧的位置 0时，表示未编码的帧为i帧在frames[0]位置，1时表示frames[0] 为p帧且已经被编码
            break;
        // 计算最后一个非B帧(last_nonb)  参考其前一个非B帧的P0(cur_nonb) 的cost
        estGroup.singleCost(curnonb, lastnonb, lastnonb);
        // cur_nonb 的propagateCost 需要先清零
        memset(frames[curnonb]->propagateCost, 0, m_cuCount * sizeof(uint16_t));
        bframes = lastnonb - curnonb - 1;//两个连续非B帧之间的B帧个数
        if (m_param->bBPyramid && bframes > 1)//如果有参考B帧存在 //此处bframes应该大于2 更好
        {
            if (m_param->bBPyramid == S265_B_PYRAMID_HIER)
            {
                int32_t gop_id = frames[lastnonb]->i_gop_size;
                while (i > curnonb)
                {
                    int32_t p0, p1, b = idx;
                    int gop_id_exist = 0;
                    for (int32_t cur = lastnonb - 1; cur > curnonb; cur--)
                    {
                        if (frames[cur]->i_gop_id == gop_id)
                        {
                            b = cur;
                            gop_id_exist = 1;
                            break;
                        }
                    }
                    gop_id--;

                    if (!gop_id_exist)
                        continue;
                    p0 = p1 = b;
                    if (!IS_S265_TYPE_I(frames[b]->sliceType))
                    {
                        for (int32_t bef = b - 1; bef >= curnonb; bef--)
                        {
                            if (frames[bef] && frames[bef]->i_bref && frames[bef]->i_level <= frames[b]->i_level)
                            {
                                p0 = bef;
                                break;
                            }
                        }
                    }
                    if (IS_S265_TYPE_B(frames[b]->sliceType))
                    {
                        for (int32_t aft = b + 1; aft <= lastnonb; aft++)
                        {
                            if (frames[aft] && frames[aft]->i_bref && frames[aft]->i_level <= frames[b]->i_level)
                            {
                                p1 = aft;
                                break;
                            }
                        }
                    }
                    if (p0 != p1)
                    {
                        estGroup.singleCost(p0, p1, b);
                        estimateCUPropagate(frames, averageDuration, p0, p1, b, frames[b]->i_bref);
                    }
                    i--;
                }
            }
            else
            {
                int middle = (bframes + 1) / 2 + curnonb; // 计算Bref帧的位置
                estGroup.singleCost(curnonb, lastnonb, middle); //b设为 中间的Bref帧 参考p0(cur_nonb) 以及p1(last_nonb) cost 先行计算
                memset(frames[middle]->propagateCost, 0, m_cuCount * sizeof(uint16_t));
                while (i > curnonb)
                {
                    int p0 = i > middle ? middle : curnonb;
                    int p1 = i < middle ? middle : lastnonb;
                    if (i != middle)// // 先对 last_nonb 到 middle 之间的B帧进行tree_progagate  再对 middle  到 cur_nonb 之间的B帧进行 tree_progagate
                    {
                        estGroup.singleCost(p0, p1, i);
                        estimateCUPropagate(frames, averageDuration, p0, p1, i, 0); // 最后的0表示当前frames[i] 为非参考帧
                    }
                    i--;
                }
                //最后将middle cost 进行propagate 到 cur_nonb与last_nonb
                estimateCUPropagate(frames, averageDuration, curnonb, lastnonb, middle, 1);
            }
        }
        else
        {
            while (i > curnonb)//所有B帧从后往前 依次进行cost计算与progagate
            {
                estGroup.singleCost(curnonb, lastnonb, i);
                estimateCUPropagate(frames, averageDuration, curnonb, lastnonb, i, 0);//所有B帧frames[i]都不会被参考
                i--;
            }
        }
        //最后将 last_nonb 的cost propagate到 cur_nonb
        estimateCUPropagate(frames, averageDuration, curnonb, lastnonb, lastnonb, 1);
        lastnonb = curnonb; //update last_nonb 后继续循环对cost 进行propagate
    }
// 到这里，last_nonb 已经更新为frames[]队列里面的第一个未编码的非B帧 如果 frames[0] 为 i 且未编码(即第二次进来) 则 last_nonb 为frames[0]
// 如果 frames[0] 为 p 则其已经编码，此时 last_nonb 为即将要编码的minigop在poc顺序上的最后一帧
    if (!m_param->lookaheadDepth)
    {
        estGroup.singleCost(0, lastnonb, lastnonb);
        estimateCUPropagate(frames, averageDuration, 0, lastnonb, lastnonb, 1);
        std::swap(frames[lastnonb]->propagateCost, frames[0]->propagateCost);
    }

    //printf("current qp minigop: %d->%d\n", frames[0]->frameNum, frames[lastnonb]->frameNum);
    //printf("set qp from: %d->%d\n", frames[lastnonb]->frameNum, frames[lastnonb + bframes + 1]->frameNum);


    cuTreeFinish(frames[lastnonb], averageDuration, lastnonb);// 计算frames[last_nonb]中的qp-delta信息
    
    // ？？? 为什么没有启用vbv 时 才进行Bref帧的cutree-qp-delta计算。
    if (m_param->bBPyramid && bframes > 1 && !m_param->rc.vbvBufferSize)
    {
        if (m_param->bBPyramid == S265_B_PYRAMID_HIER)
        {
            for (int32_t gop_id = 1; gop_id <= bframes; gop_id++)
            {
                if (frames[lastnonb + gop_id]->i_bref)
                {
                    cuTreeFinish(frames[lastnonb + gop_id], averageDuration, 0);
                }
            }
        }
        else
            cuTreeFinish(frames[lastnonb + (bframes + 1) / 2], averageDuration, 0);
    }
}


void Lookahead::cuTree2(Lowres **frames, int numframes, bool bIntra)
{
    int idx = !bIntra;
    int lastnonb, curnonb = 1;
    int bframes = 0;

    s265_emms();
    double totalDuration = 0.0;
    for (int j = 0; j <= numframes; j++)
        totalDuration += (double)m_param->fpsDenom / m_param->fpsNum;

    double averageDuration = totalDuration / (numframes + 1);

    int i = numframes;

    while (i > 0 && frames[i]->sliceType == S265_TYPE_B)
        i--;

    lastnonb = i;

    /* Lookaheadless MB-tree is not a theoretically distinct case; the same extrapolation could
     * be applied to the end of a lookahead buffer of any size.  However, it's most needed when
     * lookahead=0, so that's what's currently implemented. */
    if (!m_param->lookaheadDepth)
    {
        if (bIntra)
        {
            memset(frames[0]->propagateCost, 0, m_cuCount * sizeof(uint16_t));
            if (m_param->rc.qgSize == 8)
                memcpy(frames[0]->qpCuTreeOffset, frames[0]->qpAqOffset, m_cuCount * 4 * sizeof(double));
            else
                memcpy(frames[0]->qpCuTreeOffset, frames[0]->qpAqOffset, m_cuCount * sizeof(double));
            return;
        }
        std::swap(frames[lastnonb]->propagateCost, frames[0]->propagateCost);
        memset(frames[0]->propagateCost, 0, m_cuCount * sizeof(uint16_t));
    }
    else
    {
        if (lastnonb < idx)
            return;
        memset(frames[lastnonb]->propagateCost, 0, m_cuCount * sizeof(uint16_t));
    }

    CostEstimateGroup estGroup(*this, frames);

    if( m_param->bBPyramid == S265_B_PYRAMID_HIER )
    {
        //int first_nonb = bIntra ? 0 : frames[1]->i_gop_size ;
        int first_nonb =  0 ;
        for( int32_t t = first_nonb + 1; t <= i; t++ )
        {
            if( frames[t]->sliceType == S265_TYPE_BREF ) // i_bref
                memset( frames[t]->propagateCost, 0, m_cuCount * sizeof(uint16_t) );
        }
    }
    int lastnonb2 = lastnonb;
    int lastnonb3 = lastnonb2;

    while (i-- > idx)
    {
        curnonb = i;
        // while (frames[curnonb]->sliceType == S265_TYPE_B && curnonb > 0)
        //     curnonb--;
        while (IS_S265_TYPE_B(frames[curnonb]->sliceType) && curnonb > 0)
            curnonb--;

        // if (curnonb < idx)
        //     break;

        estGroup.singleCost(curnonb, lastnonb, lastnonb);

        memset(frames[curnonb]->propagateCost, 0, m_cuCount * sizeof(uint16_t));
        bframes = lastnonb - curnonb - 1;
        if (m_param->bBPyramid && bframes > 1)
        {
            if (m_param->bBPyramid == S265_B_PYRAMID_HIER)
            {
                int32_t gop_id = frames[lastnonb]->i_gop_size;
                while (i > curnonb)
                {
                    int32_t p0, p1, b = idx;
                    int gop_id_exist = 0;
                    for (int32_t cur = lastnonb - 1; cur > curnonb; cur--)
                    {
                        if (frames[cur]->i_gop_id == gop_id)
                        {
                            b = cur;
                            gop_id_exist = 1;
                            break;
                        }
                    }
                    gop_id--;

                    if (!gop_id_exist)
                        continue;
                    p0 = p1 = b;
                    if (!IS_S265_TYPE_I(frames[b]->sliceType))
                    {
                        for (int32_t bef = b - 1; bef >= curnonb; bef--)
                        {
                            if (frames[bef] && frames[bef]->i_bref && frames[bef]->i_level <= frames[b]->i_level)
                            {
                                p0 = bef;
                                break;
                            }
                        }
                    }
                    if (IS_S265_TYPE_B(frames[b]->sliceType))
                    {
                        for (int32_t aft = b + 1; aft <= lastnonb; aft++)
                        {
                            if (frames[aft] && frames[aft]->i_bref && frames[aft]->i_level <= frames[b]->i_level)
                            {
                                p1 = aft;
                                break;
                            }
                        }
                    }
                    if (p0 != p1)
                    {
                        estGroup.singleCost(p0, p1, b);
                        estimateCUPropagate(frames, averageDuration, p0, p1, b, frames[b]->i_bref);
                    }
                    i--;
                }
            }
            else
            {
                int middle = (bframes + 1) / 2 + curnonb;
                estGroup.singleCost(curnonb, lastnonb, middle);
                memset(frames[middle]->propagateCost, 0, m_cuCount * sizeof(uint16_t));
                while (i > curnonb)
                {
                    int p0 = i > middle ? middle : curnonb;
                    int p1 = i < middle ? middle : lastnonb;
                    if (i != middle)
                    {
                        estGroup.singleCost(p0, p1, i);
                        estimateCUPropagate(frames, averageDuration, p0, p1, i, 0);
                    }
                    i--;
                }

                estimateCUPropagate(frames, averageDuration, curnonb, lastnonb, middle, 1);
            }
        }
        else
        {
            while (i > curnonb)
            {
                estGroup.singleCost(curnonb, lastnonb, i);
                estimateCUPropagate(frames, averageDuration, curnonb, lastnonb, i, 0);
                i--;
            }
        }
        estimateCUPropagate(frames, averageDuration, curnonb, lastnonb, lastnonb, 1);
        lastnonb3 = lastnonb2;
        lastnonb2 = lastnonb;
        lastnonb = curnonb;
    }

    if (!m_param->lookaheadDepth)
    {
        estGroup.singleCost(0, lastnonb, lastnonb);
        estimateCUPropagate(frames, averageDuration, 0, lastnonb, lastnonb, 1);
        std::swap(frames[lastnonb]->propagateCost, frames[0]->propagateCost);
    }

    //printf("current qp minigop: %d->%d\n", frames[0]->frameNum, frames[lastnonb]->frameNum);
    //printf("set qp from: %d->%d\n", frames[lastnonb]->frameNum, frames[lastnonb + bframes + 1]->frameNum);
    

    //first mini gop
    if (curnonb == idx)
    //if (curnonb < idx)
    {
        cuTreeFinish(frames[lastnonb], averageDuration, lastnonb);
    }
    if (m_param->bBPyramid && bframes > 1 && !m_param->rc.vbvBufferSize)
    {
        if (m_param->bBPyramid == S265_B_PYRAMID_HIER)
        {
            for (int32_t gop_id = 1; gop_id <= bframes; gop_id++)
            {
                if (frames[lastnonb + gop_id]->i_bref)
                {
                    cuTreeFinish(frames[lastnonb + gop_id], averageDuration, 0);
                }
            }
        }
        else
            cuTreeFinish(frames[lastnonb + (bframes + 1) / 2], averageDuration, 0);
    }

        //second mini gop
    if (lastnonb3 > lastnonb2)
    {
        cuTreeFinish(frames[lastnonb + bframes + 1], averageDuration, lastnonb2 - lastnonb);
        int bframes2 = lastnonb3 - lastnonb2-1;
        if (m_param->bBPyramid && bframes2 > 1 && !m_param->rc.vbvBufferSize)
        {
            if (m_param->bBPyramid == S265_B_PYRAMID_HIER)
            {
                for (int32_t gop_id = 1; gop_id <= bframes2; gop_id++)
                {
                    if (frames[lastnonb + gop_id]->i_bref)
                    {
                        cuTreeFinish(frames[lastnonb2 + gop_id], averageDuration, 0);
                    }
                }
            }
            else
                cuTreeFinish(frames[lastnonb2 + (bframes2 + 1) / 2], averageDuration, 0);
        }
    }
}

void Lookahead::estimateCUPropagate(Lowres **frames, double averageDuration, int p0, int p1, int b, int referenced)
{
    uint16_t *refCosts[2] = { frames[p0]->propagateCost, frames[p1]->propagateCost };
    int32_t distScaleFactor = (((b - p0) << 8) + ((p1 - p0) >> 1)) / (p1 - p0);
    int32_t bipredWeight = m_param->bEnableWeightedBiPred ? 64 - (distScaleFactor >> 2) : 32;
    int32_t bipredWeights[2] = { bipredWeight, 64 - bipredWeight };
    int listDist[2] = { b - p0, p1 - b };

    memset(m_scratch, 0, m_8x8Width * sizeof(int));

    uint16_t *propagateCost = frames[b]->propagateCost;

    s265_emms();
    double fpsFactor = CLIP_DURATION((double)m_param->fpsDenom / m_param->fpsNum) / CLIP_DURATION(averageDuration);

    /* For non-referred frames the source costs are always zero, so just memset one row and re-use it. */
    if (!referenced)
        memset(frames[b]->propagateCost, 0, m_8x8Width * sizeof(uint16_t));

    int32_t strideInCU = m_8x8Width;
    for (uint16_t blocky = 0; blocky < m_8x8Height; blocky++)
    {
        int cuIndex = blocky * strideInCU;
        if (m_param->rc.qgSize == 8)
            primitives.propagateCost(m_scratch, propagateCost,
                       frames[b]->intraCost + cuIndex, frames[b]->lowresCosts[b - p0][p1 - b] + cuIndex,
                       frames[b]->invQscaleFactor8x8 + cuIndex, &fpsFactor, m_8x8Width);
        else
            primitives.propagateCost(m_scratch, propagateCost,
                       frames[b]->intraCost + cuIndex, frames[b]->lowresCosts[b - p0][p1 - b] + cuIndex,
                       frames[b]->invQscaleFactor + cuIndex, &fpsFactor, m_8x8Width);

        if (referenced)
            propagateCost += m_8x8Width;

        for (uint16_t blockx = 0; blockx < m_8x8Width; blockx++, cuIndex++)
        {
            int32_t propagate_amount = m_scratch[blockx];
            /* Don't propagate for an intra block. */
            if (propagate_amount > 0)
            {
                /* Access width-2 bitfield. */
                int32_t lists_used = frames[b]->lowresCosts[b - p0][p1 - b][cuIndex] >> LOWRES_COST_SHIFT;
                /* Follow the MVs to the previous frame(s). */
                for (uint16_t list = 0; list < 2; list++)
                {
                    if ((lists_used >> list) & 1)
                    {
#define CLIP_ADD(s, x) (s) = (uint16_t)S265_MIN((s) + (x), (1 << 16) - 1)
                        int32_t listamount = propagate_amount;
                        /* Apply bipred weighting. */
                        if (lists_used == 3)
                            listamount = (listamount * bipredWeights[list] + 32) >> 6;

                        MV *mvs = frames[b]->lowresMvs[list][listDist[list]];

                        /* Early termination for simple case of mv0. */
                        if (!mvs[cuIndex].word)
                        {
                            CLIP_ADD(refCosts[list][cuIndex], listamount);
                            continue;
                        }

                        int32_t x = mvs[cuIndex].x;
                        int32_t y = mvs[cuIndex].y;
                        int32_t cux = (x >> 5) + blockx;
                        int32_t cuy = (y >> 5) + blocky;
                        int32_t idx0 = cux + cuy * strideInCU;
                        int32_t idx1 = idx0 + 1;
                        int32_t idx2 = idx0 + strideInCU;
                        int32_t idx3 = idx0 + strideInCU + 1;
                        x &= 31;
                        y &= 31;
                        int32_t idx0weight = (32 - y) * (32 - x);
                        int32_t idx1weight = (32 - y) * x;
                        int32_t idx2weight = y * (32 - x);
                        int32_t idx3weight = y * x;

                        /* We could just clip the MVs, but pixels that lie outside the frame probably shouldn't
                         * be counted. */
                        if (cux < m_8x8Width - 1 && cuy < m_8x8Height - 1 && cux >= 0 && cuy >= 0)
                        {
                            CLIP_ADD(refCosts[list][idx0], (listamount * idx0weight + 512) >> 10);
                            CLIP_ADD(refCosts[list][idx1], (listamount * idx1weight + 512) >> 10);
                            CLIP_ADD(refCosts[list][idx2], (listamount * idx2weight + 512) >> 10);
                            CLIP_ADD(refCosts[list][idx3], (listamount * idx3weight + 512) >> 10);
                        }
                        else /* Check offsets individually */
                        {
                            if (cux < m_8x8Width && cuy < m_8x8Height && cux >= 0 && cuy >= 0)
                                CLIP_ADD(refCosts[list][idx0], (listamount * idx0weight + 512) >> 10);
                            if (cux + 1 < m_8x8Width && cuy < m_8x8Height && cux + 1 >= 0 && cuy >= 0)
                                CLIP_ADD(refCosts[list][idx1], (listamount * idx1weight + 512) >> 10);
                            if (cux < m_8x8Width && cuy + 1 < m_8x8Height && cux >= 0 && cuy + 1 >= 0)
                                CLIP_ADD(refCosts[list][idx2], (listamount * idx2weight + 512) >> 10);
                            if (cux + 1 < m_8x8Width && cuy + 1 < m_8x8Height && cux + 1 >= 0 && cuy + 1 >= 0)
                                CLIP_ADD(refCosts[list][idx3], (listamount * idx3weight + 512) >> 10);
                        }
                    }
                }
            }
        }
    }

    if (m_param->rc.vbvBufferSize && m_param->lookaheadDepth && referenced)
        cuTreeFinish(frames[b], averageDuration, b == p1 ? b - p0 : 0);
}

void Lookahead::computeCUTreeQpOffset(Lowres *frame, double averageDuration, int ref0Distance)
{
    int fpsFactor = (int)(CLIP_DURATION(averageDuration) / CLIP_DURATION((double)m_param->fpsDenom / m_param->fpsNum) * 256);
    uint32_t loopIncr = (m_param->rc.qgSize == 8) ? 8 : 16;

    double weightdelta = 0.0;
    if (ref0Distance && frame->weightedCostDelta[ref0Distance - 1] > 0)
        weightdelta = (1.0 - frame->weightedCostDelta[ref0Distance - 1]);

    uint32_t widthFullRes = frame->widthFullRes;
    uint32_t heightFullRes = frame->heightFullRes;

    if (m_param->rc.qgSize == 8)
    {
        int minAQDepth = frame->pAQLayer->minAQDepth;

        PicQPAdaptationLayer* pQPLayerMin = &frame->pAQLayer[minAQDepth];
        double* pcCuTree8x8 = pQPLayerMin->dCuTreeOffset8x8;

        for (int cuY = 0; cuY < m_8x8Height; cuY++)
        {
            for (int cuX = 0; cuX < m_8x8Width; cuX++)
            {
                const int cuXY = cuX + cuY * m_8x8Width;
                int intracost = ((frame->intraCost[cuXY] / 4) * frame->invQscaleFactor8x8[cuXY] + 128) >> 8;
                if (intracost)
                {
                    int propagateCost = ((frame->propagateCost[cuXY] / 4)  * fpsFactor + 128) >> 8;
                    double log2_ratio = S265_LOG2(intracost + propagateCost) - S265_LOG2(intracost) + weightdelta;

                    pcCuTree8x8[cuX * 2 + cuY * m_8x8Width * 4] = log2_ratio;
                    pcCuTree8x8[cuX * 2 + cuY * m_8x8Width * 4 + 1] = log2_ratio;
                    pcCuTree8x8[cuX * 2 + cuY * m_8x8Width * 4 + frame->maxBlocksInRowFullRes] = log2_ratio;
                    pcCuTree8x8[cuX * 2 + cuY * m_8x8Width * 4 + frame->maxBlocksInRowFullRes + 1] = log2_ratio;
                }
            }
        }

        for (uint32_t d = 0; d < 4; d++)
        {
            int ctuSizeIdx = 6 - g_log2Size[m_param->maxCUSize];
            int aqDepth = g_log2Size[m_param->maxCUSize] - g_log2Size[m_param->rc.qgSize];
            if (!aqLayerDepth[ctuSizeIdx][aqDepth][d])
                continue;

            PicQPAdaptationLayer* pQPLayer = &frame->pAQLayer[d];
            const uint32_t aqPartWidth = pQPLayer->aqPartWidth;
            const uint32_t aqPartHeight = pQPLayer->aqPartHeight;

            const uint32_t numAQPartInWidth = pQPLayer->numAQPartInWidth;
            const uint32_t numAQPartInHeight = pQPLayer->numAQPartInHeight;

            double* pcQP = pQPLayer->dQpOffset;
            double* pcCuTree = pQPLayer->dCuTreeOffset;

            uint32_t maxCols = frame->maxBlocksInRowFullRes;

            for (uint32_t y = 0; y < numAQPartInHeight; y++)
            {
                for (uint32_t x = 0; x < numAQPartInWidth; x++, pcQP++, pcCuTree++)
                {
                    uint32_t block_x = x * aqPartWidth;
                    uint32_t block_y = y * aqPartHeight;

                    uint32_t blockXY = 0;
                    double log2_ratio = 0;
                    for (uint32_t block_yy = block_y; block_yy < block_y + aqPartHeight && block_yy < heightFullRes; block_yy += loopIncr)
                    {
                        for (uint32_t block_xx = block_x; block_xx < block_x + aqPartWidth && block_xx < widthFullRes; block_xx += loopIncr)
                        {
                            uint32_t idx = ((block_yy / loopIncr) * (maxCols)) + (block_xx / loopIncr);

                            log2_ratio += *(pcCuTree8x8 + idx);
                            
                            blockXY++;
                        }
                    }

                    double qp_offset = (m_cuTreeStrength * log2_ratio) / blockXY;

                    *pcCuTree = *pcQP - qp_offset;
                }
            }
        }
    }
    else
    {
        int ctuSizeIdx = 6 - g_log2Size[m_param->maxCUSize];
        int aqDepth = g_log2Size[m_param->maxCUSize] - g_log2Size[m_param->rc.qgSize];
        for (uint32_t d = 0; d < 4; d++)
        {
            if (!aqLayerDepth[ctuSizeIdx][aqDepth][d]) // 1, 1, 1, 0
                continue;

            PicQPAdaptationLayer* pQPLayer = &frame->pAQLayer[d];
            const uint32_t aqPartWidth = pQPLayer->aqPartWidth;
            const uint32_t aqPartHeight = pQPLayer->aqPartHeight;

            const uint32_t numAQPartInWidth = pQPLayer->numAQPartInWidth;
            const uint32_t numAQPartInHeight = pQPLayer->numAQPartInHeight;

            double* pcQP = pQPLayer->dQpOffset;// 指针初始化
            double* pcCuTree = pQPLayer->dCuTreeOffset;// 指针初始化

            uint32_t maxCols = frame->maxBlocksInRow;

            for (uint32_t y = 0; y < numAQPartInHeight; y++)
            {
                for (uint32_t x = 0; x < numAQPartInWidth; x++, pcQP++, pcCuTree++)
                {
                    uint32_t block_x = x * aqPartWidth;
                    uint32_t block_y = y * aqPartHeight;

                    uint32_t blockXY = 0;
                    double log2_ratio = 0;
                    for (uint32_t block_yy = block_y; block_yy < block_y + aqPartHeight && block_yy < heightFullRes; block_yy += loopIncr)
                    {
                        for (uint32_t block_xx = block_x; block_xx < block_x + aqPartWidth && block_xx < widthFullRes; block_xx += loopIncr)
                        {
                            uint32_t idx = ((block_yy / loopIncr) * (maxCols)) + (block_xx / loopIncr);

                            int intraCost = (frame->intraCost[idx] * frame->invQscaleFactor[idx] + 128) >> 8;
                            int propagateCost = (frame->propagateCost[idx] * fpsFactor + 128) >> 8;

                            log2_ratio += (S265_LOG2(intraCost + propagateCost) - S265_LOG2(intraCost) + weightdelta);

                            blockXY++;// block 统计
                        }
                    }

                    double qp_offset = (m_cuTreeStrength * log2_ratio) / blockXY;// 求平均

                    *pcCuTree = *pcQP - qp_offset;

                }
            }
        }
    }
}
//ref0_distance 为0表示不需要考虑weighted_p， 不为0 则表示 (b-p0)的距离
void Lookahead::cuTreeFinish(Lowres *frame, double averageDuration, int ref0Distance)
{
    if (m_param->rc.hevcAq)
    {
        computeCUTreeQpOffset(frame, averageDuration, ref0Distance);
    }
    else
    {
        //printf("cal cuTreeFinish for frame %d\n", frame->frameNum);
        int fpsFactor = (int)(CLIP_DURATION(averageDuration) / CLIP_DURATION((double)m_param->fpsDenom / m_param->fpsNum) * 256);
        double weightdelta = 0.0;

        if (ref0Distance && frame->weightedCostDelta[ref0Distance - 1] > 0)
            weightdelta = (1.0 - frame->weightedCostDelta[ref0Distance - 1]);

        if (m_param->rc.qgSize == 8)
        {   // qgSize为8时，一个lookahead 8x8 对应以个orig 的16x16= 4个8x8
            for (int cuY = 0; cuY < m_8x8Height; cuY++)
            {
                for (int cuX = 0; cuX < m_8x8Width; cuX++)
                {
                    const int cuXY = cuX + cuY * m_8x8Width;
                    int intracost = ((frame->intraCost[cuXY]) / 4 * frame->invQscaleFactor8x8[cuXY] + 128) >> 8;
                    if (intracost)
                    {
                        int propagateCost = ((frame->propagateCost[cuXY]) / 4 * fpsFactor + 128) >> 8;
                        double log2_ratio = S265_LOG2(intracost + propagateCost) - S265_LOG2(intracost) + weightdelta;
                        //在_aq 的基础上，做进一步的调节，并将结果记录在f_qp_offset上面
                        frame->qpCuTreeOffset[cuX * 2 + cuY * m_8x8Width * 4] = frame->qpAqOffset[cuX * 2 + cuY * m_8x8Width * 4] - m_cuTreeStrength * (log2_ratio);
                        frame->qpCuTreeOffset[cuX * 2 + cuY * m_8x8Width * 4 + 1] = frame->qpAqOffset[cuX * 2 + cuY * m_8x8Width * 4 + 1] - m_cuTreeStrength * (log2_ratio);
                        frame->qpCuTreeOffset[cuX * 2 + cuY * m_8x8Width * 4 + frame->maxBlocksInRowFullRes] = frame->qpAqOffset[cuX * 2 + cuY * m_8x8Width * 4 + frame->maxBlocksInRowFullRes] - m_cuTreeStrength * (log2_ratio);
                        frame->qpCuTreeOffset[cuX * 2 + cuY * m_8x8Width * 4 + frame->maxBlocksInRowFullRes + 1] = frame->qpAqOffset[cuX * 2 + cuY * m_8x8Width * 4 + frame->maxBlocksInRowFullRes + 1] - m_cuTreeStrength * (log2_ratio);
                    }
                }
            }
        }
        else
        {    // qgSize为>8时，一个lookahead 8x8 对应以个orig 的16x16
            for (int cuIndex = 0; cuIndex < m_cuCount; cuIndex++)
            {
                int intracost = (frame->intraCost[cuIndex] * frame->invQscaleFactor[cuIndex] + 128) >> 8;
                if (intracost)
                {
                    int propagateCost = (frame->propagateCost[cuIndex] * fpsFactor + 128) >> 8;
                    double log2_ratio = S265_LOG2(intracost + propagateCost) - S265_LOG2(intracost) + weightdelta;
                    frame->qpCuTreeOffset[cuIndex] = frame->qpAqOffset[cuIndex] - m_cuTreeStrength * log2_ratio;
                }
            }
        }
    }
}

/* If MB-tree changes the quantizers, we need to recalculate the frame cost without
 * re-running lookahead. */
int64_t Lookahead::frameCostRecalculate(Lowres** frames, int p0, int p1, int b)
{

    if (frames[b]->sliceType == S265_TYPE_B)//非参考帧不受cutree 影响
        return frames[b]->costEstAq[b - p0][p1 - b];

    int64_t score = 0;
    int *rowSatd = frames[b]->rowSatds[b - p0][p1 - b];

    s265_emms();

    if (m_param->rc.hevcAq)
    {
        int minAQDepth = frames[b]->pAQLayer->minAQDepth;
        PicQPAdaptationLayer* pQPLayer = &frames[b]->pAQLayer[minAQDepth];
        double* pcQPCuTree = pQPLayer->dCuTreeOffset;

        // Use new qp offset values for qpAqOffset, qpCuTreeOffset and invQscaleFactor buffer
        for (int cuy = m_8x8Height - 1; cuy >= 0; cuy--)
        {
            rowSatd[cuy] = 0;
            for (int cux = m_8x8Width - 1; cux >= 0; cux--)
            {
                int cuxy = cux + cuy * m_8x8Width;
                int cuCost = frames[b]->lowresCosts[b - p0][p1 - b][cuxy] & LOWRES_COST_MASK;
                double qp_adj;

                if (m_param->rc.qgSize == 8)
                    qp_adj = (pcQPCuTree[cux * 2 + cuy * m_8x8Width * 4] +
                    pcQPCuTree[cux * 2 + cuy * m_8x8Width * 4 + 1] +
                    pcQPCuTree[cux * 2 + cuy * m_8x8Width * 4 + frames[b]->maxBlocksInRowFullRes] +
                    pcQPCuTree[cux * 2 + cuy * m_8x8Width * 4 + frames[b]->maxBlocksInRowFullRes + 1]) / 4;
                else
                    qp_adj = *(pcQPCuTree + cuxy);

                cuCost = (cuCost * s265_exp2fix8(qp_adj) + 128) >> 8;
                rowSatd[cuy] += cuCost;
                if ((cuy > 0 && cuy < m_8x8Height - 1 &&
                    cux > 0 && cux < m_8x8Width - 1) ||
                    m_8x8Width <= 2 || m_8x8Height <= 2)
                {
                    score += cuCost;
                }
            }
        }
    }
    else
    {
        double *qp_offset = frames[b]->qpCuTreeOffset;

        for (int cuy = m_8x8Height - 1; cuy >= 0; cuy--)
        {
            rowSatd[cuy] = 0;
            for (int cux = m_8x8Width - 1; cux >= 0; cux--)
            {
                int cuxy = cux + cuy * m_8x8Width;
                int cuCost = frames[b]->lowresCosts[b - p0][p1 - b][cuxy] & LOWRES_COST_MASK;
                double qp_adj;
                // 如果 qgroupsize 为8 , 则 在lowres上 一个 8x8 对应有 4个原始分辨率上的8x8
                if (m_param->rc.qgSize == 8)
                    qp_adj = (qp_offset[cux * 2 + cuy * m_8x8Width * 4] +
                    qp_offset[cux * 2 + cuy * m_8x8Width * 4 + 1] +
                    qp_offset[cux * 2 + cuy * m_8x8Width * 4 + frames[b]->maxBlocksInRowFullRes] +
                    qp_offset[cux * 2 + cuy * m_8x8Width * 4 + frames[b]->maxBlocksInRowFullRes + 1]) / 4;
                else
                // 否则 qgroupsize 为 16x16 以上, 则在lowres上 一个8x8对应有1个原始分辨率上的16x16
                    qp_adj = qp_offset[cuxy];
                cuCost = (cuCost * s265_exp2fix8(qp_adj) + 128) >> 8;
                rowSatd[cuy] += cuCost;
                if ((cuy > 0 && cuy < m_8x8Height - 1 &&
                    cux > 0 && cux < m_8x8Width - 1) ||
                    m_8x8Width <= 2 || m_8x8Height <= 2)
                {
                    score += cuCost;
                }
            }
        }
    }

    return score;
}


int64_t CostEstimateGroup::singleCost(int p0, int p1, int b, bool intraPenalty)
{
    LookaheadTLD& tld = m_lookahead.m_tld[m_lookahead.m_pool ? m_lookahead.m_pool->m_numWorkers : 0];
    return estimateFrameCost(tld, p0, p1, b, intraPenalty);
}

void CostEstimateGroup::add(int p0, int p1, int b)
{
    S265_CHECK(m_batchMode || !m_jobTotal, "single CostEstimateGroup instance cannot mix batch modes\n");
    m_batchMode = true;

    Estimate& e = m_estimates[m_jobTotal++];
    e.p0 = p0;
    e.p1 = p1;
    e.b = b;

    if (m_jobTotal == MAX_BATCH_SIZE)
        finishBatch();
}

void CostEstimateGroup::finishBatch()
{
    if (m_lookahead.m_pool)
        tryBondPeers(*m_lookahead.m_pool, m_jobTotal);//唤醒m_jobTotal 个sleep 线程执行processTasks();
    processTasks(-1);//有线程池时,应该不需要这里再调用processTasks了
    waitForExit();//等待所有的线程完成各自的processTasks
    m_jobTotal = m_jobAcquired = 0;
}

//有可能是通过唤醒线程池里面的线程来执行processTasks
// 也有可能是线程自己来执行 workerThreadID < 0 是
void CostEstimateGroup::processTasks(int workerThreadID)
{
    ThreadPool* pool = m_lookahead.m_pool;
    int id = workerThreadID;
    if (workerThreadID < 0)
        id = pool ? pool->m_numWorkers : 0;
    LookaheadTLD& tld = m_lookahead.m_tld[id];

    m_lock.acquire();
    while (m_jobAcquired < m_jobTotal)
    {
        int i = m_jobAcquired++;
        m_lock.release();

        if (m_batchMode)
        {/*这种模式下，estimateFrameCost不会采用帧内多slice 并行计算，一个任务不能继续分割*/
            ProfileLookaheadTime(tld.batchElapsedTime, tld.countBatches);
            ProfileScopeEvent(estCostSingle);

            Estimate& e = m_estimates[i];
            estimateFrameCost(tld, e.p0, e.p1, e.b, false);
        }
        else
        {
            ProfileLookaheadTime(tld.coopSliceElapsedTime, tld.countCoopSlices);
            ProfileScopeEvent(estCostCoop);

            S265_CHECK(i < MAX_COOP_SLICES, "impossible number of coop slices\n");

            int firstY, lastY;
            bool lastRow;
            if (m_lookahead.m_param->bEnableHME)
            {
                int numRowsPerSlice = m_lookahead.m_4x4Height / m_lookahead.m_param->lookaheadSlices;
                numRowsPerSlice = S265_MIN(S265_MAX(numRowsPerSlice, 5), m_lookahead.m_4x4Height);
                firstY = numRowsPerSlice * i;
                lastY = (i == m_jobTotal - 1) ? m_lookahead.m_4x4Height - 1 : numRowsPerSlice * (i + 1) - 1;
                lastRow = true;
                for (int cuY = lastY; cuY >= firstY; cuY--)
                {
                    for (int cuX = m_lookahead.m_4x4Width - 1; cuX >= 0; cuX--)
                        estimateCUCost(tld, cuX, cuY, m_coop.p0, m_coop.p1, m_coop.b, m_coop.bDoSearch, lastRow, i, 1);
                    lastRow = false;
                }
            }

            firstY = m_lookahead.m_numRowsPerSlice * i;
            lastY = (i == m_jobTotal - 1) ? m_lookahead.m_8x8Height - 1 : m_lookahead.m_numRowsPerSlice * (i + 1) - 1;
            lastRow = true;
            for (int cuY = lastY; cuY >= firstY; cuY--)
            {
                m_frames[m_coop.b]->rowSatds[m_coop.b - m_coop.p0][m_coop.p1 - m_coop.b][cuY] = 0;

                for (int cuX = m_lookahead.m_8x8Width - 1; cuX >= 0; cuX--)
                    estimateCUCost(tld, cuX, cuY, m_coop.p0, m_coop.p1, m_coop.b, m_coop.bDoSearch, lastRow, i, 0);

                lastRow = false;
            }
        }

        m_lock.acquire();
    }
    m_lock.release();
}

int64_t CostEstimateGroup::estimateFrameCost(LookaheadTLD& tld, int p0, int p1, int b, bool bIntraPenalty)
{
    Lowres*     fenc  = m_frames[b];
    s265_param* param = m_lookahead.m_param;
    int64_t     score = 0;

    if (fenc->costEst[b - p0][p1 - b] >= 0 && fenc->rowSatds[b - p0][p1 - b][0] != -1)
        score = fenc->costEst[b - p0][p1 - b];
    else
    {
        bool bDoSearch[2];
        bDoSearch[0] = fenc->lowresMvs[0][b - p0][0].x == 0x7FFF;
        bDoSearch[1] = p1 > b && fenc->lowresMvs[1][p1 - b][0].x == 0x7FFF;

#if CHECKED_BUILD
        S265_CHECK(!(p0 < b && fenc->lowresMvs[0][b - p0][0].x == 0x7FFE), "motion search batch duplication L0\n");
        S265_CHECK(!(p1 > b && fenc->lowresMvs[1][p1 - b][0].x == 0x7FFE), "motion search batch duplication L1\n");
        if (bDoSearch[0]) fenc->lowresMvs[0][b - p0][0].x = 0x7FFE;
        if (bDoSearch[1]) fenc->lowresMvs[1][p1 - b][0].x = 0x7FFE;
#endif

        fenc->weightedRef[b - p0].isWeighted = false;
        if (param->bEnableWeightedPred && bDoSearch[0])
            tld.weightsAnalyse(*m_frames[b], *m_frames[p0]);

        fenc->costEst[b - p0][p1 - b] = 0;
        fenc->costEstAq[b - p0][p1 - b] = 0;


        fenc->largeMvs[b-p0] = 0;
        fenc->veryLargeMvs[b-p0] = 0;
        fenc->hasSmallMvs[b-p0] = 0;

        if (!m_batchMode && m_lookahead.m_numCoopSlices > 1 && ((p1 > b) || bDoSearch[0] || bDoSearch[1]))
        {
            /*这里是说在计算一帧的cost 时，如果采用的帧内多slice并行机制，则*/
            /* Use cooperative mode if a thread pool is available and the cost estimate is
             * going to need motion searches or bidir measurements */

            memset(&m_slice, 0, sizeof(Slice) * m_lookahead.m_numCoopSlices);

            m_lock.acquire();
            S265_CHECK(!m_batchMode, "single CostEstimateGroup instance cannot mix batch modes\n");
            m_coop.p0 = p0;
            m_coop.p1 = p1;
            m_coop.b = b;
            m_coop.bDoSearch[0] = bDoSearch[0];
            m_coop.bDoSearch[1] = bDoSearch[1];
            m_jobTotal = m_lookahead.m_numCoopSlices;// 有多少个条带就有多少个任务要做
            m_jobAcquired = 0;
            m_lock.release();

            // 唤醒线程池里面的线程通过
            tryBondPeers(*m_lookahead.m_pool, m_jobTotal);// 唤醒线程池里面的线程去执行processtasks

            processTasks(-1);//子类线程 直接调用 有线程池时,应该不需要这里再调用processTasks了

            waitForExit();// 等待所有任务都完成
            // 任务汇总
            for (int i = 0; i < m_lookahead.m_numCoopSlices; i++)
            {
                fenc->costEst[b - p0][p1 - b] += m_slice[i].costEst;
                fenc->costEstAq[b - p0][p1 - b] += m_slice[i].costEstAq;
                if (p1 == b)
                {
                    fenc->intraMbs[b - p0] += m_slice[i].intraMbs;
                    fenc->veryLargeMvs[b - p0] += m_slice[i].largeMvs; 
                    fenc->hasSmallMvs[b - p0]  += m_slice[i].hasSmallMvs; 
                    fenc->largeMvs[b - p0]     += m_slice[i].largeMvs; 
                }
                    
            }
        }
        else
        {// 如果是 batchmode 或者 不实用帧内多slice 并行机制，则一整帧一整帧计算
            /* Calculate MVs for 1/16th resolution*/
            bool lastRow;
            if (param->bEnableHME)
            {
                lastRow = true;// 在1/4 * 1/4 小小图上逆序计算4x4 cost
                for (int cuY = m_lookahead.m_4x4Height - 1; cuY >= 0; cuY--)
                {
                    for (int cuX = m_lookahead.m_4x4Width - 1; cuX >= 0; cuX--)
                        estimateCUCost(tld, cuX, cuY, p0, p1, b, bDoSearch, lastRow, -1, 1);
                    lastRow = false;
                }
            }
            lastRow = true;// 在1/2 * 1/2 小图上 逆序计算8x8 cost
            for (int cuY = m_lookahead.m_8x8Height - 1; cuY >= 0; cuY--)
            {
                fenc->rowSatds[b - p0][p1 - b][cuY] = 0;

                for (int cuX = m_lookahead.m_8x8Width - 1; cuX >= 0; cuX--)
                    estimateCUCost(tld, cuX, cuY, p0, p1, b, bDoSearch, lastRow, -1, 0);

                lastRow = false;
            }
        }

        score = fenc->costEst[b - p0][p1 - b];

        if (b != p1)
            score = score * 100 / (130 + param->bFrameBias);//b frame cost 做一个偏置

        fenc->costEst[b - p0][p1 - b] = score;
    }

    if (bIntraPenalty)//因为intra的准确性没有那么高所以需要添加惩罚
        // arbitrary penalty for I-blocks after B-frames
        score += score * fenc->intraMbs[b - p0] / (tld.ncu * 8);

    return score;
}



void CostEstimateGroup::estimateCUCost(LookaheadTLD& tld, int cuX, int cuY, int p0, int p1, int b, bool bDoSearch[2], bool lastRow, int slice, bool hme)
{
    Lowres *fref0 = m_frames[p0];
    Lowres *fref1 = m_frames[p1];
    Lowres *fenc  = m_frames[b];

    ReferencePlanes *wfref0 = fenc->weightedRef[b - p0].isWeighted && !hme ? &fenc->weightedRef[b - p0] : fref0;

    const int widthInCU = hme ? m_lookahead.m_4x4Width : m_lookahead.m_8x8Width;
    const int heightInCU = hme ? m_lookahead.m_4x4Height : m_lookahead.m_8x8Height;
    const int bBidir = (b < p1);
    const int cuXY = cuX + cuY * widthInCU;
    const int cuXY_4x4 = (cuX / 2) + (cuY / 2) * widthInCU / 2;
    const int cuSize = S265_LOWRES_CU_SIZE;
    const intptr_t pelOffset = cuSize * cuX + cuSize * cuY * (hme ? fenc->lumaStride/2 : fenc->lumaStride);

    // copy src y pixle from fenc->lowresPlane[0]/fenc->lowerResPlane[0] to fencPUYuv.m_buf[0]
    if ((bBidir || bDoSearch[0] || bDoSearch[1]) && hme)
        tld.me.setSourcePU(fenc->lowerResPlane[0], fenc->lumaStride / 2, pelOffset, cuSize, cuSize, S265_HEX_SEARCH, m_lookahead.m_param->hmeSearchMethod[0], m_lookahead.m_param->hmeSearchMethod[1], 1);
    else if((bBidir || bDoSearch[0] || bDoSearch[1]) && !hme)
        tld.me.setSourcePU(fenc->lowresPlane[0], fenc->lumaStride, pelOffset, cuSize, cuSize, S265_HEX_SEARCH, m_lookahead.m_param->hmeSearchMethod[0], m_lookahead.m_param->hmeSearchMethod[1], 1);


    /* A small, arbitrary bias to avoid VBV problems caused by zero-residual lookahead blocks. */
    int lowresPenalty = 4;
    int listDist[2] = { b - p0, p1 - b};

    MV mvmin, mvmax;
    int bcost = tld.me.COST_MAX;
    int listused = 0;

    // TODO: restrict to slices boundaries
    // establish search bounds that don't cross extended frame boundaries
    mvmin.x = (int32_t)(-cuX * cuSize - 8);
    mvmin.y = (int32_t)(-cuY * cuSize - 8);
    mvmax.x = (int32_t)((widthInCU - cuX - 1) * cuSize + 8);
    mvmax.y = (int32_t)((heightInCU - cuY - 1) * cuSize + 8);

    for (int i = 0; i < 1 + bBidir; i++)
    {
        int& fencCost = hme ? fenc->lowerResMvCosts[i][listDist[i]][cuXY] : fenc->lowresMvCosts[i][listDist[i]][cuXY];
        int skipCost = INT_MAX;

        if (!bDoSearch[i])
        {
            COPY2_IF_LT(bcost, fencCost, listused, i + 1);
            continue;
        }

        int numc = 0;
        MV mvc[8], mvp;
        MV* fencMV = hme ? &fenc->lowerResMvs[i][listDist[i]][cuXY] : &fenc->lowresMvs[i][listDist[i]][cuXY];
        ReferencePlanes* fref = i ? fref1 : wfref0;
        int mvpcost = MotionEstimate::COST_MAX;

        /* Reverse-order MV prediction */
#define MVC(mv) mvc[numc++] = mv;
        if (cuX < widthInCU - 1)
            MVC(fencMV[1]);//右侧mv加入candidate
        if (!lastRow)
        {
            MVC(fencMV[widthInCU]);//下侧mv加入candidate
            if (cuX > 0)
                MVC(fencMV[widthInCU - 1]);//左下侧mv 加入candidate
            if (cuX < widthInCU - 1)
                MVC(fencMV[widthInCU + 1]);//右下侧mv 加入candidate
        }
        if (fenc->lowerResMvs[0][0] && !hme && fenc->lowerResMvCosts[i][listDist[i]][cuXY_4x4] > 0)
        {
            MVC((fenc->lowerResMvs[i][listDist[i]][cuXY_4x4]) * 2);//将 1/4*1/4小小图对应的4x4位置的mv扩大2倍后加入 candidate
        }
#undef MVC

        if (!numc) //如果上述mv 的candidate 一个也没有 设置mvp为 0
            mvp = 0;
        else
        {
            ALIGN_VAR_32(pixel, subpelbuf[S265_LOWRES_CU_SIZE * S265_LOWRES_CU_SIZE]);
            // mv_clipp
            for (int32_t idx0 = 0; idx0 < numc; idx0++)
            {
                mvc[idx0].clipped(mvmin, mvmax);
            }
            // De_duplication 去重
            bool mask[5] = { false };
            for (int32_t idx0 = 0; idx0 < numc; idx0++)
            {
                for (int32_t idx1 = idx0 + 1; idx1 < numc; idx1++)
                {
                    if (mvc[idx0].word == mvc[idx1].word)
                    {
                        mask[idx1] = true;
                    }
                }
            }
            /* measure SATD cost of each neighbor MV (estimating merge analysis)
             * and use the lowest cost MV as MVP (estimating AMVP). Since all
             * mvc[] candidates are measured here, none are passed to motionEstimate */
            for (int idx = 0; idx < numc; idx++)// iterate candidate mv find the best one
            {
                intptr_t stride = S265_LOWRES_CU_SIZE;
                /*注意 src 返回后有可能是subpelbuf 地址（需要1/4差值时 ） 
                也有可能是ref帧的小图or 小小图 对应的plane便宜后的地址
                （无需1/4分像素差值，stride 会被更改为 对应小/小小图的plane 的 Ystride） */
                if (!mask[idx])
                {
                    pixel *src = fref->lowresMC(pelOffset, mvc[idx], subpelbuf, stride, hme);
                    // 计算 fencPUYuv.m_buf[0]与参考数据（src指向的data) 之间的 satd
                    int cost = tld.me.bufSATD(src, stride);
                    COPY2_IF_LT(mvpcost, cost, mvp, mvc[idx]);// 保存最小的cost为 mvpcost 保存对应的mv candidate 为 mvp
                    /* Except for mv0 case, everyting else is likely to have enough residual to not trigger the skip. */
                    if (!mvc[idx].notZero() && (bBidir || tld.usePskip))// 如果在bBidir 的条件下，这个过程中mvc有为0的情况 则记录cost 为skipcost
                        skipCost = cost;
                    if( cost< 64 )
                        break;
                }
            }
        }
        if (skipCost < 64 && (bBidir || tld.usePskip))
        {
            fencCost = skipCost;
            *fencMV = 0;
        }
        else if (mvpcost < 64)
        {
            fencCost = mvpcost;
            *fencMV = mvp;
        }
        else
        {
            // 在非hme下 search 范围固定为16
            int searchRange = m_lookahead.m_param->bEnableHME ? (hme ? m_lookahead.m_param->hmeRange[0] : m_lookahead.m_param->hmeRange[1]) : s_merange;
            /* ME will never return a cost larger than the cost @MVP, so we do not
            * have to check that ME cost is more than the estimated merge cost */
            if(!hme) //以mvp 为起点在search范围内 搜索最佳匹配点并记录fencMV 和fencCost
                fencCost = tld.me.motionEstimate(fref, mvmin, mvmax, mvp, 0, NULL, searchRange, *fencMV, m_lookahead.m_param->maxSlices);
            else
                fencCost = tld.me.motionEstimate(fref, mvmin, mvmax, mvp, 0, NULL, searchRange, *fencMV, m_lookahead.m_param->maxSlices, fref->lowerResPlane[0]);
        }
        COPY2_IF_LT(bcost, fencCost, listused, i + 1);
    }
    if (hme)
        return;

    if (bBidir) /* B, also consider bidir */
    {
        /* NOTE: the wfref0 (weightp) is not used for BIDIR */

        /* avg(l0-mv, l1-mv) candidate */
        ALIGN_VAR_32(pixel, subpelbuf0[S265_LOWRES_CU_SIZE * S265_LOWRES_CU_SIZE]);
        ALIGN_VAR_32(pixel, subpelbuf1[S265_LOWRES_CU_SIZE * S265_LOWRES_CU_SIZE]);
        intptr_t stride0 = S265_LOWRES_CU_SIZE, stride1 = S265_LOWRES_CU_SIZE;
        pixel *src0 = fref0->lowresMC(pelOffset, fenc->lowresMvs[0][listDist[0]][cuXY], subpelbuf0, stride0, 0);
        pixel *src1 = fref1->lowresMC(pelOffset, fenc->lowresMvs[1][listDist[1]][cuXY], subpelbuf1, stride1, 0);
        ALIGN_VAR_32(pixel, ref[S265_LOWRES_CU_SIZE * S265_LOWRES_CU_SIZE]);
        primitives.pu[LUMA_8x8].pixelavg_pp[NONALIGNED](ref, S265_LOWRES_CU_SIZE, src0, stride0, src1, stride1, 32);
        // 计算 fencPUYuv.m_buf[0]与参考数据（ref指向的data) 之间的 satd
        int bicost = tld.me.bufSATD(ref, S265_LOWRES_CU_SIZE);
        COPY2_IF_LT(bcost, bicost, listused, 3);
        /* coloc candidate */
        // 如果这里要计算的话应该是 fenc->lowresMvs[0/1][listDist[0/1]][cuXY] 不同时为0,否则重复计算了
        if (fenc->lowresMvs[0][listDist[0]][cuXY].word || fenc->lowresMvs[1][listDist[1]][cuXY].word)
        {
            src0 = fref0->lowresPlane[0] + pelOffset;
            src1 = fref1->lowresPlane[0] + pelOffset;
            primitives.pu[LUMA_8x8].pixelavg_pp[NONALIGNED](ref, S265_LOWRES_CU_SIZE, src0, fref0->lumaStride, src1, fref1->lumaStride, 32);
            bicost = tld.me.bufSATD(ref, S265_LOWRES_CU_SIZE);
            COPY2_IF_LT(bcost, bicost, listused, 3);
        }
        bcost += lowresPenalty;
    }
    else /* P, also consider intra */
    {
        bcost += lowresPenalty;

        if (fenc->intraCost[cuXY] < bcost)// intra cost 的代价更小
        {
            bcost = fenc->intraCost[cuXY];
            listused = 0;
        }
    }

    /* do not include edge blocks in the frame cost estimates, they are not very accurate */
    const bool bFrameScoreCU = (cuX > 0 && cuX < widthInCU - 1 &&
                                cuY > 0 && cuY < heightInCU - 1) || widthInCU <= 2 || heightInCU <= 2;
    int bcostAq;
    if (m_lookahead.m_param->rc.qgSize == 8)
        bcostAq = (bFrameScoreCU && fenc->invQscaleFactor) ? ((bcost * fenc->invQscaleFactor8x8[cuXY] + 128) >> 8) : bcost;
    else
        bcostAq = (bFrameScoreCU && fenc->invQscaleFactor) ? ((bcost * fenc->invQscaleFactor[cuXY] +128) >> 8) : bcost;

    if (bFrameScoreCU)
    {
        if (slice < 0)//当前frame的cost 计算是按照一帧一帧计算的
        {
            fenc->costEst[b - p0][p1 - b] += bcost;
            fenc->costEstAq[b - p0][p1 - b] += bcostAq;
            if (!listused && !bBidir)
                fenc->intraMbs[b - p0]++;
        }
        else // 当前frame的cost的计算是分了多slice并行计算的 
        {
            m_slice[slice].costEst += bcost;
            m_slice[slice].costEstAq += bcostAq;
            if (!listused && !bBidir)
                m_slice[slice].intraMbs++;
        }
    }

    /* mv sts only need in case "p0 < b == p1" */
    if( bFrameScoreCU && p0 < b && b == p1 )
    {
        MV large_mv = fenc->lowresMvs[0][listDist[0]][cuXY].roundToFPel();
        if (slice < 0)//当前frame的cost 计算是按照一帧一帧计算的
        {
            if ((abs(large_mv.x) + abs(large_mv.y) > m_lookahead.i_large_mv_thres2))
            {
                fenc->largeMvs[b - p0] += 1;
            }
            if ((abs(large_mv.x) + abs(large_mv.y) > m_lookahead.i_large_mv_thres3))
            {
                fenc->veryLargeMvs[b - p0] += 1;
            }
            if (abs(large_mv.x) + abs(large_mv.y) > S265_MAX(1, m_lookahead.i_large_mv_thres / 6) &&
                m_lookahead.m_param->rc.rateControlMode)
            {
                fenc->hasSmallMvs[b - p0] += 1;
            }
        }
        else// slice 级 并行
        {
            if ((abs(large_mv.x) + abs(large_mv.y) > m_lookahead.i_large_mv_thres2))
            {
                m_slice[slice].largeMvs += 1;
            }
            if ((abs(large_mv.x) + abs(large_mv.y) > m_lookahead.i_large_mv_thres3))
            {
                m_slice[slice].veryLargeMvs += 1;
            }
            if (abs(large_mv.x) + abs(large_mv.y) > S265_MAX(1, m_lookahead.i_large_mv_thres / 6) &&
                m_lookahead.m_param->rc.rateControlMode)
            {
                m_slice[slice].hasSmallMvs += 1;
            }
        }
    }

    fenc->rowSatds[b - p0][p1 - b][cuY] += bcostAq;
    fenc->lowresCosts[b - p0][p1 - b][cuXY] = (uint16_t)(S265_MIN(bcost, LOWRES_COST_MASK) | (listused << LOWRES_COST_SHIFT));
}

}
