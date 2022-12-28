/*****************************************************************************
 * Copyright (C) 2013-2020 MulticoreWare, Inc
 *
 * Authors: Steve Borho <steve@borho.org>
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
#include "piclist.h"
#include "picyuv.h"
#include "slice.h"

using namespace S265_NS;

/** 函数功能       ： 获取参考帧List 以及 POC
/*  调用范围       ： 只在DPB::prepareEncode函数中被调用
* \参数 picList    ： DPB m_picList
*   返回值         ： null
**/
void Slice::setRefPicList(PicList& picList)
{
    if (m_sliceType == I_SLICE)
    {
        memset(m_refFrameList, 0, sizeof(m_refFrameList));
        memset(m_refReconPicList, 0, sizeof(m_refReconPicList));
        memset(m_refPOCList, 0, sizeof(m_refPOCList));//I帧 无参考帧，全部置为0
        m_numRefIdx[1] = m_numRefIdx[0] = 0;
        return;
    }

    Frame* refPic = NULL;//临时变量 用于获取参考帧
    Frame* refPicSetStCurr0[MAX_NUM_REF];//存储前向帧 指针
    Frame* refPicSetStCurr1[MAX_NUM_REF];//存储后向帧 指针
    Frame* refPicSetLtCurr[MAX_NUM_REF];//存储长期参考帧  （x265没有应用）
    int numPocStCurr0 = 0;//计数前向帧个数
    int numPocStCurr1 = 0;//计数后向帧个数
    int numPocLtCurr = 0;//计数long-term 参考帧 （x265没有应用）
    int i;//临时变量 用于计数

    for (i = 0; i < m_rps.numberOfNegativePictures; i++)//获取前向参考帧
    {
        if (m_rps.bUsed[i])
        {
            refPic = picList.getPOC(m_poc + m_rps.deltaPOC[i]);
            refPicSetStCurr0[numPocStCurr0] = refPic;
            numPocStCurr0++;
        }
    }

    for (; i < m_rps.numberOfNegativePictures + m_rps.numberOfPositivePictures; i++)//获取后向参考帧
    {
        if (m_rps.bUsed[i])
        {
            refPic = picList.getPOC(m_poc + m_rps.deltaPOC[i]);
            refPicSetStCurr1[numPocStCurr1] = refPic;
            numPocStCurr1++;
        }
    }

    S265_CHECK(m_rps.numberOfPictures == m_rps.numberOfNegativePictures + m_rps.numberOfPositivePictures,
               "unexpected picture in RPS\n");

    // ref_pic_list_init
    Frame* rpsCurrList0[MAX_NUM_REF + 1];//存储List0
    Frame* rpsCurrList1[MAX_NUM_REF + 1];//存储List1
    int numPocTotalCurr = numPocStCurr0 + numPocStCurr1 + numPocLtCurr;//参考帧个数

    int cIdx = 0;
    //将前向帧和后向帧以及长期参考帧全部存入List0
    for (i = 0; i < numPocStCurr0; i++, cIdx++)
        rpsCurrList0[cIdx] = refPicSetStCurr0[i];

    for (i = 0; i < numPocStCurr1; i++, cIdx++)
        rpsCurrList0[cIdx] = refPicSetStCurr1[i];

    for (i = 0; i < numPocLtCurr; i++, cIdx++)
        rpsCurrList0[cIdx] = refPicSetLtCurr[i];

    S265_CHECK(cIdx == numPocTotalCurr, "RPS index check fail\n");

    if (m_sliceType == B_SLICE)
    {
        cIdx = 0;
        //将后向帧和前向帧以及长期参考帧全部存入List1
        for (i = 0; i < numPocStCurr1; i++, cIdx++)
            rpsCurrList1[cIdx] = refPicSetStCurr1[i];

        for (i = 0; i < numPocStCurr0; i++, cIdx++)
            rpsCurrList1[cIdx] = refPicSetStCurr0[i];

        for (i = 0; i < numPocLtCurr; i++, cIdx++)
            rpsCurrList1[cIdx] = refPicSetLtCurr[i];

        S265_CHECK(cIdx == numPocTotalCurr, "RPS index check fail\n");
    }
     //存储实际List0列表值
    for (int rIdx = 0; rIdx < m_numRefIdx[0]; rIdx++)
    {
        cIdx = rIdx % numPocTotalCurr;
        S265_CHECK(cIdx >= 0 && cIdx < numPocTotalCurr, "RPS index check fail\n");
        m_refFrameList[0][rIdx] = rpsCurrList0[cIdx];
    }

    if (m_sliceType != B_SLICE)
    {
        m_numRefIdx[1] = 0;
        memset(m_refFrameList[1], 0, sizeof(m_refFrameList[1]));//不是B帧 ，将后向List置为0
    }
    else
    {
        //存储实际List1列表值
        for (int rIdx = 0; rIdx < m_numRefIdx[1]; rIdx++)
        {
            cIdx = rIdx % numPocTotalCurr;
            S265_CHECK(cIdx >= 0 && cIdx < numPocTotalCurr, "RPS index check fail\n");
            m_refFrameList[1][rIdx] = rpsCurrList1[cIdx];
        }
    }
    //获取参考帧POC
    for (int dir = 0; dir < 2; dir++)
        for (int numRefIdx = 0; numRefIdx < m_numRefIdx[dir]; numRefIdx++)
            m_refPOCList[dir][numRefIdx] = m_refFrameList[dir][numRefIdx]->m_poc;
}
/** 函数功能           ： 关闭当前帧的加权预测
/*  调用范围           ： 只在Slice()、FrameEncoder::compressFrame()、weightAnalyse函数中被调用
*   返回值             ： null
**/
void Slice::disableWeights()
{
    for (int l = 0; l < 2; l++)
        for (int i = 0; i < MAX_NUM_REF; i++)
            for (int yuv = 0; yuv < 3; yuv++)
            {
                WeightParam& wp = m_weightPredTable[l][i][yuv];
                wp.wtPresent = 0;
                wp.log2WeightDenom = 0;
                wp.inputWeight = 1;
                wp.inputOffset = 0;
            }
}

/* Sorts the deltaPOC and Used by current values in the RPS based on the
 * deltaPOC values.  deltaPOC values are sorted with -ve values before the +ve
 * values.  -ve values are in decreasing order.  +ve values are in increasing
 * order */

/** 函数功能           ： 将RPS列表中的deltaPOC和bused 按照 远 近 （当前帧） 近 远的次序排序
/*  调用范围           ： 只在DPB::computeRPS函数中被调用
*   返回值             ： null
**/
void RPS::sortDeltaPOC()
{
    // sort in increasing order (smallest first)
    for (int j = 1; j < numberOfPictures; j++)//将deltaPoc按照从小到大排序（即，按照poc的顺序从小到大排序）
    {
        int dPOC = deltaPOC[j];
        bool used = bUsed[j];
        for (int k = j - 1; k >= 0; k--)
        {
            int temp = deltaPOC[k];
            if (dPOC < temp)
            {
                deltaPOC[k + 1] = temp;
                bUsed[k + 1] = bUsed[k];
                deltaPOC[k] = dPOC;
                bUsed[k] = used;
            }
        }
    }

    // flip the negative values to largest first
    int numNegPics = numberOfNegativePictures;//获取前向帧个数
    for (int j = 0, k = numNegPics - 1; j < numNegPics >> 1; j++, k--)//将前向帧按照由近即远的方式换过来排序
    {
        int dPOC = deltaPOC[j];
        bool used = bUsed[j];
        deltaPOC[j] = deltaPOC[k];
        bUsed[j] = bUsed[k];
        deltaPOC[k] = dPOC;
        bUsed[k] = used;
    }

    // zy test: use only 1 ref for both postive and negative ref
    // int negativeFirst = 0;
    // int postiveFirst = numberOfNegativePictures;
    // for (int j = 0; j < numberOfPictures; j++)
    // {
    //     if (j != negativeFirst && j != postiveFirst)
    //         bUsed[j] = 0;
    // }
}
/** 函数功能       ： 返回一帧中最后实际像素在帧中的4x4块标号+1
/*  调用范围       ： 只在Encoder::encode函数中被调用
* \参数 endCUAddr  ： (一帧CTU个数)*（在CTU中4x4的个数）
*   返回值         ： 返回一帧中最后实际像素在帧中的4x4块标号+1
**/
uint32_t Slice::realEndAddress(uint32_t endCUAddr) const
{
    // Calculate end address
    uint32_t internalAddress = (endCUAddr - 1) % m_param->num4x4Partitions;//最后一个CTU的最后一个4x4块在内部的标号
    uint32_t externalAddress = (endCUAddr - 1) / m_param->num4x4Partitions;//最后一个CTU在帧中的标号
    uint32_t xmax = m_sps->picWidthInLumaSamples - (externalAddress % m_sps->numCuInWidth) * m_param->maxCUSize;//最后一个CTU的实际像素宽度上有多少像素（不含扩边）
    uint32_t ymax = m_sps->picHeightInLumaSamples - (externalAddress / m_sps->numCuInWidth) * m_param->maxCUSize;//最后一个CTU的实际像素高度上有多少像素（不含扩边）

    while (g_zscanToPelX[internalAddress] >= xmax || g_zscanToPelY[internalAddress] >= ymax)//找到实际像素右下角的4x4的zigzag标号
        internalAddress--;

    internalAddress++;//因为是从0开始计数，这样加一看看是否充满整个CTU的4x4个数 （这里可以理解为最后一个CTU含有多少实际的4x4块个数）
    if (internalAddress == m_param->num4x4Partitions)//如果最后一个CTU是一个完成的CTU
    {
        internalAddress = 0;//内部编号为0
        externalAddress++;//因为是从0开始计数，这里表示一帧含有多少个（充满状态）CTU
    }

    return externalAddress * m_param->num4x4Partitions + internalAddress;//返回一帧中最后实际像素在帧中的4x4块标号+1
}


