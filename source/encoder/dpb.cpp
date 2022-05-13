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
#include "slice.h"

#include "dpb.h"

using namespace S265_NS;

DPB::~DPB()
{
    while (!m_freeList.empty())
    {
        Frame* curFrame = m_freeList.popFront();
        curFrame->destroy();
        delete curFrame;
    }

    while (!m_picList.empty())
    {
        Frame* curFrame = m_picList.popFront();
        curFrame->destroy();
        delete curFrame;
    }
// 用了一个单链表管理复用 frameData 数据结构
    while (m_frameDataFreeList)
    {
        FrameData* next = m_frameDataFreeList->m_freeListNext;
        m_frameDataFreeList->destroy();

        m_frameDataFreeList->m_reconPic->destroy();
        delete m_frameDataFreeList->m_reconPic;

        delete m_frameDataFreeList;
        m_frameDataFreeList = next;
    }
}

// move unreferenced pictures from picList to freeList for recycle
void DPB::recycleUnreferenced()
{
    Frame *iterFrame = m_picList.first();

    while (iterFrame)
    {
        Frame *curFrame = iterFrame;
        iterFrame = iterFrame->m_next;
        if (!curFrame->m_encData->m_bHasReferences && !curFrame->m_countRefEncoders)
        {
            curFrame->m_bChromaExtended = false;

            // Reset column counter
            S265_CHECK(curFrame->m_reconRowFlag != NULL, "curFrame->m_reconRowFlag check failure");
            S265_CHECK(curFrame->m_reconColCount != NULL, "curFrame->m_reconColCount check failure");
            S265_CHECK(curFrame->m_numRows > 0, "curFrame->m_numRows check failure");

            for(int32_t row = 0; row < curFrame->m_numRows; row++)
            {
                curFrame->m_reconRowFlag[row].set(0);
                curFrame->m_reconColCount[row].set(0);
            }

            // iterator is invalidated by remove, restart scan
            m_picList.remove(*curFrame);
            iterFrame = m_picList.first();

            m_freeList.pushBack(*curFrame);
            // 将curFrame 中的 m_encData （作为一个链表结点）链接到dpb中frameData的链头
            // 或者说先将dpb中的整个m_encData链表连接到当前curFrame的m_encData结点之后,
            // 再将curFrame的m_encData作为新的头结点重新赋值给DPB中frameData的链头
            curFrame->m_encData->m_freeListNext = m_frameDataFreeList;
            m_frameDataFreeList = curFrame->m_encData;

            for (int i = 0; i < INTEGRAL_PLANE_NUM; i++)
            {
                if (curFrame->m_encData->m_meBuffer[i] != NULL)
                {
                    S265_FREE(curFrame->m_encData->m_meBuffer[i]);
                    curFrame->m_encData->m_meBuffer[i] = NULL;
                }
            }
            curFrame->m_encData = NULL;
            curFrame->m_reconPic = NULL;
        }
    }
}

void DPB::prepareEncode(Frame *newFrame)
{
    Slice* slice = newFrame->m_encData->m_slice;
    slice->m_poc = newFrame->m_poc;

    int pocCurr = slice->m_poc;
    int type = newFrame->m_lowres.sliceType;
    bool bIsKeyFrame = newFrame->m_lowres.bKeyframe;
    slice->m_nalUnitType = getNalUnitType(pocCurr, bIsKeyFrame);
    if (slice->m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL || slice->m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP)
        m_lastIDR = pocCurr;
    slice->m_lastIDR = m_lastIDR;
    slice->m_sliceType = IS_S265_TYPE_B(type) ? B_SLICE : (type == S265_TYPE_P) ? P_SLICE : I_SLICE;

    if (type == S265_TYPE_B)//非参考B帧类型,不作为参考
    {
        newFrame->m_encData->m_bHasReferences = false;

        // Adjust NAL type for unreferenced B frames (change from _R "referenced"
        // to _N "non-referenced" NAL unit type)
        switch (slice->m_nalUnitType)
        {
        case NAL_UNIT_CODED_SLICE_TRAIL_R:
            slice->m_nalUnitType = m_bTemporalSublayer ? NAL_UNIT_CODED_SLICE_TSA_N : NAL_UNIT_CODED_SLICE_TRAIL_N;
            break;
        case NAL_UNIT_CODED_SLICE_RADL_R:
            slice->m_nalUnitType = NAL_UNIT_CODED_SLICE_RADL_N;
            break;
        case NAL_UNIT_CODED_SLICE_RASL_R:
            slice->m_nalUnitType = NAL_UNIT_CODED_SLICE_RASL_N;
            break;
        default:
            break;
        }
    }
    else
    {
        /* m_bHasReferences starts out as true for non-B pictures, and is set to false
         * once no more pictures reference it */
        // 其他帧，首先全都都先标记为作为参考
        newFrame->m_encData->m_bHasReferences = true;
    }

    m_picList.pushFront(*newFrame);// 当前即将要编码的帧放到dpb的list的头部

    // Do decoding refresh marking if any
    decodingRefreshMarking(pocCurr, slice->m_nalUnitType);

    computeRPS(pocCurr, slice->isIRAP(), &slice->m_rps, slice->m_sps->maxDecPicBuffering);

    // Mark pictures in m_piclist as unreferenced if they are not included in RPS
    applyReferencePictureSet(&slice->m_rps, pocCurr);

    if (slice->m_sliceType != I_SLICE)
        slice->m_numRefIdx[0] = s265_clip3(1, newFrame->m_param->maxNumReferences, slice->m_rps.numberOfNegativePictures);
    else
        slice->m_numRefIdx[0] = S265_MIN(newFrame->m_param->maxNumReferences, slice->m_rps.numberOfNegativePictures); // Ensuring L0 contains just the -ve POC
    //slice->m_numRefIdx[1] = S265_MIN(newFrame->m_param->bBPyramid ? 2 : 1, slice->m_rps.numberOfPositivePictures);
    if (newFrame->m_param->bBPyramid == S265_B_PYRAMID_HIER)
    {
        int maxDelay = 2;
        if (newFrame->m_param->bframes > 7)
            maxDelay = 4;
        else if (newFrame->m_param->bframes > 3)
            maxDelay = 3;
        slice->m_numRefIdx[1] = S265_MIN(maxDelay, slice->m_rps.numberOfPositivePictures);
    }
    else
    {
        slice->m_numRefIdx[1] = S265_MIN(newFrame->m_param->bBPyramid ? 2 : 1, slice->m_rps.numberOfPositivePictures);
    }

    slice->setRefPicList(m_picList);

    S265_CHECK(slice->m_sliceType != B_SLICE || slice->m_numRefIdx[1], "B slice without L1 references (non-fatal)\n");

    if (slice->m_sliceType == B_SLICE)
    {
        /* TODO: the lookahead should be able to tell which reference picture
         * had the least motion residual.  We should be able to use that here to
         * select a colocation reference list and index */
        slice->m_colFromL0Flag = false;
        slice->m_colRefIdx = 0;
        slice->m_bCheckLDC = false;
    }
    else
    {
        slice->m_bCheckLDC = true;
        slice->m_colFromL0Flag = true;
        slice->m_colRefIdx = 0;
    }

    // Disable Loopfilter in bound area, because we will do slice-parallelism in future
    slice->m_sLFaseFlag = (newFrame->m_param->maxSlices > 1) ? false : ((SLFASE_CONSTANT & (1 << (pocCurr % 31))) > 0);

    /* Increment reference count of all motion-referenced frames to prevent them
     * from being recycled. These counts are decremented at the end of
     * compressFrame() */
    int numPredDir = slice->isInterP() ? 1 : slice->isInterB() ? 2 : 0;
    for (int l = 0; l < numPredDir; l++)
    {
        for (int ref = 0; ref < slice->m_numRefIdx[l]; ref++)
        {
            Frame *refpic = slice->m_refFrameList[l][ref];
            ATOMIC_INC(&refpic->m_countRefEncoders);
        }
    }
}

// void DPB::computeRPS(int curPoc, bool isRAP, RPS * rps, unsigned int maxDecPicBuffer)
// {
//     unsigned int poci = 0, numNeg = 0, numPos = 0;

//     Frame* iterPic = m_picList.first();

//     while (iterPic && (poci < maxDecPicBuffer - 1))
//     {
//         if ((iterPic->m_poc != curPoc) && iterPic->m_encData->m_bHasReferences)
//         {
//             if ((m_lastIDR >= curPoc) || (m_lastIDR <= iterPic->m_poc))
//             {
//                     rps->poc[poci] = iterPic->m_poc;
//                     rps->deltaPOC[poci] = rps->poc[poci] - curPoc;
//                     (rps->deltaPOC[poci] < 0) ? numNeg++ : numPos++;
//                     rps->bUsed[poci] = !isRAP;
//                     poci++;
//             }
//         }
//         iterPic = iterPic->m_next;
//     }

//     rps->numberOfPictures = poci;
//     rps->numberOfPositivePictures = numPos;
//     rps->numberOfNegativePictures = numNeg;

//     rps->sortDeltaPOC();
// }

void DPB::computeRPS(int curPoc, bool isRAP, RPS * rps, unsigned int maxDecPicBuffer)
{
    unsigned int poci = 0, numNeg = 0, numPos = 0;


    Frame* curPic = m_picList.first();
    while (curPic)
    {
        if (curPic->m_poc == curPoc)
        {
            break;
        }
        curPic = curPic->m_next;
    }
    int cur_temporal_id = curPic->m_lowres.i_temporal_id;
    int cur_level = curPic->m_lowres.i_level;

    if (m_dpb_method)
    {

        if (S265_B_PYRAMID_HIER == m_pyramid_type)
        //if (0)
        {
            Frame *tmpPic = m_picList.first();
            Frame *rmCanList[MAX_NUM_REF];
            int rmCanScoreList[MAX_NUM_REF];
            int rmCanNum = 0;
            int totalRefNum = 0;
            int maxLevel = 5;

            //寻找可丢弃的参考帧
            while (tmpPic)
            {
                if ((tmpPic->m_poc != curPoc) && tmpPic->m_encData->m_bHasReferences)
                {
                    int score = 100000;
                    if (cur_level == 0)
                    {
                        tmpPic->m_lowres.i_ref_value = -1;
                    }
                    if (cur_temporal_id > 0 && tmpPic->m_lowres.i_temporal_id >= cur_temporal_id)
                    {
                        score = 0;
                        tmpPic->m_lowres.i_ref_value = -2;
                    }
                    else if (cur_temporal_id == 0 && tmpPic->m_lowres.i_temporal_id > cur_temporal_id)
                    {
                        score = 0;
                        tmpPic->m_lowres.i_ref_value = -2;
                    }
                    rmCanScoreList[rmCanNum] = score + tmpPic->m_lowres.i_ref_value * 1000 + (maxLevel - tmpPic->m_lowres.i_temporal_id) * 100 + tmpPic->m_poc;
                    rmCanList[rmCanNum++] = tmpPic;
                    totalRefNum++;
                }
                tmpPic = tmpPic->m_next;
            }
            if (totalRefNum > maxDecPicBuffer - 1)
            {
                //如果除去可丢弃帧依然大于maxDecPicBuffer - 1，那么舍弃掉所有可丢弃帧，由后面流程根据需要舍弃较远的i_temporal_id为0的参考帧
                // if (totalRefNum - rmCanNum >= maxDecPicBuffer - 1)
                // {
                //     for (int i = 0; i < rmCanNum; i++)
                //     {
                //         rmCanList[i]->m_encData->m_bHasReferences = false;
                //     }
                // }
                // else
                {
                    int tmpScore;
                    //舍弃rmNum数量的参考帧，根据score大小
                    int rmNum = totalRefNum - maxDecPicBuffer + 1;
                    //找出score最小的几个，排到头部
                    for (int i = 0; i < rmNum; i++)
                    {
                        for (int j = rmCanNum - 1; j > 0; j--)
                        {
                            if (rmCanScoreList[j] < rmCanScoreList[j - 1])
                            {
                                tmpPic = rmCanList[j];
                                tmpScore = rmCanScoreList[j];
                                rmCanList[j] = rmCanList[j - 1];
                                rmCanScoreList[j] = rmCanScoreList[j - 1];
                                rmCanList[j - 1] = tmpPic;
                                rmCanScoreList[j - 1] = tmpScore;
                            }
                        }
                        rmCanList[i]->m_encData->m_bHasReferences = false;
                    }
                }
            }
        }
    }

    Frame* iterPic = m_picList.first();
    while (iterPic && (poci < maxDecPicBuffer - 1))
    {
        // 找出与当前帧不想等的用作参考的帧
        if ((iterPic->m_poc != curPoc) && iterPic->m_encData->m_bHasReferences)
        {
            if (!m_dpb_method)
            {
                if (cur_temporal_id > 0 && iterPic->m_lowres.i_temporal_id >= cur_temporal_id)
                {
                    iterPic->m_encData->m_bHasReferences = false;
                    continue;
                }
                if (cur_temporal_id == 0 && iterPic->m_lowres.i_temporal_id > cur_temporal_id)
                {
                    iterPic->m_encData->m_bHasReferences = false;
                    continue;
                }
            }
            //如果当前的编码帧的poc 位于最近编码的idr帧之前,则dpb中所有的帧都计入rps,否则,只有位于dir后面的帧才能作为当前编码帧的参考帧
            if ((m_lastIDR >= curPoc) || (m_lastIDR <= iterPic->m_poc))
            {
                    rps->poc[poci] = iterPic->m_poc;
                    rps->deltaPOC[poci] = rps->poc[poci] - curPoc;
                    (rps->deltaPOC[poci] < 0) ? numNeg++ : numPos++;
                    rps->bUsed[poci] = !isRAP;
                    poci++;
            }
        }
        iterPic = iterPic->m_next;
    }

    rps->numberOfPictures = poci;
    rps->numberOfPositivePictures = numPos;
    rps->numberOfNegativePictures = numNeg;
    //negtive（前向） 按照poc降序排列，postive（后向） 按照poc升序排列
    rps->sortDeltaPOC();
}

/* Marking reference pictures when an IDR/CRA is encountered. */
void DPB::decodingRefreshMarking(int pocCurr, NalUnitType nalUnitType)
{
    if (nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL || nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP)
    {
        /* If the nal_unit_type is IDR, all pictures in the reference picture
         * list are marked as "unused for reference" */
        // IDR 全部标记为非参考
        Frame* iterFrame = m_picList.first();
        while (iterFrame)
        {
            if (iterFrame->m_poc != pocCurr)
                iterFrame->m_encData->m_bHasReferences = false;
            iterFrame = iterFrame->m_next;
        }
    }
    else // CRA or No IDR
    {
        if (m_bRefreshPending && pocCurr > m_pocCRA)
        {
            /* If the bRefreshPending flag is true (a deferred decoding refresh
             * is pending) and the current temporal reference is greater than
             * the temporal reference of the latest CRA picture (pocCRA), mark
             * all reference pictures except the latest CRA picture as "unused
             * for reference" and set the bRefreshPending flag to false */
            // 将poc 为与CRA 之前的帧 全部标记为不可参考
            // 只保留当前poc 和 the latest CRA
            Frame* iterFrame = m_picList.first();
            while (iterFrame)
            {
                if (iterFrame->m_poc != pocCurr && iterFrame->m_poc != m_pocCRA)
                    iterFrame->m_encData->m_bHasReferences = false;
                iterFrame = iterFrame->m_next;
            }

            m_bRefreshPending = false;
        }
        if (nalUnitType == NAL_UNIT_CODED_SLICE_CRA)
        {
            /* If the nal_unit_type is CRA, set the bRefreshPending flag to true
             * and pocCRA to the temporal reference of the current picture */
            m_bRefreshPending = true;
            m_pocCRA = pocCurr;
        }
    }

    /* Note that the current picture is already placed in the reference list and
     * its marking is not changed.  If the current picture has a nal_ref_idc
     * that is not 0, it will remain marked as "used for reference" */
}

/** Function for applying picture marking based on the Reference Picture Set */
//在dpb中找出所有不属于rps中的参考帧，标记为不用于参考
void DPB::applyReferencePictureSet(RPS *rps, int curPoc)
{
    // loop through all pictures in the reference picture buffer
    Frame* iterFrame = m_picList.first();
    while (iterFrame)
    {
        if (iterFrame->m_poc != curPoc && iterFrame->m_encData->m_bHasReferences)
        {
            // loop through all pictures in the Reference Picture Set
            // to see if the picture should be kept as reference picture
            bool referenced = false;
            for (int i = 0; i < rps->numberOfPositivePictures + rps->numberOfNegativePictures; i++)
            {
                if (iterFrame->m_poc == curPoc + rps->deltaPOC[i])
                {
                    referenced = true;
                    break;
                }
            }
            if (!referenced)
                iterFrame->m_encData->m_bHasReferences = false;
        }
        iterFrame = iterFrame->m_next;
    }
}

/* deciding the nal_unit_type */
NalUnitType DPB::getNalUnitType(int curPOC, bool bIsKeyFrame)
{
    if (!curPOC)
        return NAL_UNIT_CODED_SLICE_IDR_N_LP;
    if (bIsKeyFrame)
        return m_bOpenGOP ? NAL_UNIT_CODED_SLICE_CRA : m_bhasLeadingPicture ? NAL_UNIT_CODED_SLICE_IDR_W_RADL : NAL_UNIT_CODED_SLICE_IDR_N_LP;
    if (m_pocCRA && curPOC < m_pocCRA)
        // All leading pictures are being marked as TFD pictures here since
        // current encoder uses all reference pictures while encoding leading
        // pictures. An encoder can ensure that a leading picture can be still
        // decodable when random accessing to a CRA/CRANT/BLA/BLANT picture by
        // controlling the reference pictures used for encoding that leading
        // picture. Such a leading picture need not be marked as a TFD picture.
        return NAL_UNIT_CODED_SLICE_RASL_R;

    if (m_lastIDR && curPOC < m_lastIDR)
        return NAL_UNIT_CODED_SLICE_RADL_R;

    return NAL_UNIT_CODED_SLICE_TRAIL_R;
}
