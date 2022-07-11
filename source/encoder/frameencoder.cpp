/*****************************************************************************
 * Copyright (C) 2013-2020 MulticoreWare, Inc
 *
 * Authors: Chung Shin Yee <shinyee@multicorewareinc.com>
 *          Min Chen <chenm003@163.com>
 *          Steve Borho <steve@borho.org>
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
#include "wavefront.h"
#include "param.h"

#include "encoder.h"
#include "frameencoder.h"
#include "common.h"
#include "slicetype.h"
#include "nal.h"

namespace S265_NS {
void weightAnalyse(Slice& slice, Frame& frame, s265_param& param);

FrameEncoder::FrameEncoder()
{
    m_prevOutputTime = s265_mdate();
    m_reconfigure = false;
    m_isFrameEncoder = true;
    m_threadActive = true;
    m_slicetypeWaitTime = 0;
    m_activeWorkerCount = 0;
    m_completionCount = 0;
    m_outStreams = NULL;
    m_backupStreams = NULL;
    m_substreamSizes = NULL;
    m_nr = NULL;
    m_tld = NULL;
    m_rows = NULL;
    m_top = NULL;
    m_param = NULL;
    m_frame = NULL;
    m_cuGeoms = NULL;
    m_ctuGeomMap = NULL;
    m_localTldIdx = 0;
    memset(&m_rce, 0, sizeof(RateControlEntry));
}

void FrameEncoder::destroy()
{
    if (m_pool)
    {
        if (!m_jpId)// 首个jobprovider 负责new/delete threadlocaldata
        {
            int numTLD = m_pool->m_numWorkers;
            if (!m_param->bEnableWavefront)
                numTLD += m_pool->m_numProviders;
            for (int i = 0; i < numTLD; i++)
                m_tld[i].destroy();
            delete [] m_tld;
        }
    }
    else
    {
        m_tld->destroy();
        delete m_tld;
    }

    delete[] m_rows;
    delete[] m_outStreams;
    delete[] m_backupStreams;
    S265_FREE(m_sliceBaseRow);
    S265_FREE((void*)m_bAllRowsStop);
    S265_FREE((void*)m_vbvResetTriggerRow);
    S265_FREE(m_sliceMaxBlockRow);
    S265_FREE(m_cuGeoms);
    S265_FREE(m_ctuGeomMap);
    S265_FREE(m_substreamSizes);
    S265_FREE(m_nr);

    m_frameFilter.destroy();

    if (m_param->bEmitHRDSEI)
    {
        delete m_rce.picTimingSEI;
        delete m_rce.hrdTiming;
    }
}

bool FrameEncoder::init(Encoder *top, int numRows, int numCols)
{
    m_top = top;
    m_param = top->m_param;
    m_numRows = numRows;
    m_numCols = numCols;
    m_reconfigure = false;
    m_filterRowDelay = ((m_param->bEnableSAO && m_param->bSaoNonDeblocked)
                        || (!m_param->bEnableLoopFilter && m_param->bEnableSAO)) ?
                        2 : (m_param->bEnableSAO || m_param->bEnableLoopFilter ? 1 : 0);
    m_filterRowDelayCus = m_filterRowDelay * numCols;
    m_rows = new CTURow[m_numRows];
    bool ok = !!m_numRows;

    m_sliceBaseRow = S265_MALLOC(uint32_t, m_param->maxSlices + 1);//加多了一个1
    m_bAllRowsStop = S265_MALLOC(bool, m_param->maxSlices);// 每个slice 都需要有个标志为，来表示是否其所有的行需要stop
    m_vbvResetTriggerRow = S265_MALLOC(int, m_param->maxSlices);
    ok &= !!m_sliceBaseRow;
    // 多slice 编码时，均分cut row 行
    m_sliceGroupSize = (uint16_t)(m_numRows + m_param->maxSlices - 1) / m_param->maxSlices;
    uint32_t sliceGroupSizeAccu = (m_numRows << 8) / m_param->maxSlices;// 为了提高精度，放大256倍
    uint32_t rowSum = sliceGroupSizeAccu;
    uint32_t sidx = 0;
    for (uint32_t i = 0; i < m_numRows; i++)
    {
        const uint32_t rowRange = (rowSum >> 8);
        if ((i >= rowRange) & (sidx != m_param->maxSlices - 1))
        {
            rowSum += sliceGroupSizeAccu;
            m_sliceBaseRow[++sidx] = i;// 目的是为了求第slice_idx个slice 的起始ctu_row index
        }
    }
    S265_CHECK(sidx < m_param->maxSlices, "sliceID check failed!");
    m_sliceBaseRow[0] = 0;
    m_sliceBaseRow[m_param->maxSlices] = m_numRows;

    m_sliceMaxBlockRow = S265_MALLOC(uint32_t, m_param->maxSlices + 1);//加多了一个1
    ok &= !!m_sliceMaxBlockRow;
    // 多slice 编码时，均分block row 行
    uint32_t maxBlockRows = (m_param->sourceHeight + (16 - 1)) / 16;
    sliceGroupSizeAccu = (maxBlockRows << 8) / m_param->maxSlices;// 为了提高精度，放大256倍
    rowSum = sliceGroupSizeAccu;
    sidx = 0;
    for (uint32_t i = 0; i < maxBlockRows; i++)
    {
        const uint32_t rowRange = (rowSum >> 8);
        if ((i >= rowRange) & (sidx != m_param->maxSlices - 1))
        {
            rowSum += sliceGroupSizeAccu;
            m_sliceMaxBlockRow[++sidx] = i;// 目的是为了求第slice_idx个slice 的起始block_row index
        }
    }
    m_sliceMaxBlockRow[0] = 0;
    m_sliceMaxBlockRow[m_param->maxSlices] = maxBlockRows;

    /* determine full motion search range */
    /*计算依赖参考帧的搜索范围*/
    int range  = m_param->searchRange;       /* fpel search 有参数设定的参考范围*/
    range += !!(m_param->searchMethod < 2);  /* diamond/hex range check lag  由dia/hex 算法决定的需要额外多一个整像素范围*/
    range += NTAPS_LUMA / 2;                 /* subpel filter half-length 亚像素搜素需要的差值决定需要额外多4个整像素范围 */
    range += 2 + (MotionEstimate::hpelIterationCount(m_param->subpelRefine) + 1) / 2; /* subpel refine steps */
    m_refLagRows = /*(m_param->maxSlices > 1 ? 1 : 0) +*/ 1 + ((range + m_param->maxCUSize - 1) / m_param->maxCUSize);
    //m_refLagRows 一般是3个ctu行

    // NOTE: 2 times of numRows because both Encoder and Filter in same queue
    //ctu 行级编码任务数量 以及 ctu 行级滤波 任务数量 放在一个队列 
    if (!WaveFront::init(m_numRows * 2))//调用父类waveFront的init 函数
    {
        s265_log(m_param, S265_LOG_ERROR, "unable to initialize wavefront queue\n");
        m_pool = NULL;
    }
    //帧级filter 任务的初始化
    m_frameFilter.init(top, this, numRows, numCols);

    // initialize HRD parameters of SPS
    if (m_param->bEmitHRDSEI)
    {
        m_rce.picTimingSEI = new SEIPictureTiming;
        m_rce.hrdTiming = new HRDTiming;

        ok &= m_rce.picTimingSEI && m_rce.hrdTiming;
    }

    if (m_param->noiseReductionIntra || m_param->noiseReductionInter)
        m_nr = S265_MALLOC(NoiseReduction, 1);
    if (m_nr)
        memset(m_nr, 0, sizeof(NoiseReduction));
    else
        m_param->noiseReductionIntra = m_param->noiseReductionInter = 0;

    // 7.4.7.1 - Ceil( Log2( PicSizeInCtbsY ) ) bits
    {
        unsigned long tmp;
        CLZ(tmp, (numRows * numCols - 1));
        m_sliceAddrBits = (uint16_t)(tmp + 1);
    }

    return ok;
}

/* Generate a complete list of unique geom sets for the current picture dimensions */
bool FrameEncoder::initializeGeoms()
{
    /* Geoms only vary between CTUs in the presence of picture edges */
    int maxCUSize = m_param->maxCUSize;
    int minCUSize = m_param->minCUSize;
    int heightRem = m_param->sourceHeight & (maxCUSize - 1);
    int widthRem = m_param->sourceWidth & (maxCUSize - 1);
    int allocGeoms = 1; // body 初始化为一类
    if (heightRem && widthRem)
        allocGeoms = 4; // body, right, bottom, corner 4类型 宽高都不为LCU 对齐
    else if (heightRem || widthRem)
        allocGeoms = 2; // body, right or bottom // 2类型 宽高有一边不为LCU 对齐

    m_ctuGeomMap = S265_MALLOC(uint32_t, m_numRows * m_numCols);
    m_cuGeoms = S265_MALLOC(CUGeom, allocGeoms * CUGeom::MAX_GEOMS);// MAX_GEOMS: 85
    if (!m_cuGeoms || !m_ctuGeomMap)
        return false;

    // body
    CUData::calcCTUGeoms(maxCUSize, maxCUSize, maxCUSize, minCUSize, m_cuGeoms);
    //首先全部初始化为第0个 CUGeom
    memset(m_ctuGeomMap, 0, sizeof(uint32_t) * m_numRows * m_numCols);// 第一类

    if (allocGeoms == 1)
        return true;

    int countGeoms = 1;
    if (widthRem)
    {
        // right 右边剩余部分s
        CUData::calcCTUGeoms(widthRem, maxCUSize, maxCUSize, minCUSize, m_cuGeoms + countGeoms * CUGeom::MAX_GEOMS);
        for (uint32_t i = 0; i < m_numRows; i++)
        {
            uint32_t ctuAddr = m_numCols * (i + 1) - 1;
            //右边的ctu 指向 第一个CUGeom
            m_ctuGeomMap[ctuAddr] = countGeoms * CUGeom::MAX_GEOMS;// 第2类
        }
        countGeoms++;
    }
    if (heightRem)
    {
        // bottom 底边剩余部分
        CUData::calcCTUGeoms(maxCUSize, heightRem, maxCUSize, minCUSize, m_cuGeoms + countGeoms * CUGeom::MAX_GEOMS);
        for (uint32_t i = 0; i < m_numCols; i++)
        {
            uint32_t ctuAddr = m_numCols * (m_numRows - 1) + i;
            //底边的ctu 指向 第2个CUGeom
            m_ctuGeomMap[ctuAddr] = countGeoms * CUGeom::MAX_GEOMS;// 第3类
        }
        countGeoms++;

        if (widthRem)
        {
            // corner 右下角剩余部分
            CUData::calcCTUGeoms(widthRem, heightRem, maxCUSize, minCUSize, m_cuGeoms + countGeoms * CUGeom::MAX_GEOMS);

            uint32_t ctuAddr = m_numCols * m_numRows - 1;
            //最后一个ctu 指向 第3个CUGeom
            m_ctuGeomMap[ctuAddr] = countGeoms * CUGeom::MAX_GEOMS;
            countGeoms++;
        }
        S265_CHECK(countGeoms == allocGeoms, "geometry match check failure\n");//第4类
    }

    return true;
}

bool FrameEncoder::startCompressFrame(Frame* curFrame)
{
    // 当该线程完成输出一个压缩帧时，会记录时间点 随后会调用有可能 调用 slicetypedecide 然后然后再进入该startcompressFrame函数
    // 从这这段时间可以衡量slicetypedicision的耗时
    m_slicetypeWaitTime = s265_mdate() - m_prevOutputTime;
    m_frame = curFrame;
    // 这个 m_sliceType 是 FrameEncoder 的基类’WaveFront‘的基类’JobProvider‘的成员
    m_sliceType = curFrame->m_lowres.sliceType; //使用帧类型作为优先级判断标准，越重要的类更小的值的越需要更高的优先级
    curFrame->m_encData->m_frameEncoderID = m_jpId;
    curFrame->m_encData->m_jobProvider = this;
    curFrame->m_encData->m_slice->m_mref = m_mref;

    if (!m_cuGeoms)
    {// 每个线程对象只做一次初始化
        if (!initializeGeoms())
            return false;
    }
    m_enable.trigger();// 仅仅一个启动的动作,触发 compressFrame 线程 进入新一轮的循环
    return true;
}
// 线程函数
// 子类FrameEncoder 的 threadMain 覆盖基类thread 的threadMain
void FrameEncoder::threadMain()
{
    THREAD_NAME("Frame", m_jpId);

    if (m_pool)
    {
        m_pool->setCurrentThreadAffinity();

        /* the first FE（frame_encoder） on each NUMA node is responsible for allocating thread
         * local data for all worker threads in that pool. If WPP is disabled, then
         * each FE(frame_encoder) also needs a TLD instance */
        if (!m_jpId)// 首个jobprovider 负责 new/delete threadlocaldata
        {
            int numTLD = m_pool->m_numWorkers;
            if (!m_param->bEnableWavefront)
                numTLD += m_pool->m_numProviders;

            m_tld = new ThreadLocalData[numTLD];
            for (int i = 0; i < numTLD; i++)
            {
                m_tld[i].analysis.initSearch(*m_param, m_top->m_scalingList);
                m_tld[i].analysis.create(m_tld);
            }
            // 该pool 有多少个’领导‘
            for (int i = 0; i < m_pool->m_numProviders; i++)
            {
                //如果第i个’领导‘ 是一个FrameENcoder
                if (m_pool->m_jpTable[i]->m_isFrameEncoder) /* ugh; over-allocation and other issues here */
                {
                    //
                    FrameEncoder *peer = dynamic_cast<FrameEncoder*>(m_pool->m_jpTable[i]);
                    peer->m_tld = m_tld;
                }
            }
        }

        if (m_param->bEnableWavefront)
            m_localTldIdx = -1; // cause exception if used
        else
            m_localTldIdx = m_pool->m_numWorkers + m_jpId;
    }
    else
    {
        m_tld = new ThreadLocalData;
        m_tld->analysis.initSearch(*m_param, m_top->m_scalingList);
        m_tld->analysis.create(NULL);
        m_localTldIdx = 0;
    }

    m_done.trigger();     /* 触发 m_frameEncoder[i]->m_done.wait()继续 signal that thread is initialized */
    //挂起，进入等待 m_enable.trigger() 信号，该信号由FrameEncoder::startCompressFrame(）函数触发
    m_enable.wait();      /* Encoder::encode() triggers this event */

    while (m_threadActive)
    {
        compressFrame();
        m_done.trigger(); /*  触发FrameEncoder::getEncodedPicture() 继续*/
        m_enable.wait(); /*挂起等待 由FrameEncoder::startCompressFrame(）函数触发的信号*/
    }
}

void FrameEncoder::WeightAnalysis::processTasks(int /* workerThreadId */)
{
    Frame* frame = master.m_frame;
    weightAnalyse(*frame->m_encData->m_slice, *frame, *master.m_param);
}


uint32_t getBsLength( int32_t code )
{
    uint32_t ucode = (code <= 0) ? -code << 1 : (code << 1) - 1;

    ++ucode;
    unsigned long idx;
    CLZ( idx, ucode );
    uint32_t length = (uint32_t)idx * 2 + 1;

    return length;
}

bool FrameEncoder::writeToneMapInfo(s265_sei_payload *payload)
{
    bool payloadChange = false;
    if (m_top->m_prevTonemapPayload.payload != NULL && payload->payloadSize == m_top->m_prevTonemapPayload.payloadSize)
    {
        if (memcmp(m_top->m_prevTonemapPayload.payload, payload->payload, payload->payloadSize) != 0)
            payloadChange = true;
    }
    else
    {
        payloadChange = true;
        if (m_top->m_prevTonemapPayload.payload != NULL)
            s265_free(m_top->m_prevTonemapPayload.payload);
        m_top->m_prevTonemapPayload.payload = (uint8_t*)s265_malloc(sizeof(uint8_t)* payload->payloadSize);
    }

    if (payloadChange)
    {
        m_top->m_prevTonemapPayload.payloadType = payload->payloadType;
        m_top->m_prevTonemapPayload.payloadSize = payload->payloadSize;
        memcpy(m_top->m_prevTonemapPayload.payload, payload->payload, payload->payloadSize);
    }

    bool isIDR = m_frame->m_lowres.sliceType == S265_TYPE_IDR;
    return (payloadChange || isIDR);
}

void FrameEncoder::writeTrailingSEIMessages()
{
    Slice* slice = m_frame->m_encData->m_slice;
    int planes = (m_param->internalCsp != S265_CSP_I400) ? 3 : 1;
    int32_t payloadSize = 0;

    if (m_param->decodedPictureHashSEI == 1)
    {
        m_seiReconPictureDigest.m_method = SEIDecodedPictureHash::MD5;
        for (int i = 0; i < planes; i++)
            MD5Final(&m_seiReconPictureDigest.m_state[i], m_seiReconPictureDigest.m_digest[i]);
        payloadSize = 1 + 16 * planes;
    }
    else if (m_param->decodedPictureHashSEI == 2)
    {
        m_seiReconPictureDigest.m_method = SEIDecodedPictureHash::CRC;
        for (int i = 0; i < planes; i++)
            crcFinish(m_seiReconPictureDigest.m_crc[i], m_seiReconPictureDigest.m_digest[i]);
        payloadSize = 1 + 2 * planes;
    }
    else if (m_param->decodedPictureHashSEI == 3)
    {
        m_seiReconPictureDigest.m_method = SEIDecodedPictureHash::CHECKSUM;
        for (int i = 0; i < planes; i++)
            checksumFinish(m_seiReconPictureDigest.m_checksum[i], m_seiReconPictureDigest.m_digest[i]);
        payloadSize = 1 + 4 * planes;
    }

    m_seiReconPictureDigest.setSize(payloadSize);
    m_seiReconPictureDigest.writeSEImessages(m_bs, *slice->m_sps, NAL_UNIT_SUFFIX_SEI, m_nalList, false);
}
// 开始一帧的编码,called by frameEncode threads
void FrameEncoder::compressFrame()
{
    ProfileScopeEvent(frameThread);

    m_startCompressTime = s265_mdate();
    m_totalActiveWorkerCount = 0;
    m_activeWorkerCountSamples = 0;
    m_totalWorkerElapsedTime = 0;
    m_totalNoWorkerTime = 0;
    m_countRowBlocks = 0;
    m_allRowsAvailableTime = 0;
    m_stallStartTime = 0;

    m_completionCount = 0;
    memset((void*)m_bAllRowsStop, 0, sizeof(bool) * m_param->maxSlices);// 该帧中 所有slice 的该标志清零
    memset((void*)m_vbvResetTriggerRow, -1, sizeof(int) * m_param->maxSlices);
    m_rowSliceTotalBits[0] = 0;
    m_rowSliceTotalBits[1] = 0;

    m_SSDY = m_SSDU = m_SSDV = 0;
    m_ssim = 0;
    m_ssimCnt = 0;
    memset(&(m_frame->m_encData->m_frameStats), 0, sizeof(m_frame->m_encData->m_frameStats));

    if (!m_param->bHistBasedSceneCut && m_param->rc.aqMode != S265_AQ_EDGE && m_param->recursionSkipMode == EDGE_BASED_RSKIP)
    {
        int height = m_frame->m_fencPic->m_picHeight;
        int width = m_frame->m_fencPic->m_picWidth;
        intptr_t stride = m_frame->m_fencPic->m_stride;

        if (!computeEdge(m_frame->m_edgeBitPic, m_frame->m_fencPic->m_picOrg[0], NULL, stride, height, width, false, 1))
        {
            s265_log(m_param, S265_LOG_ERROR, " Failed to compute edge !");
        }
    }

    /* Emit access unit delimiter unless this is the first frame and the user is
     * not repeating headers (since AUD is supposed to be the first NAL in the access
     * unit) */
    Slice* slice = m_frame->m_encData->m_slice;

    if (m_param->bEnableEndOfSequence && m_frame->m_lowres.sliceType == S265_TYPE_IDR && m_frame->m_poc)
    {
        m_bs.resetBits();
        m_nalList.serialize(NAL_UNIT_EOS, m_bs);
    }

    if (m_param->bEnableAccessUnitDelimiters && (m_frame->m_poc || m_param->bRepeatHeaders))
    {
        m_bs.resetBits();
        m_entropyCoder.setBitstream(&m_bs);
        m_entropyCoder.codeAUD(*slice);
        m_bs.writeByteAlignment();
        m_nalList.serialize(NAL_UNIT_ACCESS_UNIT_DELIMITER, m_bs);
        if (m_param->bSingleSeiNal)
            m_bs.resetBits();
    }
    if (m_frame->m_lowres.bKeyframe && m_param->bRepeatHeaders)
    {
        if (m_param->bOptRefListLengthPPS)
        {
            ScopedLock refIdxLock(m_top->m_sliceRefIdxLock);
            m_top->updateRefIdx();
        }
        m_top->getStreamHeaders(m_nalList, m_entropyCoder, m_bs);
    }

    // Weighted Prediction parameters estimation.
    bool bUseWeightP = slice->m_sliceType == P_SLICE && slice->m_pps->bUseWeightPred;
    bool bUseWeightB = slice->m_sliceType == B_SLICE && slice->m_pps->bUseWeightedBiPred;

    if (bUseWeightP || bUseWeightB)
    {
#if DETAILED_CU_STATS
        m_cuStats.countWeightAnalyze++;
        ScopedElapsedTime time(m_cuStats.weightAnalyzeTime);
#endif
        //WeightAnalysis wa(*this);
        //if (m_pool && wa.tryBondPeers(*this, 1))//唤醒线程池里面的1个线程去执行processTask 从而执行 weightAnalyse
        //    /* use an idle worker for weight analysis */
        //    wa.waitForExit();// 等待线程执行完, ??? 这里只有一个任务 为啥需要让别的线程去做？自己在这里等？？？
        //else
            weightAnalyse(*slice, *m_frame, *m_param);// 本线程直接调用
    }
    else
        slice->disableWeights();

    // Generate motion references
    int numPredDir = slice->isInterP() ? 1 : slice->isInterB() ? 2 : 0;
    for (int l = 0; l < numPredDir; l++)
    {
        for (int ref = 0; ref < slice->m_numRefIdx[l]; ref++)
        {
            WeightParam *w = NULL;
            if ((bUseWeightP || bUseWeightB) && slice->m_weightPredTable[l][ref][0].wtPresent)
                w = slice->m_weightPredTable[l][ref];
            slice->m_refReconPicList[l][ref] = slice->m_refFrameList[l][ref]->m_reconPic;
            // 根据是否需要加权，对参考帧进行加权与否
            m_mref[l][ref].init(slice->m_refReconPicList[l][ref], w, *m_param);
        }
    }

    int numTLD;
    if (m_pool)
        numTLD = m_param->bEnableWavefront ? m_pool->m_numWorkers : m_pool->m_numWorkers + m_pool->m_numProviders;
    else
        numTLD = 1;

    /* Get the QP for this frame from rate control. This call may block until
     * frames ahead of it in encode order have called rateControlEnd() */
    //里面会阻塞，等当前帧之前的帧完成对rateControlEnd 的访问 
    int qp = m_top->m_rateControl->rateControlStart(m_frame, &m_rce, m_top);
    m_rce.newQp = qp;

    if (m_nr) // noise reduction 相关
    {
        if (qp > QP_MAX_SPEC && m_frame->m_param->rc.vbvBufferSize)
        {
            for (int i = 0; i < numTLD; i++)
            {
                m_tld[i].analysis.m_quant.m_frameNr[m_jpId].offset = m_top->m_offsetEmergency[qp - QP_MAX_SPEC - 1];
                m_tld[i].analysis.m_quant.m_frameNr[m_jpId].residualSum = m_top->m_residualSumEmergency;
                m_tld[i].analysis.m_quant.m_frameNr[m_jpId].count = m_top->m_countEmergency;
            }
        }
        else
        {
            if (m_param->noiseReductionIntra || m_param->noiseReductionInter)
            {
                for (int i = 0; i < numTLD; i++)
                {
                    m_tld[i].analysis.m_quant.m_frameNr[m_jpId].offset = m_tld[i].analysis.m_quant.m_frameNr[m_jpId].nrOffsetDenoise;
                    m_tld[i].analysis.m_quant.m_frameNr[m_jpId].residualSum = m_tld[i].analysis.m_quant.m_frameNr[m_jpId].nrResidualSum;
                    m_tld[i].analysis.m_quant.m_frameNr[m_jpId].count = m_tld[i].analysis.m_quant.m_frameNr[m_jpId].nrCount;
                }
            }
            else
            {
                for (int i = 0; i < numTLD; i++)
                    m_tld[i].analysis.m_quant.m_frameNr[m_jpId].offset = NULL;
            }
        }
    }

    /* Clip slice QP to 0-51 spec range before encoding */
    slice->m_sliceQp = s265_clip3(-QP_BD_OFFSET, QP_MAX_SPEC, qp);
    if (m_param->bHDR10Opt)
    {
        int qpCb = s265_clip3(-12, 0, (int)floor((m_top->m_cB * ((-.46) * qp + 9.26)) + 0.5 ));
        int qpCr = s265_clip3(-12, 0, (int)floor((m_top->m_cR * ((-.46) * qp + 9.26)) + 0.5 ));
        slice->m_chromaQpOffset[0] = slice->m_pps->chromaQpOffset[0] + qpCb < -12 ? (qpCb + (-12 - (slice->m_pps->chromaQpOffset[0] + qpCb))) : qpCb;
        slice->m_chromaQpOffset[1] = slice->m_pps->chromaQpOffset[1] + qpCr < -12 ? (qpCr + (-12 - (slice->m_pps->chromaQpOffset[1] + qpCr))) : qpCr;
    }

    if (m_param->bOptQpPPS && m_param->bRepeatHeaders)
    {
        ScopedLock qpLock(m_top->m_sliceQpLock);
        for (int i = 0; i < (QP_MAX_MAX + 1); i++)
        {
            int delta = slice->m_sliceQp - (i + 1);
            int codeLength = getBsLength( delta );
            m_top->m_iBitsCostSum[i] += codeLength;
        }
        m_top->m_iFrameNum++;
    }
    m_initSliceContext.resetEntropy(*slice);

    m_frameFilter.start(m_frame, m_initSliceContext);

    /* ensure all rows are blocked prior to initializing row CTU counters */
    WaveFront::clearEnabledRowMask();

    /* reset entropy coders and compute slice id */
    m_entropyCoder.load(m_initSliceContext);
    for (uint32_t sliceId = 0; sliceId < m_param->maxSlices; sliceId++)// slice 循环
        for (uint32_t row = m_sliceBaseRow[sliceId]; row < m_sliceBaseRow[sliceId + 1]; row++)//每次slice里面的行数循环
            m_rows[row].init(m_initSliceContext, sliceId);   

    // reset slice counter for rate control update
    m_sliceCnt = 0;

    uint32_t numSubstreams = m_param->bEnableWavefront ? slice->m_sps->numCuInHeight : m_param->maxSlices;
    // 多slice 下 一定得wpp
    S265_CHECK(m_param->bEnableWavefront || (m_param->maxSlices == 1), "Multiple slices without WPP unsupport now!");
    if (!m_outStreams)
    {
        m_outStreams = new Bitstream[numSubstreams];
        if (!m_param->bEnableWavefront)
            m_backupStreams = new Bitstream[numSubstreams];
        m_substreamSizes = S265_MALLOC(uint32_t, numSubstreams);
        if (!slice->m_bUseSao)
        {
            for (uint32_t i = 0; i < numSubstreams; i++)
                m_rows[i].rowGoOnCoder.setBitstream(&m_outStreams[i]);
        }
    }
    else
    {
        for (uint32_t i = 0; i < numSubstreams; i++)
        {
            m_outStreams[i].resetBits();
            if (!slice->m_bUseSao)
                m_rows[i].rowGoOnCoder.setBitstream(&m_outStreams[i]);
            else
                m_rows[i].rowGoOnCoder.setBitstream(NULL);
        }
    }

    m_rce.encodeOrder = m_frame->m_encodeOrder;
    int prevBPSEI = m_rce.encodeOrder ? m_top->m_lastBPSEI : 0;

    if (m_frame->m_lowres.bKeyframe)// I 帧
    {
        if (m_param->bEmitHRDSEI)
        {
            SEIBufferingPeriod* bpSei = &m_top->m_rateControl->m_bufPeriodSEI;

            // since the temporal layer HRD is not ready, we assumed it is fixed
            bpSei->m_auCpbRemovalDelayDelta = 1;
            bpSei->m_cpbDelayOffset = 0;
            bpSei->m_dpbDelayOffset = 0;
            bpSei->m_concatenationFlag = (m_param->bEnableHRDConcatFlag && !m_frame->m_poc) ? true : false;

            // hrdFullness() calculates the initial CPB removal delay and offset
            m_top->m_rateControl->hrdFullness(bpSei);
            bpSei->writeSEImessages(m_bs, *slice->m_sps, NAL_UNIT_PREFIX_SEI, m_nalList, m_param->bSingleSeiNal);

            m_top->m_lastBPSEI = m_rce.encodeOrder;
        }

        if (m_frame->m_lowres.sliceType == S265_TYPE_IDR && m_param->bEmitIDRRecoverySEI)
        {
            /* Recovery Point SEI require the SPS to be "activated" */
            SEIRecoveryPoint sei;
            sei.m_recoveryPocCnt = 0;
            sei.m_exactMatchingFlag = true;
            sei.m_brokenLinkFlag = false;
            sei.writeSEImessages(m_bs, *slice->m_sps, NAL_UNIT_PREFIX_SEI, m_nalList, m_param->bSingleSeiNal);
        }
    }

    if (m_param->bEmitHRDSEI)
    {
        SEIPictureTiming *sei = m_rce.picTimingSEI;
        const VUI *vui = &slice->m_sps->vuiParameters;
        const HRDInfo *hrd = &vui->hrdParameters;
        int poc = slice->m_poc;

        if (vui->hrdParametersPresentFlag)
        {
            // The m_aucpbremoval delay specifies how many clock ticks the
            // access unit associated with the picture timing SEI message has to
            // wait after removal of the access unit with the most recent
            // buffering period SEI message
            sei->m_auCpbRemovalDelay = S265_MIN(S265_MAX(1, m_rce.encodeOrder - prevBPSEI), (1 << hrd->cpbRemovalDelayLength));
            sei->m_picDpbOutputDelay = slice->m_sps->numReorderPics + poc - m_rce.encodeOrder;
        }

        sei->writeSEImessages(m_bs, *slice->m_sps, NAL_UNIT_PREFIX_SEI, m_nalList, m_param->bSingleSeiNal);
    }

    if (m_param->preferredTransferCharacteristics > -1 && slice->isIRAP())
    {
        SEIAlternativeTC m_seiAlternativeTC;
        m_seiAlternativeTC.m_preferredTransferCharacteristics = m_param->preferredTransferCharacteristics;
        m_seiAlternativeTC.writeSEImessages(m_bs, *slice->m_sps, NAL_UNIT_PREFIX_SEI, m_nalList, m_param->bSingleSeiNal);
    }

    /* Write user SEI */
    for (int i = 0; i < m_frame->m_userSEI.numPayloads; i++)
    {
        s265_sei_payload *payload = &m_frame->m_userSEI.payloads[i];
        if (payload->payloadType == USER_DATA_UNREGISTERED)
        {
            SEIuserDataUnregistered sei;
            sei.m_userData = payload->payload;
            sei.setSize(payload->payloadSize);
            sei.writeSEImessages(m_bs, *slice->m_sps, NAL_UNIT_PREFIX_SEI, m_nalList, m_param->bSingleSeiNal);
        }
        else if (payload->payloadType == USER_DATA_REGISTERED_ITU_T_T35)
        {
            bool writeSei = m_param->bDhdr10opt ? writeToneMapInfo(payload) : true;
            if (writeSei)
            {
                SEIuserDataRegistered sei;
                sei.m_userData = payload->payload;
                sei.setSize(payload->payloadSize);
                sei.writeSEImessages(m_bs, *slice->m_sps, NAL_UNIT_PREFIX_SEI, m_nalList, m_param->bSingleSeiNal);
            }
        }
        else
            s265_log(m_param, S265_LOG_ERROR, "Unrecognized SEI type\n");
    }

    bool isSei = ((m_frame->m_lowres.bKeyframe && m_param->bRepeatHeaders) || m_param->bEmitHRDSEI ||
                  (m_frame->m_lowres.sliceType == S265_TYPE_IDR && m_param->bEmitIDRRecoverySEI) ||
                   m_frame->m_userSEI.numPayloads);

    if (isSei && m_param->bSingleSeiNal)
    {
        m_bs.writeByteAlignment();
        m_nalList.serialize(NAL_UNIT_PREFIX_SEI, m_bs);
    }
    /* CQP and CRF (without capped VBV) doesn't use mid-frame statistics to 
     * tune RateControl parameters for other frames.
     * Hence, for these modes, update m_startEndOrder and unlock RC for previous threads waiting in
     * RateControlEnd here, after the slice contexts are initialized. For the rest - ABR
     * and VBV, unlock only after rateControlUpdateStats of this frame is called */
    if (m_param->rc.rateControlMode != S265_RC_ABR && !m_top->m_rateControl->m_isVbv)
    {// 这里是非码空情况下，不需要统计 bits,在这里提前 自增m_startEndOrder，以让之前的线程在执行RateControlEnd 不需要等待
        m_top->m_rateControl->m_startEndOrder.incr();// 非码空 update+1 by frameEncoder threads 
        // 当前帧的编码序号不足一个 frameNumThreads 循环，
        if (m_rce.encodeOrder < m_param->frameNumThreads - 1)
            m_top->m_rateControl->m_startEndOrder.incr(); // 非码空提前 end+1 by frameEncoder threads  faked rateControlEnd calls for negative frames
    }

    if (m_param->bDynamicRefine)
        computeAvgTrainingData();

    /* Analyze CTU rows, most of the hard work is done here.  Frame is
     * compressed in a wave-front pattern if WPP is enabled. Row based loop
     * filters runs behind the CTU compression and reconstruction */

    for (uint32_t sliceId = 0; sliceId < m_param->maxSlices; sliceId++)    
        m_rows[m_sliceBaseRow[sliceId]].active = true;
    
    if (m_param->bEnableWavefront)
    {
        int i = 0;
        //多个slice时，每个slice有多少个ctu_row,
        for (uint32_t rowInSlice = 0; rowInSlice < m_sliceGroupSize; rowInSlice++)
        {
            //每个slice的同编号row对应到不同slice时
            for (uint32_t sliceId = 0; sliceId < m_param->maxSlices; sliceId++)
            {
                const uint32_t sliceStartRow = m_sliceBaseRow[sliceId];
                const uint32_t sliceEndRow = m_sliceBaseRow[sliceId + 1] - 1;
                const uint32_t row = sliceStartRow + rowInSlice;
                if (row > sliceEndRow)// 超出图片底部了
                    continue;// 注意只能 continue 不能用break, 会漏掉一些行
                m_row_to_idx[row] = i;// 每一行使用一个rowprocess任务 记录任务id
                m_idx_to_row[i] = row;// wpp下每个任务负责的row 行号
                i += 1;
            }
        }
    }

    if (m_param->bEnableWavefront)// wpp 下
    {   //m_sliceGroupSize: 均分下来每个slice有多少行ctu
        for (uint32_t rowInSlice = 0; rowInSlice < m_sliceGroupSize; rowInSlice++)
        {
            for (uint32_t sliceId = 0; sliceId < m_param->maxSlices; sliceId++)
            {
            // 这里启动行级编码器的顺序为:第（1...到最后一个）slice 的第一行,先启动，然后再第（1..到最后一个）slice的第二行启动，依次类推
                const uint32_t sliceStartRow = m_sliceBaseRow[sliceId];
                const uint32_t sliceEndRow = m_sliceBaseRow[sliceId + 1] - 1;
                const uint32_t row = sliceStartRow + rowInSlice;

                S265_CHECK(row < m_numRows, "slices row fault was detected");

                if (row > sliceEndRow)
                    continue; // 注意只能 continue 不能用break, 会漏掉一些行

                // block until all reference frames have reconstructed the rows we need
                for (int l = 0; l < numPredDir; l++)//如果B帧，有两个方向的reference P帧:1个 I帧: 0个
                {
                    for (int ref = 0; ref < slice->m_numRefIdx[l]; ref++)
                    {
                        Frame *refpic = slice->m_refFrameList[l][ref];

                        // NOTE: we unnecessary wait row that beyond current slice boundary
                        //m_refLagRows: fpp 帧级并行时 参考帧依赖
                        const int rowIdx = S265_MIN(sliceEndRow, (row + m_refLagRows));
                        //等待依赖的参考帧部分完成重建
                        while (refpic->m_reconRowFlag[rowIdx].get() == 0)//==0:表示还没有完成，==1:表示重建完成
                            refpic->m_reconRowFlag[rowIdx].waitForChange(0);// 0-->1 重建完成

                        if ((bUseWeightP || bUseWeightB) && m_mref[l][ref].isWeighted)
                            m_mref[l][ref].applyWeight(rowIdx, m_numRows, sliceEndRow, sliceId);
                    }
                }
                // 清除外部参考依赖bit
                enableRowEncoder(m_row_to_idx[row]); /* clear external dependency for this row */
                if (!rowInSlice)//对于每个slice 的首行CTU,
                {
                    m_row0WaitTime = s265_mdate();
                    // 清除内部依赖bit
                    enqueueRowEncoder(m_row_to_idx[row]); /* clear internal dependency, start wavefront */
                }
                // framencoder 通过继承 wavefronts 又进一步继承了 jobprovider 
                // wpp下 入口1:
                tryWakeOne(); //-->WaveFront::findJob 唤醒一个线程 去找对应的jobprovider 取出一个任务执行 此处（执行 FrameEncoder 的processRow 里面的 encode 任务)
            } // end of loop rowInSlice
        } // end of loop sliceId

        m_allRowsAvailableTime = s265_mdate();
        tryWakeOne(); /* ensure one thread is active or help-wanted flag is set prior to blocking */
        static const int block_ms = 250;
        while (m_completionEvent.timedWait(block_ms))
            tryWakeOne();
    }
    else //非wpp 下
    {
        for (uint32_t i = 0; i < m_numRows + m_filterRowDelay; i++)
        {
            // compress
            if (i < m_numRows)
            {
                // block until all reference frames have reconstructed the rows we need
                for (int l = 0; l < numPredDir; l++)
                {
                    int list = l;
                    for (int ref = 0; ref < slice->m_numRefIdx[list]; ref++)
                    {
                        Frame *refpic = slice->m_refFrameList[list][ref];

                        const int rowIdx = S265_MIN(m_numRows - 1, (i + m_refLagRows));
                        while (refpic->m_reconRowFlag[rowIdx].get() == 0)
                            refpic->m_reconRowFlag[rowIdx].waitForChange(0);

                        if ((bUseWeightP || bUseWeightB) && m_mref[l][ref].isWeighted)
                            m_mref[list][ref].applyWeight(rowIdx, m_numRows, m_numRows, 0);
                    }
                }

                if (!i)
                    m_row0WaitTime = s265_mdate();
                else if (i == m_numRows - 1)
                    m_allRowsAvailableTime = s265_mdate();
                // 非wpp下,由线程自己直接调用执行一行编码任务
                processRowEncoder(i, m_tld[m_localTldIdx]);
            }

            // filter
            if (i >= m_filterRowDelay)
                // 同样，非wpp下,由线程自己直接调用执行一个ctu行的filter任务
                m_frameFilter.processRow(i - m_filterRowDelay);
        }
    }
#if ENABLE_LIBVMAF
    vmafFrameLevelScore();
#endif

    if (m_param->maxSlices > 1)
    {
        PicYuv *reconPic = m_frame->m_reconPic;
        uint32_t height = reconPic->m_picHeight;
        initDecodedPictureHashSEI(0, 0, height);
    } 

    if (m_param->bDynamicRefine && m_top->m_startPoint <= m_frame->m_encodeOrder) //Avoid collecting data that will not be used by future frames.
        collectDynDataFrame();

    if (m_param->csvLogLevel >= 1)
    {
        for (uint32_t i = 0; i < m_numRows; i++)
        {
            m_frame->m_encData->m_frameStats.cntIntraNxN += m_rows[i].rowStats.cntIntraNxN;
            m_frame->m_encData->m_frameStats.totalCu += m_rows[i].rowStats.totalCu;
            m_frame->m_encData->m_frameStats.totalCtu += m_rows[i].rowStats.totalCtu;
            m_frame->m_encData->m_frameStats.lumaDistortion += m_rows[i].rowStats.lumaDistortion;
            m_frame->m_encData->m_frameStats.chromaDistortion += m_rows[i].rowStats.chromaDistortion;
            m_frame->m_encData->m_frameStats.psyEnergy += m_rows[i].rowStats.psyEnergy;
            m_frame->m_encData->m_frameStats.ssimEnergy += m_rows[i].rowStats.ssimEnergy;
            m_frame->m_encData->m_frameStats.resEnergy += m_rows[i].rowStats.resEnergy;
            for (uint32_t depth = 0; depth <= m_param->maxCUDepth; depth++)
            {
                m_frame->m_encData->m_frameStats.cntSkipCu[depth] += m_rows[i].rowStats.cntSkipCu[depth];
                m_frame->m_encData->m_frameStats.cntMergeCu[depth] += m_rows[i].rowStats.cntMergeCu[depth];
                for (int m = 0; m < INTER_MODES; m++)
                    m_frame->m_encData->m_frameStats.cuInterDistribution[depth][m] += m_rows[i].rowStats.cuInterDistribution[depth][m];
                for (int n = 0; n < INTRA_MODES; n++)
                    m_frame->m_encData->m_frameStats.cuIntraDistribution[depth][n] += m_rows[i].rowStats.cuIntraDistribution[depth][n];
            }
        }
        m_frame->m_encData->m_frameStats.percentIntraNxN = (double)(m_frame->m_encData->m_frameStats.cntIntraNxN * 100) / m_frame->m_encData->m_frameStats.totalCu;

        for (uint32_t depth = 0; depth <= m_param->maxCUDepth; depth++)
        {
            m_frame->m_encData->m_frameStats.percentSkipCu[depth] = (double)(m_frame->m_encData->m_frameStats.cntSkipCu[depth] * 100) / m_frame->m_encData->m_frameStats.totalCu;
            m_frame->m_encData->m_frameStats.percentMergeCu[depth] = (double)(m_frame->m_encData->m_frameStats.cntMergeCu[depth] * 100) / m_frame->m_encData->m_frameStats.totalCu;
            for (int n = 0; n < INTRA_MODES; n++)
                m_frame->m_encData->m_frameStats.percentIntraDistribution[depth][n] = (double)(m_frame->m_encData->m_frameStats.cuIntraDistribution[depth][n] * 100) / m_frame->m_encData->m_frameStats.totalCu;
            uint64_t cuInterRectCnt = 0; // sum of Nx2N, 2NxN counts
            cuInterRectCnt += m_frame->m_encData->m_frameStats.cuInterDistribution[depth][1] + m_frame->m_encData->m_frameStats.cuInterDistribution[depth][2];
            m_frame->m_encData->m_frameStats.percentInterDistribution[depth][0] = (double)(m_frame->m_encData->m_frameStats.cuInterDistribution[depth][0] * 100) / m_frame->m_encData->m_frameStats.totalCu;
            m_frame->m_encData->m_frameStats.percentInterDistribution[depth][1] = (double)(cuInterRectCnt * 100) / m_frame->m_encData->m_frameStats.totalCu;
            m_frame->m_encData->m_frameStats.percentInterDistribution[depth][2] = (double)(m_frame->m_encData->m_frameStats.cuInterDistribution[depth][3] * 100) / m_frame->m_encData->m_frameStats.totalCu;
        }
    }

    if (m_param->csvLogLevel >= 2)
    {
        m_frame->m_encData->m_frameStats.avgLumaDistortion = (double)(m_frame->m_encData->m_frameStats.lumaDistortion) / m_frame->m_encData->m_frameStats.totalCtu;
        m_frame->m_encData->m_frameStats.avgChromaDistortion = (double)(m_frame->m_encData->m_frameStats.chromaDistortion) / m_frame->m_encData->m_frameStats.totalCtu;
        m_frame->m_encData->m_frameStats.avgPsyEnergy = (double)(m_frame->m_encData->m_frameStats.psyEnergy) / m_frame->m_encData->m_frameStats.totalCtu;
        m_frame->m_encData->m_frameStats.avgSsimEnergy = (double)(m_frame->m_encData->m_frameStats.ssimEnergy) / m_frame->m_encData->m_frameStats.totalCtu;
        m_frame->m_encData->m_frameStats.avgResEnergy = (double)(m_frame->m_encData->m_frameStats.resEnergy) / m_frame->m_encData->m_frameStats.totalCtu;
    }

    m_bs.resetBits();
    m_entropyCoder.load(m_initSliceContext);
    m_entropyCoder.setBitstream(&m_bs);

    // finish encode of each CTU row, only required when SAO is enabled
    if (slice->m_bUseSao)
        encodeSlice(0);

    m_entropyCoder.setBitstream(&m_bs);

    if (m_param->maxSlices > 1)
    {
        uint32_t nextSliceRow = 0;

        for(uint32_t sliceId = 0; sliceId < m_param->maxSlices; sliceId++)
        {
            m_bs.resetBits();

            const uint32_t sliceAddr = nextSliceRow * m_numCols;
            if (m_param->bOptRefListLengthPPS)
            {
                ScopedLock refIdxLock(m_top->m_sliceRefIdxLock);
                m_top->analyseRefIdx(slice->m_numRefIdx);
            }
            m_entropyCoder.codeSliceHeader(*slice, *m_frame->m_encData, sliceAddr, m_sliceAddrBits, slice->m_sliceQp);

            // Find rows of current slice
            const uint32_t prevSliceRow = nextSliceRow;
            while(nextSliceRow < m_numRows && m_rows[nextSliceRow].sliceId == sliceId)
                nextSliceRow++;

            // serialize each row, record final lengths in slice header
            uint32_t maxStreamSize = m_nalList.serializeSubstreams(&m_substreamSizes[prevSliceRow], (nextSliceRow - prevSliceRow), &m_outStreams[prevSliceRow]);

            // complete the slice header by writing WPP row-starts
            m_entropyCoder.setBitstream(&m_bs);
            if (slice->m_pps->bEntropyCodingSyncEnabled)
                m_entropyCoder.codeSliceHeaderWPPEntryPoints(&m_substreamSizes[prevSliceRow], (nextSliceRow - prevSliceRow - 1), maxStreamSize);
            
            m_bs.writeByteAlignment();

            m_nalList.serialize(slice->m_nalUnitType, m_bs);
        }
    }
    else
    {
        if (m_param->bOptRefListLengthPPS)
        {
            ScopedLock refIdxLock(m_top->m_sliceRefIdxLock);
            m_top->analyseRefIdx(slice->m_numRefIdx);
        }
        m_entropyCoder.codeSliceHeader(*slice, *m_frame->m_encData, 0, 0, slice->m_sliceQp);

        // serialize each row, record final lengths in slice header
        uint32_t maxStreamSize = m_nalList.serializeSubstreams(m_substreamSizes, numSubstreams, m_outStreams);

        // complete the slice header by writing WPP row-starts
        m_entropyCoder.setBitstream(&m_bs);
        if (slice->m_pps->bEntropyCodingSyncEnabled)
            m_entropyCoder.codeSliceHeaderWPPEntryPoints(m_substreamSizes, (slice->m_sps->numCuInHeight - 1), maxStreamSize);
        m_bs.writeByteAlignment();

        m_nalList.serialize(slice->m_nalUnitType, m_bs);
    }

    if (m_param->decodedPictureHashSEI)
        writeTrailingSEIMessages();

    uint64_t bytes = 0;
    for (uint32_t i = 0; i < m_nalList.m_numNal; i++)
    {
        int type = m_nalList.m_nal[i].type;

        // exclude SEI
        if (type != NAL_UNIT_PREFIX_SEI && type != NAL_UNIT_SUFFIX_SEI)
        {
            bytes += m_nalList.m_nal[i].sizeBytes;
            // and exclude start code prefix
            //减去起始码，第一个nalunit ,sps_nalunit,pps_nalunit 使用4字节起始码，其他使用3字节起始码
            bytes -= (!i || type == NAL_UNIT_SPS || type == NAL_UNIT_PPS) ? 4 : 3;
        }
    }
    m_accessUnitBits = bytes << 3;

    int filler = 0;
    /* rateControlEnd may also block for earlier frames to call rateControlUpdateStats */
    if (m_top->m_rateControl->rateControlEnd(m_frame, m_accessUnitBits, &m_rce, &filler) < 0)
        m_top->m_aborted = true;

    if (filler > 0)
    {
        filler = (filler - FILLER_OVERHEAD * 8) >> 3;
        m_bs.resetBits();
        while (filler > 0)
        {
            m_bs.write(0xff, 8);
            filler--;
        }
        m_bs.writeByteAlignment();
        m_nalList.serialize(NAL_UNIT_FILLER_DATA, m_bs);
        bytes += m_nalList.m_nal[m_nalList.m_numNal - 1].sizeBytes;
        bytes -= 3; //exclude start code prefix
        m_accessUnitBits = bytes << 3;
    }

    if (m_frame->m_rpu.payloadSize)
    {
        m_bs.resetBits();
        for (int i = 0; i < m_frame->m_rpu.payloadSize; i++)
            m_bs.write(m_frame->m_rpu.payload[i], 8);
        m_nalList.serialize(NAL_UNIT_UNSPECIFIED, m_bs);
    }

    m_endCompressTime = s265_mdate();

    /* Decrement referenced frame reference counts, allow them to be recycled */
    for (int l = 0; l < numPredDir; l++)
    {
        for (int ref = 0; ref < slice->m_numRefIdx[l]; ref++)
        {
            Frame *refpic = slice->m_refFrameList[l][ref];
            ATOMIC_DEC(&refpic->m_countRefEncoders);
        }
    }

    if (m_nr)
    {
        bool nrEnabled = (m_rce.newQp < QP_MAX_SPEC || !m_param->rc.vbvBufferSize) && (m_param->noiseReductionIntra || m_param->noiseReductionInter);

        if (nrEnabled)
        {
            /* Accumulate NR statistics from all worker threads */
            for (int i = 0; i < numTLD; i++)
            {
                NoiseReduction* nr = &m_tld[i].analysis.m_quant.m_frameNr[m_jpId];
                for (int cat = 0; cat < MAX_NUM_TR_CATEGORIES; cat++)
                {
                    for (int coeff = 0; coeff < MAX_NUM_TR_COEFFS; coeff++)
                        m_nr->nrResidualSum[cat][coeff] += nr->nrResidualSum[cat][coeff];

                    m_nr->nrCount[cat] += nr->nrCount[cat];
                }
            }

            noiseReductionUpdate();

            /* Copy updated NR coefficients back to all worker threads */
            for (int i = 0; i < numTLD; i++)
            {
                NoiseReduction* nr = &m_tld[i].analysis.m_quant.m_frameNr[m_jpId];
                memcpy(nr->nrOffsetDenoise, m_nr->nrOffsetDenoise, sizeof(uint16_t)* MAX_NUM_TR_CATEGORIES * MAX_NUM_TR_COEFFS);
                memset(nr->nrCount, 0, sizeof(uint32_t)* MAX_NUM_TR_CATEGORIES);
                memset(nr->nrResidualSum, 0, sizeof(uint32_t)* MAX_NUM_TR_CATEGORIES * MAX_NUM_TR_COEFFS);
            }
        }
    }

#if DETAILED_CU_STATS
    /* Accumulate CU statistics from each worker thread, we could report
     * per-frame stats here, but currently we do not. */
    for (int i = 0; i < numTLD; i++)
        m_cuStats.accumulate(m_tld[i].analysis.m_stats[m_jpId], *m_param);
#endif

    m_endFrameTime = s265_mdate();  
}

void FrameEncoder::initDecodedPictureHashSEI(int row, int cuAddr, int height)
{
    PicYuv *reconPic = m_frame->m_reconPic;
    uint32_t width = reconPic->m_picWidth;	
    intptr_t stride = reconPic->m_stride;
    uint32_t maxCUHeight = m_param->maxCUSize;

    const uint32_t hChromaShift = CHROMA_H_SHIFT(m_param->internalCsp);
    const uint32_t vChromaShift = CHROMA_V_SHIFT(m_param->internalCsp);

    if (m_param->decodedPictureHashSEI == 1)
    {
        if (!row)
            MD5Init(&m_seiReconPictureDigest.m_state[0]);

        updateMD5Plane(m_seiReconPictureDigest.m_state[0], reconPic->getLumaAddr(cuAddr), width, height, stride);
        if (m_param->internalCsp != S265_CSP_I400)
        {
            if (!row)
            {
                MD5Init(&m_seiReconPictureDigest.m_state[1]);
                MD5Init(&m_seiReconPictureDigest.m_state[2]);
            }

            width >>= hChromaShift;
            height >>= vChromaShift;
            stride = reconPic->m_strideC;

            updateMD5Plane(m_seiReconPictureDigest.m_state[1], reconPic->getCbAddr(cuAddr), width, height, stride);
            updateMD5Plane(m_seiReconPictureDigest.m_state[2], reconPic->getCrAddr(cuAddr), width, height, stride);
        }
    }
    else if (m_param->decodedPictureHashSEI == 2)
    {

        if (!row)
            m_seiReconPictureDigest.m_crc[0] = 0xffff;

        updateCRC(reconPic->getLumaAddr(cuAddr), m_seiReconPictureDigest.m_crc[0], height, width, stride);
        if (m_param->internalCsp != S265_CSP_I400)
        {
            width >>= hChromaShift;
            height >>= vChromaShift;
            stride = reconPic->m_strideC;
            m_seiReconPictureDigest.m_crc[1] = m_seiReconPictureDigest.m_crc[2] = 0xffff;

            updateCRC(reconPic->getCbAddr(cuAddr), m_seiReconPictureDigest.m_crc[1], height, width, stride);
            updateCRC(reconPic->getCrAddr(cuAddr), m_seiReconPictureDigest.m_crc[2], height, width, stride);
        }
    }
    else if (m_param->decodedPictureHashSEI == 3)
    {
        if (!row)
            m_seiReconPictureDigest.m_checksum[0] = 0;

        updateChecksum(reconPic->m_picOrg[0], m_seiReconPictureDigest.m_checksum[0], height, width, stride, row, maxCUHeight);
        if (m_param->internalCsp != S265_CSP_I400)
        {
            width >>= hChromaShift;
            height >>= vChromaShift;
            stride = reconPic->m_strideC;
            maxCUHeight >>= vChromaShift;

            if (!row)
                m_seiReconPictureDigest.m_checksum[1] = m_seiReconPictureDigest.m_checksum[2] = 0;

            updateChecksum(reconPic->m_picOrg[1], m_seiReconPictureDigest.m_checksum[1], height, width, stride, row, maxCUHeight);
            updateChecksum(reconPic->m_picOrg[2], m_seiReconPictureDigest.m_checksum[2], height, width, stride, row, maxCUHeight);
        }
    }
}

void FrameEncoder::encodeSlice(uint32_t sliceAddr)
{
    Slice* slice = m_frame->m_encData->m_slice;
    const uint32_t widthInLCUs = slice->m_sps->numCuInWidth;
    const uint32_t lastCUAddr = (slice->m_endCUAddr + m_param->num4x4Partitions - 1) / m_param->num4x4Partitions;
    const uint32_t numSubstreams = m_param->bEnableWavefront ? slice->m_sps->numCuInHeight : 1;

    SAOParam* saoParam = slice->m_sps->bUseSAO && slice->m_bUseSao ? m_frame->m_encData->m_saoParam : NULL;
    for (uint32_t cuAddr = sliceAddr; cuAddr < lastCUAddr; cuAddr++)
    {
        uint32_t col = cuAddr % widthInLCUs;
        uint32_t row = cuAddr / widthInLCUs;
        uint32_t subStrm = row % numSubstreams;
        CUData* ctu = m_frame->m_encData->getPicCTU(cuAddr);

        m_entropyCoder.setBitstream(&m_outStreams[subStrm]);

        // Synchronize cabac probabilities with upper-right CTU if it's available and we're at the start of a line.
        if (m_param->bEnableWavefront && !col && row)
        {
            m_entropyCoder.copyState(m_initSliceContext);
            m_entropyCoder.loadContexts(m_rows[row - 1].bufferedEntropy);
        }

        // Initialize slice context
        if (ctu->m_bFirstRowInSlice && !col)
            m_entropyCoder.load(m_initSliceContext);

        if (saoParam)
        {
            if (saoParam->bSaoFlag[0] || saoParam->bSaoFlag[1])
            {
                int mergeLeft = col && saoParam->ctuParam[0][cuAddr].mergeMode == SAO_MERGE_LEFT;
                int mergeUp = !ctu->m_bFirstRowInSlice && saoParam->ctuParam[0][cuAddr].mergeMode == SAO_MERGE_UP;
                if (col)
                    m_entropyCoder.codeSaoMerge(mergeLeft);
                if (!ctu->m_bFirstRowInSlice && !mergeLeft)
                    m_entropyCoder.codeSaoMerge(mergeUp);
                if (!mergeLeft && !mergeUp)
                {
                    if (saoParam->bSaoFlag[0])
                        m_entropyCoder.codeSaoOffset(saoParam->ctuParam[0][cuAddr], 0);
                    if (saoParam->bSaoFlag[1])
                    {
                        m_entropyCoder.codeSaoOffset(saoParam->ctuParam[1][cuAddr], 1);
                        m_entropyCoder.codeSaoOffset(saoParam->ctuParam[2][cuAddr], 2);
                    }
                }
            }
            else
            {
                for (int i = 0; i < (m_param->internalCsp != S265_CSP_I400 ? 3 : 1); i++)
                    saoParam->ctuParam[i][cuAddr].reset();
            }
        }

        // final coding (bitstream generation) for this CU
        m_entropyCoder.encodeCTU(*ctu, m_cuGeoms[m_ctuGeomMap[cuAddr]]);

        if (m_param->bEnableWavefront)
        {
            if (col == 1)
                // Store probabilities of second CTU in line into buffer
                m_rows[row].bufferedEntropy.loadContexts(m_entropyCoder);

            if (col == widthInLCUs - 1)
                m_entropyCoder.finishSlice();
        }
    }

    if (!m_param->bEnableWavefront)
        m_entropyCoder.finishSlice();
}
// row 更应该使用 id 表示
// 这里只有在wpp下才能走进来
void FrameEncoder::processRow(int row, int threadId)
{
    int64_t startTime = s265_mdate();
    //先加1，再返回更新后的值
    if (ATOMIC_INC(&m_activeWorkerCount) == 1 && m_stallStartTime)
        m_totalNoWorkerTime += s265_mdate() - m_stallStartTime;

    // 因为编码和滤波分开来了,row(其实表示的是id),每个row 对应有两个id的任务，如果偶数id为编码则基数id为filter,否则反之
    const uint32_t realRow = m_idx_to_row[row >> 1];
    const uint32_t typeNum = m_idx_to_row[row & 1];

    if (!typeNum)//0 表示:编码任务
        processRowEncoder(realRow, m_tld[threadId]);
    else//否则 表示:filter任务
    {
        m_frameFilter.processRow(realRow);// deblock +sao

        // NOTE: Active next row
        if (realRow != m_sliceBaseRow[m_rows[realRow].sliceId + 1] - 1)// 非slice的最后一天行ctu 做完了filter时
            enqueueRowFilter(m_row_to_idx[realRow + 1]);//提交下一行的ctu filter 任务
    }
        // 先减1，再返回更新后的值
    if (ATOMIC_DEC(&m_activeWorkerCount) == 0)
        m_stallStartTime = s265_mdate();

    m_totalWorkerElapsedTime += s265_mdate() - startTime; // not thread safe, but good enough
}

// Called by worker threads （行编码任务）
void FrameEncoder::processRowEncoder(int intRow, ThreadLocalData& tld)
{
    const uint32_t row = (uint32_t)intRow;
    CTURow& curRow = m_rows[row];

    if (m_param->bEnableWavefront)// wpp  下 需要先拿到锁
    {
        ScopedLock self(curRow.lock);// 锁 active 和busy 变量
        if (!curRow.active) //
            /* VBV restart is in progress, exit out */
            return;
        if (curRow.busy)// 改行表示已经有其他的线程正在处理，出错
        {
            /* On multi-socket Windows servers, we have seen problems with
             * ATOMIC_CAS which resulted in multiple worker threads processing
             * the same CU row, which often resulted in bad pointer accesses. We
             * believe the problem is fixed, but are leaving this check in place
             * to prevent crashes in case it is not */
            s265_log(m_param, S265_LOG_WARNING,
                     "internal error - simultaneous row access detected. Please report HW to s265-devel@videolan.org\n");
            return;
        }
        curRow.busy = true;// 接下来要进行该row的编码了，设置标志为 busy状态
    }

    /* When WPP is enabled, every row has its own row coder instance. Otherwise
     * they share row 0 */
    Entropy& rowCoder = m_param->bEnableWavefront ? curRow.rowGoOnCoder : m_rows[0].rowGoOnCoder;
    FrameData& curEncData = *m_frame->m_encData;
    Slice *slice = curEncData.m_slice;

    const uint32_t numCols = m_numCols;//一行有多少个CTU
    const uint32_t lineStartCUAddr = row * numCols;//该编码ctu行的起始CTU 的addr
    bool bIsVbv = m_param->rc.vbvBufferSize > 0 && m_param->rc.vbvMaxBitrate > 0;

    const uint32_t sliceId = curRow.sliceId;
    uint32_t maxBlockCols = (m_frame->m_fencPic->m_picWidth + (16 - 1)) / 16;// 以16为宽度的帧级_block宽度
    uint32_t noOfBlocks = m_param->maxCUSize / 16;// 对于一个ctu 有几个16的宽度
    const uint32_t bFirstRowInSlice = ((row == 0) || (m_rows[row - 1].sliceId != curRow.sliceId)) ? 1 : 0;//slice 的首行标志
    const uint32_t bLastRowInSlice = ((row == m_numRows - 1) || (m_rows[row + 1].sliceId != curRow.sliceId)) ? 1 : 0;// slice的尾行
    const uint32_t endRowInSlicePlus1 = m_sliceBaseRow[sliceId + 1];
    const uint32_t rowInSlice = row - m_sliceBaseRow[sliceId];// 该row 位于该slice的第几行

    // Load SBAC coder context from previous row and initialize row state.
    if (bFirstRowInSlice && !curRow.completed)//slice的首行 并且首个ctu 时
        rowCoder.load(m_initSliceContext);

    // calculate mean QP for consistent deltaQP signalling calculation
    //计算一个CTU行内的meanQp
    if (m_param->bOptCUDeltaQP)
    {
        ScopedLock self(curRow.lock);// // 锁 avgQPComputed 变量
        if (!curRow.avgQPComputed)//如果该行的avgQP 还没有被计算
        {
            if (m_param->bEnableWavefront || !row)// wpp下每一行都计算,非wpp下只有第0行才计算
            {
                double meanQPOff = 0;
                bool isReferenced = IS_REFERENCED(m_frame);
                //如果被参考则qp_offsets 使用 cutree和aq 共同作用后的offset, 否则使用aq作用后的offset
                double *qpoffs = (isReferenced && m_param->rc.cuTree) ? m_frame->m_lowres.qpCuTreeOffset : m_frame->m_lowres.qpAqOffset;
                if (qpoffs)// qp offset 不为空时
                {
                    uint32_t loopIncr = (m_param->rc.qgSize == 8) ? 8 : 16;// 只有在qgsize 为 8时，才按照那个8x8的unit 进行统计，否则一律按照16x16 的unit 进行统计

                    uint32_t cuYStart = 0, height = m_frame->m_fencPic->m_picHeight;// 初始化为 统计一整帧的aq_offset
                    if (m_param->bEnableWavefront)//wpp 下，每一行只统计该行ctu内的 aq_offset
                    {
                        cuYStart = intRow * m_param->maxCUSize;//row行*一个ctu的高度
                        height = cuYStart + m_param->maxCUSize;// wpp下 高度只有一个ctu行的高度
                    }

                    uint32_t qgSize = m_param->rc.qgSize, width = m_frame->m_fencPic->m_picWidth;
                    uint32_t maxOffsetCols = (m_frame->m_fencPic->m_picWidth + (loopIncr - 1)) / loopIncr;//一帧中以 aqunit 为单位的宽度
                    uint32_t count = 0;
                    for (uint32_t cuY = cuYStart; cuY < height && (cuY < m_frame->m_fencPic->m_picHeight); cuY += qgSize)//外围按照qgsize 的高度进行统计
                    {
                        for (uint32_t cuX = 0; cuX < width; cuX += qgSize)//宽度也是qgsize
                        {
                            double qp_offset = 0;
                            uint32_t cnt = 0;
                            //一个qgsize 内按照 8x8（qgsize=8时）或者 16x16的unit 进行统计
                            for (uint32_t block_yy = cuY; block_yy < cuY + qgSize && block_yy < m_frame->m_fencPic->m_picHeight; block_yy += loopIncr)
                            {
                                for (uint32_t block_xx = cuX; block_xx < cuX + qgSize && block_xx < width; block_xx += loopIncr)
                                {
                                    int idx = ((block_yy / loopIncr) * (maxOffsetCols)) + (block_xx / loopIncr);// 按照 8x8 or 16x16的单位 进行 寻址
                                    qp_offset += qpoffs[idx];// 累加qpoffset
                                    cnt++;// qpoffset的数量计数累加
                                }
                            }
                            qp_offset /= cnt;//求得一个qgsize内的平均qpoffset
                            meanQPOff += qp_offset;
                            count++;//qpsize 计数累加
                        }
                    }
                    meanQPOff /= count;//求得一个frame or 一行CTU内的平均meanQPoff
                }
                rowCoder.m_meanQP = slice->m_sliceQp + meanQPOff;// 该rowCoder 使用的meanQp
            }
            else//非wpp&&也不是第0行, 则每一行的meanqp 使用帧级统计得倒的meanqp
            {
                rowCoder.m_meanQP = m_rows[0].rowGoOnCoder.m_meanQP;
            }
            curRow.avgQPComputed = 1;
        }
    }

    // Initialize restrict on MV range in slices
    // slice mv 的 边界 （这里 应该是 slice 只能参考参考帧中的同一个slice区域？？？？）
    tld.analysis.m_sliceMinY = -(int32_t)(rowInSlice * m_param->maxCUSize * 4) + 3 * 4;
    tld.analysis.m_sliceMaxY = (int32_t)((endRowInSlicePlus1 - 1 - row) * (m_param->maxCUSize * 4) - 4 * 4);

    // Handle single row slice
    if (tld.analysis.m_sliceMaxY < tld.analysis.m_sliceMinY)
        tld.analysis.m_sliceMaxY = tld.analysis.m_sliceMinY = 0;// 当slice只有一个CTU row的时候


    while (curRow.completed < numCols)//一行中 ctu 还没有编码完成
    {
        ProfileScopeEvent(encodeCTU);

        const uint32_t col = curRow.completed;// 列地址
        const uint32_t cuAddr = lineStartCUAddr + col;//该row 行ctu的起始地址+列地址
        CUData* ctu = curEncData.getPicCTU(cuAddr);
        const uint32_t bLastCuInSlice = (bLastRowInSlice & (col == numCols - 1)) ? 1 : 0;// slice 最后一行同时最后一列
        // ctu的一些周边信息的初始化
        ctu->initCTU(*m_frame, cuAddr, slice->m_sliceQp, bFirstRowInSlice, bLastRowInSlice, bLastCuInSlice);

        if (bIsVbv)//如果开启了vbv
        {
            if (col == 0 && !m_param->bEnableWavefront)// 非wpp且 是一行row的首列时
            {   // backup 一行的初始状态，谨防需要重新编码
                m_backupStreams[0].copyBits(&m_outStreams[0]);
                curRow.bufferedEntropy.copyState(rowCoder);
                curRow.bufferedEntropy.loadContexts(rowCoder);
            }
            if (bFirstRowInSlice && m_vbvResetTriggerRow[curRow.sliceId] != intRow)
            {   //该slice的首行并且该row 所在slice没有被标记为从首行开始重新编码,取该首行row的qp为m_avgQpRc
                curEncData.m_rowStat[row].rowQp = curEncData.m_avgQpRc;
                curEncData.m_rowStat[row].rowQpScale = s265_qp2qScale(curEncData.m_avgQpRc);//同时设置qpscale
            }

            FrameData::RCStatCU& cuStat = curEncData.m_cuStat[cuAddr];
            if (m_param->bEnableWavefront && rowInSlice >= col && !bFirstRowInSlice && m_vbvResetTriggerRow[curRow.sliceId] != intRow)
                //wpp下，在非slice首行ctu行的斜对角线上以及右下边部分的ctu上，如果该row 所在slice没有被标记为从该row行开始重新编码
                cuStat.baseQp = curEncData.m_cuStat[cuAddr - numCols + 1].baseQp;//取右上角ctu的baseQp作为当前ctu的初始baseqp
            else if (!m_param->bEnableWavefront && !bFirstRowInSlice && m_vbvResetTriggerRow[curRow.sliceId] != intRow)
                //非wpp下，非slice首行ctu且该row 所在slice没有被标记为从该row行开始重新编码
                cuStat.baseQp = curEncData.m_rowStat[row - 1].rowQp;// 取上一行的rowQp作为当前ctu的初始baseqp
            else
                cuStat.baseQp = curEncData.m_rowStat[row].rowQp;// 否则使用当前row的rowQp作为当前ctu的初始baseqp

            /* TODO: use defines from slicetype.h for lowres block size */
            uint32_t block_y = (ctu->m_cuPelY >> m_param->maxLog2CUSize) * noOfBlocks;//ctu的pix坐标y转换为16x16为单位的坐标
            uint32_t block_x = (ctu->m_cuPelX >> m_param->maxLog2CUSize) * noOfBlocks;//ctu的pix坐标x转换为16x16为单位的坐标
            cuStat.vbvCost = 0;
            cuStat.intraVbvCost = 0;
            // 计算当前ctu 所覆盖的16x16的block区域的 cost 和intracost
            for (uint32_t h = 0; h < noOfBlocks && block_y < m_sliceMaxBlockRow[sliceId + 1]; h++, block_y++)
            {
                uint32_t idx = block_x + (block_y * maxBlockCols);

                for (uint32_t w = 0; w < noOfBlocks && (block_x + w) < maxBlockCols; w++, idx++)
                {
                    cuStat.vbvCost += m_frame->m_lowres.lowresCostForRc[idx] & LOWRES_COST_MASK;
                    cuStat.intraVbvCost += m_frame->m_lowres.intraCost[idx];
                }
            }
        }
        else//如果没有开启vbv,则当前ctu使用的baseQp 使用帧级m_avgQpRc为初始值
            curEncData.m_cuStat[cuAddr].baseQp = curEncData.m_avgQpRc;

        if (m_param->bEnableWavefront && !col && !bFirstRowInSlice)
        {//wpp下，对于非首行ctu的首列ctu，使用上一行中buffered商编码状态作为当前row的初始初始状态
            // Load SBAC coder context from previous row and initialize row state.
            rowCoder.copyState(m_initSliceContext);
            rowCoder.loadContexts(m_rows[row - 1].bufferedEntropy);
        }
        // ？？？dynamicRd 先不看
        if (m_param->dynamicRd && (int32_t)(m_rce.qpaRc - m_rce.qpNoVbv) > 0)
            ctu->m_vbvAffected = true;

        // Does all the CU analysis, returns best top level mode decision
        // 主要的分析函数入口
        Mode& best = tld.analysis.compressCTU(*ctu, *m_frame, m_cuGeoms[m_ctuGeomMap[cuAddr]], rowCoder);

        /* startPoint > encodeOrder is true when the start point changes for
        a new GOP but few frames from the previous GOP is still incomplete.
        The data of frames in this interval will not be used by any future frames. */
        // ？？？
        if (m_param->bDynamicRefine && m_top->m_startPoint <= m_frame->m_encodeOrder)
            collectDynDataRow(*ctu, &curRow.rowStats);

        // take a sample of the current active worker count
        ATOMIC_ADD(&m_totalActiveWorkerCount, m_activeWorkerCount);
        ATOMIC_INC(&m_activeWorkerCountSamples);

        /* advance top-level row coder to include the context of this CTU.
         * if SAO is disabled, rowCoder writes the final CTU bitstream */
        rowCoder.encodeCTU(*ctu, m_cuGeoms[m_ctuGeomMap[cuAddr]]);//entropy and bitstream output

        if (m_param->bEnableWavefront && col == 1)
            // Save CABAC state for next row
            // col = 1 时，需要保存entropycontext 状态 同步给下一行row作为其初始状态
            curRow.bufferedEntropy.loadContexts(rowCoder);

        /* SAO parameter estimation using non-deblocked pixels for CTU bottom and right boundary areas */
        if (slice->m_bUseSao && m_param->bSaoNonDeblocked)
            m_frameFilter.m_parallelFilter[row].m_sao.calcSaoStatsCu_BeforeDblk(m_frame, col, row);

        /* Deblock with idle threading */
        if (m_param->bEnableLoopFilter | slice->m_bUseSao)
        {
            // NOTE: in VBV mode, we may reencode anytime, so we can't do Deblock stage-Horizon and SAO
            //在vbv的时候，可能需要重新编码某些行，所以
            if (!bIsVbv)
            {// 非vbv 下 不会有重新编码，可以让deblock and sao 进行工作了
                // Delay one row to avoid intra prediction conflict
                if (m_pool && !bFirstRowInSlice)
                {// 线程池 并且非slice首行时
                    int allowCol = col;

                    // avoid race condition on last column
                    if (rowInSlice >= 2)//从slice的第2行开始(从0开始算法)
                    {   // 如果改ctu 是row的最后一列，则允许上一行从其更上一行的已经完成了所有deblock的ctu相同位置的ctu开始进行deblock
                        // 否则不是最后一列，则允许上一行从其更上一行中准备处理的ctu的相同位置ctu进行滤波处理
                        allowCol = S265_MIN(((col == numCols - 1) ? m_frameFilter.m_parallelFilter[row - 2].m_lastDeblocked.get()
                                                                  : m_frameFilter.m_parallelFilter[row - 2].m_lastCol.get()), (int)col);
                    }
                    //设置当前行的上一行所允许的滤波col 位置
                    m_frameFilter.m_parallelFilter[row - 1].m_allowedCol.set(allowCol);
                }

                // Last Row may start early
                if (m_pool && bLastRowInSlice)
                {//开启了线程池，且当前行为slice的最后一行
                    // Deblocking last row
                    int allowCol = col;

                    // avoid race condition on last column
                    if (rowInSlice >= 2)
                    {   // 如果该ctu 是该最后一行row的最后一列，则允许本身行从其上一行已经完成了所有deblock的ctu相同位置的ctu开始进行deblock
                        // 否则不是最后一列，则允许当前行从上一行中准备处理的ctu的相同位置ctu进行滤波处理
                        allowCol = S265_MIN(((col == numCols - 1) ? m_frameFilter.m_parallelFilter[row - 1].m_lastDeblocked.get()
                                                                  : m_frameFilter.m_parallelFilter[row - 1].m_lastCol.get()), (int)col);
                    }
                    //设置当前行的所允许的滤波col 位置
                    m_frameFilter.m_parallelFilter[row].m_allowedCol.set(allowCol);
                }
            } // end of !bIsVbv
        }
        // Both Loopfilter and SAO Disabled
        else
        {
            m_frameFilter.m_parallelFilter[row].processPostCu(col);
        }

        // Completed CU processing
        curRow.completed++;//已完成编码的ctu数量++ （注意不需要管 deblock 和sao)

        // 统计
        FrameStats frameLog;
        curEncData.m_rowStat[row].sumQpAq += collectCTUStatistics(*ctu, &frameLog);

        curRow.rowStats.totalCtu++;
        curRow.rowStats.lumaDistortion   += best.lumaDistortion;
        curRow.rowStats.chromaDistortion += best.chromaDistortion;
        curRow.rowStats.psyEnergy        += best.psyEnergy;
        curRow.rowStats.ssimEnergy       += best.ssimEnergy;
        curRow.rowStats.resEnergy        += best.resEnergy;
        curRow.rowStats.cntIntraNxN      += frameLog.cntIntraNxN;
        curRow.rowStats.totalCu          += frameLog.totalCu;
        for (uint32_t depth = 0; depth <= m_param->maxCUDepth; depth++)
        {
            curRow.rowStats.cntSkipCu[depth] += frameLog.cntSkipCu[depth];
            curRow.rowStats.cntMergeCu[depth] += frameLog.cntMergeCu[depth];
            for (int m = 0; m < INTER_MODES; m++)
                curRow.rowStats.cuInterDistribution[depth][m] += frameLog.cuInterDistribution[depth][m];
            for (int n = 0; n < INTRA_MODES; n++)
                curRow.rowStats.cuIntraDistribution[depth][n] += frameLog.cuIntraDistribution[depth][n];
        }

        curEncData.m_cuStat[cuAddr].totalBits = best.totalBits;
        s265_emms();

        if (bIsVbv)
        {   
            // Update encoded bits, satdCost, baseQP for each CU if tune grain is disabled
            FrameData::RCStatCU& cuStat = curEncData.m_cuStat[cuAddr];    
            if ((m_param->bEnableWavefront && ((cuAddr == m_sliceBaseRow[sliceId] * numCols) || !m_param->rc.bEnableConstVbv)) || !m_param->bEnableWavefront)
            {  //非wpp下 或者wpp下的(非constvbv||该ctu为当前slice的首个ctu时
                curEncData.m_rowStat[row].rowSatd += cuStat.vbvCost;
                curEncData.m_rowStat[row].rowIntraSatd += cuStat.intraVbvCost;
                curEncData.m_rowStat[row].encodedBits += cuStat.totalBits;
                curEncData.m_rowStat[row].sumQpRc += cuStat.baseQp;
                curEncData.m_rowStat[row].numEncodedCUs = cuAddr; // 标记当前行统计的信息已经统计到了当前ctu位置
            }
            
            // If current block is at row end checkpoint, call vbv ratecontrol.
            if (!m_param->bEnableWavefront && col == numCols - 1)
            { //非wpp下，ctu行最后一个ctu编码完成
                double qpBase = curEncData.m_cuStat[cuAddr].baseQp;
                //检查是否需要重新编码
                curRow.reEncode = m_top->m_rateControl->rowVbvRateControl(m_frame, row, &m_rce, qpBase, m_sliceBaseRow, sliceId);
                qpBase = s265_clip3((double)m_param->rc.qpMin, (double)m_param->rc.qpMax, qpBase);
                curEncData.m_rowStat[row].rowQp = qpBase;
                curEncData.m_rowStat[row].rowQpScale = s265_qp2qScale(qpBase);
                if (curRow.reEncode < 0)
                {// 需要重新编码
                    s265_log(m_param, S265_LOG_DEBUG, "POC %d row %d - encode restart required for VBV, to %.2f from %.2f\n",
                        m_frame->m_poc, row, qpBase, curEncData.m_cuStat[cuAddr].baseQp);

                    m_vbvResetTriggerRow[curRow.sliceId] = row;//设置当前ctu所属slice需要从当前行开始重新编码
                    m_outStreams[0].copyBits(&m_backupStreams[0]);//恢复输出bit 状态为初始状态

                    rowCoder.copyState(curRow.bufferedEntropy);//恢复state
                    rowCoder.loadContexts(curRow.bufferedEntropy);//恢复context

                    curRow.completed = 0;//completed 重置0
                    memset(&curRow.rowStats, 0, sizeof(curRow.rowStats));//统计数据重新清零
                    curEncData.m_rowStat[row].numEncodedCUs = 0;
                    curEncData.m_rowStat[row].encodedBits = 0;
                    curEncData.m_rowStat[row].rowSatd = 0;
                    curEncData.m_rowStat[row].rowIntraSatd = 0;
                    curEncData.m_rowStat[row].sumQpRc = 0;
                    curEncData.m_rowStat[row].sumQpAq = 0;
                }
            }
            // If current block is at row diagonal checkpoint, call vbv ratecontrol.
            // wpp下 处在对角线上的ctu 且非slice的首行
            // 放在对角线上检查的原因： 与wpp依赖关系构成 trick 保证每次只有一个线程进入该process
            else if (m_param->bEnableWavefront && rowInSlice == col && !bFirstRowInSlice)
            {
                if (m_param->rc.bEnableConstVbv)
                {
                    uint32_t startCuAddr = numCols * row;//当前行起始ctu位于frame中的地址
                    uint32_t EndCuAddr = startCuAddr + col;// 已经completed了ctu的位于frame中的地址为止

                    // 当前行统计当前行已经编码的ctu
                    // 往上走 剩余的行如果还没完成编码，则每行统计两个ctu，否则，不再统计
                    for (int32_t r = row; r >= (int32_t)m_sliceBaseRow[sliceId]; r--)//从当前slice的当前行往后推到slice的起始行为止
                    {
                        for (uint32_t c = startCuAddr; c <= EndCuAddr && c <= numCols * (r + 1) - 1; c++)
                        {
                            curEncData.m_rowStat[r].rowSatd += curEncData.m_cuStat[c].vbvCost;
                            curEncData.m_rowStat[r].rowIntraSatd += curEncData.m_cuStat[c].intraVbvCost;
                            curEncData.m_rowStat[r].encodedBits += curEncData.m_cuStat[c].totalBits;
                            curEncData.m_rowStat[r].sumQpRc += curEncData.m_cuStat[c].baseQp;
                            curEncData.m_rowStat[r].numEncodedCUs = c;//更新每一行的已经统计到了c位置为止的ctu 信息
                        }
                        if (curRow.reEncode < 0)
                            break;
                        startCuAddr = EndCuAddr - numCols;//下一次统计的起始位置为当前行的结束位置往上退一行
                        EndCuAddr = startCuAddr + 1;//一行只统计两个CTU的信息
                        if( EndCuAddr > numCols * r ) break; //到第r行截止就好了，再往后推是那些行已经全部编码完毕了
                    }
                }
                double qpBase = curEncData.m_cuStat[cuAddr].baseQp;
                //wpp下 在对角线上的ctu上检查是否满足vbv
                curRow.reEncode = m_top->m_rateControl->rowVbvRateControl(m_frame, row, &m_rce, qpBase, m_sliceBaseRow, sliceId);
                qpBase = s265_clip3((double)m_param->rc.qpMin, (double)m_param->rc.qpMax, qpBase);
                curEncData.m_rowStat[row].rowQp = qpBase;
                curEncData.m_rowStat[row].rowQpScale = s265_qp2qScale(qpBase);

                if (curRow.reEncode < 0)
                {// 如果需要重编码
                    s265_log(m_param, S265_LOG_DEBUG, "POC %d row %d - encode restart required for VBV, to %.2f from %.2f\n",
                             m_frame->m_poc, row, qpBase, curEncData.m_cuStat[cuAddr].baseQp);

                    // prevent the WaveFront::findJob() method from providing new jobs
                    // 注意：注意这里之所以不用加锁：因为每次只有一个线程进入该处理过程标记该flag
                    // 其他编码线程需要访问到该flag，如果访问到了正确的值则直接重新编码那一行
                    // 如果还没有访问到正确的值，则继续编码一个ctu，到一下个ctu可再次访问该flag，然后再重新编码对应行

                    m_vbvResetTriggerRow[curRow.sliceId] = row;//记录该slice需要从该row行开始重新编码
                    m_bAllRowsStop[curRow.sliceId] = true;// 该lice 的 curRow 需要重新编码，则curRow后面的所有row 都需要重新编码标志该slice的所有row 需要stop

                    for (uint32_t r = m_sliceBaseRow[sliceId + 1] - 1; r >= row; r--)// 从该slice 的最后一行开始回退到当前row 为止
                    {
                        CTURow& stopRow = m_rows[r];

                        if (r != row)//如果是在当前 row 之后的行，wpp下 有可能其他的row 已经被其他的线程在编码的状态
                        {
                            /* if row was active (ready to be run) clear active bit and bitmap bit for this row */
                            stopRow.lock.acquire();//获取该行的锁
                            while (stopRow.active)// 该行已经准要要编码了
                            {
                                if (dequeueRow(m_row_to_idx[r] * 2))//重置dependency 标志 （*2 表示的编码的任务，+1 表示对应row的滤波任务）
                                    stopRow.active = false;//成功清除了dependency标记后，设置active 为false
                                else
                                {
                                    /* we must release the row lock to allow the thread to exit */
                                    stopRow.lock.release();//没能够成功清除标记，则需要先释放锁，等待其他线程重新持有锁后根据m_bAllRowsStop 为ture 来退出
                                    GIVE_UP_TIME();
                                    stopRow.lock.acquire();// 再重新持有锁,去判断 active 是否已经被设置为false了
                                }
                            }
                            stopRow.lock.release(); //释放锁

                            bool bRowBusy = true;
                            do
                            {
                                stopRow.lock.acquire();// 重新获取该行的锁
                                bRowBusy = stopRow.busy;//获取对应的该行的状态
                                stopRow.lock.release();

                                if (bRowBusy)//如果该行为Busy状态，则继续等待退出busy状态
                                {
                                    GIVE_UP_TIME();
                                }
                            }
                            while (bRowBusy);
                        }
                        // 重置该行的所有状态
                        m_outStreams[r].resetBits();
                        stopRow.completed = 0;// 这里是安全的因为，在标记了该行需要重新编码后，会等待正在编码的线程完成其正在编码的ctu 的completed++,之后再被清零
                        memset(&stopRow.rowStats, 0, sizeof(stopRow.rowStats));
                        curEncData.m_rowStat[r].numEncodedCUs = 0;
                        curEncData.m_rowStat[r].encodedBits = 0;
                        curEncData.m_rowStat[r].rowSatd = 0;
                        curEncData.m_rowStat[r].rowIntraSatd = 0;
                        curEncData.m_rowStat[r].sumQpRc = 0;
                        curEncData.m_rowStat[r].sumQpAq = 0;
                    }

                    m_bAllRowsStop[curRow.sliceId] = false;// 可以放开条件，让其他线程继续去执行该slice的row的编码任务了
                }
            }
        }

        if (m_param->bEnableWavefront && curRow.completed >= 2 && !bLastRowInSlice &&
            (!m_bAllRowsStop[curRow.sliceId] || intRow + 1 < m_vbvResetTriggerRow[curRow.sliceId]))
        {   //wpp下 该行已经编码了2个ctu，该行不是slice最后一行 并且 该slice没有被标记为需要stop 或者 虽有标记了但是下一行还没有到达重新编码的的起始行
            /* activate next row */
            ScopedLock below(m_rows[row + 1].lock);

            if (m_rows[row + 1].active == false &&
                m_rows[row + 1].completed + 2 <= curRow.completed)//wpp 条件满足了
            {
                m_rows[row + 1].active = true;
                // 编码下一行所需的内部依赖解决了,清除内部依赖
                enqueueRowEncoder(m_row_to_idx[row + 1]);
                //wpp下 入口2://-->WaveFront::findJob 唤醒一个线程 去找对应的jobprovider 取出一个任务执行 此处（执行 FrameEncoder 的processRow 里面的 encode 任务)
                tryWakeOne(); /* wake up a sleeping thread or set the help wanted flag */
            }
        }

        ScopedLock self(curRow.lock);
        if ((m_bAllRowsStop[curRow.sliceId] && intRow > m_vbvResetTriggerRow[curRow.sliceId]) ||
            (!bFirstRowInSlice && ((curRow.completed < numCols - 1) || (m_rows[row - 1].completed < numCols)) && m_rows[row - 1].completed < curRow.completed + 2))
        { // 如果该slice 被标记为需要从某一行开始（m_vbvResetTriggerRow）重新编码 && 且当前行刚好位于该行之后 或者
          // 该行不为slice的首行&&并且(当前编码的ctu不为该行最后一个ctu || 或者上一行还没有完成所有ctu编码)并且其依赖的wpp条件前一行没有领先该行2个ctu编码时,
            curRow.active = false; //当前任务被阻塞需要停止
            curRow.busy = false; //标记当前行不在busy 状态
            ATOMIC_INC(&m_countRowBlocks);
            return;
        }

    }// 完成了 第 rowInSlice 行ctu的编码了

    /* this row of CTUs has been compressed */
    if (m_param->bEnableWavefront && m_param->rc.bEnableConstVbv)
    {//wpp下
        if (bLastRowInSlice)// 整个slice 已经在编码最后一行了    注意: 这里统计所有还没有统计到的ctu的信息 
        {   //从slice的起始行到最后一行
            for (uint32_t r = m_sliceBaseRow[sliceId]; r < m_sliceBaseRow[sliceId + 1]; r++)
            {   // 统计每一行中还没有completed的ctu的信息累加
                for (uint32_t c = curEncData.m_rowStat[r].numEncodedCUs + 1; c < numCols * (r + 1); c++)
                {
                    curEncData.m_rowStat[r].rowSatd += curEncData.m_cuStat[c].vbvCost;
                    curEncData.m_rowStat[r].rowIntraSatd += curEncData.m_cuStat[c].intraVbvCost;
                    curEncData.m_rowStat[r].encodedBits += curEncData.m_cuStat[c].totalBits;
                    curEncData.m_rowStat[r].sumQpRc += curEncData.m_cuStat[c].baseQp;
                    curEncData.m_rowStat[r].numEncodedCUs = c;
                }
            }
        }
    }

    /* If encoding with ABR, update update bits and complexity in rate control
     * after a number of rows so the next frame's rateControlStart has more
     * accurate data for estimation. At the start of the encode we update stats
     * after half the frame is encoded, but after this initial period we update
     * after refLagRows (the number of rows reference frames must have completed
     * before referencees may begin encoding) */
    if (m_param->rc.rateControlMode == S265_RC_ABR || bIsVbv)
    {// 在有码率控制的情况下
        uint32_t rowCount = 0;
        uint32_t maxRows = m_sliceBaseRow[sliceId + 1] - m_sliceBaseRow[sliceId];

        if (!m_rce.encodeOrder)//对于首帧需要等到最后一行编码结束
            rowCount = maxRows - 1; 
        else if ((uint32_t)m_rce.encodeOrder <= 2 * (m_param->fpsNum / m_param->fpsDenom))
            rowCount = S265_MIN((maxRows + 1) / 2, maxRows - 1);//对于前面两秒时间内的帧数，需要等到帧的一半ctu 行编码完成
        else// 对于超过2s后的帧，只需要等待 refLagRow 行
            rowCount = S265_MIN(m_refLagRows / m_param->maxSlices, maxRows - 1);

        if (rowInSlice == rowCount)//当前slice需要等待的CTU行编码完了
        {
            m_rowSliceTotalBits[sliceId] = 0;
            if (bIsVbv && !(m_param->rc.bEnableConstVbv && m_param->bEnableWavefront))
            {
                for (uint32_t i = m_sliceBaseRow[sliceId]; i < rowCount + m_sliceBaseRow[sliceId]; i++)
                    m_rowSliceTotalBits[sliceId] += curEncData.m_rowStat[i].encodedBits;//累加slice 中的每一行ctu 编码产生的bits
            }
            else
            {
                uint32_t startAddr = m_sliceBaseRow[sliceId] * numCols;//当前slice的首个ctu在frame中地址
                uint32_t finishAddr = startAddr + rowCount * numCols;// 当前slcie的最后一个ctu 在frame中的地址
                
                for (uint32_t cuAddr = startAddr; cuAddr < finishAddr; cuAddr++)
                    m_rowSliceTotalBits[sliceId] += curEncData.m_cuStat[cuAddr].totalBits;// 按照ctu 进行统计产生的bits
            }
            // slice num 先自加 再访问, 
            if (ATOMIC_INC(&m_sliceCnt) == (int)m_param->maxSlices)
            {   //如果当前frame的slice 全部编码完毕，则重新统计bits
                m_rce.rowTotalBits = 0;
                for (uint32_t i = 0; i < m_param->maxSlices; i++)
                    m_rce.rowTotalBits += m_rowSliceTotalBits[i];// 所有slice产生的bits 累加
                m_top->m_rateControl->rateControlUpdateStats(&m_rce);
            }
        }
    }

    /* flush row bitstream (if WPP and no SAO) or flush frame if no WPP and no SAO */
    /* end_of_sub_stream_one_bit / end_of_slice_segment_flag */
       if (!slice->m_bUseSao && (m_param->bEnableWavefront || bLastRowInSlice))
               rowCoder.finishSlice();


    /* Processing left Deblock block with current threading */
    if ((m_param->bEnableLoopFilter | slice->m_bUseSao) & (rowInSlice >= 2))
    {
        /* Check conditional to start previous row process with current threading */
        if (m_frameFilter.m_parallelFilter[row - 2].m_lastDeblocked.get() == (int)numCols)
        {
            /* stop threading on current row and restart it */
            m_frameFilter.m_parallelFilter[row - 1].m_allowedCol.set(numCols);
            m_frameFilter.m_parallelFilter[row - 1].processTasks(-1);
        }
    }

    /* trigger row-wise loop filters */
    if (m_param->bEnableWavefront)
    {
        if (rowInSlice >= m_filterRowDelay)
        {
            // 外部依赖关系解决
            enableRowFilter(m_row_to_idx[row - m_filterRowDelay]);

            /* NOTE: Activate filter if first row (row 0) */
            if (rowInSlice == m_filterRowDelay)// 第一次由 此处提交任务，后面的任务提交由work完成一次filter 任务提交下一行的filter 任务
                enqueueRowFilter(m_row_to_idx[row - m_filterRowDelay]);
            tryWakeOne();// -->WaveFront::findJob 唤醒一个线程 去找对应的jobprovider 取出一个任务执行 此处（执行 FrameEncoder 的processRow 里面的 filter 任务)
        }

        if (bLastRowInSlice)
        { // slice的最后一行时, 最后几行的的外部依赖关系 自然已经满足了
            for (uint32_t i = endRowInSlicePlus1 - m_filterRowDelay; i < endRowInSlicePlus1; i++)
            {
                enableRowFilter(m_row_to_idx[i]);
            }
            tryWakeOne();// -->WaveFront::findJob 唤醒一个线程 去找对应的jobprovider 取出一个任务执行 此处（执行 FrameEncoder 的processRow 里面的 filter 任务)
        }

        // handle specially case - single row slice 单slice单ctu row的情况
        if  (bFirstRowInSlice & bLastRowInSlice)
        {
            enqueueRowFilter(m_row_to_idx[row]);
            tryWakeOne();// --> WaveFront::findJob
        }
    }
    // 标记 curRow 已经处理完了 
    curRow.busy = false;

    // CHECK_ME: Does it always FALSE condition?
    if (ATOMIC_INC(&m_completionCount) == 2 * (int)m_numRows)
        m_completionEvent.trigger();
}

void FrameEncoder::collectDynDataRow(CUData& ctu, FrameStats* rowStats)
{
    for (uint32_t i = 0; i < S265_REFINE_INTER_LEVELS; i++)
    {
        for (uint32_t depth = 0; depth < m_param->maxCUDepth; depth++)
        {
            int offset = (depth * S265_REFINE_INTER_LEVELS) + i;
            if (ctu.m_collectCUCount[offset])
            {
                rowStats->rowVarDyn[offset] += ctu.m_collectCUVariance[offset];
                rowStats->rowRdDyn[offset] += ctu.m_collectCURd[offset];
                rowStats->rowCntDyn[offset] += ctu.m_collectCUCount[offset];
            }
        }
    }
}

void FrameEncoder::collectDynDataFrame()
{
    for (uint32_t row = 0; row < m_numRows; row++)
    {
        for (uint32_t refLevel = 0; refLevel < S265_REFINE_INTER_LEVELS; refLevel++)
        {
            for (uint32_t depth = 0; depth < m_param->maxCUDepth; depth++)
            {
                int offset = (depth * S265_REFINE_INTER_LEVELS) + refLevel;
                int curFrameIndex = m_frame->m_encodeOrder - m_top->m_startPoint;
                int index = (curFrameIndex * S265_REFINE_INTER_LEVELS * m_param->maxCUDepth) + offset;
                if (m_rows[row].rowStats.rowCntDyn[offset])
                {
                    m_top->m_variance[index] += m_rows[row].rowStats.rowVarDyn[offset];
                    m_top->m_rdCost[index] += m_rows[row].rowStats.rowRdDyn[offset];
                    m_top->m_trainingCount[index] += m_rows[row].rowStats.rowCntDyn[offset];
                }
            }
        }
    }
}

void FrameEncoder::computeAvgTrainingData()
{
    if (m_frame->m_lowres.bScenecut || m_frame->m_lowres.bKeyframe)
    {
        m_top->m_startPoint = m_frame->m_encodeOrder;
        int size = (m_param->keyframeMax + m_param->lookaheadDepth) * m_param->maxCUDepth * S265_REFINE_INTER_LEVELS;
        memset(m_top->m_variance, 0, size * sizeof(uint64_t));
        memset(m_top->m_rdCost, 0, size * sizeof(uint64_t));
        memset(m_top->m_trainingCount, 0, size * sizeof(uint32_t));
    }
    if (m_frame->m_encodeOrder - m_top->m_startPoint < 2 * m_param->frameNumThreads)
        m_frame->m_classifyFrame = false;
    else
        m_frame->m_classifyFrame = true;

    int size = m_param->maxCUDepth * S265_REFINE_INTER_LEVELS;
    memset(m_frame->m_classifyRd, 0, size * sizeof(uint64_t));
    memset(m_frame->m_classifyVariance, 0, size * sizeof(uint64_t));
    memset(m_frame->m_classifyCount, 0, size * sizeof(uint32_t));
    if (m_frame->m_classifyFrame)
    {
        uint32_t limit = m_frame->m_encodeOrder - m_top->m_startPoint - m_param->frameNumThreads;
        for (uint32_t i = 1; i < limit; i++)
        {
            for (uint32_t j = 0; j < S265_REFINE_INTER_LEVELS; j++)
            {
                for (uint32_t depth = 0; depth < m_param->maxCUDepth; depth++)
                {
                    int offset = (depth * S265_REFINE_INTER_LEVELS) + j;
                    int index = (i* S265_REFINE_INTER_LEVELS * m_param->maxCUDepth) + offset;
                    if (m_top->m_trainingCount[index])
                    {
                        m_frame->m_classifyRd[offset] += m_top->m_rdCost[index] / m_top->m_trainingCount[index];
                        m_frame->m_classifyVariance[offset] += m_top->m_variance[index] / m_top->m_trainingCount[index];
                        m_frame->m_classifyCount[offset] += m_top->m_trainingCount[index];
                    }
                }
            }
        }
        /* Calculates the average feature values of historic frames that are being considered for the current frame */
        int historyCount = m_frame->m_encodeOrder - m_param->frameNumThreads - m_top->m_startPoint - 1;
        if (historyCount)
        {
            for (uint32_t j = 0; j < S265_REFINE_INTER_LEVELS; j++)
            {
                for (uint32_t depth = 0; depth < m_param->maxCUDepth; depth++)
                {
                    int offset = (depth * S265_REFINE_INTER_LEVELS) + j;
                    m_frame->m_classifyRd[offset] /= historyCount;
                    m_frame->m_classifyVariance[offset] /= historyCount;
                }
            }
        }
    }
}

/* collect statistics about CU coding decisions, return total QP */
int FrameEncoder::collectCTUStatistics(const CUData& ctu, FrameStats* log)
{
    int totQP = 0;
    uint32_t depth = 0;
    for (uint32_t absPartIdx = 0; absPartIdx < ctu.m_numPartitions; absPartIdx += ctu.m_numPartitions >> (depth * 2))
    {
        depth = ctu.m_cuDepth[absPartIdx];
        // 按照4X4 来存储的m_qp
        totQP += ctu.m_qp[absPartIdx] * (ctu.m_numPartitions >> (depth * 2));
    }

    if (m_param->csvLogLevel >= 1)
    {
        if (ctu.m_slice->m_sliceType == I_SLICE)
        {
            depth = 0;
            for (uint32_t absPartIdx = 0; absPartIdx < ctu.m_numPartitions; absPartIdx += ctu.m_numPartitions >> (depth * 2))
            {
                depth = ctu.m_cuDepth[absPartIdx];

                log->totalCu++;
                log->cntIntra[depth]++;

                if (ctu.m_predMode[absPartIdx] == MODE_NONE)
                {
                    log->totalCu--;
                    log->cntIntra[depth]--;
                }
                else if (ctu.m_partSize[absPartIdx] != SIZE_2Nx2N)
                {
                    /* TODO: log intra modes at absPartIdx +0 to +3 */
                    S265_CHECK(ctu.m_log2CUSize[absPartIdx] == 3 && ctu.m_slice->m_sps->quadtreeTULog2MinSize < 3, "Intra NxN found at improbable depth\n");
                    log->cntIntraNxN++;
                    log->cntIntra[depth]--;
                }
                else if (ctu.m_lumaIntraDir[absPartIdx] > 1)
                    log->cuIntraDistribution[depth][ANGULAR_MODE_ID]++;
                else
                    log->cuIntraDistribution[depth][ctu.m_lumaIntraDir[absPartIdx]]++;
            }
        }
        else
        {
            depth = 0;
            for (uint32_t absPartIdx = 0; absPartIdx < ctu.m_numPartitions; absPartIdx += ctu.m_numPartitions >> (depth * 2))
            {
                depth = ctu.m_cuDepth[absPartIdx];

                log->totalCu++;

                if (ctu.m_predMode[absPartIdx] == MODE_NONE)
                    log->totalCu--;
                else if (ctu.isSkipped(absPartIdx))
                {
                    if (ctu.m_mergeFlag[0])
                        log->cntMergeCu[depth]++;
                    else
                        log->cntSkipCu[depth]++;
                }
                else if (ctu.isInter(absPartIdx))
                {
                    log->cntInter[depth]++;

                    if (ctu.m_partSize[absPartIdx] < AMP_ID)
                        log->cuInterDistribution[depth][ctu.m_partSize[absPartIdx]]++;
                    else
                        log->cuInterDistribution[depth][AMP_ID]++;
                }
                else if (ctu.isIntra(absPartIdx))
                {
                    log->cntIntra[depth]++;

                    if (ctu.m_partSize[absPartIdx] != SIZE_2Nx2N)
                    {
                        S265_CHECK(ctu.m_log2CUSize[absPartIdx] == 3 && ctu.m_slice->m_sps->quadtreeTULog2MinSize < 3, "Intra NxN found at improbable depth\n");
                        log->cntIntraNxN++;
                        log->cntIntra[depth]--;
                        /* TODO: log intra modes at absPartIdx +0 to +3 */
                    }
                    else if (ctu.m_lumaIntraDir[absPartIdx] > 1)
                        log->cuIntraDistribution[depth][ANGULAR_MODE_ID]++;
                    else
                        log->cuIntraDistribution[depth][ctu.m_lumaIntraDir[absPartIdx]]++;
                }
            }
        }
    }

    return totQP;
}

/* DCT-domain noise reduction / adaptive deadzone from libavcodec */
void FrameEncoder::noiseReductionUpdate()
{
    static const uint32_t maxBlocksPerTrSize[4] = {1 << 18, 1 << 16, 1 << 14, 1 << 12};

    for (int cat = 0; cat < MAX_NUM_TR_CATEGORIES; cat++)
    {
        int trSize = cat & 3;
        int coefCount = 1 << ((trSize + 2) * 2);

        if (m_nr->nrCount[cat] > maxBlocksPerTrSize[trSize])
        {
            for (int i = 0; i < coefCount; i++)
                m_nr->nrResidualSum[cat][i] >>= 1;
            m_nr->nrCount[cat] >>= 1;
        }

        int nrStrength = cat < 8 ? m_param->noiseReductionIntra : m_param->noiseReductionInter;
        uint64_t scaledCount = (uint64_t)nrStrength * m_nr->nrCount[cat];

        for (int i = 0; i < coefCount; i++)
        {
            uint64_t value = scaledCount + m_nr->nrResidualSum[cat][i] / 2;
            uint64_t denom = m_nr->nrResidualSum[cat][i] + 1;
            m_nr->nrOffsetDenoise[cat][i] = (uint16_t)(value / denom);
        }

        // Don't denoise DC coefficients
        m_nr->nrOffsetDenoise[cat][0] = 0;
    }
}
#if ENABLE_LIBVMAF
void FrameEncoder::vmafFrameLevelScore()
{
    //PicYuv *fenc = m_frame->m_fencPic;
    PicYuv *fenc = m_frame->m_originalPic;
    PicYuv *recon = m_frame->m_reconPic;

    s265_vmaf_framedata *vmafframedata = (s265_vmaf_framedata*)s265_malloc(sizeof(s265_vmaf_framedata));
    if (!vmafframedata)
    {
        s265_log(NULL, S265_LOG_ERROR, "vmaf frame data alloc failed\n");
    }

    vmafframedata->height = fenc->m_picHeight;
    vmafframedata->width = fenc->m_picWidth;
    vmafframedata->frame_set = 0;
    vmafframedata->internalBitDepth = m_param->internalBitDepth;
    vmafframedata->reference_frame = fenc;
    vmafframedata->distorted_frame = recon;

    fenc->m_vmafScore = s265_calculate_vmaf_framelevelscore(vmafframedata);

    if (vmafframedata)
    s265_free(vmafframedata);
}
#endif
// 当调用次函数时，如果该FrameEncoder 对应线程已经在编码（m_frame不为空）
// 则需要等待该编码线程完成该帧的编码
// 当然可能该编码线程已经编码完成了
Frame *FrameEncoder::getEncodedPicture(NALList& output)
{
    if (m_frame) // m_frame 不为空则表示该帧级编码器有帧在编码，需要等待其编码完成
    {
        /* block here until worker thread completes */
        m_done.wait();// 等待 compressFrame(); 完后后的 m_done.trigger()

        Frame *ret = m_frame;
        m_frame = NULL;
        output.takeContents(m_nalList);
        m_prevOutputTime = s265_mdate();
        return ret;
    }

    return NULL;
}
}
