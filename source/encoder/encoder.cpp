/*****************************************************************************
 * Copyright (C) 2013-2020 MulticoreWare, Inc
 *
 * Authors: Steve Borho <steve@borho.org>
 *          Min Chen <chenm003@163.com>
 *          Praveen Kumar Tiwari <praveen@multicorewareinc.com>
 *          Aruna Matheswaran <aruna@multicorewareinc.com>
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
#include "primitives.h"
#include "threadpool.h"
#include "param.h"
#include "frame.h"
#include "framedata.h"
#include "picyuv.h"

#include "bitcost.h"
#include "encoder.h"
#include "slicetype.h"
#include "frameencoder.h"
#include "ratecontrol.h"
#include "dpb.h"
#include "nal.h"

#include "s265.h"

#if _MSC_VER
#pragma warning(disable: 4996) // POSIX functions are just fine, thanks
#endif

namespace S265_NS {
const char g_sliceTypeToChar[] = {'B', 'P', 'I'};

/* Dolby Vision profile specific settings */
typedef struct
{
    int bEmitHRDSEI;
    int bEnableVideoSignalTypePresentFlag;
    int bEnableColorDescriptionPresentFlag;
    int bEnableAccessUnitDelimiters;
    int bAnnexB;

    /* VUI parameters specific to Dolby Vision Profile */
    int videoFormat;
    int bEnableVideoFullRangeFlag;
    int transferCharacteristics;
    int colorPrimaries;
    int matrixCoeffs;

    int doviProfileId;
}DolbyVisionProfileSpec;

DolbyVisionProfileSpec dovi[] =
{
    { 1, 1, 1, 1, 1, 5, 1,  2, 2, 2, 50 },
    { 1, 1, 1, 1, 1, 5, 0, 16, 9, 9, 81 },
    { 1, 1, 1, 1, 1, 5, 0,  1, 1, 1, 82 }
};

}

/* Threshold for motion vection, based on expermental result.
 * TODO: come up an algorithm for adoptive threshold */
#define PU_2Nx2N 1
#define MAX_CHROMA_QP_OFFSET 12

using namespace S265_NS;

Encoder::Encoder()
{
    m_aborted = false;
    m_reconfigure = false;
    m_reconfigureRc = false;
    m_encodedFrameNum = 0;
    m_pocLast = -1;
    m_curEncoder = 0;
    m_numLumaWPFrames = 0;
    m_numChromaWPFrames = 0;
    m_numLumaWPBiFrames = 0;
    m_numChromaWPBiFrames = 0;
    m_lookahead = NULL;
    m_rateControl = NULL;
    m_dpb = NULL;
    m_exportedPic = NULL;
    m_numDelayedPic = 0;
    m_outputCount = 0;
    m_param = NULL;
    m_latestParam = NULL;
    m_threadPool = NULL;
    m_naluFile = NULL;
    m_offsetEmergency = NULL;
    m_iFrameNum = 0;
    m_iPPSQpMinus26 = 0;
    m_rpsInSpsCount = 0;
    m_cB = 1.0;
    m_cR = 1.0;
    for (int i = 0; i < S265_MAX_FRAME_THREADS; i++)
        m_frameEncoder[i] = NULL;
    MotionEstimate::initScales();

#if ENABLE_HDR10_PLUS
    m_hdr10plus_api = hdr10plus_api_get();
    m_numCimInfo = 0;
    m_cim = NULL;
#endif

#if SVT_HEVC
    m_svtAppData = NULL;
#endif
    m_prevTonemapPayload.payload = NULL;
    m_startPoint = 0;
    m_saveCTUSize = 0;
    m_edgePic = NULL;
    m_edgeHistThreshold = 0;
    m_chromaHistThreshold = 0.0;
    m_scaledEdgeThreshold = 0.0;
    m_scaledChromaThreshold = 0.0;
    m_zoneIndex = 0;
}

inline char *strcatFilename(const char *input, const char *suffix)
{
    char *output = S265_MALLOC(char, strlen(input) + strlen(suffix) + 1);
    if (!output)
    {
        s265_log(NULL, S265_LOG_ERROR, "unable to allocate memory for filename\n");
        return NULL;
    }
    strcpy(output, input);
    strcat(output, suffix);
    return output;
}

void Encoder::create()
{
    if (!primitives.pu[0].sad)
    {
        // this should be an impossible condition when using our public API, and indicates a serious bug.
        s265_log(m_param, S265_LOG_ERROR, "Primitives must be initialized before encoder is created\n");
        abort();
    }

    s265_param* p = m_param;

    int rows = (p->sourceHeight + p->maxCUSize - 1) >> g_log2Size[p->maxCUSize];
    int cols = (p->sourceWidth  + p->maxCUSize - 1) >> g_log2Size[p->maxCUSize];

    if (m_param->bHistBasedSceneCut)
    {
        m_planeSizes[0] = (m_param->sourceWidth >> s265_cli_csps[p->internalCsp].width[0]) * (m_param->sourceHeight >> s265_cli_csps[m_param->internalCsp].height[0]);
        uint32_t pixelbytes = m_param->internalBitDepth > 8 ? 2 : 1;
        m_edgePic = S265_MALLOC(pixel, m_planeSizes[0] * pixelbytes);
        m_edgeHistThreshold = m_param->edgeTransitionThreshold;
        m_chromaHistThreshold = s265_min(m_edgeHistThreshold * 10.0, MAX_SCENECUT_THRESHOLD);
        m_scaledEdgeThreshold = s265_min(m_edgeHistThreshold * SCENECUT_STRENGTH_FACTOR, MAX_SCENECUT_THRESHOLD);
        m_scaledChromaThreshold = s265_min(m_chromaHistThreshold * SCENECUT_STRENGTH_FACTOR, MAX_SCENECUT_THRESHOLD);
        if (m_param->sourceBitDepth != m_param->internalBitDepth)
        {
            int size = m_param->sourceWidth * m_param->sourceHeight;
            int hshift = CHROMA_H_SHIFT(m_param->internalCsp);
            int vshift = CHROMA_V_SHIFT(m_param->internalCsp);
            int widthC = m_param->sourceWidth >> hshift;
            int heightC = m_param->sourceHeight >> vshift;

            m_inputPic[0] = S265_MALLOC(pixel, size);
            if (m_param->internalCsp != S265_CSP_I400)
            {
                for (int j = 1; j < 3; j++)
                {
                    m_inputPic[j] = S265_MALLOC(pixel, widthC * heightC);
                }
            }
        }
    }

    // Do not allow WPP if only one row or fewer than 3 columns, it is pointless and unstable
    if (rows == 1 || cols < 3)
    {
        s265_log(p, S265_LOG_WARNING, "Too few rows/columns, --wpp disabled\n");
        p->bEnableWavefront = 0;
    }

    bool allowPools = !p->numaPools || strcmp(p->numaPools, "none");

    // Trim the thread pool if --wpp, --pme, and --pmode are disabled
    if (!p->bEnableWavefront && !p->bDistributeModeAnalysis && !p->bDistributeMotionEstimation && !p->lookaheadSlices)
        allowPools = false;

    m_numPools = 0;
    if (allowPools)
        // 创建用于编码的若干线程池
        m_threadPool = ThreadPool::allocThreadPools(p, m_numPools, 0);// 此处m_numPools 是引用传递 0表示非lookahead 线程池
    else
    {
        if (!p->frameNumThreads)
        {
            // auto-detect frame threads
            int cpuCount = ThreadPool::getCpuCount();
            ThreadPool::getFrameThreadsCount(p, cpuCount);
        }
    }

    if (!m_numPools)
    {
        // issue warnings if any of these features were requested
        if (p->bEnableWavefront)
            s265_log(p, S265_LOG_WARNING, "No thread pool allocated, --wpp disabled\n");
        if (p->bDistributeMotionEstimation)
            s265_log(p, S265_LOG_WARNING, "No thread pool allocated, --pme disabled\n");
        if (p->bDistributeModeAnalysis)
            s265_log(p, S265_LOG_WARNING, "No thread pool allocated, --pmode disabled\n");
        if (p->lookaheadSlices)
            s265_log(p, S265_LOG_WARNING, "No thread pool allocated, --lookahead-slices disabled\n");

        // disable all pool features if the thread pool is disabled or unusable.
        p->bEnableWavefront = p->bDistributeModeAnalysis = p->bDistributeMotionEstimation = p->lookaheadSlices = 0;
    }

    s265_log(p, S265_LOG_INFO, "Slices                              : %d\n", p->maxSlices);

    char buf[128];
    int len = 0;
    if (p->bEnableWavefront)
        len += sprintf(buf + len, "wpp(%d rows)", rows);
    if (p->bDistributeModeAnalysis)
        len += sprintf(buf + len, "%spmode", len ? "+" : "");
    if (p->bDistributeMotionEstimation)
        len += sprintf(buf + len, "%spme ", len ? "+" : "");
    if (!len)
        strcpy(buf, "none");

    s265_log(p, X265_LOG_INFO, "frame threads / pool features       : %d / %s\n", p->frameNumThreads, buf);
    // 根据帧级编码线程数量创建若干个帧编码器实例

    for (int i = 0; i < m_param->frameNumThreads; i++)
    {
        m_frameEncoder[i] = new FrameEncoder;
        m_frameEncoder[i]->m_nalList.m_annexB = !!m_param->bAnnexB;
    }
    /*
    一个m_frameEncoder 类似一个领导，一定会归属到某一个线程池, 一个线程池可能挂多个领导
    */
    if (m_numPools)
    {   // 注意 线程池的个数 一定是小于framenumthreads的个数
        // 任务分配
        for (int i = 0; i < m_param->frameNumThreads; i++)
        {
            //帧级编码线程依次轮训分配到各个线程池 做领导派发任务
            int pool = i % m_numPools;//第i个m_frameEncoder应该分配到哪个线程池
            m_frameEncoder[i]->m_pool = &m_threadPool[pool];//在第i个m_frameEncoder 中该帧级编码实例所属线程池 第i个领导管理第pool个线程池
            // 该第i个m_frameEncoder是其所属线程池的第几个任务派发者（生产者）'第i个领导属于他管理的线程池中的第几个领导'
            m_frameEncoder[i]->m_jpId = m_threadPool[pool].m_numProviders++;// jobprovider id 加1;
            //FrameEncoder 是派生自 wavefront，而wavefront派生自 jobprovider
            //将第i个领导安排给 第pool个线程池的领导们中第jpid的位置
            m_threadPool[pool].m_jpTable[m_frameEncoder[i]->m_jpId] = m_frameEncoder[i];
            //一个线程池可能对应分配到多个(一般不超过两个)帧编码任务，每个帧编码将作为一个独立的jobprovider 记录在jpTable中
        }
        //启动所用线程池的所有worker线程
        for (int i = 0; i < m_numPools; i++)
            m_threadPool[i].start();
    }
    else
    {
        /* CU stats and noise-reduction buffers are indexed by jpId, so it cannot be left as -1 */
        for (int i = 0; i < m_param->frameNumThreads; i++)
            m_frameEncoder[i]->m_jpId = 0;//
    }

    if (!m_scalingList.init())
    {
        s265_log(m_param, S265_LOG_ERROR, "Unable to allocate scaling list arrays\n");
        m_aborted = true;
        return;
    }
    else if (!m_param->scalingLists || !strcmp(m_param->scalingLists, "off"))
        m_scalingList.m_bEnabled = false;
    else if (!strcmp(m_param->scalingLists, "default"))
        m_scalingList.setDefaultScalingList();
    else if (m_scalingList.parseScalingList(m_param->scalingLists))
        m_aborted = true;
    int pools = m_numPools;
    ThreadPool* lookAheadThreadPool = 0;
    if (m_param->lookaheadThreads > 0)
    {// 如果设置了lookaheadthreads 则表示lookahead 使用单独的线程池
        lookAheadThreadPool = ThreadPool::allocThreadPools(p, pools, 1);// 注意: pools 的值在里面被修改为1
    }
    else//否则,共用encoder的线程池
        lookAheadThreadPool = m_threadPool;
    m_lookahead = new Lookahead(m_param, lookAheadThreadPool);
    if (pools)
    {   
        // lookahead 只有一个线程池0，这里 将‘领导’ m_lookahead 追加给该线程池
        m_lookahead->m_jpId = lookAheadThreadPool[0].m_numProviders++;
        lookAheadThreadPool[0].m_jpTable[m_lookahead->m_jpId] = m_lookahead;
    }
    if (m_param->lookaheadThreads > 0)// 如果设置了lookaheadthreads 则表示lookahead 有自己单独的线程池
        for (int i = 0; i < pools; i++)
            lookAheadThreadPool[i].start();//lookahead自己的线程池启动
    m_lookahead->m_numPools = pools;// 注意这里限定了为 1
    m_dpb = new DPB(m_param);
    m_rateControl = new RateControl(*m_param, this);
    if (!m_param->bResetZoneConfig)
    {
        zoneReadCount = new ThreadSafeInteger[m_param->rc.zonefileCount];
        zoneWriteCount = new ThreadSafeInteger[m_param->rc.zonefileCount];
    }

    initVPS(&m_vps);
    initSPS(&m_sps);
    initPPS(&m_pps);
   
    if (m_param->rc.vbvBufferSize)
    {
        m_offsetEmergency = (uint16_t(*)[MAX_NUM_TR_CATEGORIES][MAX_NUM_TR_COEFFS])S265_MALLOC(uint16_t, MAX_NUM_TR_CATEGORIES * MAX_NUM_TR_COEFFS * (QP_MAX_MAX - QP_MAX_SPEC));
        if (!m_offsetEmergency)
        {
            s265_log(m_param, S265_LOG_ERROR, "Unable to allocate memory\n");
            m_aborted = true;
            return;
        }

        bool scalingEnabled = m_scalingList.m_bEnabled;
        if (!scalingEnabled)
        {
            m_scalingList.setDefaultScalingList();
            m_scalingList.setupQuantMatrices(m_sps.chromaFormatIdc);
        }
        else
            m_scalingList.setupQuantMatrices(m_sps.chromaFormatIdc);

        for (int q = 0; q < QP_MAX_MAX - QP_MAX_SPEC; q++)
        {
            for (int cat = 0; cat < MAX_NUM_TR_CATEGORIES; cat++)
            {
                uint16_t *nrOffset = m_offsetEmergency[q][cat];

                int trSize = cat & 3;

                int coefCount = 1 << ((trSize + 2) * 2);

                /* Denoise chroma first then luma, then DC. */
                int dcThreshold = (QP_MAX_MAX - QP_MAX_SPEC) * 2 / 3;
                int lumaThreshold = (QP_MAX_MAX - QP_MAX_SPEC) * 2 / 3;
                int chromaThreshold = 0;

                int thresh = (cat < 4 || (cat >= 8 && cat < 12)) ? lumaThreshold : chromaThreshold;

                double quantF = (double)(1ULL << (q / 6 + 16 + 8));

                for (int i = 0; i < coefCount; i++)
                {
                    /* True "emergency mode": remove all DCT coefficients */
                    if (q == QP_MAX_MAX - QP_MAX_SPEC - 1)
                    {
                        nrOffset[i] = INT16_MAX;
                        continue;
                    }

                    int iThresh = i == 0 ? dcThreshold : thresh;
                    if (q < iThresh)
                    {
                        nrOffset[i] = 0;
                        continue;
                    }

                    int numList = (cat >= 8) * 3 + ((int)!iThresh);

                    double pos = (double)(q - iThresh + 1) / (QP_MAX_MAX - QP_MAX_SPEC - iThresh);
                    double start = quantF / (m_scalingList.m_quantCoef[trSize][numList][QP_MAX_SPEC % 6][i]);

                    // Formula chosen as an exponential scale to vaguely mimic the effects of a higher quantizer.
                    double bias = (pow(2, pos * (QP_MAX_MAX - QP_MAX_SPEC)) * 0.003 - 0.003) * start;
                    nrOffset[i] = (uint16_t)S265_MIN(bias + 0.5, INT16_MAX);
                }
            }
        }

        if (!scalingEnabled)
        {
            m_scalingList.m_bEnabled = false;
            m_scalingList.m_bDataPresent = false;
            m_scalingList.setupQuantMatrices(m_sps.chromaFormatIdc);
        }
    }
    else
        m_scalingList.setupQuantMatrices(m_sps.chromaFormatIdc);

    int numRows = (m_param->sourceHeight + m_param->maxCUSize - 1) / m_param->maxCUSize;
    int numCols = (m_param->sourceWidth  + m_param->maxCUSize - 1) / m_param->maxCUSize;
    
    for (int i = 0; i < m_param->frameNumThreads; i++)
    {//每个帧级线程对应一个帧编码实例
        if (!m_frameEncoder[i]->init(this, numRows, numCols))
        {
            s265_log(m_param, S265_LOG_ERROR, "Unable to initialize frame encoder, aborting\n");
            m_aborted = true;
        }
    }

    for (int i = 0; i < m_param->frameNumThreads; i++)
    {
        m_frameEncoder[i]->start();//调用基类的start()函数 启动并完成线程的初始化 
        //阻塞等待线程的初始化工作完成
        m_frameEncoder[i]->m_done.wait(); /* 1 对应 duwait for thread to initialize */
    }

    if (m_param->bEmitHRDSEI)
        m_rateControl->initHRD(m_sps);

    if (!m_rateControl->init(m_sps))
        m_aborted = true;
    if (!m_lookahead->create())
        m_aborted = true;

    initRefIdx();

    m_bZeroLatency = !m_param->bframes && !m_param->lookaheadDepth && m_param->frameNumThreads == 1 && m_param->maxSlices == 1;
    m_aborted |= parseLambdaFile(m_param);

    m_encodeStartTime = s265_mdate();

    m_nalList.m_annexB = !!m_param->bAnnexB;

    if (m_param->naluFile)
    {
        m_naluFile = s265_fopen(m_param->naluFile, "r");
        if (!m_naluFile)
        {
            s265_log_file(NULL, S265_LOG_ERROR, "%s file not found or Failed to open\n", m_param->naluFile);
            m_aborted = true;
        }
        else
             m_enableNal = 1;
    }
    else
         m_enableNal = 0;

#if ENABLE_HDR10_PLUS
    if (m_bToneMap)
        m_numCimInfo = m_hdr10plus_api->hdr10plus_json_to_movie_cim(m_param->toneMapFile, m_cim);
#endif
    if (m_param->bDynamicRefine)
    {
        /* Allocate memory for 1 GOP and reuse it for the subsequent GOPs */
        int size = (m_param->keyframeMax + m_param->lookaheadDepth) * m_param->maxCUDepth * S265_REFINE_INTER_LEVELS;
        CHECKED_MALLOC_ZERO(m_variance, uint64_t, size);
        CHECKED_MALLOC_ZERO(m_rdCost, uint64_t, size);
        CHECKED_MALLOC_ZERO(m_trainingCount, uint32_t, size);
        return;
    fail:
        m_aborted = true;
    }
}

void Encoder::stopJobs()
{
    if (m_rateControl)
        m_rateControl->terminate(); // unblock all blocked RC calls

    if (m_lookahead)
        m_lookahead->stopJobs();
    
    for (int i = 0; i < m_param->frameNumThreads; i++)
    {
        if (m_frameEncoder[i])
        {
            m_frameEncoder[i]->getEncodedPicture(m_nalList);
            m_frameEncoder[i]->m_threadActive = false;//设置线程while循环结束条件
            m_frameEncoder[i]->m_enable.trigger();//唤起挂起的线程继续
            m_frameEncoder[i]->stop();//等待线程join(退出)
        }
    }

    if (m_threadPool)
    {
        for (int i = 0; i < m_numPools; i++)
            m_threadPool[i].stopWorkers();
    }
}

int Encoder::copySlicetypePocAndSceneCut(int *slicetype, int *poc, int *sceneCut)
{
    Frame *FramePtr = m_dpb->m_picList.getCurFrame();
    if (FramePtr != NULL)
    {
        *slicetype = FramePtr->m_lowres.sliceType;
        *poc = FramePtr->m_encData->m_slice->m_poc;
        *sceneCut = FramePtr->m_lowres.bScenecut;
    }
    else
    {
        s265_log(NULL, S265_LOG_WARNING, "Frame is still in lookahead pipeline, this API must be called after (poc >= lookaheadDepth + bframes + 2) condition check\n");
        return -1;
    }
    return 0;
}
// 仅提供api调用 编码器本身不会调用
int Encoder::getRefFrameList(PicYuv** l0, PicYuv** l1, int sliceType, int poc, int* pocL0, int* pocL1)
{
    if (!(IS_S265_TYPE_I(sliceType)))
    {
        Frame *framePtr = m_dpb->m_picList.getPOC(poc);
        if (framePtr != NULL)
        {
            for (int j = 0; j < framePtr->m_encData->m_slice->m_numRefIdx[0]; j++)    // check only for --ref=n number of frames.
            {
                if (framePtr->m_encData->m_slice->m_refFrameList[0][j] && framePtr->m_encData->m_slice->m_refFrameList[0][j]->m_reconPic != NULL)
                {
                    int l0POC = framePtr->m_encData->m_slice->m_refFrameList[0][j]->m_poc;
                    pocL0[j] = l0POC;
                    Frame* l0Fp = m_dpb->m_picList.getPOC(l0POC);
                    while (l0Fp->m_reconRowFlag[l0Fp->m_numRows - 1].get() == 0)
                        l0Fp->m_reconRowFlag[l0Fp->m_numRows - 1].waitForChange(0); /* If recon is not ready, current frame encoder has to wait. */
                    l0[j] = l0Fp->m_reconPic;
                }
            }
            for (int j = 0; j < framePtr->m_encData->m_slice->m_numRefIdx[1]; j++)    // check only for --ref=n number of frames.
            {
                if (framePtr->m_encData->m_slice->m_refFrameList[1][j] && framePtr->m_encData->m_slice->m_refFrameList[1][j]->m_reconPic != NULL)
                {
                    int l1POC = framePtr->m_encData->m_slice->m_refFrameList[1][j]->m_poc;
                    pocL1[j] = l1POC;
                    Frame* l1Fp = m_dpb->m_picList.getPOC(l1POC);
                    while (l1Fp->m_reconRowFlag[l1Fp->m_numRows - 1].get() == 0)
                        l1Fp->m_reconRowFlag[l1Fp->m_numRows - 1].waitForChange(0); /* If recon is not ready, current frame encoder has to wait. */
                    l1[j] = l1Fp->m_reconPic;
                }
            }
        }
        else
        {
            s265_log(NULL, S265_LOG_WARNING, "Current frame is not in DPB piclist.\n");
            return 1;
        }
    }
    else
    {
        s265_log(NULL, S265_LOG_ERROR, "I frames does not have a refrence List\n");
        return -1;
    }
    return 0;
}

void Encoder::destroy()
{
#if ENABLE_HDR10_PLUS
    if (m_bToneMap)
        m_hdr10plus_api->hdr10plus_clear_movie(m_cim, m_numCimInfo);
#endif

    if (m_param->bDynamicRefine)
    {
        S265_FREE(m_variance);
        S265_FREE(m_rdCost);
        S265_FREE(m_trainingCount);
    }
    if (m_exportedPic)
    {
        ATOMIC_DEC(&m_exportedPic->m_countRefEncoders);
        m_exportedPic = NULL;
    }

    if (m_param->bHistBasedSceneCut)
    {
        if (m_edgePic != NULL)
        {
            S265_FREE_ZERO(m_edgePic);
        }

        if (m_param->sourceBitDepth != m_param->internalBitDepth)
        {
            S265_FREE_ZERO(m_inputPic[0]);
            if (m_param->internalCsp != S265_CSP_I400)
            {
                for (int i = 1; i < 3; i++)
                {
                    S265_FREE_ZERO(m_inputPic[i]);
                }
            }
        }
    }

    for (int i = 0; i < m_param->frameNumThreads; i++)
    {
        if (m_frameEncoder[i])
        {
            m_frameEncoder[i]->destroy();
            delete m_frameEncoder[i];
        }
    }

    // thread pools can be cleaned up now that all the JobProviders are
    // known to be shutdown
    delete [] m_threadPool;

    if (m_lookahead)
    {
        m_lookahead->destroy();
        delete m_lookahead;
    }

    delete m_dpb;
    if (!m_param->bResetZoneConfig && m_param->rc.zonefileCount)
    {
        delete[] zoneReadCount;
        delete[] zoneWriteCount;
    }
    if (m_rateControl)
    {
        m_rateControl->destroy();
        delete m_rateControl;
    }

    S265_FREE(m_offsetEmergency);

    if (m_latestParam != NULL && m_latestParam != m_param)
    {
        if (m_latestParam->scalingLists != m_param->scalingLists)
            free((char*)m_latestParam->scalingLists);

        PARAM_NS::s265_param_free(m_latestParam);
    }
    if (m_naluFile)
        fclose(m_naluFile);

#ifdef SVT_HEVC
    S265_FREE(m_svtAppData);
#endif
    if (m_param)
    {
        if (m_param->csvfpt)
            fclose(m_param->csvfpt);
        /* release string arguments that were strdup'd */
        free((char*)m_param->rc.lambdaFileName);
        free((char*)m_param->rc.statFileName);
        free((char*)m_param->rc.sharedMemName);
        free((char*)m_param->scalingLists);
        free((char*)m_param->csvfn);
        free((char*)m_param->numaPools);
        free((char*)m_param->masteringDisplayColorVolume);
        free((char*)m_param->toneMapFile);
        PARAM_NS::s265_param_free(m_param);
    }
}

void Encoder::updateVbvPlan(RateControl* rc)
{
    for (int i = 0; i < m_param->frameNumThreads; i++)
    {
        FrameEncoder *encoder = m_frameEncoder[i];
        if (encoder->m_rce.isActive && encoder->m_rce.poc != rc->m_curSlice->m_poc)
        {
            int64_t bits = m_param->rc.bEnableConstVbv ? (int64_t)encoder->m_rce.frameSizePlanned : (int64_t)S265_MAX(encoder->m_rce.frameSizeEstimated, encoder->m_rce.frameSizePlanned);
            rc->m_bufferFill -= bits;
            rc->m_bufferFill = S265_MAX(rc->m_bufferFill, 0);
            rc->m_bufferFill += encoder->m_rce.bufferRate;
            rc->m_bufferFill = S265_MIN(rc->m_bufferFill, rc->m_bufferSize);
            if (rc->m_2pass)
                rc->m_predictedBits += bits;
        }
    }
}

void Encoder::calcRefreshInterval(Frame* frameEnc)
{
    Slice* slice = frameEnc->m_encData->m_slice;
    uint32_t numBlocksInRow = slice->m_sps->numCuInWidth;
    FrameData::PeriodicIR* pir = &frameEnc->m_encData->m_pir;
    if (slice->m_sliceType == I_SLICE)
    {
        pir->framesSinceLastPir = 0;
        m_bQueuedIntraRefresh = 0;
        /* PIR is currently only supported with ref == 1, so any intra frame effectively refreshes
         * the whole frame and counts as an intra refresh. */
        pir->pirEndCol = numBlocksInRow;
    }
    else if (slice->m_sliceType == P_SLICE)
    {
        Frame* ref = frameEnc->m_encData->m_slice->m_refFrameList[0][0];
        int pocdiff = frameEnc->m_poc - ref->m_poc;
        int numPFramesInGOP = m_param->keyframeMax / pocdiff;
        int increment = (numBlocksInRow + numPFramesInGOP - 1) / numPFramesInGOP;
        pir->pirEndCol = ref->m_encData->m_pir.pirEndCol;
        pir->framesSinceLastPir = ref->m_encData->m_pir.framesSinceLastPir + pocdiff;
        if (pir->framesSinceLastPir >= m_param->keyframeMax ||
            (m_bQueuedIntraRefresh && pir->pirEndCol >= numBlocksInRow))
        {
            pir->pirEndCol = 0;
            pir->framesSinceLastPir = 0;
            m_bQueuedIntraRefresh = 0;
            frameEnc->m_lowres.bKeyframe = 1;
        }
        pir->pirStartCol = pir->pirEndCol;
        pir->pirEndCol += increment;
        /* If our intra refresh has reached the right side of the frame, we're done. */
        // 按列进行intra_refresh
        // 可以改为按行进行 intra_refresh
        if (pir->pirEndCol >= numBlocksInRow)
        {
            pir->pirEndCol = numBlocksInRow;
        }
    }
}

void Encoder::copyUserSEIMessages(Frame *frame, const s265_picture* pic_in)
{
    s265_sei_payload toneMap;
    toneMap.payload = NULL;
    int toneMapPayload = 0;

#if ENABLE_HDR10_PLUS
    if (m_bToneMap)
    {
        int currentPOC = m_pocLast;
        if (currentPOC < m_numCimInfo)
        {
            int32_t i = 0;
            toneMap.payloadSize = 0;
            while (m_cim[currentPOC][i] == 0xFF)
                toneMap.payloadSize += m_cim[currentPOC][i++];
            toneMap.payloadSize += m_cim[currentPOC][i];

            toneMap.payload = (uint8_t*)s265_malloc(sizeof(uint8_t) * toneMap.payloadSize);
            toneMap.payloadType = USER_DATA_REGISTERED_ITU_T_T35;
            memcpy(toneMap.payload, &m_cim[currentPOC][i + 1], toneMap.payloadSize);
            toneMapPayload = 1;
        }
    }
#endif
    /* seiMsg will contain SEI messages specified in a fixed file format in POC order.
    * Format of the file : <POC><space><PREFIX><space><NAL UNIT TYPE>/<SEI TYPE><space><SEI Payload> */
    s265_sei_payload seiMsg;
    seiMsg.payload = NULL;
    int userPayload = 0;
    if (m_enableNal)
    {
        readUserSeiFile(seiMsg, m_pocLast);
        if (seiMsg.payload)
            userPayload = 1;;
    }

    int numPayloads = pic_in->userSEI.numPayloads + toneMapPayload + userPayload;
    frame->m_userSEI.numPayloads = numPayloads;

    if (frame->m_userSEI.numPayloads)
    {
        if (!frame->m_userSEI.payloads)
        {
            frame->m_userSEI.payloads = new s265_sei_payload[numPayloads];
            for (int i = 0; i < numPayloads; i++)
                frame->m_userSEI.payloads[i].payload = NULL;
        }
        for (int i = 0; i < numPayloads; i++)
        {
            s265_sei_payload input;
            if ((i == (numPayloads - 1)) && toneMapPayload)
                input = toneMap;
            else if (m_enableNal)
                input = seiMsg;
            else
                input = pic_in->userSEI.payloads[i];

            if (!frame->m_userSEI.payloads[i].payload)
                frame->m_userSEI.payloads[i].payload = new uint8_t[input.payloadSize];
            memcpy(frame->m_userSEI.payloads[i].payload, input.payload, input.payloadSize);
            frame->m_userSEI.payloads[i].payloadSize = input.payloadSize;
            frame->m_userSEI.payloads[i].payloadType = input.payloadType;
        }
        if (toneMap.payload)
            s265_free(toneMap.payload);
        if (seiMsg.payload)
            s265_free(seiMsg.payload);
    }
}

//Find Sum of Squared Difference (SSD) between two pictures
uint64_t Encoder::computeSSD(pixel *fenc, pixel *rec, intptr_t stride, uint32_t width, uint32_t height)
{
    uint64_t ssd = 0;

    if ((width | height) & 3)
    {
        /* Slow Path */
        for (uint32_t y = 0; y < height; y++)
        {
            for (uint32_t x = 0; x < width; x++)
            {
                int diff = (int)(fenc[x] - rec[x]);
                ssd += diff * diff;
            }

            fenc += stride;
            rec += stride;
        }

        return ssd;
    }

    uint32_t y = 0;

    /* Consume rows in ever narrower chunks of height */
    for (int size = BLOCK_64x64; size >= BLOCK_4x4 && y < height; size--)
    {
        uint32_t rowHeight = 1 << (size + 2);

        for (; y + rowHeight <= height; y += rowHeight)
        {
            uint32_t y1, x = 0;

            /* Consume each row using the largest square blocks possible */
            if (size == BLOCK_64x64 && !(stride & 31))
                for (; x + 64 <= width; x += 64)
                    ssd += primitives.cu[BLOCK_64x64].sse_pp(fenc + x, stride, rec + x, stride);

            if (size >= BLOCK_32x32 && !(stride & 15))
                for (; x + 32 <= width; x += 32)
                    for (y1 = 0; y1 + 32 <= rowHeight; y1 += 32)
                        ssd += primitives.cu[BLOCK_32x32].sse_pp(fenc + y1 * stride + x, stride, rec + y1 * stride + x, stride);

            if (size >= BLOCK_16x16)
                for (; x + 16 <= width; x += 16)
                    for (y1 = 0; y1 + 16 <= rowHeight; y1 += 16)
                        ssd += primitives.cu[BLOCK_16x16].sse_pp(fenc + y1 * stride + x, stride, rec + y1 * stride + x, stride);

            if (size >= BLOCK_8x8)
                for (; x + 8 <= width; x += 8)
                    for (y1 = 0; y1 + 8 <= rowHeight; y1 += 8)
                        ssd += primitives.cu[BLOCK_8x8].sse_pp(fenc + y1 * stride + x, stride, rec + y1 * stride + x, stride);

            for (; x + 4 <= width; x += 4)
                for (y1 = 0; y1 + 4 <= rowHeight; y1 += 4)
                    ssd += primitives.cu[BLOCK_4x4].sse_pp(fenc + y1 * stride + x, stride, rec + y1 * stride + x, stride);

            fenc += stride * rowHeight;
            rec += stride * rowHeight;
        }
    }
    return ssd;
}

void Encoder::copyPicture(s265_picture *dest, const s265_picture *src)
{
    dest->poc = src->poc;
    dest->pts = src->pts;
    dest->userSEI = src->userSEI;
    dest->bitDepth = src->bitDepth;
    dest->framesize = src->framesize;
    dest->height = src->height;
    dest->width = src->width;
    dest->colorSpace = src->colorSpace;
    dest->userSEI = src->userSEI;
    dest->rpu.payload = src->rpu.payload;
    dest->picStruct = src->picStruct;
    dest->stride[0] = src->stride[0];
    dest->stride[1] = src->stride[1];
    dest->stride[2] = src->stride[2];
    memcpy(dest->planes[0], src->planes[0], src->framesize * sizeof(char));
    dest->planes[1] = (char*)dest->planes[0] + src->stride[0] * src->height;
    dest->planes[2] = (char*)dest->planes[1] + src->stride[1] * (src->height >> s265_cli_csps[src->colorSpace].height[1]);
}

bool Encoder::computeHistograms(s265_picture *pic)
{
    pixel *src = NULL, *planeV = NULL, *planeU = NULL;
    uint32_t widthC, heightC;
    int hshift, vshift;

    hshift = CHROMA_H_SHIFT(pic->colorSpace);
    vshift = CHROMA_V_SHIFT(pic->colorSpace);
    widthC = pic->width >> hshift;
    heightC = pic->height >> vshift;

    if (pic->bitDepth == S265_DEPTH)
    {
        src = (pixel*)pic->planes[0];
        if (m_param->internalCsp != S265_CSP_I400)
        {
            planeU = (pixel*)pic->planes[1];
            planeV = (pixel*)pic->planes[2];
        }
    }
    else if (pic->bitDepth == 8 && S265_DEPTH > 8)
    {
        int shift = (S265_DEPTH - 8);
        uint8_t *yChar, *uChar, *vChar;

        yChar = (uint8_t*)pic->planes[0];
        primitives.planecopy_cp(yChar, pic->stride[0] / sizeof(*yChar), m_inputPic[0], pic->stride[0] / sizeof(*yChar), pic->width, pic->height, shift);
        src = m_inputPic[0];
        if (m_param->internalCsp != S265_CSP_I400)
        {
            uChar = (uint8_t*)pic->planes[1];
            vChar = (uint8_t*)pic->planes[2];
            primitives.planecopy_cp(uChar, pic->stride[1] / sizeof(*uChar), m_inputPic[1], pic->stride[1] / sizeof(*uChar), widthC, heightC, shift);
            primitives.planecopy_cp(vChar, pic->stride[2] / sizeof(*vChar), m_inputPic[2], pic->stride[2] / sizeof(*vChar), widthC, heightC, shift);
            planeU = m_inputPic[1];
            planeV = m_inputPic[2];
        }
    }
    else
    {
        uint16_t *yShort, *uShort, *vShort;
        /* mask off bits that are supposed to be zero */
        uint16_t mask = (1 << S265_DEPTH) - 1;
        int shift = abs(pic->bitDepth - S265_DEPTH);

        yShort = (uint16_t*)pic->planes[0];
        uShort = (uint16_t*)pic->planes[1];
        vShort = (uint16_t*)pic->planes[2];

        if (pic->bitDepth > S265_DEPTH)
        {
            /* shift right and mask pixels to final size */
            primitives.planecopy_sp(yShort, pic->stride[0] / sizeof(*yShort), m_inputPic[0], pic->stride[0] / sizeof(*yShort), pic->width, pic->height, shift, mask);
            if (m_param->internalCsp != S265_CSP_I400)
            {
                primitives.planecopy_sp(uShort, pic->stride[1] / sizeof(*uShort), m_inputPic[1], pic->stride[1] / sizeof(*uShort), widthC, heightC, shift, mask);
                primitives.planecopy_sp(vShort, pic->stride[2] / sizeof(*vShort), m_inputPic[2], pic->stride[2] / sizeof(*vShort), widthC, heightC, shift, mask);
            }
        }
        else /* Case for (pic.bitDepth < S265_DEPTH) */
        {
            /* shift left and mask pixels to final size */
            primitives.planecopy_sp_shl(yShort, pic->stride[0] / sizeof(*yShort), m_inputPic[0], pic->stride[0] / sizeof(*yShort), pic->width, pic->height, shift, mask);
            if (m_param->internalCsp != S265_CSP_I400)
            {
                primitives.planecopy_sp_shl(uShort, pic->stride[1] / sizeof(*uShort), m_inputPic[1], pic->stride[1] / sizeof(*uShort), widthC, heightC, shift, mask);
                primitives.planecopy_sp_shl(vShort, pic->stride[2] / sizeof(*vShort), m_inputPic[2], pic->stride[2] / sizeof(*vShort), widthC, heightC, shift, mask);
            }
        }

        src = m_inputPic[0];
        planeU = m_inputPic[1];
        planeV = m_inputPic[2];
    }

    size_t bufSize = sizeof(pixel) * m_planeSizes[0];
    memset(m_edgePic, 0, bufSize);

    if (!computeEdge(m_edgePic, src, NULL, pic->width, pic->height, pic->width, false, 1))
    {
        s265_log(m_param, S265_LOG_ERROR, "Failed to compute edge!");
        return false;
    }

    pixel pixelVal;
    int32_t *edgeHist = m_curEdgeHist;
    memset(edgeHist, 0, EDGE_BINS * sizeof(int32_t));
    for (uint32_t i = 0; i < m_planeSizes[0]; i++)
    {
        if (m_edgePic[i])
            edgeHist[1]++;
        else
            edgeHist[0]++;
    }

    /* Y Histogram Calculation */
    int32_t *yHist = m_curYUVHist[0];
    memset(yHist, 0, HISTOGRAM_BINS * sizeof(int32_t));
    for (uint32_t i = 0; i < m_planeSizes[0]; i++)
    {
        pixelVal = src[i];
        yHist[pixelVal]++;
    }

    if (pic->colorSpace != S265_CSP_I400)
    {
        /* U Histogram Calculation */
        int32_t *uHist = m_curYUVHist[1];
        memset(uHist, 0, sizeof(m_curYUVHist[1]));
        for (uint32_t i = 0; i < m_planeSizes[1]; i++)
        {
            pixelVal = planeU[i];
            uHist[pixelVal]++;
        }

        /* V Histogram Calculation */
        pixelVal = 0;
        int32_t *vHist = m_curYUVHist[2];
        memset(vHist, 0, sizeof(m_curYUVHist[2]));
        for (uint32_t i = 0; i < m_planeSizes[2]; i++)
        {
            pixelVal = planeV[i];
            vHist[pixelVal]++;
        }
    }
    return true;
}

// 计算柱状图之间的 sad (the current and prev pictures)
void Encoder::computeHistogramSAD(double *normalizedMaxUVSad, double *normalizedEdgeSad, int curPoc)
{

    if (curPoc == 0)
    {   /* first frame is scenecut by default no sad computation for the same. */
        *normalizedMaxUVSad = 0.0;
        *normalizedEdgeSad = 0.0;
    }
    else
    {
        /* compute sum of absolute differences of histogram bins of chroma and luma edge response between the current and prev pictures. */
        int32_t edgeHistSad = 0;
        int32_t uHistSad = 0;
        int32_t vHistSad = 0;
        double normalizedUSad = 0.0;
        double normalizedVSad = 0.0;

        for (int j = 0; j < HISTOGRAM_BINS; j++)
        {
            if (j < 2)
            {
                edgeHistSad += abs(m_curEdgeHist[j] - m_prevEdgeHist[j]);
            }
            uHistSad += abs(m_curYUVHist[1][j] - m_prevYUVHist[1][j]);
            vHistSad += abs(m_curYUVHist[2][j] - m_prevYUVHist[2][j]);
        }
        *normalizedEdgeSad = normalizeRange(edgeHistSad, 0, 2 * m_planeSizes[0], 0.0, 1.0);
        normalizedUSad = normalizeRange(uHistSad, 0, 2 * m_planeSizes[1], 0.0, 1.0);
        normalizedVSad = normalizeRange(vHistSad, 0, 2 * m_planeSizes[2], 0.0, 1.0);
        *normalizedMaxUVSad = s265_max(normalizedUSad, normalizedVSad);
    }

    /* store histograms of previous frame for reference */
    memcpy(m_prevEdgeHist, m_curEdgeHist, sizeof(m_curEdgeHist));
    memcpy(m_prevYUVHist, m_curYUVHist, sizeof(m_curYUVHist));
}

double Encoder::normalizeRange(int32_t value, int32_t minValue, int32_t maxValue, double rangeStart, double rangeEnd)
{
    return (double)(value - minValue) * (rangeEnd - rangeStart) / (maxValue - minValue) + rangeStart;
}

void Encoder::findSceneCuts(s265_picture *pic, bool& bDup, double maxUVSad, double edgeSad, bool& isMaxThres, bool& isHardSC)
{
    double minEdgeT = m_edgeHistThreshold * MIN_EDGE_FACTOR;
    double minChromaT = minEdgeT * SCENECUT_CHROMA_FACTOR;
    double maxEdgeT = m_edgeHistThreshold * MAX_EDGE_FACTOR;
    double maxChromaT = maxEdgeT * SCENECUT_CHROMA_FACTOR;
    pic->frameData.bScenecut = false;

    if (pic->poc == 0)
    {
        /* for first frame */
        pic->frameData.bScenecut = false;
        bDup = false;
    }
    else
    {
        if (edgeSad == 0.0 && maxUVSad == 0.0)
        {
            bDup = true;
        }
        else if (edgeSad < minEdgeT && maxUVSad < minChromaT)
        {
            pic->frameData.bScenecut = false;
        }
        else if (edgeSad > maxEdgeT && maxUVSad > maxChromaT)
        {
            pic->frameData.bScenecut = true;
            isMaxThres = true;
            isHardSC = true;
        }
        else if (edgeSad > m_scaledEdgeThreshold || maxUVSad >= m_scaledChromaThreshold
                 || (edgeSad > m_edgeHistThreshold && maxUVSad >= m_chromaHistThreshold))
        {
            pic->frameData.bScenecut = true;
            bDup = false;
            if (edgeSad > m_scaledEdgeThreshold || maxUVSad >= m_scaledChromaThreshold)
                isHardSC = true;
        }
    }
}

/**
 * Feed one new input frame into the encoder, get one frame out. If pic_in is
 * NULL, a flush condition is implied and pic_in must be NULL for all subsequent
 * calls for this encoder instance.
 *
 * pic_in  input original YUV picture or NULL
 * pic_out pointer to reconstructed picture struct
 *
 * returns 0 if no frames are currently available for output
 *         1 if frame was output, m_nalList contains access unit
 *         negative on malloc error or abort */

//区分大小写 encode 不是类Encoder的构造函数
int Encoder::encode(const x265_picture* pic_in, x265_picture* pic_out)
{
#if CHECKED_BUILD || _DEBUG
    if (g_checkFailures)
    {
        s265_log(m_param, S265_LOG_ERROR, "encoder aborting because of internal error\n");
        return -1;
    }
#endif
    if (m_aborted)
        return -1;

    const s265_picture* inputPic = NULL;
    bool bdropFrame = false;
    bool isMaxThres = false;
    bool isHardSC = false;

    if (m_exportedPic)
    {
        ATOMIC_DEC(&m_exportedPic->m_countRefEncoders);
        m_exportedPic = NULL;
        m_dpb->recycleUnreferenced();
    }
    if ((pic_in && (!m_param->chunkEnd || (m_encodedFrameNum < m_param->chunkEnd))))
    {
        if (m_param->bHistBasedSceneCut && pic_in)
        {
            s265_picture *pic = (s265_picture *) pic_in;

            if (pic->poc == 0)
            {
                /* for entire encode compute the chroma plane sizes only once */
                for (int i = 1; i < s265_cli_csps[m_param->internalCsp].planes; i++)
                    m_planeSizes[i] = (pic->width >> s265_cli_csps[m_param->internalCsp].width[i]) * (pic->height >> s265_cli_csps[m_param->internalCsp].height[i]);
            }

            if (computeHistograms(pic))
            {
                double maxUVSad = 0.0, edgeSad = 0.0;
                computeHistogramSAD(&maxUVSad, &edgeSad, pic_in->poc);
                findSceneCuts(pic, bdropFrame, maxUVSad, edgeSad, isMaxThres, isHardSC);
            }
        }


        if (m_latestParam->forceFlush == 1)
        {
            m_lookahead->setLookaheadQueue();
            m_latestParam->forceFlush = 0;
        }
        if (m_latestParam->forceFlush == 2)
        {
            m_lookahead->m_filled = false;
            m_latestParam->forceFlush = 0;
        }

        if (pic_in->bitDepth < 8 || pic_in->bitDepth > 16)
        {
            s265_log(m_param, S265_LOG_ERROR, "Input bit depth (%d) must be between 8 and 16\n",
                        pic_in->bitDepth);
            return -1;
        }

        inputPic = pic_in;

        Frame *inFrame;
        s265_param *p = (m_reconfigure || m_reconfigureRc) ? m_latestParam : m_param;
        if (m_dpb->m_freeList.empty())
        {// 内存池技术，freeList是一个Frame的pool 如果没有可用的则new一个
            inFrame = new Frame;
            //Frame主要包括编码完成的数据，重建帧的YUV数据和要编码的YUV数据
            inFrame->m_encodeStartTime = s265_mdate();
            if (inFrame->create(p, inputPic->quantOffsets))
            {
                /* the first PicYuv created is asked to generate the CU and block unit offset
                 * arrays which are then shared with all subsequent PicYuv (orig and recon) 
                 * allocated by this top level encoder */
                if (m_sps.cuOffsetY)
                {
                    inFrame->m_fencPic->m_cuOffsetY = m_sps.cuOffsetY;
                    inFrame->m_fencPic->m_buOffsetY = m_sps.buOffsetY;
                    if (m_param->internalCsp != S265_CSP_I400)
                    {
                        inFrame->m_fencPic->m_cuOffsetC = m_sps.cuOffsetC;
                        inFrame->m_fencPic->m_buOffsetC = m_sps.buOffsetC;
                    }
                }
                else
                {
                    if (!inFrame->m_fencPic->createOffsets(m_sps))
                    {
                        m_aborted = true;
                        s265_log(m_param, S265_LOG_ERROR, "memory allocation failure, aborting encode\n");
                        inFrame->destroy();
                        delete inFrame;
                        return -1;
                    }
                    else
                    {
                        m_sps.cuOffsetY = inFrame->m_fencPic->m_cuOffsetY;
                        m_sps.buOffsetY = inFrame->m_fencPic->m_buOffsetY;
                        if (m_param->internalCsp != S265_CSP_I400)
                        {
                            m_sps.cuOffsetC = inFrame->m_fencPic->m_cuOffsetC;
                            m_sps.cuOffsetY = inFrame->m_fencPic->m_cuOffsetY;
                            m_sps.buOffsetC = inFrame->m_fencPic->m_buOffsetC;
                            m_sps.buOffsetY = inFrame->m_fencPic->m_buOffsetY;
                        }
                    }
                }
                if (m_param->recursionSkipMode == EDGE_BASED_RSKIP && m_param->bHistBasedSceneCut)
                {
                    pixel* src = m_edgePic;
                    primitives.planecopy_pp_shr(src, inFrame->m_fencPic->m_picWidth, inFrame->m_edgeBitPic, inFrame->m_fencPic->m_stride,
                        inFrame->m_fencPic->m_picWidth, inFrame->m_fencPic->m_picHeight, 0);
                }
            }
            else
            {
                m_aborted = true;
                s265_log(m_param, S265_LOG_ERROR, "memory allocation failure, aborting encode\n");
                inFrame->destroy();
                delete inFrame;
                return -1;
            }
        }
        else
        {
            inFrame = m_dpb->m_freeList.popBack();// 内存池技术，如果有可用的直接pop一个frame
            inFrame->m_encodeStartTime = s265_mdate();
            /* Set lowres scencut and satdCost here to aovid overwriting ANALYSIS_READ
               decision by lowres init*/
            inFrame->m_lowres.bScenecut = false;
            inFrame->m_lowres.satdCost = (int64_t)-1;
            inFrame->m_lowresInit = false;
            inFrame->m_isInsideWindow = 0;
        }

        /* Copy input picture into a Frame and PicYuv, send to lookahead */
        inFrame->m_fencPic->copyFromPicture(*inputPic, *m_param, m_sps.conformanceWindow.rightOffset, m_sps.conformanceWindow.bottomOffset);
        if ((m_param->bEnablePsnr || m_param->bEnableSsim) && m_param->mctf.enable)
        {
            inFrame->m_filteredPic->copyFromPicture(*inputPic, *m_param, m_sps.conformanceWindow.rightOffset, m_sps.conformanceWindow.bottomOffset);
        }
        inFrame->m_poc       = ++m_pocLast;//输入帧序号自加
        inFrame->m_userData  = inputPic->userData;
        inFrame->m_pts       = inputPic->pts;
        if (m_param->bHistBasedSceneCut)
        {
            inFrame->m_lowres.bScenecut = (inputPic->frameData.bScenecut == 1) ? true : false;
            inFrame->m_lowres.m_bIsMaxThres = isMaxThres;
            if (m_param->radl && m_param->keyframeMax != m_param->keyframeMin)
                inFrame->m_lowres.m_bIsHardScenecut = isHardSC;
        }

        if ((m_param->bEnableSceneCutAwareQp & BACKWARD) && m_param->rc.bStatRead)
        {
            RateControlEntry * rcEntry = NULL;
            rcEntry = &(m_rateControl->m_rce2Pass[inFrame->m_poc]);
            if(rcEntry->scenecut)
            {
                int backwardWindow = S265_MIN(int((m_param->bwdScenecutWindow / 1000.0) * (m_param->fpsNum / m_param->fpsDenom)), p->lookaheadDepth);
                for (int i = 1; i <= backwardWindow; i++)
                {
                    int frameNum = inFrame->m_poc - i;
                    Frame * frame = m_lookahead->m_inputQueue.getPOC(frameNum);
                    if (frame)
                        frame->m_isInsideWindow = BACKWARD_WINDOW;
                }
            }
        }
        inFrame->m_forceqp   = inputPic->forceqp;
        inFrame->m_param     = (m_reconfigure || m_reconfigureRc) ? m_latestParam : m_param;
        inFrame->m_picStruct = inputPic->picStruct;

        copyUserSEIMessages(inFrame, inputPic);

        /*Copy Dolby Vision RPU from inputPic to frame*/
        if (inputPic->rpu.payloadSize)
        {
            inFrame->m_rpu.payloadSize = inputPic->rpu.payloadSize;
            inFrame->m_rpu.payload = new uint8_t[inputPic->rpu.payloadSize];
            memcpy(inFrame->m_rpu.payload, inputPic->rpu.payload, inputPic->rpu.payloadSize);
        }

        if (inputPic->quantOffsets != NULL)
        {
            int cuCount;
            if (m_param->rc.qgSize == 8)
                //每个8x8 有自己的qp_deltal
                cuCount = inFrame->m_lowres.maxBlocksInRowFullRes * inFrame->m_lowres.maxBlocksInColFullRes;
            else
                //每个16x16 有自己的qp_deltal
                cuCount = inFrame->m_lowres.maxBlocksInRow * inFrame->m_lowres.maxBlocksInCol;
            memcpy(inFrame->m_quantOffsets, inputPic->quantOffsets, cuCount * sizeof(float));
        }

        if (m_pocLast == 0)
            m_firstPts = inFrame->m_pts;// 首帧pts 记录
        if (m_bframeDelay && m_pocLast == m_bframeDelay)//当输入帧等与bframe_delay 时,记录delaytime
            m_bframeDelayTime = inFrame->m_pts - m_firstPts;

        /* Encoder holds a reference count until stats collection is finished */
        //记录引用了改帧的encoder数
        ATOMIC_INC(&inFrame->m_countRefEncoders);

        if ((m_param->rc.aqMode || m_param->bEnableWeightedPred || m_param->bEnableWeightedBiPred) &&
            (m_param->rc.cuTree && m_param->rc.bStatRead))
        {
            if (!m_rateControl->cuTreeReadFor2Pass(inFrame))
            {
                m_aborted = 1;
                return -1;
            }
        }

        /* Use the frame types from the first pass, if available */
        int sliceType = (m_param->rc.bStatRead) ? m_rateControl->rateControlSliceType(inFrame->m_poc) : inputPic->sliceType;

        if (m_param->bUseRcStats && inputPic->rcData)
        {
            RcStats* rc = (RcStats*)inputPic->rcData;
            m_rateControl->m_accumPQp = rc->cumulativePQp;
            m_rateControl->m_accumPNorm = rc->cumulativePNorm;
            m_rateControl->m_isNextGop = true;
            for (int j = 0; j < 3; j++)
                m_rateControl->m_lastQScaleFor[j] = rc->lastQScaleFor[j];
            m_rateControl->m_wantedBitsWindow = rc->wantedBitsWindow;
            m_rateControl->m_cplxrSum = rc->cplxrSum;
            m_rateControl->m_totalBits = rc->totalBits;
            m_rateControl->m_encodedBits = rc->encodedBits;
            m_rateControl->m_shortTermCplxSum = rc->shortTermCplxSum;
            m_rateControl->m_shortTermCplxCount = rc->shortTermCplxCount;
            if (m_rateControl->m_isVbv)
            {
                m_rateControl->m_bufferFillFinal = rc->bufferFillFinal;
                for (int i = 0; i < 4; i++)
                {
                    m_rateControl->m_pred[i].coeff = rc->coeff[i];
                    m_rateControl->m_pred[i].count = rc->count[i];
                    m_rateControl->m_pred[i].offset = rc->offset[i];
                }
            }
            m_param->bUseRcStats = 0;
        }

        if (m_reconfigureRc)
            inFrame->m_reconfigureRc = true;
        // 添加到inputQeune 用于预分析,如果帧数满了会自动触发lookahead线程池里面的线程worker
        m_lookahead->addPicture(*inFrame, sliceType);
        m_numDelayedPic++;
    }
    else if (m_latestParam->forceFlush == 2)
        m_lookahead->m_filled = true;
    else
        m_lookahead->flush();
    
    // 循环启用帧级编码器
    FrameEncoder *curEncoder = m_frameEncoder[m_curEncoder]; // get     当前的帧级编码器
    m_curEncoder = (m_curEncoder + 1) % m_param->frameNumThreads;// 记录用于编码下一帧的encoder对象在数组中的位置

    int ret = 0;

    /* Normal operation is to wait for the current frame encoder to complete its current frame
     * and then to give it a new frame to work on.  In zero-latency mode, we must encode this
     * input picture before returning so the order must be reversed. This do/while() loop allows
     * us to alternate the order of the calls without ugly code replication */
    Frame* outFrame = NULL;
    Frame* frameEnc = NULL;
    int pass = 0;
    do
    {
        /* getEncodedPicture() should block until the FrameEncoder has completed
         * encoding the frame.  This is how back-pressure through the API is
         * accomplished when the encoder is full */
        if (!m_bZeroLatency || pass)// 在输入下一帧前,先取走一个output
            outFrame = curEncoder->getEncodedPicture(m_nalList);// 如果先前有帧编码，则需要等待其编码完成, 这里面会block 或者说等待
        if (outFrame)
        {//如果成功取到输出
            Slice *slice = outFrame->m_encData->m_slice;
            s265_frame_stats* frameData = NULL;

            if (pic_out)//大部分情况下,pic_out 为null,只有在某些特定的参数配置如 输出recon/analysis 复用等时,才启用
            {
                PicYuv *recpic = outFrame->m_reconPic;
                pic_out->poc = slice->m_poc;
                pic_out->bitDepth = S265_DEPTH;
                pic_out->userData = outFrame->m_userData;
                pic_out->colorSpace = m_param->internalCsp;
                frameData = &(pic_out->frameData);

                pic_out->pts = outFrame->m_pts;
                pic_out->dts = outFrame->m_dts;
                pic_out->reorderedPts = outFrame->m_reorderedPts;
                pic_out->sliceType = outFrame->m_lowres.sliceType;
                pic_out->planes[0] = recpic->m_picOrg[0];
                pic_out->stride[0] = (int)(recpic->m_stride * sizeof(pixel));
                if (m_param->internalCsp != S265_CSP_I400)
                {
                    pic_out->planes[1] = recpic->m_picOrg[1];
                    pic_out->stride[1] = (int)(recpic->m_strideC * sizeof(pixel));
                    pic_out->planes[2] = recpic->m_picOrg[2];
                    pic_out->stride[2] = (int)(recpic->m_strideC * sizeof(pixel));
                }
            }
            if (m_param->internalCsp == S265_CSP_I400)
            {
                if (slice->m_sliceType == P_SLICE)
                {
                    if (slice->m_weightPredTable[0][0][0].wtPresent)
                        m_numLumaWPFrames++;
                }
                else if (slice->m_sliceType == B_SLICE)
                {
                    bool bLuma = false;
                    for (int l = 0; l < 2; l++)
                    {
                        if (slice->m_weightPredTable[l][0][0].wtPresent)
                            bLuma = true;
                    }
                    if (bLuma)
                        m_numLumaWPBiFrames++;
                }
            }
            else
            {   //统计加权预测的的frame 数量
                if (slice->m_sliceType == P_SLICE)
                {
                    if (slice->m_weightPredTable[0][0][0].wtPresent)
                        m_numLumaWPFrames++;
                    if (slice->m_weightPredTable[0][0][1].wtPresent ||
                        slice->m_weightPredTable[0][0][2].wtPresent)
                        m_numChromaWPFrames++;
                }
                else if (slice->m_sliceType == B_SLICE)
                {
                    bool bLuma = false, bChroma = false;
                    for (int l = 0; l < 2; l++)
                    {
                        if (slice->m_weightPredTable[l][0][0].wtPresent)
                            bLuma = true;
                        if (slice->m_weightPredTable[l][0][1].wtPresent ||
                            slice->m_weightPredTable[l][0][2].wtPresent)
                            bChroma = true;
                    }

                    if (bLuma)
                        m_numLumaWPBiFrames++;
                    if (bChroma)
                        m_numChromaWPBiFrames++;
                }
            }
            if (m_aborted)
                return -1;

            if ((m_outputCount + 1)  >= m_param->chunkStart)
                finishFrameStats(outFrame, curEncoder, frameData, m_pocLast);

            /* Write RateControl Frame level stats in multipass encodes */
            if (m_param->rc.bStatWrite)
                if (m_rateControl->writeRateControlFrameStats(outFrame, &curEncoder->m_rce))
                    m_aborted = true;
            if (pic_out)
            { 
                /* m_rcData is allocated for every frame */
                pic_out->rcData = outFrame->m_rcData;
                outFrame->m_rcData->qpaRc = outFrame->m_encData->m_avgQpRc;
                outFrame->m_rcData->qRceq = curEncoder->m_rce.qRceq;
                outFrame->m_rcData->qpNoVbv = curEncoder->m_rce.qpNoVbv;
                outFrame->m_rcData->coeffBits = outFrame->m_encData->m_frameStats.coeffBits;
                outFrame->m_rcData->miscBits = outFrame->m_encData->m_frameStats.miscBits;
                outFrame->m_rcData->mvBits = outFrame->m_encData->m_frameStats.mvBits;
                outFrame->m_rcData->qScale = outFrame->m_rcData->newQScale = s265_qp2qScale(outFrame->m_encData->m_avgQpRc);
                outFrame->m_rcData->poc = curEncoder->m_rce.poc;
                outFrame->m_rcData->encodeOrder = curEncoder->m_rce.encodeOrder;
                outFrame->m_rcData->sliceType = curEncoder->m_rce.sliceType;
                outFrame->m_rcData->keptAsRef = curEncoder->m_rce.sliceType == B_SLICE && !IS_REFERENCED(outFrame) ? 0 : 1;
                outFrame->m_rcData->qpAq = outFrame->m_encData->m_avgQpAq;
                outFrame->m_rcData->iCuCount = outFrame->m_encData->m_frameStats.percent8x8Intra * m_rateControl->m_ncu;
                outFrame->m_rcData->pCuCount = outFrame->m_encData->m_frameStats.percent8x8Inter * m_rateControl->m_ncu;
                outFrame->m_rcData->skipCuCount = outFrame->m_encData->m_frameStats.percent8x8Skip  * m_rateControl->m_ncu;
                outFrame->m_rcData->currentSatd = curEncoder->m_rce.coeffBits;
            }

            /* Allow this frame to be recycled if no frame encoders are using it for reference */
            if (!pic_out)
            {
                ATOMIC_DEC(&outFrame->m_countRefEncoders);
                m_dpb->recycleUnreferenced();
            }
            else
                m_exportedPic = outFrame;
            
            m_outputCount++;
            if (m_param->chunkEnd == m_outputCount)
                m_numDelayedPic = 0;
            else
                m_numDelayedPic--;

            ret = 1;
        }

        /* pop a single frame from decided list, then provide to frame encoder
         * curEncoder is guaranteed to be idle at this point */
        if (!pass)
            frameEnc = m_lookahead->getDecidedPicture();// 从已经决策好帧类型的list里面取出来一帧
        //如果可以取到帧,则进一步送给编码器编码
        if (frameEnc && !pass && (!m_param->chunkEnd || (m_encodedFrameNum < m_param->chunkEnd)))
        {
            if ((m_param->bEnableSceneCutAwareQp & FORWARD) && m_param->rc.bStatRead)
            {
                RateControlEntry * rcEntry;
                rcEntry = &(m_rateControl->m_rce2Pass[frameEnc->m_poc]);

                if (rcEntry->scenecut)
                {
                    if (m_rateControl->m_lastScenecut == -1)
                        m_rateControl->m_lastScenecut = frameEnc->m_poc;
                    else
                    {
                        int maxWindowSize = int((m_param->fwdScenecutWindow / 1000.0) * (m_param->fpsNum / m_param->fpsDenom) + 0.5);
                        if (frameEnc->m_poc > (m_rateControl->m_lastScenecut + maxWindowSize))
                            m_rateControl->m_lastScenecut = frameEnc->m_poc;
                    }
                }
            }

            if (m_param->bResetZoneConfig)
            {
                for (int i = 0; i < m_param->rc.zonefileCount; i++)
                {
                    if (m_param->rc.zones[i].startFrame == frameEnc->m_poc)
                        s265_encoder_reconfig(this, m_param->rc.zones[i].zoneParam);
                }
            }

            if (frameEnc->m_reconfigureRc && m_reconfigureRc)
            {
                s265_copy_params(m_param, m_latestParam);
                m_rateControl->reconfigureRC();
                m_reconfigureRc = false;
            }
            if (frameEnc->m_reconfigureRc && !m_reconfigureRc)
                frameEnc->m_reconfigureRc = false;
            if (curEncoder->m_reconfigure)
            {
                /* One round robin cycle of FE reconfigure is complete */
                /* Safe to copy m_latestParam to Encoder::m_param, encoder reconfigure complete */
                for (int frameEncId = 0; frameEncId < m_param->frameNumThreads; frameEncId++)
                    m_frameEncoder[frameEncId]->m_reconfigure = false;
                s265_copy_params(m_param, m_latestParam);
                m_reconfigure = false;
            }

            /* Initiate reconfigure for this FE if necessary */
            curEncoder->m_param = m_reconfigure ? m_latestParam : m_param;
            curEncoder->m_reconfigure = m_reconfigure;

            /* give this frame a FrameData instance before encoding */
            if (m_dpb->m_frameDataFreeList)//如果dpb中 m_encData链表不为空
            {
                frameEnc->m_encData = m_dpb->m_frameDataFreeList;//取走当前的framedata(表头)给 当前需要编码的frameEnc的->m_encData
                m_dpb->m_frameDataFreeList = m_dpb->m_frameDataFreeList->m_freeListNext;//表头指向后一个
                frameEnc->reinit(m_sps);
                frameEnc->m_param = m_reconfigure ? m_latestParam : m_param;
                frameEnc->m_encData->m_param = m_reconfigure ? m_latestParam : m_param;
            }
            else
            {   // 否则如果dpb中 m_encData链表为空，new 一个, 后面会由dpb中m_encData链表管理回收
                frameEnc->allocEncodeData(m_reconfigure ? m_latestParam : m_param, m_sps);
                Slice* slice = frameEnc->m_encData->m_slice;
                slice->m_sps = &m_sps;
                slice->m_pps = &m_pps;
                slice->m_param = m_param;
                slice->m_maxNumMergeCand = m_param->maxNumMergeCand;
                slice->m_endCUAddr = slice->realEndAddress(m_sps.numCUsInFrame * m_param->num4x4Partitions);
            }
            if (m_param->searchMethod == S265_SEA && frameEnc->m_lowres.sliceType != S265_TYPE_B)
            {
                int padX = m_param->maxCUSize + 32;
                int padY = m_param->maxCUSize + 16;
                //int padx = frameEnc->m_fencPic->m_lumaMarginX;
                //int pady = frameEnc->m_fencPic->m_lumaMarginY;
                uint32_t numCuInHeight = (frameEnc->m_encData->m_reconPic->m_picHeight + m_param->maxCUSize - 1) / m_param->maxCUSize;
                int maxHeight = numCuInHeight * m_param->maxCUSize;
                for (int i = 0; i < INTEGRAL_PLANE_NUM; i++)
                {
                    frameEnc->m_encData->m_meBuffer[i] = S265_MALLOC(uint32_t, frameEnc->m_reconPic->m_stride * (maxHeight + (2 * padY)));
                    if (frameEnc->m_encData->m_meBuffer[i])
                    {
                        memset(frameEnc->m_encData->m_meBuffer[i], 0, sizeof(uint32_t)* frameEnc->m_reconPic->m_stride * (maxHeight + (2 * padY)));
                        frameEnc->m_encData->m_meIntegral[i] = frameEnc->m_encData->m_meBuffer[i] + frameEnc->m_encData->m_reconPic->m_stride * padY + padX;
                    }
                    else
                        s265_log(m_param, S265_LOG_ERROR, "SEA motion search: POC %d Integral buffer[%d] unallocated\n", frameEnc->m_poc, i);
                }
            }

            if (m_param->bOptQpPPS && frameEnc->m_lowres.bKeyframe && m_param->bRepeatHeaders)
            {
                ScopedLock qpLock(m_sliceQpLock);
                if (m_iFrameNum > 0)
                {
                    //Search the least cost
                    int64_t iLeastCost = m_iBitsCostSum[0];
                    int iLeastId = 0;
                    for (int i = 1; i < QP_MAX_MAX + 1; i++)
                    {
                        if (iLeastCost > m_iBitsCostSum[i])
                        {
                            iLeastId = i;
                            iLeastCost = m_iBitsCostSum[i];
                        }
                    }
                    /* If last slice Qp is close to (26 + m_iPPSQpMinus26) or outputs is all I-frame video,
                       we don't need to change m_iPPSQpMinus26. */
                    if (m_iFrameNum > 1)
                        m_iPPSQpMinus26 = (iLeastId + 1) - 26;
                    m_iFrameNum = 0;
                }

                for (int i = 0; i < QP_MAX_MAX + 1; i++)
                    m_iBitsCostSum[i] = 0;
            }

            frameEnc->m_encData->m_slice->m_iPPSQpMinus26 = m_iPPSQpMinus26;
            frameEnc->m_encData->m_slice->numRefIdxDefault[0] = m_pps.numRefIdxDefault[0];
            frameEnc->m_encData->m_slice->numRefIdxDefault[1] = m_pps.numRefIdxDefault[1];
            frameEnc->m_encData->m_slice->m_iNumRPSInSPS = m_sps.spsrpsNum;

            curEncoder->m_rce.encodeOrder = frameEnc->m_encodeOrder = m_encodedFrameNum++;// 每送一个已经决策好了的帧给编码编码，编码num++

            if (m_bframeDelay)
            {
                //由pts 生成dts
                int64_t *prevReorderedPts = m_prevReorderedPts;
                frameEnc->m_dts = m_encodedFrameNum > m_bframeDelay
                    ? prevReorderedPts[(m_encodedFrameNum - m_bframeDelay) % m_bframeDelay]
                    : frameEnc->m_reorderedPts - m_bframeDelayTime;
                prevReorderedPts[m_encodedFrameNum % m_bframeDelay] = frameEnc->m_reorderedPts;
            }
            else
                frameEnc->m_dts = frameEnc->m_reorderedPts;

            /* determine references, setup RPS, etc */
            /*参考帧列表建立，排序，rps 构建 等*/
            m_dpb->prepareEncode(frameEnc);
            if (!!m_param->selectiveSAO)
            {
                Slice* slice = frameEnc->m_encData->m_slice;
                slice->m_bUseSao = curEncoder->m_frameFilter.m_useSao = 1;
                switch (m_param->selectiveSAO)
                {
                case 3: if (!IS_REFERENCED(frameEnc))// 3: 非参考帧不做sao
                            slice->m_bUseSao = curEncoder->m_frameFilter.m_useSao = 0;
                        break;
                        //2: 所有B帧不做sao
                case 2: if (!!m_param->bframes && slice->m_sliceType == B_SLICE)
                            slice->m_bUseSao = curEncoder->m_frameFilter.m_useSao = 0;
                        break;
                        //1: 除I帧以外的帧不做sao
                case 1: if (slice->m_sliceType != I_SLICE)
                            slice->m_bUseSao = curEncoder->m_frameFilter.m_useSao = 0;
                        break;
                       //4: 所有帧都做sao
                }
            }
            else // 0 所有帧都不做SAO
            {
                Slice* slice = frameEnc->m_encData->m_slice;
                slice->m_bUseSao = curEncoder->m_frameFilter.m_useSao = 0;
            }

            if (m_param->rc.rateControlMode != S265_RC_CQP)
                m_lookahead->getEstimatedPictureCost(frameEnc);
            if (m_param->bIntraRefresh)
                 calcRefreshInterval(frameEnc);// intraRefresh 列刷新位置计算

            /* Allow FrameEncoder::compressFrame() to start in the frame encoder thread */
            //启动编码器编码,触发 一个帧级编码线程 发起compressFrame 任务
            if (!curEncoder->startCompressFrame(frameEnc))
                m_aborted = true;
        }
        else if (m_encodedFrameNum)//从lookahead 中取不到了,但是先前已经有送编码帧进行编码了,代表实际需要编码的帧输入结束了
            m_rateControl->setFinalFrameCount(m_encodedFrameNum);
    }
    while (m_bZeroLatency && ++pass < 2);//如果是zerolatency,需要继续取帧循环取输出

    return ret;
}

int Encoder::reconfigureParam(s265_param* encParam, s265_param* param)
{
    if (isReconfigureRc(encParam, param) && !param->rc.zonefileCount)
    {
        /* VBV can't be turned ON if it wasn't ON to begin with and can't be turned OFF if it was ON to begin with*/
        if (param->rc.vbvMaxBitrate > 0 && param->rc.vbvBufferSize > 0 &&
            encParam->rc.vbvMaxBitrate > 0 && encParam->rc.vbvBufferSize > 0)
        {
            m_reconfigureRc |= encParam->rc.vbvMaxBitrate != param->rc.vbvMaxBitrate;
            m_reconfigureRc |= encParam->rc.vbvBufferSize != param->rc.vbvBufferSize;
            if (m_reconfigureRc && m_param->bEmitHRDSEI)
                s265_log(m_param, S265_LOG_WARNING, "VBV parameters cannot be changed when HRD is in use.\n");
            else
            {
                encParam->rc.vbvMaxBitrate = param->rc.vbvMaxBitrate;
                encParam->rc.vbvBufferSize = param->rc.vbvBufferSize;
            }
        }
        m_reconfigureRc |= encParam->rc.bitrate != param->rc.bitrate;
        encParam->rc.bitrate = param->rc.bitrate;
        m_reconfigureRc |= encParam->rc.rfConstant != param->rc.rfConstant;
        encParam->rc.rfConstant = param->rc.rfConstant;
    }
    else
    {
        encParam->maxNumReferences = param->maxNumReferences; // never uses more refs than specified in stream headers
        encParam->bEnableFastIntra = param->bEnableFastIntra;
        encParam->bEnableEarlySkip = param->bEnableEarlySkip;
        encParam->recursionSkipMode = param->recursionSkipMode;
        encParam->searchMethod = param->searchMethod;
        /* Scratch buffer prevents me_range from being increased for esa/tesa */
        if (param->searchRange < encParam->searchRange)
            encParam->searchRange = param->searchRange;
        /* We can't switch out of subme=0 during encoding. */
        if (encParam->subpelRefine)
            encParam->subpelRefine = param->subpelRefine;
        encParam->rdoqLevel = param->rdoqLevel;
        encParam->rdLevel = param->rdLevel;
        encParam->bEnableRectInter = param->bEnableRectInter;
        encParam->maxNumMergeCand = param->maxNumMergeCand;
        encParam->bIntraInBFrames = param->bIntraInBFrames;
        if (param->scalingLists && !encParam->scalingLists)
            encParam->scalingLists = strdup(param->scalingLists);

        encParam->rc.aqMode = param->rc.aqMode;
        encParam->rc.aqStrength = param->rc.aqStrength;
        encParam->noiseReductionInter = param->noiseReductionInter;
        encParam->noiseReductionIntra = param->noiseReductionIntra;

        encParam->limitModes = param->limitModes;
        encParam->bEnableSplitRdSkip = param->bEnableSplitRdSkip;
        encParam->bCULossless = param->bCULossless;
        encParam->bEnableRdRefine = param->bEnableRdRefine;
        encParam->limitTU = param->limitTU;
        encParam->bEnableTSkipFast = param->bEnableTSkipFast;
        encParam->rdPenalty = param->rdPenalty;
        encParam->dynamicRd = param->dynamicRd;
        encParam->bEnableTransformSkip = param->bEnableTransformSkip;
        encParam->bEnableAMP = param->bEnableAMP;
		if (param->confWinBottomOffset == 0 && param->confWinRightOffset == 0)
		{
			encParam->confWinBottomOffset = param->confWinBottomOffset;
			encParam->confWinRightOffset = param->confWinRightOffset;
		}
        /* Resignal changes in params in Parameter Sets */
        m_sps.maxAMPDepth = (m_sps.bUseAMP = param->bEnableAMP && param->bEnableAMP) ? param->maxCUDepth : 0;
        m_pps.bTransformSkipEnabled = param->bEnableTransformSkip ? 1 : 0;

    }
    encParam->forceFlush = param->forceFlush;
    /* To add: Loop Filter/deblocking controls, transform skip, signhide require PPS to be resent */
    /* To add: SAO, temporal MVP, AMP, TU depths require SPS to be resent, at every CVS boundary */
    return s265_check_params(encParam);
}

bool Encoder::isReconfigureRc(s265_param* latestParam, s265_param* param_in)
{
    return (latestParam->rc.vbvMaxBitrate != param_in->rc.vbvMaxBitrate
        || latestParam->rc.vbvBufferSize != param_in->rc.vbvBufferSize
        || latestParam->rc.bitrate != param_in->rc.bitrate
        || latestParam->rc.rfConstant != param_in->rc.rfConstant);
}

void EncStats::addPsnr(double psnrY, double psnrU, double psnrV)
{
    m_psnrSumY += psnrY;
    m_psnrSumU += psnrU;
    m_psnrSumV += psnrV;
}

void EncStats::addBits(uint64_t bits)
{
    m_accBits += bits;
    m_numPics++;
}

void EncStats::addSsim(double ssim)
{
    m_globalSsim += ssim;
}

void EncStats::addQP(double aveQp)
{
    m_totalQp += aveQp;
}

char* Encoder::statsString(EncStats& stat, char* buffer)
{
    double fps = (double)m_param->fpsNum / m_param->fpsDenom;
    double scale = fps / 1000 / (double)stat.m_numPics;

    int len = sprintf(buffer, "%6u, ", stat.m_numPics);

    len += sprintf(buffer + len, "Avg QP:%2.2lf", stat.m_totalQp / (double)stat.m_numPics);
    len += sprintf(buffer + len, "  kb/s: %-8.2lf", stat.m_accBits * scale);
    if (m_param->bEnablePsnr)
    {
        len += sprintf(buffer + len, "  PSNR Mean: Y:%.3lf U:%.3lf V:%.3lf",
                       stat.m_psnrSumY / (double)stat.m_numPics,
                       stat.m_psnrSumU / (double)stat.m_numPics,
                       stat.m_psnrSumV / (double)stat.m_numPics);
    }
    if (m_param->bEnableSsim)
    {
        sprintf(buffer + len, "  SSIM Mean: %.6lf (%.3lfdB)",
                stat.m_globalSsim / (double)stat.m_numPics,
                s265_ssim2dB(stat.m_globalSsim / (double)stat.m_numPics));
    }
    return buffer;
}

void Encoder::printSummary()
{
    if (m_param->logLevel < S265_LOG_INFO)
        return;

    char buffer[200];
    if (m_analyzeI.m_numPics)
        s265_log(m_param, S265_LOG_INFO, "frame I: %s\n", statsString(m_analyzeI, buffer));
    if (m_analyzeP.m_numPics)
        s265_log(m_param, S265_LOG_INFO, "frame P: %s\n", statsString(m_analyzeP, buffer));
    if (m_analyzeB.m_numPics)
        s265_log(m_param, S265_LOG_INFO, "frame B: %s\n", statsString(m_analyzeB, buffer));
    if (m_param->bEnableWeightedPred && m_analyzeP.m_numPics)
    {
        s265_log(m_param, S265_LOG_INFO, "Weighted P-Frames: Y:%.1f%% UV:%.1f%%\n",
            (float)100.0 * m_numLumaWPFrames / m_analyzeP.m_numPics,
            (float)100.0 * m_numChromaWPFrames / m_analyzeP.m_numPics);
    }
    if (m_param->bEnableWeightedBiPred && m_analyzeB.m_numPics)
    {
        s265_log(m_param, S265_LOG_INFO, "Weighted B-Frames: Y:%.1f%% UV:%.1f%%\n",
            (float)100.0 * m_numLumaWPBiFrames / m_analyzeB.m_numPics,
            (float)100.0 * m_numChromaWPBiFrames / m_analyzeB.m_numPics);
    }
    int pWithB = 0;
    for (int i = 0; i <= m_param->bframes; i++)
        pWithB += m_lookahead->m_histogram[i];

    if (pWithB)
    {
        int p = 0;
        for (int i = 0; i <= m_param->bframes; i++)
            p += sprintf(buffer + p, "%.1f%% ", 100. * m_lookahead->m_histogram[i] / pWithB);

        s265_log(m_param, S265_LOG_INFO, "consecutive B-frames: %s\n", buffer);
    }
    if (m_param->bLossless)
    {
        float frameSize = (float)(m_param->sourceWidth - m_sps.conformanceWindow.rightOffset) *
                                 (m_param->sourceHeight - m_sps.conformanceWindow.bottomOffset);
        float uncompressed = frameSize * S265_DEPTH * m_analyzeAll.m_numPics;

        s265_log(m_param, S265_LOG_INFO, "lossless compression ratio %.2f::1\n", uncompressed / m_analyzeAll.m_accBits);
    }
    if (m_param->bMultiPassOptRPS && m_param->rc.bStatRead)
    {
        s265_log(m_param, S265_LOG_INFO, "RPS in SPS: %d frames (%.2f%%), RPS not in SPS: %d frames (%.2f%%)\n", 
            m_rpsInSpsCount, (float)100.0 * m_rpsInSpsCount / m_rateControl->m_numEntries, 
            m_rateControl->m_numEntries - m_rpsInSpsCount, 
            (float)100.0 * (m_rateControl->m_numEntries - m_rpsInSpsCount) / m_rateControl->m_numEntries);
    }

    if (m_analyzeAll.m_numPics)
    {
        int p = 0;
        double elapsedEncodeTime = (double)(s265_mdate() - m_encodeStartTime) / 1000000;
        double elapsedVideoTime = (double)m_analyzeAll.m_numPics * m_param->fpsDenom / m_param->fpsNum;
        double bitrate = (0.001f * m_analyzeAll.m_accBits) / elapsedVideoTime;

        p += sprintf(buffer + p, "\nencoded %d frames in %.2fs (%.2f fps), %.2f kb/s, Avg QP:%2.2lf", m_analyzeAll.m_numPics,
                     elapsedEncodeTime, m_analyzeAll.m_numPics / elapsedEncodeTime, bitrate, m_analyzeAll.m_totalQp / (double)m_analyzeAll.m_numPics);

        if (m_param->bEnablePsnr)
        {
            double globalPsnr = (m_analyzeAll.m_psnrSumY * 6 + m_analyzeAll.m_psnrSumU + m_analyzeAll.m_psnrSumV) / (8 * m_analyzeAll.m_numPics);
            double yPsnr = m_analyzeAll.m_psnrSumY / m_analyzeAll.m_numPics;
            double uPsnr = m_analyzeAll.m_psnrSumU / m_analyzeAll.m_numPics;
            double vPsnr = m_analyzeAll.m_psnrSumV / m_analyzeAll.m_numPics;
            //p += sprintf(buffer + p, ", Global PSNR: %.3f", globalPsnr);
            p += sprintf(buffer + p, ", Y PSNR: %.3f, U PSNR: %.3f, V PSNR: %.3f, Global PSNR: %.3f", yPsnr, uPsnr, vPsnr, globalPsnr);
        }

        if (m_param->bEnableSsim)
            p += sprintf(buffer + p, ", SSIM Mean Y: %.7f (%6.3f dB)", m_analyzeAll.m_globalSsim / m_analyzeAll.m_numPics, s265_ssim2dB(m_analyzeAll.m_globalSsim / m_analyzeAll.m_numPics));

        sprintf(buffer + p, "\n");
        general_log(m_param, NULL, S265_LOG_INFO, buffer);
    }
    else
        general_log(m_param, NULL, S265_LOG_INFO, "\nencoded 0 frames\n");

#if DETAILED_CU_STATS
    /* Summarize stats from all frame encoders */
    CUStats cuStats;
    for (int i = 0; i < m_param->frameNumThreads; i++)
        cuStats.accumulate(m_frameEncoder[i]->m_cuStats, *m_param);

    if (!cuStats.totalCTUTime)
        return;

    int totalWorkerCount = 0;
    for (int i = 0; i < m_numPools; i++)
        totalWorkerCount += m_threadPool[i].m_numWorkers;

    int64_t  batchElapsedTime, coopSliceElapsedTime;
    uint64_t batchCount, coopSliceCount;
    m_lookahead->getWorkerStats(batchElapsedTime, batchCount, coopSliceElapsedTime, coopSliceCount);
    int64_t lookaheadWorkerTime = m_lookahead->m_slicetypeDecideElapsedTime + m_lookahead->m_preLookaheadElapsedTime +
                                  batchElapsedTime + coopSliceElapsedTime;

    int64_t totalWorkerTime = cuStats.totalCTUTime + cuStats.loopFilterElapsedTime + cuStats.pmodeTime +
                              cuStats.pmeTime + lookaheadWorkerTime + cuStats.weightAnalyzeTime;
    int64_t elapsedEncodeTime = s265_mdate() - m_encodeStartTime;

    int64_t interRDOTotalTime = 0, intraRDOTotalTime = 0;
    uint64_t interRDOTotalCount = 0, intraRDOTotalCount = 0;
    for (uint32_t i = 0; i <= m_param->maxCUDepth; i++)
    {
        interRDOTotalTime += cuStats.interRDOElapsedTime[i];
        intraRDOTotalTime += cuStats.intraRDOElapsedTime[i];
        interRDOTotalCount += cuStats.countInterRDO[i];
        intraRDOTotalCount += cuStats.countIntraRDO[i];
    }

    /* Time within compressCTU() and pmode tasks not captured by ME, Intra mode selection, or RDO (2Nx2N merge, 2Nx2N bidir, etc) */
    int64_t unaccounted = (cuStats.totalCTUTime + cuStats.pmodeTime) -
                          (cuStats.intraAnalysisElapsedTime + cuStats.motionEstimationElapsedTime + interRDOTotalTime + intraRDOTotalTime);

#define ELAPSED_SEC(val)  ((double)(val) / 1000000)
#define ELAPSED_MSEC(val) ((double)(val) / 1000)

    if (m_param->bDistributeMotionEstimation && cuStats.countPMEMasters)
    {
        s265_log(m_param, S265_LOG_INFO, "CU: %%%05.2lf time spent in motion estimation, averaging %.3lf CU inter modes per CTU\n",
                 100.0 * (cuStats.motionEstimationElapsedTime + cuStats.pmeTime) / totalWorkerTime,
                 (double)cuStats.countMotionEstimate / cuStats.totalCTUs);
        s265_log(m_param, S265_LOG_INFO, "CU: %.3lf PME masters per inter CU, each blocked an average of %.3lf ns\n",
                 (double)cuStats.countPMEMasters / cuStats.countMotionEstimate,
                 (double)cuStats.pmeBlockTime / cuStats.countPMEMasters);
        s265_log(m_param, S265_LOG_INFO, "CU:       %.3lf slaves per PME master, each took an average of %.3lf ms\n",
                 (double)cuStats.countPMETasks / cuStats.countPMEMasters,
                 ELAPSED_MSEC(cuStats.pmeTime) / cuStats.countPMETasks);
    }
    else
    {
        s265_log(m_param, S265_LOG_INFO, "CU: %%%05.2lf time spent in motion estimation, averaging %.3lf CU inter modes per CTU\n",
                 100.0 * cuStats.motionEstimationElapsedTime / totalWorkerTime,
                 (double)cuStats.countMotionEstimate / cuStats.totalCTUs);

        if (cuStats.skippedMotionReferences[0] || cuStats.skippedMotionReferences[1] || cuStats.skippedMotionReferences[2])
            s265_log(m_param, S265_LOG_INFO, "CU: Skipped motion searches per depth %%%.2lf %%%.2lf %%%.2lf %%%.2lf\n",
                     100.0 * cuStats.skippedMotionReferences[0] / cuStats.totalMotionReferences[0],
                     100.0 * cuStats.skippedMotionReferences[1] / cuStats.totalMotionReferences[1],
                     100.0 * cuStats.skippedMotionReferences[2] / cuStats.totalMotionReferences[2],
                     100.0 * cuStats.skippedMotionReferences[3] / cuStats.totalMotionReferences[3]);
    }
    s265_log(m_param, S265_LOG_INFO, "CU: %%%05.2lf time spent in intra analysis, averaging %.3lf Intra PUs per CTU\n",
             100.0 * cuStats.intraAnalysisElapsedTime / totalWorkerTime,
             (double)cuStats.countIntraAnalysis / cuStats.totalCTUs);
    if (cuStats.skippedIntraCU[0] || cuStats.skippedIntraCU[1] || cuStats.skippedIntraCU[2])
        s265_log(m_param, S265_LOG_INFO, "CU: Skipped intra CUs at depth %%%.2lf %%%.2lf %%%.2lf\n",
                 100.0 * cuStats.skippedIntraCU[0] / cuStats.totalIntraCU[0],
                 100.0 * cuStats.skippedIntraCU[1] / cuStats.totalIntraCU[1],
                 100.0 * cuStats.skippedIntraCU[2] / cuStats.totalIntraCU[2]);
    s265_log(m_param, S265_LOG_INFO, "CU: %%%05.2lf time spent in inter RDO, measuring %.3lf inter/merge predictions per CTU\n",
             100.0 * interRDOTotalTime / totalWorkerTime,
             (double)interRDOTotalCount / cuStats.totalCTUs);
    s265_log(m_param, S265_LOG_INFO, "CU: %%%05.2lf time spent in intra RDO, measuring %.3lf intra predictions per CTU\n",
             100.0 * intraRDOTotalTime / totalWorkerTime,
             (double)intraRDOTotalCount / cuStats.totalCTUs);
    s265_log(m_param, S265_LOG_INFO, "CU: %%%05.2lf time spent in loop filters, average %.3lf ms per call\n",
             100.0 * cuStats.loopFilterElapsedTime / totalWorkerTime,
             ELAPSED_MSEC(cuStats.loopFilterElapsedTime) / cuStats.countLoopFilter);
    if (cuStats.countWeightAnalyze && cuStats.weightAnalyzeTime)
    {
        s265_log(m_param, S265_LOG_INFO, "CU: %%%05.2lf time spent in weight analysis, average %.3lf ms per call\n",
                 100.0 * cuStats.weightAnalyzeTime / totalWorkerTime,
                 ELAPSED_MSEC(cuStats.weightAnalyzeTime) / cuStats.countWeightAnalyze);
    }
    if (m_param->bDistributeModeAnalysis && cuStats.countPModeMasters)
    {
        s265_log(m_param, S265_LOG_INFO, "CU: %.3lf PMODE masters per CTU, each blocked an average of %.3lf ns\n",
                 (double)cuStats.countPModeMasters / cuStats.totalCTUs,
                 (double)cuStats.pmodeBlockTime / cuStats.countPModeMasters);
        s265_log(m_param, S265_LOG_INFO, "CU:       %.3lf slaves per PMODE master, each took average of %.3lf ms\n",
                 (double)cuStats.countPModeTasks / cuStats.countPModeMasters,
                 ELAPSED_MSEC(cuStats.pmodeTime) / cuStats.countPModeTasks);
    }

    s265_log(m_param, S265_LOG_INFO, "CU: %%%05.2lf time spent in slicetypeDecide (avg %.3lfms) and prelookahead (avg %.3lfms)\n",
             100.0 * lookaheadWorkerTime / totalWorkerTime,
             ELAPSED_MSEC(m_lookahead->m_slicetypeDecideElapsedTime) / m_lookahead->m_countSlicetypeDecide,
             ELAPSED_MSEC(m_lookahead->m_preLookaheadElapsedTime) / m_lookahead->m_countPreLookahead);

    s265_log(m_param, S265_LOG_INFO, "CU: %%%05.2lf time spent in other tasks\n",
             100.0 * unaccounted / totalWorkerTime);

    if (intraRDOTotalTime && intraRDOTotalCount)
    {
        s265_log(m_param, S265_LOG_INFO, "CU: Intra RDO time  per depth %%%05.2lf %%%05.2lf %%%05.2lf %%%05.2lf\n",
                 100.0 * cuStats.intraRDOElapsedTime[0] / intraRDOTotalTime,  // 64
                 100.0 * cuStats.intraRDOElapsedTime[1] / intraRDOTotalTime,  // 32
                 100.0 * cuStats.intraRDOElapsedTime[2] / intraRDOTotalTime,  // 16
                 100.0 * cuStats.intraRDOElapsedTime[3] / intraRDOTotalTime); // 8
        s265_log(m_param, S265_LOG_INFO, "CU: Intra RDO calls per depth %%%05.2lf %%%05.2lf %%%05.2lf %%%05.2lf\n",
                 100.0 * cuStats.countIntraRDO[0] / intraRDOTotalCount,  // 64
                 100.0 * cuStats.countIntraRDO[1] / intraRDOTotalCount,  // 32
                 100.0 * cuStats.countIntraRDO[2] / intraRDOTotalCount,  // 16
                 100.0 * cuStats.countIntraRDO[3] / intraRDOTotalCount); // 8
    }

    if (interRDOTotalTime && interRDOTotalCount)
    {
        s265_log(m_param, S265_LOG_INFO, "CU: Inter RDO time  per depth %%%05.2lf %%%05.2lf %%%05.2lf %%%05.2lf\n",
                 100.0 * cuStats.interRDOElapsedTime[0] / interRDOTotalTime,  // 64
                 100.0 * cuStats.interRDOElapsedTime[1] / interRDOTotalTime,  // 32
                 100.0 * cuStats.interRDOElapsedTime[2] / interRDOTotalTime,  // 16
                 100.0 * cuStats.interRDOElapsedTime[3] / interRDOTotalTime); // 8
        s265_log(m_param, S265_LOG_INFO, "CU: Inter RDO calls per depth %%%05.2lf %%%05.2lf %%%05.2lf %%%05.2lf\n",
                 100.0 * cuStats.countInterRDO[0] / interRDOTotalCount,  // 64
                 100.0 * cuStats.countInterRDO[1] / interRDOTotalCount,  // 32
                 100.0 * cuStats.countInterRDO[2] / interRDOTotalCount,  // 16
                 100.0 * cuStats.countInterRDO[3] / interRDOTotalCount); // 8
    }

    s265_log(m_param, S265_LOG_INFO, "CU: " S265_LL " %dX%d CTUs compressed in %.3lf seconds, %.3lf CTUs per worker-second\n",
             cuStats.totalCTUs, m_param->maxCUSize, m_param->maxCUSize,
             ELAPSED_SEC(totalWorkerTime),
             cuStats.totalCTUs / ELAPSED_SEC(totalWorkerTime));

    if (m_threadPool)
        s265_log(m_param, S265_LOG_INFO, "CU: %.3lf average worker utilization, %%%05.2lf of theoretical maximum utilization\n",
                 (double)totalWorkerTime / elapsedEncodeTime,
                 100.0 * totalWorkerTime / (elapsedEncodeTime * totalWorkerCount));

#undef ELAPSED_SEC
#undef ELAPSED_MSEC
#endif
}

void Encoder::fetchStats(s265_stats *stats, size_t statsSizeBytes)
{
    if (statsSizeBytes >= sizeof(stats))
    {
        stats->globalPsnrY = m_analyzeAll.m_psnrSumY;
        stats->globalPsnrU = m_analyzeAll.m_psnrSumU;
        stats->globalPsnrV = m_analyzeAll.m_psnrSumV;
        stats->encodedPictureCount = m_analyzeAll.m_numPics;
        stats->totalWPFrames = m_numLumaWPFrames;
        stats->accBits = m_analyzeAll.m_accBits;
        stats->elapsedEncodeTime = (double)(s265_mdate() - m_encodeStartTime) / 1000000;
        if (stats->encodedPictureCount > 0)
        {
            stats->globalSsim = m_analyzeAll.m_globalSsim / stats->encodedPictureCount;
            stats->globalPsnr = (stats->globalPsnrY * 6 + stats->globalPsnrU + stats->globalPsnrV) / (8 * stats->encodedPictureCount);
            stats->elapsedVideoTime = (double)stats->encodedPictureCount * m_param->fpsDenom / m_param->fpsNum;
            stats->bitrate = (0.001f * stats->accBits) / stats->elapsedVideoTime;
        }
        else
        {
            stats->globalSsim = 0;
            stats->globalPsnr = 0;
            stats->bitrate = 0;
            stats->elapsedVideoTime = 0;
        }

        double fps = (double)m_param->fpsNum / m_param->fpsDenom;
        double scale = fps / 1000;

        stats->statsI.numPics = m_analyzeI.m_numPics;
        stats->statsI.avgQp   = m_analyzeI.m_totalQp / (double)m_analyzeI.m_numPics;
        stats->statsI.bitrate = m_analyzeI.m_accBits * scale / (double)m_analyzeI.m_numPics;
        stats->statsI.psnrY   = m_analyzeI.m_psnrSumY / (double)m_analyzeI.m_numPics;
        stats->statsI.psnrU   = m_analyzeI.m_psnrSumU / (double)m_analyzeI.m_numPics;
        stats->statsI.psnrV   = m_analyzeI.m_psnrSumV / (double)m_analyzeI.m_numPics;
        stats->statsI.ssim    = s265_ssim2dB(m_analyzeI.m_globalSsim / (double)m_analyzeI.m_numPics);

        stats->statsP.numPics = m_analyzeP.m_numPics;
        stats->statsP.avgQp   = m_analyzeP.m_totalQp / (double)m_analyzeP.m_numPics;
        stats->statsP.bitrate = m_analyzeP.m_accBits * scale / (double)m_analyzeP.m_numPics;
        stats->statsP.psnrY   = m_analyzeP.m_psnrSumY / (double)m_analyzeP.m_numPics;
        stats->statsP.psnrU   = m_analyzeP.m_psnrSumU / (double)m_analyzeP.m_numPics;
        stats->statsP.psnrV   = m_analyzeP.m_psnrSumV / (double)m_analyzeP.m_numPics;
        stats->statsP.ssim    = s265_ssim2dB(m_analyzeP.m_globalSsim / (double)m_analyzeP.m_numPics);

        stats->statsB.numPics = m_analyzeB.m_numPics;
        stats->statsB.avgQp   = m_analyzeB.m_totalQp / (double)m_analyzeB.m_numPics;
        stats->statsB.bitrate = m_analyzeB.m_accBits * scale / (double)m_analyzeB.m_numPics;
        stats->statsB.psnrY   = m_analyzeB.m_psnrSumY / (double)m_analyzeB.m_numPics;
        stats->statsB.psnrU   = m_analyzeB.m_psnrSumU / (double)m_analyzeB.m_numPics;
        stats->statsB.psnrV   = m_analyzeB.m_psnrSumV / (double)m_analyzeB.m_numPics;
        stats->statsB.ssim    = s265_ssim2dB(m_analyzeB.m_globalSsim / (double)m_analyzeB.m_numPics);
        if (m_param->csvLogLevel >= 2 || m_param->maxCLL || m_param->maxFALL)
        {
            stats->maxCLL = m_analyzeAll.m_maxCLL;
            stats->maxFALL = (uint16_t)(m_analyzeAll.m_maxFALL / m_analyzeAll.m_numPics);
        }
    }
    /* If new statistics are added to s265_stats, we must check here whether the
     * structure provided by the user is the new structure or an older one (for
     * future safety) */
}

void Encoder::finishFrameStats(Frame* curFrame, FrameEncoder *curEncoder, s265_frame_stats* frameStats, int inPoc)
{
    PicYuv* reconPic = curFrame->m_reconPic;
    uint64_t bits = curEncoder->m_accessUnitBits;

    //===== calculate PSNR =====
    int width  = reconPic->m_picWidth - m_sps.conformanceWindow.rightOffset;
    int height = reconPic->m_picHeight - m_sps.conformanceWindow.bottomOffset;
    int size = width * height;

    int maxvalY = 255 << (S265_DEPTH - 8);
    int maxvalC = 255 << (S265_DEPTH - 8);
    double refValueY = (double)maxvalY * maxvalY * size;
    double refValueC = (double)maxvalC * maxvalC * size / 4.0;
    uint64_t ssdY, ssdU, ssdV;

    ssdY = curEncoder->m_SSDY;
    ssdU = curEncoder->m_SSDU;
    ssdV = curEncoder->m_SSDV;
    double psnrY = (ssdY ? 10.0 * log10(refValueY / (double)ssdY) : 99.99);
    double psnrU = (ssdU ? 10.0 * log10(refValueC / (double)ssdU) : 99.99);
    double psnrV = (ssdV ? 10.0 * log10(refValueC / (double)ssdV) : 99.99);

    FrameData& curEncData = *curFrame->m_encData;
    Slice* slice = curEncData.m_slice;

    //===== add bits, psnr and ssim =====
    m_analyzeAll.addBits(bits);
    m_analyzeAll.addQP(curEncData.m_avgQpAq);

    if (m_param->bEnablePsnr)
        m_analyzeAll.addPsnr(psnrY, psnrU, psnrV);

    double ssim = 0.0;
    if (m_param->bEnableSsim && curEncoder->m_ssimCnt)
    {
        ssim = curEncoder->m_ssim / curEncoder->m_ssimCnt;
        m_analyzeAll.addSsim(ssim);
    }
    if (slice->isIntra())
    {
        m_analyzeI.addBits(bits);
        m_analyzeI.addQP(curEncData.m_avgQpAq);
        if (m_param->bEnablePsnr)
            m_analyzeI.addPsnr(psnrY, psnrU, psnrV);
        if (m_param->bEnableSsim)
            m_analyzeI.addSsim(ssim);
    }
    else if (slice->isInterP())
    {
        m_analyzeP.addBits(bits);
        m_analyzeP.addQP(curEncData.m_avgQpAq);
        if (m_param->bEnablePsnr)
            m_analyzeP.addPsnr(psnrY, psnrU, psnrV);
        if (m_param->bEnableSsim)
            m_analyzeP.addSsim(ssim);
    }
    else if (slice->isInterB())
    {
        m_analyzeB.addBits(bits);
        m_analyzeB.addQP(curEncData.m_avgQpAq);
        if (m_param->bEnablePsnr)
            m_analyzeB.addPsnr(psnrY, psnrU, psnrV);
        if (m_param->bEnableSsim)
            m_analyzeB.addSsim(ssim);
    }
    if (m_param->csvLogLevel >= 2 || m_param->maxCLL || m_param->maxFALL)
    {
        m_analyzeAll.m_maxFALL += curFrame->m_fencPic->m_avgLumaLevel;
        m_analyzeAll.m_maxCLL = S265_MAX(m_analyzeAll.m_maxCLL, curFrame->m_fencPic->m_maxLumaLevel);
    }
    char c = (slice->isIntra() ? (curFrame->m_lowres.sliceType == S265_TYPE_IDR ? 'I' : 'i') : slice->isInterP() ? 'P' : 'B');
    int poc = slice->m_poc;
    if (!IS_REFERENCED(curFrame))
        c += 32; // lower case if unreferenced

    if (frameStats)
    {
        const int picOrderCntLSB = slice->m_poc - slice->m_lastIDR;

        frameStats->encoderOrder = m_outputCount;
        frameStats->sliceType = c;
        frameStats->poc = picOrderCntLSB;
        frameStats->qp = curEncData.m_avgQpAq;
        frameStats->bits = bits;
        frameStats->bScenecut = curFrame->m_lowres.bScenecut;
        if (m_param->csvLogLevel >= 2)
            frameStats->ipCostRatio = curFrame->m_lowres.ipCostRatio;
        frameStats->bufferFill = m_rateControl->m_bufferFillActual;
        frameStats->bufferFillFinal = m_rateControl->m_bufferFillFinal;
        if (m_param->csvLogLevel >= 2)
            frameStats->unclippedBufferFillFinal = m_rateControl->m_unclippedBufferFillFinal;
        frameStats->frameLatency = inPoc - poc;
        if (m_param->rc.rateControlMode == S265_RC_CRF)
            frameStats->rateFactor = curEncData.m_rateFactor;
        frameStats->psnrY = psnrY;
        frameStats->psnrU = psnrU;
        frameStats->psnrV = psnrV;
        double psnr = (psnrY * 6 + psnrU + psnrV) / 8;
        frameStats->psnr = psnr;
        frameStats->ssim = ssim;
        if (!slice->isIntra())
        {
            for (int ref = 0; ref < MAX_NUM_REF; ref++)
                frameStats->list0POC[ref] = ref < slice->m_numRefIdx[0] ? slice->m_refPOCList[0][ref] - slice->m_lastIDR : -1;

            if (!slice->isInterP())
            {
                for (int ref = 0; ref < MAX_NUM_REF; ref++)
                    frameStats->list1POC[ref] = ref < slice->m_numRefIdx[1] ? slice->m_refPOCList[1][ref] - slice->m_lastIDR : -1;
            }
        }
#define ELAPSED_MSEC(start, end) (((double)(end) - (start)) / 1000)
        if (m_param->csvLogLevel >= 2)
        {
#if ENABLE_LIBVMAF
            frameStats->vmafFrameScore = curFrame->m_fencPic->m_vmafScore;
#endif
            frameStats->decideWaitTime = ELAPSED_MSEC(0, curEncoder->m_slicetypeWaitTime);
            frameStats->row0WaitTime = ELAPSED_MSEC(curEncoder->m_startCompressTime, curEncoder->m_row0WaitTime);
            frameStats->wallTime = ELAPSED_MSEC(curEncoder->m_row0WaitTime, curEncoder->m_endCompressTime);
            frameStats->refWaitWallTime = ELAPSED_MSEC(curEncoder->m_row0WaitTime, curEncoder->m_allRowsAvailableTime);
            frameStats->totalCTUTime = ELAPSED_MSEC(0, curEncoder->m_totalWorkerElapsedTime);
            frameStats->stallTime = ELAPSED_MSEC(0, curEncoder->m_totalNoWorkerTime);
            frameStats->totalFrameTime = ELAPSED_MSEC(curFrame->m_encodeStartTime, s265_mdate());
            if (curEncoder->m_totalActiveWorkerCount)
                frameStats->avgWPP = (double)curEncoder->m_totalActiveWorkerCount / curEncoder->m_activeWorkerCountSamples;
            else
                frameStats->avgWPP = 1;
            frameStats->countRowBlocks = curEncoder->m_countRowBlocks;

            frameStats->avgChromaDistortion = curFrame->m_encData->m_frameStats.avgChromaDistortion;
            frameStats->avgLumaDistortion = curFrame->m_encData->m_frameStats.avgLumaDistortion;
            frameStats->avgPsyEnergy = curFrame->m_encData->m_frameStats.avgPsyEnergy;
            frameStats->avgResEnergy = curFrame->m_encData->m_frameStats.avgResEnergy;
            frameStats->maxLumaLevel = curFrame->m_fencPic->m_maxLumaLevel;
            frameStats->minLumaLevel = curFrame->m_fencPic->m_minLumaLevel;
            frameStats->avgLumaLevel = curFrame->m_fencPic->m_avgLumaLevel;

            frameStats->maxChromaULevel = curFrame->m_fencPic->m_maxChromaULevel;
            frameStats->minChromaULevel = curFrame->m_fencPic->m_minChromaULevel;
            frameStats->avgChromaULevel = curFrame->m_fencPic->m_avgChromaULevel;

            frameStats->maxChromaVLevel = curFrame->m_fencPic->m_maxChromaVLevel;
            frameStats->minChromaVLevel = curFrame->m_fencPic->m_minChromaVLevel;
            frameStats->avgChromaVLevel = curFrame->m_fencPic->m_avgChromaVLevel;

            if (curFrame->m_encData->m_frameStats.totalPu[4] == 0)
                frameStats->puStats.percentNxN = 0;
            else
                frameStats->puStats.percentNxN = (double)(curFrame->m_encData->m_frameStats.cnt4x4 / (double)curFrame->m_encData->m_frameStats.totalPu[4]) * 100;
            for (uint32_t depth = 0; depth <= m_param->maxCUDepth; depth++)
            {
                if (curFrame->m_encData->m_frameStats.totalPu[depth] == 0)
                {
                    frameStats->puStats.percentSkipPu[depth] = 0;
                    frameStats->puStats.percentIntraPu[depth] = 0;
                    frameStats->puStats.percentAmpPu[depth] = 0;
                    for (int i = 0; i < INTER_MODES - 1; i++)
                    {
                        frameStats->puStats.percentInterPu[depth][i] = 0;
                        frameStats->puStats.percentMergePu[depth][i] = 0;
                    }
                }
                else
                {
                    frameStats->puStats.percentSkipPu[depth] = (double)(curFrame->m_encData->m_frameStats.cntSkipPu[depth] / (double)curFrame->m_encData->m_frameStats.totalPu[depth]) * 100;
                    frameStats->puStats.percentIntraPu[depth] = (double)(curFrame->m_encData->m_frameStats.cntIntraPu[depth] / (double)curFrame->m_encData->m_frameStats.totalPu[depth]) * 100;
                    frameStats->puStats.percentAmpPu[depth] = (double)(curFrame->m_encData->m_frameStats.cntAmp[depth] / (double)curFrame->m_encData->m_frameStats.totalPu[depth]) * 100;
                    for (int i = 0; i < INTER_MODES - 1; i++)
                    {
                        frameStats->puStats.percentInterPu[depth][i] = (double)(curFrame->m_encData->m_frameStats.cntInterPu[depth][i] / (double)curFrame->m_encData->m_frameStats.totalPu[depth]) * 100;
                        frameStats->puStats.percentMergePu[depth][i] = (double)(curFrame->m_encData->m_frameStats.cntMergePu[depth][i] / (double)curFrame->m_encData->m_frameStats.totalPu[depth]) * 100;
                    }
                }
            }
        }

        if (m_param->csvLogLevel >= 1)
        {
            frameStats->cuStats.percentIntraNxN = curFrame->m_encData->m_frameStats.percentIntraNxN;

            for (uint32_t depth = 0; depth <= m_param->maxCUDepth; depth++)
            {
                frameStats->cuStats.percentSkipCu[depth] = curFrame->m_encData->m_frameStats.percentSkipCu[depth];
                frameStats->cuStats.percentMergeCu[depth] = curFrame->m_encData->m_frameStats.percentMergeCu[depth];
                frameStats->cuStats.percentInterDistribution[depth][0] = curFrame->m_encData->m_frameStats.percentInterDistribution[depth][0];
                frameStats->cuStats.percentInterDistribution[depth][1] = curFrame->m_encData->m_frameStats.percentInterDistribution[depth][1];
                frameStats->cuStats.percentInterDistribution[depth][2] = curFrame->m_encData->m_frameStats.percentInterDistribution[depth][2];
                for (int n = 0; n < INTRA_MODES; n++)
                    frameStats->cuStats.percentIntraDistribution[depth][n] = curFrame->m_encData->m_frameStats.percentIntraDistribution[depth][n];
            }
        }
    }
}

#if defined(_MSC_VER)
#pragma warning(disable: 4800) // forcing int to bool
#pragma warning(disable: 4127) // conditional expression is constant
#endif

void Encoder::initRefIdx()
{
    int j = 0;

    for (j = 0; j < MAX_NUM_REF_IDX; j++)
    {
        m_refIdxLastGOP.numRefIdxl0[j] = 0;
        m_refIdxLastGOP.numRefIdxl1[j] = 0;
    }

    return;
}

void Encoder::analyseRefIdx(int *numRefIdx)
{
    int i_l0 = 0;
    int i_l1 = 0;

    i_l0 = numRefIdx[0];
    i_l1 = numRefIdx[1];

    if ((0 < i_l0) && (MAX_NUM_REF_IDX > i_l0))
        m_refIdxLastGOP.numRefIdxl0[i_l0]++;
    if ((0 < i_l1) && (MAX_NUM_REF_IDX > i_l1))
        m_refIdxLastGOP.numRefIdxl1[i_l1]++;

    return;
}

void Encoder::updateRefIdx()
{
    int i_max_l0 = 0;
    int i_max_l1 = 0;
    int j = 0;

    i_max_l0 = 0;
    i_max_l1 = 0;
    m_refIdxLastGOP.numRefIdxDefault[0] = 1;
    m_refIdxLastGOP.numRefIdxDefault[1] = 1;
    for (j = 0; j < MAX_NUM_REF_IDX; j++)
    {
        if (i_max_l0 < m_refIdxLastGOP.numRefIdxl0[j])
        {
            i_max_l0 = m_refIdxLastGOP.numRefIdxl0[j];
            m_refIdxLastGOP.numRefIdxDefault[0] = j;
        }
        if (i_max_l1 < m_refIdxLastGOP.numRefIdxl1[j])
        {
            i_max_l1 = m_refIdxLastGOP.numRefIdxl1[j];
            m_refIdxLastGOP.numRefIdxDefault[1] = j;
        }
    }

    m_pps.numRefIdxDefault[0] = m_refIdxLastGOP.numRefIdxDefault[0];
    m_pps.numRefIdxDefault[1] = m_refIdxLastGOP.numRefIdxDefault[1];
    initRefIdx();

    return;
}

void Encoder::getStreamHeaders(NALList& list, Entropy& sbacCoder, Bitstream& bs)
{
    sbacCoder.setBitstream(&bs);

    if (m_param->dolbyProfile && !m_param->bRepeatHeaders)
    {
        bs.resetBits();
        bs.write(0x10, 8);
        list.serialize(NAL_UNIT_ACCESS_UNIT_DELIMITER, bs);
    }
    
    /* headers for start of bitstream */
    bs.resetBits();
    sbacCoder.codeVPS(m_vps);
    bs.writeByteAlignment();
    list.serialize(NAL_UNIT_VPS, bs);

    bs.resetBits();
    sbacCoder.codeSPS(m_sps, m_scalingList, m_vps.ptl);
    bs.writeByteAlignment();
    list.serialize(NAL_UNIT_SPS, bs);

    bs.resetBits();
    sbacCoder.codePPS(m_pps, (m_param->maxSlices <= 1), m_iPPSQpMinus26);
    bs.writeByteAlignment();
    list.serialize(NAL_UNIT_PPS, bs);

    if (m_param->bSingleSeiNal)
        bs.resetBits();

    if (m_param->bEmitHDR10SEI)
    {
        if (m_param->bEmitCLL)
        {
            SEIContentLightLevel cllsei;
            cllsei.max_content_light_level = m_param->maxCLL;
            cllsei.max_pic_average_light_level = m_param->maxFALL;
            cllsei.writeSEImessages(bs, m_sps, NAL_UNIT_PREFIX_SEI, list, m_param->bSingleSeiNal);
        }

        if (m_param->masteringDisplayColorVolume)
        {
            SEIMasteringDisplayColorVolume mdsei;
            if (mdsei.parse(m_param->masteringDisplayColorVolume))
                mdsei.writeSEImessages(bs, m_sps, NAL_UNIT_PREFIX_SEI, list, m_param->bSingleSeiNal);
            else
                s265_log(m_param, S265_LOG_WARNING, "unable to parse mastering display color volume info\n");
        }
    }

    if (m_param->bEmitInfoSEI)
    {
        char *opts = s265_param2string(m_param, m_sps.conformanceWindow.rightOffset, m_sps.conformanceWindow.bottomOffset);
        if (opts)
        {
            char *buffer = S265_MALLOC(char, strlen(opts) + strlen(PFX(version_str)) +
                strlen(PFX(build_info_str)) + 200);
            if (buffer)
            {
                sprintf(buffer, "s265 (build %d) - %s:%s - H.265/HEVC codec - "
                    "Copyright 2013-2018 (c) Multicoreware, Inc - "
                    "http://s265.org - options: %s",
                    S265_BUILD, PFX(version_str), PFX(build_info_str), opts);

                SEIuserDataUnregistered idsei;
                idsei.m_userData = (uint8_t*)buffer;
                idsei.setSize((uint32_t)strlen(buffer));
                idsei.writeSEImessages(bs, m_sps, NAL_UNIT_PREFIX_SEI, list, m_param->bSingleSeiNal);

                S265_FREE(buffer);
            }

            S265_FREE(opts);
        }
    }

    if (m_param->bEmitHRDSEI)
    {
        /* Picture Timing and Buffering Period SEI require the SPS to be "activated" */
        SEIActiveParameterSets sei;
        sei.m_selfContainedCvsFlag = true;
        sei.m_noParamSetUpdateFlag = true;
        sei.writeSEImessages(bs, m_sps, NAL_UNIT_PREFIX_SEI, list, m_param->bSingleSeiNal);
    }
}

void Encoder::getEndNalUnits(NALList& list, Bitstream& bs)
{
    NALList nalList;
    bs.resetBits();

    if (m_param->bEnableEndOfSequence)
        nalList.serialize(NAL_UNIT_EOS, bs);
    if (m_param->bEnableEndOfBitstream)
        nalList.serialize(NAL_UNIT_EOB, bs);

    list.takeContents(nalList);
}

void Encoder::initVPS(VPS *vps)
{
    /* Note that much of the VPS is initialized by determineLevel() */
    vps->ptl.progressiveSourceFlag = true;
    vps->ptl.nonPackedConstraintFlag = false;
    vps->ptl.frameOnlyConstraintFlag = true;
}

void Encoder::initSPS(SPS *sps)
{
    sps->conformanceWindow = m_conformanceWindow;
    sps->chromaFormatIdc = m_param->internalCsp;
    sps->picWidthInLumaSamples = m_param->sourceWidth;
    sps->picHeightInLumaSamples = m_param->sourceHeight;
    sps->numCuInWidth = (m_param->sourceWidth + m_param->maxCUSize - 1) / m_param->maxCUSize;
    sps->numCuInHeight = (m_param->sourceHeight + m_param->maxCUSize - 1) / m_param->maxCUSize;
    sps->numCUsInFrame = sps->numCuInWidth * sps->numCuInHeight;
    sps->numPartitions = m_param->num4x4Partitions;
    sps->numPartInCUSize = 1 << m_param->unitSizeDepth;

    sps->log2MinCodingBlockSize = m_param->maxLog2CUSize - m_param->maxCUDepth;
    sps->log2DiffMaxMinCodingBlockSize = m_param->maxCUDepth;
    uint32_t maxLog2TUSize = (uint32_t)g_log2Size[m_param->maxTUSize];
    sps->quadtreeTULog2MaxSize = S265_MIN((uint32_t)m_param->maxLog2CUSize, maxLog2TUSize);
    sps->quadtreeTULog2MinSize = 2;
    sps->quadtreeTUMaxDepthInter = m_param->tuQTMaxInterDepth;
    sps->quadtreeTUMaxDepthIntra = m_param->tuQTMaxIntraDepth;

    sps->bUseSAO = m_param->bEnableSAO;

    sps->bUseAMP = m_param->bEnableAMP;
    sps->maxAMPDepth = m_param->bEnableAMP ? m_param->maxCUDepth : 0;

    sps->maxTempSubLayers = m_param->bEnableTemporalSubLayers ? 2 : 1;
    sps->maxDecPicBuffering = m_vps.maxDecPicBuffering;
    sps->numReorderPics = m_vps.numReorderPics;
    sps->maxLatencyIncrease = m_vps.maxLatencyIncrease = m_param->bframes;

    sps->bUseStrongIntraSmoothing = m_param->bEnableStrongIntraSmoothing;
    sps->bTemporalMVPEnabled = m_param->bEnableTemporalMvp;
    sps->bEmitVUITimingInfo = m_param->bEmitVUITimingInfo;
    sps->bEmitVUIHRDInfo = m_param->bEmitVUIHRDInfo;
    sps->log2MaxPocLsb = m_param->log2MaxPocLsb;
    int maxDeltaPOC = (m_param->bframes + 2) * (!!m_param->bBPyramid + 1) * 2;
    while ((1 << sps->log2MaxPocLsb) <= maxDeltaPOC * 2)
        sps->log2MaxPocLsb++;

    if (sps->log2MaxPocLsb != m_param->log2MaxPocLsb)
        s265_log(m_param, S265_LOG_WARNING, "Reset log2MaxPocLsb to %d to account for all POC values\n", sps->log2MaxPocLsb);

    VUI& vui = sps->vuiParameters;
    vui.aspectRatioInfoPresentFlag = !!m_param->vui.aspectRatioIdc;
    vui.aspectRatioIdc = m_param->vui.aspectRatioIdc;
    vui.sarWidth = m_param->vui.sarWidth;
    vui.sarHeight = m_param->vui.sarHeight;

    vui.overscanInfoPresentFlag = m_param->vui.bEnableOverscanInfoPresentFlag;
    vui.overscanAppropriateFlag = m_param->vui.bEnableOverscanAppropriateFlag;

    vui.videoSignalTypePresentFlag = m_param->vui.bEnableVideoSignalTypePresentFlag;
    vui.videoFormat = m_param->vui.videoFormat;
    vui.videoFullRangeFlag = m_param->vui.bEnableVideoFullRangeFlag;

    vui.colourDescriptionPresentFlag = m_param->vui.bEnableColorDescriptionPresentFlag;
    vui.colourPrimaries = m_param->vui.colorPrimaries;
    vui.transferCharacteristics = m_param->vui.transferCharacteristics;
    vui.matrixCoefficients = m_param->vui.matrixCoeffs;

    vui.defaultDisplayWindow.bEnabled = m_param->vui.bEnableDefaultDisplayWindowFlag;
    vui.defaultDisplayWindow.rightOffset = m_param->vui.defDispWinRightOffset;
    vui.defaultDisplayWindow.topOffset = m_param->vui.defDispWinTopOffset;
    vui.defaultDisplayWindow.bottomOffset = m_param->vui.defDispWinBottomOffset;
    vui.defaultDisplayWindow.leftOffset = m_param->vui.defDispWinLeftOffset;
    vui.hrdParametersPresentFlag = m_param->bEmitHRDSEI;

    vui.timingInfo.numUnitsInTick = m_param->fpsDenom;
    vui.timingInfo.timeScale = m_param->fpsNum;
}

void Encoder::initPPS(PPS *pps)
{
    bool bIsVbv = m_param->rc.vbvBufferSize > 0 && m_param->rc.vbvMaxBitrate > 0;

    if (!m_param->bLossless && (m_param->rc.aqMode || bIsVbv || m_param->bAQMotion))
    {
        pps->bUseDQP = true;
        pps->maxCuDQPDepth = g_log2Size[m_param->maxCUSize] - g_log2Size[m_param->rc.qgSize];
        S265_CHECK(pps->maxCuDQPDepth <= 3, "max CU DQP depth cannot be greater than 3\n");
    }
    else
    {
        pps->bUseDQP = false;
        pps->maxCuDQPDepth = 0;
    }

    pps->chromaQpOffset[0] = m_param->cbQpOffset;
    pps->chromaQpOffset[1] = m_param->crQpOffset;
    pps->pps_slice_chroma_qp_offsets_present_flag = m_param->bHDR10Opt;

    pps->bConstrainedIntraPred = m_param->bEnableConstrainedIntra;
    pps->bUseWeightPred = m_param->bEnableWeightedPred;
    pps->bUseWeightedBiPred = m_param->bEnableWeightedBiPred;
    pps->bTransquantBypassEnabled = m_param->bCULossless || m_param->bLossless;
    pps->bTransformSkipEnabled = m_param->bEnableTransformSkip;
    pps->bSignHideEnabled = m_param->bEnableSignHiding;

    pps->bDeblockingFilterControlPresent = !m_param->bEnableLoopFilter || m_param->deblockingFilterBetaOffset || m_param->deblockingFilterTCOffset;
    pps->bPicDisableDeblockingFilter = !m_param->bEnableLoopFilter;
    pps->deblockingFilterBetaOffsetDiv2 = m_param->deblockingFilterBetaOffset;
    pps->deblockingFilterTcOffsetDiv2 = m_param->deblockingFilterTCOffset;

    pps->bEntropyCodingSyncEnabled = m_param->bEnableWavefront;

    pps->numRefIdxDefault[0] = 1;
    pps->numRefIdxDefault[1] = 1;
}

void Encoder::configureZone(s265_param *p, s265_param *zone)
{
    if (m_param->bResetZoneConfig)
    {
        p->maxNumReferences = zone->maxNumReferences;
        p->bEnableFastIntra = zone->bEnableFastIntra;
        p->bEnableEarlySkip = zone->bEnableEarlySkip;
        p->recursionSkipMode = zone->recursionSkipMode;
        p->searchMethod = zone->searchMethod;
        p->searchRange = zone->searchRange;
        p->subpelRefine = zone->subpelRefine;
        p->rdoqLevel = zone->rdoqLevel;
        p->rdLevel = zone->rdLevel;
        p->bEnableRectInter = zone->bEnableRectInter;
        p->maxNumMergeCand = zone->maxNumMergeCand;
        p->bIntraInBFrames = zone->bIntraInBFrames;
        if (zone->scalingLists)
            p->scalingLists = strdup(zone->scalingLists);

        p->rc.aqMode = zone->rc.aqMode;
        p->rc.aqStrength = zone->rc.aqStrength;
        p->noiseReductionInter = zone->noiseReductionInter;
        p->noiseReductionIntra = zone->noiseReductionIntra;

        p->limitModes = zone->limitModes;
        p->bEnableSplitRdSkip = zone->bEnableSplitRdSkip;
        p->bCULossless = zone->bCULossless;
        p->bEnableRdRefine = zone->bEnableRdRefine;
        p->limitTU = zone->limitTU;
        p->bEnableTSkipFast = zone->bEnableTSkipFast;
        p->rdPenalty = zone->rdPenalty;
        p->dynamicRd = zone->dynamicRd;
        p->bEnableTransformSkip = zone->bEnableTransformSkip;
        p->bEnableAMP = zone->bEnableAMP;

        if (m_param->rc.rateControlMode == S265_RC_ABR)
            p->rc.bitrate = zone->rc.bitrate;
        if (m_param->rc.rateControlMode == S265_RC_CRF)
            p->rc.rfConstant = zone->rc.rfConstant;
        if (m_param->rc.rateControlMode == S265_RC_CQP)
        {
            p->rc.qp = zone->rc.qp;
            p->rc.aqMode = S265_AQ_NONE;
            p->rc.hevcAq = 0;
        }
        p->radl = zone->radl;
    }
    memcpy(zone, p, sizeof(s265_param));
}

void Encoder::configureDolbyVisionParams(s265_param* p)
{
    uint32_t doviProfile = 0;

    while (dovi[doviProfile].doviProfileId != p->dolbyProfile && doviProfile + 1 < sizeof(dovi) / sizeof(dovi[0]))
        doviProfile++;

    p->bEmitHRDSEI = dovi[doviProfile].bEmitHRDSEI;
    p->vui.bEnableVideoSignalTypePresentFlag = dovi[doviProfile].bEnableVideoSignalTypePresentFlag;
    p->vui.bEnableColorDescriptionPresentFlag = dovi[doviProfile].bEnableColorDescriptionPresentFlag;
    p->bEnableAccessUnitDelimiters = dovi[doviProfile].bEnableAccessUnitDelimiters;
    p->bAnnexB = dovi[doviProfile].bAnnexB;
    p->vui.videoFormat = dovi[doviProfile].videoFormat;
    p->vui.bEnableVideoFullRangeFlag = dovi[doviProfile].bEnableVideoFullRangeFlag;
    p->vui.transferCharacteristics = dovi[doviProfile].transferCharacteristics;
    p->vui.colorPrimaries = dovi[doviProfile].colorPrimaries;
    p->vui.matrixCoeffs = dovi[doviProfile].matrixCoeffs;

    if (dovi[doviProfile].doviProfileId == 81)
        p->bEmitHDR10SEI = p->bEmitCLL = 1;

    if (dovi[doviProfile].doviProfileId == 50)
        p->crQpOffset = 3;
}

void Encoder::configure(s265_param *p)
{
    this->m_param = p;
    this->m_externalFlush = false;

    if (p->keyframeMax < 0)
    {
        /* A negative max GOP size indicates the user wants only one I frame at
         * the start of the stream. Set an infinite GOP distance and disable
         * adaptive I frame placement */
        p->keyframeMax = INT_MAX;
        p->scenecutThreshold = 0;
        p->bHistBasedSceneCut = 0;
        p->bEnableTradScdInHscd = 1;
    }
    else if (p->keyframeMax <= 1)
    {
        p->keyframeMax = 1;

        // disable lookahead for all-intra encodes
        p->bFrameAdaptive = 0;
        p->bframes = 0;
        p->bOpenGOP = 0;
        p->bRepeatHeaders = 1;
        p->lookaheadDepth = 0;
        p->bframes = 0;
        p->scenecutThreshold = 0;
        p->bHistBasedSceneCut = 0;
        p->bEnableTradScdInHscd = 1;
        p->bFrameAdaptive = 0;
        p->rc.cuTree = 0;
        p->bEnableWeightedPred = 0;
        p->bEnableWeightedBiPred = 0;
        p->bIntraRefresh = 0;

        /* SPSs shall have sps_max_dec_pic_buffering_minus1[ sps_max_sub_layers_minus1 ] equal to 0 only */
        p->maxNumReferences = 1;
    }
    if (!p->keyframeMin)
    {
        double fps = (double)p->fpsNum / p->fpsDenom;
        p->keyframeMin = S265_MIN((int)fps, p->keyframeMax / 10);
    }
    p->keyframeMin = S265_MAX(1, p->keyframeMin);

    if (!p->bframes)
        p->bBPyramid = 0;
    if (!p->rdoqLevel)
        p->psyRdoq = 0;

    /* Disable features which are not supported by the current RD level */
    if (p->rdLevel < 3)
    {
        if (p->bCULossless)             /* impossible */
            s265_log(p, S265_LOG_WARNING, "--cu-lossless disabled, requires --rdlevel 3 or higher\n");
        if (p->bEnableTransformSkip)    /* impossible */
            s265_log(p, S265_LOG_WARNING, "--tskip disabled, requires --rdlevel 3 or higher\n");
        p->bCULossless = p->bEnableTransformSkip = 0;
    }
    if (p->rdLevel < 2)
    {
        if (p->bDistributeModeAnalysis) /* not useful */
            s265_log(p, S265_LOG_WARNING, "--pmode disabled, requires --rdlevel 2 or higher\n");
        p->bDistributeModeAnalysis = 0;

        p->psyRd = 0;                   /* impossible */

        if (p->bEnableRectInter)        /* broken, not very useful */
            s265_log(p, S265_LOG_WARNING, "--rect disabled, requires --rdlevel 2 or higher\n");
        p->bEnableRectInter = 0;
    }

    if (!p->bEnableRectInter)          /* not useful */
        p->bEnableAMP = false;

    /* In 444, chroma gets twice as much resolution, so halve quality when psy-rd is enabled */
    if (p->internalCsp == S265_CSP_I444 && p->psyRd)
    {
        if (!p->cbQpOffset && !p->crQpOffset)
        {
            p->cbQpOffset = MAX_CHROMA_QP_OFFSET / 2;
            p->crQpOffset = MAX_CHROMA_QP_OFFSET / 2;
            s265_log(p, S265_LOG_WARNING, "halving the quality when psy-rd is enabled for 444 input."
                     " Setting cbQpOffset = %d and crQpOffset = %d\n", p->cbQpOffset, p->crQpOffset);
        }
    }

    if (p->bLossless)
    {
        p->rc.rateControlMode = S265_RC_CQP;
        p->rc.qp = 4; // An oddity, QP=4 is more lossless than QP=0 and gives better lambdas
        p->bEnableSsim = 0;
        p->bEnablePsnr = 0;
    }

    if (p->rc.rateControlMode == S265_RC_CQP)
    {
        p->rc.aqMode = S265_AQ_NONE;
        p->rc.hevcAq = 0;
        p->rc.bitrate = 0;
        p->rc.cuTree = 0;
        p->rc.aqStrength = 0;
    }

    if (p->rc.aqMode == 0 && p->rc.cuTree)
    {
        p->rc.aqMode = S265_AQ_VARIANCE;
        p->rc.aqStrength = 0.0;
    }

    if (p->lookaheadDepth == 0 && p->rc.cuTree && !p->rc.bStatRead)
    {
        s265_log(p, S265_LOG_WARNING, "cuTree disabled, requires lookahead to be enabled\n");
        p->rc.cuTree = 0;
    }

    if (p->maxTUSize > p->maxCUSize)
    {
        s265_log(p, S265_LOG_WARNING, "Max TU size should be less than or equal to max CU size, setting max TU size = %d\n", p->maxCUSize);
        p->maxTUSize = p->maxCUSize;
    }
    if (p->rc.aqStrength == 0 && p->rc.cuTree == 0)
    {
        p->rc.aqMode = S265_AQ_NONE;
        p->rc.hevcAq = 0;
    }
    if (p->rc.aqMode == S265_AQ_NONE && p->rc.cuTree == 0)
        p->rc.aqStrength = 0;
    if (p->rc.hevcAq && p->rc.aqMode)
    {
        s265_log(p, S265_LOG_WARNING, "hevc-aq enabled, disabling other aq-modes\n");
    }

    if (p->totalFrames && p->totalFrames <= 2 * ((float)p->fpsNum) / p->fpsDenom && p->rc.bStrictCbr)
        p->lookaheadDepth = p->totalFrames;
    if (p->bIntraRefresh)
    {
        int numCuInWidth = (m_param->sourceWidth + m_param->maxCUSize - 1) / m_param->maxCUSize;
        if (p->maxNumReferences > 1)
        {
            s265_log(p,  S265_LOG_WARNING, "Max References > 1 + intra-refresh is not supported , setting max num references = 1\n");
            p->maxNumReferences = 1;
        }

        if (p->bBPyramid && p->bframes)
            s265_log(p,  S265_LOG_WARNING, "B pyramid cannot be enabled when max references is 1, Disabling B pyramid\n");
        p->bBPyramid = 0;


        if (p->bOpenGOP)
        {
            s265_log(p,  S265_LOG_WARNING, "Open Gop disabled, Intra Refresh is not compatible with openGop\n");
            p->bOpenGOP = 0;
        }

        s265_log(p,  S265_LOG_WARNING, "Scenecut is disabled when Intra Refresh is enabled\n");

        if (((float)numCuInWidth - 1) / m_param->keyframeMax > 1)
            s265_log(p,  S265_LOG_WARNING, "Keyint value is very low.It leads to frequent intra refreshes, can be almost every frame."
                     "Prefered use case would be high keyint value or an API call to refresh when necessary\n");

    }

    if (p->selectiveSAO && !p->bEnableSAO)
    {
        p->bEnableSAO = 1;
        s265_log(p, S265_LOG_WARNING, "SAO turned ON when selective-sao is ON\n");
    }

    if (!p->selectiveSAO && p->bEnableSAO)
        p->selectiveSAO = 4;

    if (p->rc.rfConstantMin > p->rc.rfConstant)
    {
        s265_log(m_param, S265_LOG_WARNING, "CRF min must be less than CRF\n");
        p->rc.rfConstantMin = 0;
    }

    if (p->scaleFactor)
    {
        if (p->scaleFactor == 1)
        {
            p->scaleFactor = 0;
        }
    }

    if (p->limitTU && (p->interRefine || p->bDynamicRefine))
    {
        s265_log(p, S265_LOG_WARNING, "Inter refinement does not support limitTU. Disabling limitTU.\n");
        p->limitTU = 0;
    }

    if (p->bDistributeModeAnalysis && (p->limitReferences >> 1) && 1)
    {
        s265_log(p, S265_LOG_WARNING, "Limit reference options 2 and 3 are not supported with pmode. Disabling limit reference\n");
        p->limitReferences = 0;
    }

    if (p->bEnableTemporalSubLayers && !p->bframes)
    {
        s265_log(p, S265_LOG_WARNING, "B frames not enabled, temporal sublayer disabled\n");
        p->bEnableTemporalSubLayers = 0;
    }

    int pyramidDelay = 2;
    if( p->bBPyramid== S265_B_PYRAMID_HIER )
    {
        if( p->bframes > 7 )
            pyramidDelay = 4;
        else if(p->bframes > 3 )
            pyramidDelay = 3;
    }
    m_bframeDelay = p->bframes ? (p->bBPyramid ? pyramidDelay : 1) : 0;

    p->bFrameBias = S265_MIN(S265_MAX(-90, p->bFrameBias), 100);
    p->scenecutBias = (double)(p->scenecutBias / 100);

    if (p->logLevel < S265_LOG_INFO)
    {
        /* don't measure these metrics if they will not be reported */
        p->bEnablePsnr = 0;
        p->bEnableSsim = 0;
    }
    /* Warn users trying to measure PSNR/SSIM with psy opts on. */
    if (p->bEnablePsnr || p->bEnableSsim)
    {
        const char *s = NULL;

        if (p->psyRd || p->psyRdoq)
        {
            s = p->bEnablePsnr ? "psnr" : "ssim";
            s265_log(p, S265_LOG_WARNING, "--%s used with psy on: results will be invalid!\n", s);
        }
        else if (!p->rc.aqMode && p->bEnableSsim)
        {
            s265_log(p, S265_LOG_WARNING, "--ssim used with AQ off: results will be invalid!\n");
            s = "ssim";
        }
        else if (p->rc.aqStrength > 0 && p->bEnablePsnr)
        {
            s265_log(p, S265_LOG_WARNING, "--psnr used with AQ on: results will be invalid!\n");
            s = "psnr";
        }
        if (s)
            s265_log(p, S265_LOG_WARNING, "--tune %s should be used if attempting to benchmark %s!\n", s, s);
    }
    if (p->searchMethod == S265_SEA && (p->bDistributeMotionEstimation || p->bDistributeModeAnalysis))
    {
        s265_log(p, S265_LOG_WARNING, "Disabling pme and pmode: --pme and --pmode cannot be used with SEA motion search!\n");
        p->bDistributeMotionEstimation = 0;
        p->bDistributeModeAnalysis = 0;
    }

    if ((p->rc.bStatWrite || p->rc.bStatRead) && p->rc.dataShareMode != S265_SHARE_MODE_FILE && p->rc.dataShareMode != S265_SHARE_MODE_SHAREDMEM)
    {
        p->rc.dataShareMode = S265_SHARE_MODE_FILE;
    }

    if (!p->rc.bStatRead || p->rc.rateControlMode != S265_RC_CRF)
    {
        p->rc.bEncFocusedFramesOnly = 0;
    }

    /* some options make no sense if others are disabled */
    p->bSaoNonDeblocked &= p->bEnableSAO;
    p->bEnableTSkipFast &= p->bEnableTransformSkip;
    p->bLimitSAO &= p->bEnableSAO;
    /* initialize the conformance window */
    m_conformanceWindow.bEnabled = false;
    m_conformanceWindow.rightOffset = 0;
    m_conformanceWindow.topOffset = 0;
    m_conformanceWindow.bottomOffset = 0;
    m_conformanceWindow.leftOffset = 0;

    uint32_t padsize = 0;

    /* set pad size if width is not multiple of the minimum CU size */
    if (p->confWinRightOffset)
    {
        if ((p->sourceWidth + p->confWinRightOffset) & (p->minCUSize - 1))
        {
            s265_log(p, S265_LOG_ERROR, "Incompatible conformance window right offset."
                                          " This when added to the source width should be a multiple of minCUSize\n");
            m_aborted = true;
        }
        else
        {
            p->sourceWidth += p->confWinRightOffset;
            m_conformanceWindow.bEnabled = true;
            m_conformanceWindow.rightOffset = p->confWinRightOffset;
        }
    }
    else if (p->sourceWidth & (p->minCUSize - 1))
    {
        uint32_t rem = p->sourceWidth & (p->minCUSize - 1);
        padsize = p->minCUSize - rem;
        p->sourceWidth += padsize;

        m_conformanceWindow.bEnabled = true;
        m_conformanceWindow.rightOffset = padsize;
    }

    if (p->bEnableRdRefine && (p->rdLevel < 5 || !p->rc.aqMode))
    {
        p->bEnableRdRefine = false;
        s265_log(p, S265_LOG_WARNING, "--rd-refine disabled, requires RD level > 4 and adaptive quant\n");
    }

    if (p->bOptCUDeltaQP && p->rdLevel < 5)
    {
        p->bOptCUDeltaQP = false;
        s265_log(p, S265_LOG_WARNING, "--opt-cu-delta-qp disabled, requires RD level > 4\n");
    }

    if (p->limitTU && p->tuQTMaxInterDepth < 2)
    {
        p->limitTU = 0;
        s265_log(p, S265_LOG_WARNING, "limit-tu disabled, requires tu-inter-depth > 1\n");
    }
    bool bIsVbv = m_param->rc.vbvBufferSize > 0 && m_param->rc.vbvMaxBitrate > 0;
    if (!m_param->bLossless && (m_param->rc.aqMode || bIsVbv || m_param->bAQMotion))
    {
        if (p->rc.qgSize < S265_MAX(8, p->minCUSize))
        {
            p->rc.qgSize = S265_MAX(8, p->minCUSize);
            s265_log(p, S265_LOG_WARNING, "QGSize should be greater than or equal to 8 and minCUSize, setting QGSize = %d\n", p->rc.qgSize);
        }
        if (p->rc.qgSize > p->maxCUSize)
        {
            p->rc.qgSize = p->maxCUSize;
            s265_log(p, S265_LOG_WARNING, "QGSize should be less than or equal to maxCUSize, setting QGSize = %d\n", p->rc.qgSize);
        }
    }
    else
        m_param->rc.qgSize = p->maxCUSize;

    if (m_param->dynamicRd && (!bIsVbv || !p->rc.aqMode || p->rdLevel > 4))
    {
        p->dynamicRd = 0;
        s265_log(p, S265_LOG_WARNING, "Dynamic-rd disabled, requires RD <= 4, VBV and aq-mode enabled\n");
    }

#ifdef ENABLE_HDR10_PLUS
    if (m_param->bDhdr10opt && m_param->toneMapFile == NULL)
    {
        s265_log(p, S265_LOG_WARNING, "Disabling dhdr10-opt. dhdr10-info must be enabled.\n");
        m_param->bDhdr10opt = 0;
    }

    if (m_param->toneMapFile)
    {
        if (!s265_fopen(p->toneMapFile, "r"))
        {
            s265_log(p, S265_LOG_ERROR, "Unable to open tone-map file.\n");
            m_bToneMap = 0;
            m_param->toneMapFile = NULL;
            m_aborted = true;
        }
        else
            m_bToneMap = 1;
    }
    else
        m_bToneMap = 0;
#else
    if (m_param->toneMapFile)
    {
        s265_log(p, S265_LOG_WARNING, "--dhdr10-info disabled. Enable HDR10_PLUS in cmake.\n");
        m_bToneMap = 0;
        m_param->toneMapFile = NULL;
    }
    else if (m_param->bDhdr10opt)
    {
        s265_log(p, S265_LOG_WARNING, "Disabling dhdr10-opt. dhdr10-info must be enabled.\n");
        m_param->bDhdr10opt = 0;
    }
#endif
    /* set pad size if height is not multiple of the minimum CU size */
    if (p->confWinBottomOffset)
    {
        if ((p->sourceHeight + p->confWinBottomOffset) & (p->minCUSize - 1))
        {
            s265_log(p, S265_LOG_ERROR, "Incompatible conformance window bottom offset."
                " This when added to the source height should be a multiple of minCUSize\n");
            m_aborted = true;
        }
        else
        {
            p->sourceHeight += p->confWinBottomOffset;
            m_conformanceWindow.bEnabled = true;
            m_conformanceWindow.bottomOffset = p->confWinBottomOffset;
        }
    }
    else if(p->sourceHeight & (p->minCUSize - 1))
    {
        uint32_t rem = p->sourceHeight & (p->minCUSize - 1);
        padsize = p->minCUSize - rem;
        p->sourceHeight += padsize;
        m_conformanceWindow.bEnabled = true;
        m_conformanceWindow.bottomOffset = padsize;
    }

    if (p->bLogCuStats)
        s265_log(p, S265_LOG_WARNING, "--cu-stats option is now deprecated\n");

    if (p->log2MaxPocLsb < 4)
    {
        s265_log(p, S265_LOG_WARNING, "maximum of the picture order count can not be less than 4\n");
        p->log2MaxPocLsb = 4;
    }

    if (p->maxSlices < 1)
    {
        s265_log(p, S265_LOG_WARNING, "maxSlices can not be less than 1, force set to 1\n");
        p->maxSlices = 1;
    }
    const uint32_t numRows = (p->sourceHeight + p->maxCUSize - 1) / p->maxCUSize;
    const uint32_t slicesLimit = S265_MIN(numRows, NALList::MAX_NAL_UNITS - 1);
    if (p->maxSlices > slicesLimit)
    {
        s265_log(p, S265_LOG_WARNING, "maxSlices can not be more than min(rows, MAX_NAL_UNITS-1), force set to %d\n", slicesLimit);
        p->maxSlices = slicesLimit;
    }
    if (p->bHDR10Opt)
    {
        if (p->internalCsp != S265_CSP_I420 || p->internalBitDepth != 10 || p->vui.colorPrimaries != 9 ||
            p->vui.transferCharacteristics != 16 || p->vui.matrixCoeffs != 9)
        {
            s265_log(p, S265_LOG_ERROR, "Recommended Settings for HDR10-opt: colour primaries should be BT.2020,\n"
                                        "                                            transfer characteristics should be SMPTE ST.2084,\n"
                                        "                                            matrix coeffs should be BT.2020,\n"
                                        "                                            the input video should be 10 bit 4:2:0\n"
                                        "                                            Disabling hdr10-opt.\n");
            p->bHDR10Opt = 0;
        }
    }

    if (m_param->toneMapFile || p->bHDR10Opt || p->bEmitHDR10SEI)
    {
        if (!p->bRepeatHeaders)
        {
            p->bRepeatHeaders = 1;
            s265_log(p, S265_LOG_WARNING, "Turning on repeat-headers for HDR compatibility\n");
        }
    }

    p->maxLog2CUSize = g_log2Size[p->maxCUSize];
    p->maxCUDepth    = p->maxLog2CUSize - g_log2Size[p->minCUSize];
    p->unitSizeDepth = p->maxLog2CUSize - LOG2_UNIT_SIZE;
    p->num4x4Partitions = (1U << (p->unitSizeDepth << 1));

    if (p->radl && p->bOpenGOP)
    {
        p->radl = 0;
        s265_log(p, S265_LOG_WARNING, "Radl requires closed gop structure. Disabling radl.\n");
    }

    if ((p->chunkStart || p->chunkEnd) && p->bOpenGOP && m_param->bResetZoneConfig)
    {
        p->chunkStart = p->chunkEnd = 0;
        s265_log(p, S265_LOG_WARNING, "Chunking requires closed gop structure. Disabling chunking.\n");
    }

    if (p->chunkEnd < p->chunkStart)
    {
        p->chunkStart = p->chunkEnd = 0;
        s265_log(p, S265_LOG_WARNING, "chunk-end cannot be less than chunk-start. Disabling chunking.\n");
    }

    if (p->dolbyProfile)     // Default disabled.
        configureDolbyVisionParams(p);

    if (p->rc.zonefileCount && p->rc.zoneCount)
    {
        p->rc.zoneCount = 0;
        s265_log(p, S265_LOG_WARNING, "Only zone or zonefile can be used. Enabling only zonefile\n");
    }

    if (m_param->rc.zonefileCount && p->bOpenGOP)
    {
        p->bOpenGOP = 0;
        s265_log(p, S265_LOG_WARNING, "Zone encoding requires closed gop structure. Enabling closed GOP.\n");
    }

    if (m_param->rc.zonefileCount && !p->bRepeatHeaders)
    {
        p->bRepeatHeaders = 1;
        s265_log(p, S265_LOG_WARNING, "Turning on repeat - headers for zone encoding\n");
    }

    if (m_param->bEnableHME)
    {
        if (m_param->sourceHeight < 540)
        {
            s265_log(p, S265_LOG_WARNING, "Source height < 540p is too low for HME. Disabling HME.\n");
            p->bEnableHME = 0;
        }
    }

    if (m_param->bEnableHME)
    {
        if (m_param->searchMethod != m_param->hmeSearchMethod[2])
            m_param->searchMethod = m_param->hmeSearchMethod[2];
        if (m_param->searchRange != m_param->hmeRange[2])
            m_param->searchRange = m_param->hmeRange[2];
    }

    if (p->bHistBasedSceneCut && !p->edgeTransitionThreshold)
    {
        p->edgeTransitionThreshold = 0.03;
        s265_log(p, S265_LOG_WARNING, "using  default threshold %.2lf for scene cut detection.\n", p->edgeTransitionThreshold);
    }

    if (!p->bHistBasedSceneCut && !p->bEnableTradScdInHscd)
    {
        p->bEnableTradScdInHscd = 1;
        s265_log(p, S265_LOG_WARNING, "option --no-traditional-scenecut requires --hist-scenecut to be enabled.\n");
    }
}

/* Toggle between two consecutive CTU rows. The save's CTU is copied
twice consecutively in the first and second CTU row of load*/

int Encoder::getCUIndex(cuLocation* cuLoc, uint32_t* count, int bytes, int flag)
{
    int index = 0;
    cuLoc->switchCondition += bytes;
    int isBoundaryW = (*count % (m_param->num4x4Partitions * cuLoc->widthInCU) == 0);

    /* Width boundary case :
    Skip to appropriate index when out of boundary cases occur
    Out of boundary may occur when the out of bound pixels along
    the width in low resoultion is greater than half of the maxCUSize */
    if (cuLoc->skipWidth && isBoundaryW)
    {
        if (flag)
            index++;
        else
        {
            /* Number of 4x4 blocks in out of bound region */
            int outOfBound = m_param->maxCUSize / 2;
            uint32_t sum = (uint32_t)pow((outOfBound >> 2), 2);
            index += sum;
        }
        cuLoc->switchCondition += m_param->num4x4Partitions;
    }

    /* Completed writing 2 CTUs - move to the last remembered index of the next CTU row*/
    if (cuLoc->switchCondition == 2 * m_param->num4x4Partitions)
    {
        if (isBoundaryW)
            cuLoc->evenRowIndex = *count + (m_param->num4x4Partitions * cuLoc->widthInCU); // end of row - skip to the next even row
        else
            cuLoc->evenRowIndex = *count;
        *count = cuLoc->oddRowIndex;

        /* Height boundary case :
        Skip to appropriate index when out of boundary cases occur
        Out of boundary may occur when the out of bound pixels along
        the height in low resoultion is greater than half of the maxCUSize */
        int isBoundaryH = (*count >= (m_param->num4x4Partitions * cuLoc->heightInCU * cuLoc->widthInCU));
        if (cuLoc->skipHeight && isBoundaryH)
        {
            if (flag)
                index += 2;
            else
            {
                int outOfBound = m_param->maxCUSize / 2;
                uint32_t sum = (uint32_t)(2 * pow((abs(outOfBound) >> 2), 2));
                index += sum;
            }
            *count = cuLoc->evenRowIndex;
            cuLoc->switchCondition = 0;
        }
    }
    /* Completed writing 4 CTUs - move to the last remembered index of
    the previous CTU row to copy the next save CTU's data*/
    else if (cuLoc->switchCondition == 4 * m_param->num4x4Partitions)
    {
        if (isBoundaryW)
            cuLoc->oddRowIndex = *count + (m_param->num4x4Partitions * cuLoc->widthInCU); // end of row - skip to the next odd row
        else
            cuLoc->oddRowIndex = *count;
        *count = cuLoc->evenRowIndex;
        cuLoc->switchCondition = 0;
    }
    return index;
}

/*      save                        load
                       CTU0    CTU1    CTU2    CTU3
        2NxN          2Nx2N   2Nx2N   2Nx2N   2Nx2N
        NX2N          2Nx2N   2Nx2N   2Nx2N   2Nx2N
        2NxnU          2NxN    2NxN   2Nx2N   2Nx2N
        2NxnD         2Nx2N   2Nx2N    2NxN    2NxN
        nLx2N          Nx2N   2Nx2N    Nx2N   2Nx2N
        nRx2N         2Nx2N    Nx2N    2Nx2N   Nx2N
*/
int Encoder::getPuShape(puOrientation* puOrient, int partSize, int numCTU)
{
    puOrient->isRect = true;
    if (partSize == SIZE_Nx2N)
        puOrient->isVert = true;
    if (partSize >= SIZE_2NxnU) // All AMP modes
    {
        puOrient->isAmp = true;
        puOrient->isRect = false;
        if (partSize == SIZE_2NxnD && numCTU > 1)
            return SIZE_2NxN;
        else if (partSize == SIZE_2NxnU && numCTU < 2)
            return SIZE_2NxN;
        else if (partSize == SIZE_nLx2N)
        {
            puOrient->isVert = true;
            if (!(numCTU % 2))
                return SIZE_Nx2N;
        }
        else if (partSize == SIZE_nRx2N)
        {
            puOrient->isVert = true;
            if (numCTU % 2)
                return SIZE_Nx2N;
        }
    }
    return SIZE_2Nx2N;
}

void Encoder::printReconfigureParams()
{
    if (!(m_reconfigure || m_reconfigureRc))
        return;
    s265_param* oldParam = m_param;
    s265_param* newParam = m_latestParam;
    
    s265_log(newParam, S265_LOG_DEBUG, "Reconfigured param options, input Frame: %d\n", m_pocLast + 1);

    char tmp[60];
#define TOOLCMP(COND1, COND2, STR)  if (COND1 != COND2) { sprintf(tmp, STR, COND1, COND2); s265_log(newParam, S265_LOG_DEBUG, tmp); }
    TOOLCMP(oldParam->maxNumReferences, newParam->maxNumReferences, "ref=%d to %d\n");
    TOOLCMP(oldParam->bEnableFastIntra, newParam->bEnableFastIntra, "fast-intra=%d to %d\n");
    TOOLCMP(oldParam->bEnableEarlySkip, newParam->bEnableEarlySkip, "early-skip=%d to %d\n");
    TOOLCMP(oldParam->recursionSkipMode, newParam->recursionSkipMode, "rskip=%d to %d\n");
    TOOLCMP(oldParam->searchMethod, newParam->searchMethod, "me=%d to %d\n");
    TOOLCMP(oldParam->searchRange, newParam->searchRange, "merange=%d to %d\n");
    TOOLCMP(oldParam->subpelRefine, newParam->subpelRefine, "subme= %d to %d\n");
    TOOLCMP(oldParam->rdLevel, newParam->rdLevel, "rd=%d to %d\n");
    TOOLCMP(oldParam->rdoqLevel, newParam->rdoqLevel, "rdoq=%d to %d\n" );
    TOOLCMP(oldParam->bEnableRectInter, newParam->bEnableRectInter, "rect=%d to %d\n");
    TOOLCMP(oldParam->maxNumMergeCand, newParam->maxNumMergeCand, "max-merge=%d to %d\n");
    TOOLCMP(oldParam->bIntraInBFrames, newParam->bIntraInBFrames, "b-intra=%d to %d\n");
    TOOLCMP(oldParam->scalingLists, newParam->scalingLists, "scalinglists=%s to %s\n");
    TOOLCMP(oldParam->rc.vbvMaxBitrate, newParam->rc.vbvMaxBitrate, "vbv-maxrate=%d to %d\n");
    TOOLCMP(oldParam->rc.vbvBufferSize, newParam->rc.vbvBufferSize, "vbv-bufsize=%d to %d\n");
    TOOLCMP(oldParam->rc.bitrate, newParam->rc.bitrate, "bitrate=%d to %d\n");
    TOOLCMP(oldParam->rc.rfConstant, newParam->rc.rfConstant, "crf=%f to %f\n");
}

void Encoder::readUserSeiFile(s265_sei_payload& seiMsg, int curPoc)
{
    char line[1024];
    while (fgets(line, sizeof(line), m_naluFile))
    {
        int poc = atoi(strtok(line, " "));
        char *prefix = strtok(NULL, " ");
        int nalType = atoi(strtok(NULL, "/"));
        int payloadType = atoi(strtok(NULL, " "));
        char *base64Encode = strtok(NULL, "\n");
        int base64EncodeLength = (int)strlen(base64Encode);
        char *base64Decode = SEI::base64Decode(base64Encode, base64EncodeLength);
        if (nalType == NAL_UNIT_PREFIX_SEI && (!strcmp(prefix, "PREFIX")))
        {
            int currentPOC = curPoc;
            if (currentPOC == poc)
            {
                seiMsg.payloadSize = (base64EncodeLength / 4) * 3;
                seiMsg.payload = (uint8_t*)s265_malloc(sizeof(uint8_t) * seiMsg.payloadSize);
                if (!seiMsg.payload)
                {
                    s265_log(m_param, S265_LOG_ERROR, "Unable to allocate memory for SEI payload\n");
                    break;
                }
                if (payloadType == 4)
                    seiMsg.payloadType = USER_DATA_REGISTERED_ITU_T_T35;
                else if (payloadType == 5)
                    seiMsg.payloadType = USER_DATA_UNREGISTERED;
                else
                {
                    s265_log(m_param, S265_LOG_WARNING, "Unsupported SEI payload Type for frame %d\n", poc);
                    break;
                }
                memcpy(seiMsg.payload, base64Decode, seiMsg.payloadSize);
                break;
            }
        }
        else
        {
            s265_log(m_param, S265_LOG_WARNING, "SEI message for frame %d is not inserted. Will support only PREFIX SEI messages.\n", poc);
            break;
        }
    }
}

bool Encoder::computeSPSRPSIndex()
{
    RPS* rpsInSPS = m_sps.spsrps;
    int* rpsNumInPSP = &m_sps.spsrpsNum;
    int  beginNum = m_sps.numGOPBegin;
    int  endNum;
    RPS* rpsInRec;
    RPS* rpsInIdxList;
    RPS* thisRpsInSPS;
    RPS* thisRpsInList;
    RPSListNode* headRpsIdxList = NULL;
    RPSListNode* tailRpsIdxList = NULL;
    RPSListNode* rpsIdxListIter = NULL;
    RateControlEntry *rce2Pass = m_rateControl->m_rce2Pass;
    int numEntries = m_rateControl->m_numEntries;
    RateControlEntry *rce;
    int idx = 0;
    int pos = 0;
    int resultIdx[64];
    memset(rpsInSPS, 0, sizeof(RPS) * MAX_NUM_SHORT_TERM_RPS);

    // find out all RPS date in current GOP
    beginNum++;
    endNum = beginNum;
    if (!m_param->bRepeatHeaders)
    {
        endNum = numEntries;
    }
    else
    {
        while (endNum < numEntries)
        {
            rce = &rce2Pass[endNum];
            if (rce->sliceType == I_SLICE)
            {
                if (m_param->keyframeMin && (endNum - beginNum + 1 < m_param->keyframeMin))
                {
                    endNum++;
                    continue;
                }
                break;
            }
            endNum++;
        }
    }
    m_sps.numGOPBegin = endNum;

    // find out all kinds of RPS
    for (int i = beginNum; i < endNum; i++)
    {
        rce = &rce2Pass[i];
        rpsInRec = &rce->rpsData;
        rpsIdxListIter = headRpsIdxList;
        // i frame don't recode RPS info
        if (rce->sliceType != I_SLICE)
        {
            while (rpsIdxListIter)
            {
                rpsInIdxList = rpsIdxListIter->rps;
                if (rpsInRec->numberOfPictures == rpsInIdxList->numberOfPictures
                    && rpsInRec->numberOfNegativePictures == rpsInIdxList->numberOfNegativePictures
                    && rpsInRec->numberOfPositivePictures == rpsInIdxList->numberOfPositivePictures)
                {
                    for (pos = 0; pos < rpsInRec->numberOfPictures; pos++)
                    {
                        if (rpsInRec->deltaPOC[pos] != rpsInIdxList->deltaPOC[pos]
                            || rpsInRec->bUsed[pos] != rpsInIdxList->bUsed[pos])
                            break;
                    }
                    if (pos == rpsInRec->numberOfPictures)    // if this type of RPS has exist
                    {
                        rce->rpsIdx = rpsIdxListIter->idx;
                        rpsIdxListIter->count++;
                        // sort RPS type link after reset RPS type count.
                        RPSListNode* next = rpsIdxListIter->next;
                        RPSListNode* prior = rpsIdxListIter->prior;
                        RPSListNode* iter = prior;
                        if (iter)
                        {
                            while (iter)
                            {
                                if (iter->count > rpsIdxListIter->count)
                                    break;
                                iter = iter->prior;
                            }
                            if (iter)
                            {
                                prior->next = next;
                                if (next)
                                    next->prior = prior;
                                else
                                    tailRpsIdxList = prior;
                                rpsIdxListIter->next = iter->next;
                                rpsIdxListIter->prior = iter;
                                iter->next->prior = rpsIdxListIter;
                                iter->next = rpsIdxListIter;
                            }
                            else
                            {
                                prior->next = next;
                                if (next)
                                    next->prior = prior;
                                else
                                    tailRpsIdxList = prior;
                                headRpsIdxList->prior = rpsIdxListIter;
                                rpsIdxListIter->next = headRpsIdxList;
                                rpsIdxListIter->prior = NULL;
                                headRpsIdxList = rpsIdxListIter;
                            }
                        }
                        break;
                    }
                }
                rpsIdxListIter = rpsIdxListIter->next;
            }
            if (!rpsIdxListIter)  // add new type of RPS
            {
                RPSListNode* newIdxNode = new RPSListNode();
                if (newIdxNode == NULL)
                    goto fail;
                newIdxNode->rps = rpsInRec;
                newIdxNode->idx = idx++;
                newIdxNode->count = 1;
                newIdxNode->next = NULL;
                newIdxNode->prior = NULL;
                if (!tailRpsIdxList)
                    tailRpsIdxList = headRpsIdxList = newIdxNode;
                else
                {
                    tailRpsIdxList->next = newIdxNode;
                    newIdxNode->prior = tailRpsIdxList;
                    tailRpsIdxList = newIdxNode;
                }
                rce->rpsIdx = newIdxNode->idx;
            }
        }
        else
        {
            rce->rpsIdx = -1;
        }
    }

    // get commonly RPS set
    memset(resultIdx, 0, sizeof(resultIdx));
    if (idx > MAX_NUM_SHORT_TERM_RPS)
        idx = MAX_NUM_SHORT_TERM_RPS;

    *rpsNumInPSP = idx;
    rpsIdxListIter = headRpsIdxList;
    for (int i = 0; i < idx; i++)
    {
        resultIdx[i] = rpsIdxListIter->idx;
        m_rpsInSpsCount += rpsIdxListIter->count;
        thisRpsInSPS = rpsInSPS + i;
        thisRpsInList = rpsIdxListIter->rps;
        thisRpsInSPS->numberOfPictures = thisRpsInList->numberOfPictures;
        thisRpsInSPS->numberOfNegativePictures = thisRpsInList->numberOfNegativePictures;
        thisRpsInSPS->numberOfPositivePictures = thisRpsInList->numberOfPositivePictures;
        for (pos = 0; pos < thisRpsInList->numberOfPictures; pos++)
        {
            thisRpsInSPS->deltaPOC[pos] = thisRpsInList->deltaPOC[pos];
            thisRpsInSPS->bUsed[pos] = thisRpsInList->bUsed[pos];
        }
        rpsIdxListIter = rpsIdxListIter->next;
    }

    //reset every frame's RPS index
    for (int i = beginNum; i < endNum; i++)
    {
        int j;
        rce = &rce2Pass[i];
        for (j = 0; j < idx; j++)
        {
            if (rce->rpsIdx == resultIdx[j])
            {
                rce->rpsIdx = j;
                break;
            }
        }

        if (j == idx)
            rce->rpsIdx = -1;
    }

    rpsIdxListIter = headRpsIdxList;
    while (rpsIdxListIter)
    {
        RPSListNode* freeIndex = rpsIdxListIter;
        rpsIdxListIter = rpsIdxListIter->next;
        delete freeIndex;
    }
    return true;

fail:
    rpsIdxListIter = headRpsIdxList;
    while (rpsIdxListIter)
    {
        RPSListNode* freeIndex = rpsIdxListIter;
        rpsIdxListIter = rpsIdxListIter->next;
        delete freeIndex;
    }
    return false;
}

