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

#ifndef S265_ENCODER_H
#define S265_ENCODER_H

#include "common.h"
#include "slice.h"
#include "threading.h"
#include "scalinglist.h"
#include "s265.h"
#include "nal.h"
#include "framedata.h"
#include "svt.h"
#ifdef ENABLE_HDR10_PLUS
    #include "dynamicHDR10/hdr10plus.h"
#endif
struct s265_encoder {};
namespace S265_NS {
// private namespace
extern const char g_sliceTypeToChar[3];

class Entropy;

#ifdef SVT_HEVC
typedef struct SvtAppContext
{
    EB_COMPONENTTYPE*          svtEncoderHandle;
    EB_H265_ENC_CONFIGURATION* svtHevcParams;

    // Buffer Pools
    EB_BUFFERHEADERTYPE*       inputPictureBuffer;
    uint64_t                   byteCount;
    uint64_t                   outFrameCount;

}SvtAppContext;
#endif

struct EncStats
{
    double        m_psnrSumY;
    double        m_psnrSumU;
    double        m_psnrSumV;
    double        m_globalSsim;
    double        m_totalQp;
    double        m_maxFALL;
    uint64_t      m_accBits;
    uint32_t      m_numPics;
    uint16_t      m_maxCLL;

    EncStats()
    {
        m_psnrSumY = m_psnrSumU = m_psnrSumV = m_globalSsim = 0;
        m_accBits = 0;
        m_numPics = 0;
        m_totalQp = 0;
        m_maxCLL = 0;
        m_maxFALL = 0;
    }

    void addQP(double aveQp);

    void addPsnr(double psnrY, double psnrU, double psnrV);

    void addBits(uint64_t bits);

    void addSsim(double ssim);
};

#define MAX_NUM_REF_IDX 64

struct RefIdxLastGOP
{
    int numRefIdxDefault[2];
    int numRefIdxl0[MAX_NUM_REF_IDX];
    int numRefIdxl1[MAX_NUM_REF_IDX];
};

struct RPSListNode
{
    int idx;
    int count;
    RPS* rps;
    RPSListNode* next;
    RPSListNode* prior;
};

struct cuLocation
{
    bool skipWidth;
    bool skipHeight;
    uint32_t heightInCU;
    uint32_t widthInCU;
    uint32_t oddRowIndex;
    uint32_t evenRowIndex;
    uint32_t switchCondition;

    void init(s265_param* param)
    {
        skipHeight = false;
        skipWidth = false;
        heightInCU = (param->sourceHeight + param->maxCUSize - 1) >> param->maxLog2CUSize;
        widthInCU = (param->sourceWidth + param->maxCUSize - 1) >> param->maxLog2CUSize;
        evenRowIndex = 0;
        oddRowIndex = param->num4x4Partitions * widthInCU;
        switchCondition = 0; // To switch between odd and even rows
    }
};

struct puOrientation
{
    bool isVert;
    bool isRect;
    bool isAmp;

    void init()
    {
        isRect = false;
        isAmp = false;
        isVert = false;
    }
};

class FrameEncoder;
class DPB;
class Lookahead;
class RateControl;
class ThreadPool;
class FrameData;

#define MAX_SCENECUT_THRESHOLD 1.0
#define SCENECUT_STRENGTH_FACTOR 2.0
#define MIN_EDGE_FACTOR 0.5
#define MAX_EDGE_FACTOR 1.5
#define SCENECUT_CHROMA_FACTOR 10.0

class Encoder : public s265_encoder
{
public:

    uint32_t           m_residualSumEmergency[MAX_NUM_TR_CATEGORIES][MAX_NUM_TR_COEFFS];
    uint32_t           m_countEmergency[MAX_NUM_TR_CATEGORIES];
    uint16_t           (*m_offsetEmergency)[MAX_NUM_TR_CATEGORIES][MAX_NUM_TR_COEFFS];

    int64_t            m_firstPts;
    int64_t            m_bframeDelayTime;
    int64_t            m_prevReorderedPts[4]; //hier B in 15bframs is 4
    int64_t            m_encodeStartTime;

    int                m_pocLast;         // time index (POC)  初始化为-1,每输入一帧自加1
    int                m_encodedFrameNum; //已经送给编码器进行编码的帧数量统计
    int                m_outputCount;     // 从编码器已经输出的帧数统计
    int                m_bframeDelay;//类似num_reordered frames
    int                m_numPools;
    int                m_curEncoder;

    // weighted prediction
    int                m_numLumaWPFrames;    // number of P frames with weighted luma reference
    int                m_numChromaWPFrames;  // number of P frames with weighted chroma reference
    int                m_numLumaWPBiFrames;  // number of B frames with weighted luma reference
    int                m_numChromaWPBiFrames; // number of B frames with weighted chroma reference
    int                m_conformanceMode;
    int                m_lastBPSEI;
    uint32_t           m_numDelayedPic;


    ThreadPool*        m_threadPool;//编码器中创建的线程池，供需要工作线程的任务使用
    FrameEncoder*      m_frameEncoder[S265_MAX_FRAME_THREADS];//编码器创建几个帧并行编码单元，每个有自己独立的后台线程，等待编码分配过来的帧
    DPB*               m_dpb;
    Frame*             m_exportedPic;
    FILE*              m_naluFile;
    s265_param*        m_param;
    s265_param*        m_latestParam;     // Holds latest param during a reconfigure
    RateControl*       m_rateControl;
    Lookahead*         m_lookahead;//任务成员，创建后不带线程，Encoder 通过其接口addPicture来驱动它唤醒工作线程来处理帧队列，生成待编码器由调用者通过getDecidedPicture取出

    bool               m_externalFlush;
    /* Collect statistics globally */
    EncStats           m_analyzeAll;
    EncStats           m_analyzeI;
    EncStats           m_analyzeP;
    EncStats           m_analyzeB;
    VPS                m_vps;
    SPS                m_sps;
    PPS                m_pps;
    NALList            m_nalList;
    ScalingList        m_scalingList;      // quantization matrix information
    Window             m_conformanceWindow;

    bool               m_bZeroLatency;     // s265_encoder_encode() returns NALs for the input picture, zero lag
    bool               m_aborted;          // fatal error detected
    bool               m_reconfigure;      // Encoder reconfigure in progress
    bool               m_reconfigureRc;
    bool               m_reconfigureZone;

    int                m_saveCtuDistortionLevel;

    /* Begin intra refresh when one not in progress or else begin one as soon as the current 
     * one is done. Requires bIntraRefresh to be set.*/
    int                m_bQueuedIntraRefresh;

    /* For optimising slice QP */
    Lock               m_sliceQpLock;
    int                m_iFrameNum;   
    int                m_iPPSQpMinus26;
    int64_t            m_iBitsCostSum[QP_MAX_MAX + 1];
    Lock               m_sliceRefIdxLock;
    RefIdxLastGOP      m_refIdxLastGOP;

    Lock               m_rpsInSpsLock;
    int                m_rpsInSpsCount;
    /* For HDR*/
    double             m_cB;
    double             m_cR;

    int                m_bToneMap; // Enables tone-mapping
    int                m_enableNal;

    /* For histogram based scene-cut detection */
    pixel*             m_edgePic;
    pixel*             m_inputPic[3];
    int32_t            m_curYUVHist[3][HISTOGRAM_BINS];
    int32_t            m_prevYUVHist[3][HISTOGRAM_BINS];
    int32_t            m_curEdgeHist[2];
    int32_t            m_prevEdgeHist[2];
    uint32_t           m_planeSizes[3];
    double             m_edgeHistThreshold;
    double             m_chromaHistThreshold;
    double             m_scaledEdgeThreshold;
    double             m_scaledChromaThreshold;

#ifdef ENABLE_HDR10_PLUS
    const hdr10plus_api     *m_hdr10plus_api;
    uint8_t                 **m_cim;
    int                     m_numCimInfo;
#endif

#ifdef SVT_HEVC
    SvtAppContext*          m_svtAppData;
#endif

    s265_sei_payload        m_prevTonemapPayload;

    int                     m_zoneIndex;

    /* Collect frame level feature data */
    uint64_t*               m_rdCost;
    uint64_t*               m_variance;
    uint32_t*               m_trainingCount;
    int32_t                 m_startPoint;
    Lock                    m_dynamicRefineLock;

    bool                    m_saveCTUSize;


    ThreadSafeInteger* zoneReadCount;
    ThreadSafeInteger* zoneWriteCount;

    Encoder();
    ~Encoder()
    {
#ifdef ENABLE_HDR10_PLUS
        if (m_prevTonemapPayload.payload != NULL)
            S265_FREE(m_prevTonemapPayload.payload);
#endif
    };

    void create();
    void stopJobs();
    void destroy();

    int encode(const s265_picture* pic, s265_picture *pic_out);

    int reconfigureParam(s265_param* encParam, s265_param* param);

    bool isReconfigureRc(s265_param* latestParam, s265_param* param_in);

    int copySlicetypePocAndSceneCut(int *slicetype, int *poc, int *sceneCut);

    int getRefFrameList(PicYuv** l0, PicYuv** l1, int sliceType, int poc, int* pocL0, int* pocL1);

    void getStreamHeaders(NALList& list, Entropy& sbacCoder, Bitstream& bs);

    void getEndNalUnits(NALList& list, Bitstream& bs);

    void fetchStats(s265_stats* stats, size_t statsSizeBytes);

    void printSummary();

    void printReconfigureParams();

    char* statsString(EncStats&, char*);

    void configure(s265_param *param);

    void configureZone(s265_param *p, s265_param *zone);

    void updateVbvPlan(RateControl* rc);

    int getCUIndex(cuLocation* cuLoc, uint32_t* count, int bytes, int flag);

    int getPuShape(puOrientation* puOrient, int partSize, int numCTU);

    void finishFrameStats(Frame* pic, FrameEncoder *curEncoder, s265_frame_stats* frameStats, int inPoc);

    void readUserSeiFile(s265_sei_payload& seiMsg, int poc);

    void calcRefreshInterval(Frame* frameEnc);

    uint64_t computeSSD(pixel *fenc, pixel *rec, intptr_t stride, uint32_t width, uint32_t height);


    void copyPicture(s265_picture *dest, const s265_picture *src);

    bool computeHistograms(s265_picture *pic);
    void computeHistogramSAD(double *maxUVNormalizedSAD, double *edgeNormalizedSAD, int curPoc);
    double normalizeRange(int32_t value, int32_t minValue, int32_t maxValue, double rangeStart, double rangeEnd);
    void findSceneCuts(s265_picture *pic, bool& bDup, double m_maxUVSADVal, double m_edgeSADVal, bool& isMaxThres, bool& isHardSC);

    void initRefIdx();
    void analyseRefIdx(int *numRefIdx);
    void updateRefIdx();

    void copyUserSEIMessages(Frame *frame, const s265_picture* pic_in);

    void configureDolbyVisionParams(s265_param* p);

protected:

    void initVPS(VPS *vps);
    void initSPS(SPS *sps);
    void initPPS(PPS *pps);
};
}

#endif // ifndef S265_ENCODER_H
