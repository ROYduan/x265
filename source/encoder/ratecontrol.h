/*****************************************************************************
 * Copyright (C) 2013-2020 MulticoreWare, Inc
 *
 * Authors: Sumalatha Polureddy <sumalatha@multicorewareinc.com>
 *          Aarthi Priya Thirumalai <aarthi@multicorewareinc.com>
 *          Xun Xu, PPLive Corporation <xunxu@pptv.com>
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

#ifndef S265_RATECONTROL_H
#define S265_RATECONTROL_H

#include "common.h"
#include "sei.h"

namespace S265_NS {
// encoder namespace

class Encoder;
class Frame;
class SEIBufferingPeriod;
struct SPS;
#define BASE_FRAME_DURATION 0.04

/* Arbitrary limitations as a sanity check. */
#define MAX_FRAME_DURATION 1.00
#define MIN_FRAME_DURATION 0.01

#define MIN_AMORTIZE_FRAME 10
#define MIN_AMORTIZE_FRACTION 0.2
#define CLIP_DURATION(f) s265_clip3(MIN_FRAME_DURATION, MAX_FRAME_DURATION, f)

/*Scenecut Aware QP*/
#define WINDOW1_DELTA           1.0 /* The offset for the frames coming in the window-1*/
#define WINDOW2_DELTA           0.7 /* The offset for the frames coming in the window-2*/
#define WINDOW3_DELTA           0.4 /* The offset for the frames coming in the window-3*/

struct Predictor
{
    double coeffMin;
    double coeff;
    double count;
    double decay;
    double offset;
};

struct HRDTiming
{
    double cpbInitialAT;
    double cpbFinalAT;
    double dpbOutputTime;
    double cpbRemovalTime;
};

struct RateControlEntry
{
    Predictor  rowPreds[4][2];//[B/P/I/Bref][pred_s/pred_intra]
    Predictor* rowPred[2];

    int64_t currentSatd;
    int64_t lastSatd;      /* Contains the picture cost of the previous frame, required for resetAbr and VBV */
    int64_t leadingNoBSatd;
    int64_t rowTotalBits;  /* update cplxrsum and totalbits at the end of 2 rows */
    double  blurredComplexity;
    double  qpaRc;
    double  qpAq;
    double  qRceq;
    double  qpPrev;
    double  frameSizePlanned;  /* frame Size decided by RateCotrol before encoding the frame */
    double  bufferRate;
    double  movingAvgSum;
    double  rowCplxrSum;
    double  qpNoVbv;
    double  bufferFill;
    double  bufferFillFinal;
    double  bufferFillActual;
    double  targetFill;
    bool    vbvEndAdj;
    double  frameDuration;
    double  clippedDuration;
    double  frameSizeEstimated; /* hold frameSize, updated from cu level vbv rc */
    double  frameSizeMaximum;   /* max frame Size according to minCR restrictions and level of the video */
    int     sliceType;
    int     bframes;
    int     poc;
    int     encodeOrder;// 当前编码器编码的帧 是第几个帧num（每送一个已经决策好了的帧给编码器，帧order 就会++ß）
    bool    bLastMiniGopBFrame;
    bool    isActive;
    double  amortizeFrames;
    double  amortizeFraction;
    /* Required in 2-pass rate control */
    uint64_t expectedBits; /* total expected bits up to the current frame (current one excluded) */
    double   iCuCount;
    double   pCuCount;
    double   skipCuCount;
    double   expectedVbv;
    double   qScale;
    double   newQScale;
    double   newQp;
    int      miscBits;
    bool     keptAsRef;
    bool     scenecut;
    bool     isIdr;
    SEIPictureTiming *picTimingSEI;
    HRDTiming        *hrdTiming;
    int      rpsIdx;
    RPS      rpsData;
    bool     isFadeEnd;
};

class RateControl
{
public:

    s265_param* m_param;
    Slice*      m_curSlice;      /* all info about the current frame */
    SliceType   m_sliceType;     /* Current frame type */
    int         m_ncu;           /* number of CUs in a frame */
    int         m_qp;            /* updated qp for current frame */

    bool   m_isAbr;
    bool   m_isVbv;// 是否开启了vbv
    bool   m_isCbr;
    bool   m_singleFrameVbv;
    bool   m_isGrainEnabled;
    bool   m_isAbrReset;
    bool   m_isNextGop;
    bool   m_initVbv;
    int    m_lastAbrResetPoc;

    int    m_lastScenecut;
    int    m_lastScenecutAwareIFrame;
    double m_rateTolerance;
    double m_frameDuration;     /* current frame duration in seconds */
    double m_bitrate;
    double m_rateFactorConstant;
    double m_bufferSize;
    double m_bufferFillFinal;  /* real buffer as of the last finished frame */
    double m_unclippedBufferFillFinal; /* real unclipped buffer as of the last finished frame used to log in CSV*/
    double m_bufferFill;       /* planned buffer, if all in-progress frames hit their bit budget */
    double m_bufferRate;       /* # of bits added to buffer_fill after each frame */
    double m_vbvMaxRate;       /* in kbps */
    double m_rateFactorMaxIncrement; /* Don't allow RF above (CRF + this value). */
    double m_rateFactorMaxDecrement; /* don't allow RF below (this value). */
    double m_avgPFrameQp;
    double m_bufferExcess;
    double m_minBufferFill;
    double m_maxBufferFill;
    bool   m_isFirstMiniGop;
    Predictor m_pred[3][4];       /* [3]:SIMPLE/NORMAL/COMPLEX Slice predictors to preidct bits for each Slice type - [4]:I,P,Bref and B */
    int64_t m_leadingNoBSatd;
    int     m_predType;       /* Type of slice predictors to be used - depends on the slice type b/p/I/Bref */
    int     m_sceneType;
    double  m_ipOffset;
    double  m_pbOffset;
    int64_t m_bframeBits;
    int64_t m_currentSatd;
    int     m_qpConstant[3];
    int     m_lastNonBPictType;
    int     m_framesDone;        /* # of frames passed through RateCotrol already 完成了ratecontrolstart调用后的帧数*/

    double  m_cplxrSum;          /* sum of bits*qscale/rceq */
    double  m_wantedBitsWindow;  /* target bitrate * window */
    double  m_accumPQp;          /* for determining I-frame quant */
    double  m_accumPNorm;
    double  m_lastQScaleFor[4];  /* last qscale for a specific m_predType, used for max_diff & ipb factor stuff */
    double  m_lstep;
    double  m_lmin[4];
    double  m_lmax[4];
    double  m_shortTermCplxSum;
    double  m_shortTermCplxCount;
    double  m_lastRceq;
    double  m_qCompress;
    int64_t m_totalBits;        /* total bits used for already encoded frames (after ammortization) */
    int64_t m_encodedBits;      /* bits used for encoded frames (without ammortization) */
    double  m_fps;
    int64_t m_satdCostWindow[50];
    int64_t m_encodedBitsWindow[50];
    int     m_sliderPos;
    int64_t m_lastRemovedSatdCost;
    double  m_movingAvgSum;

    /* To detect a pattern of low detailed static frames in single pass ABR using satdcosts */
    int64_t m_lastBsliceSatdCost;
    int     m_numBframesInPattern;
    bool    m_isPatternPresent;
    bool    m_isSceneTransition;
    int     m_lastPredictorReset;
    double  m_qpToEncodedBits[QP_MAX_MAX + 1];
    /* a common variable on which rateControlStart, rateControlEnd and rateControUpdateStats waits to
     * sync the calls to these functions. For example
     * -F2:
     * rceStart  10
     * rceUpdate 10
     * rceEnd    9
     * rceStart  11
     * rceUpdate 11
     * rceEnd    10
     * rceStart  12
     * rceUpdate 12
     * rceEnd    11 */
    // 此变量用于控制整个编码器中码率控制，何时可以执行rateControlStart,何时可以执行rateControlEnd
    // 变量的变换情况是 start后在update中+1, 然后在end中+1
    // 执行条件是，在调用start 前 需要等在并行编码中的最早的一帧完成end的调用
    // 在调用end之前，需要等在并行编码中的最新的一帧完成update调用
    ThreadSafeInteger m_startEndOrder;

    int     m_finalFrameCount;   /* set when encoder begins flushing */
    bool    m_bTerminated;       /* set true when encoder is closing */

    /* hrd stuff */
    SEIBufferingPeriod m_bufPeriodSEI;
    double  m_nominalRemovalTime;
    double  m_prevCpbFinalAT;

    /* 2 pass */
    int     m_numEntries;
    Encoder* m_top;

    RateControl(s265_param& p, Encoder *enc);
    bool init(const SPS& sps);
    void reconfigureRC();

    void setFinalFrameCount(int count);
    void terminate();          /* un-block all waiting functions so encoder may close */
    void destroy();

    // to be called for each curFrame to process RateControl and set QP
    int  rateControlStart(Frame* curFrame, RateControlEntry* rce, Encoder* enc);
    void rateControlUpdateStats(RateControlEntry* rce);
    int  rateControlEnd(Frame* curFrame, int64_t bits, RateControlEntry* rce, int *filler);
    int  rowVbvRateControl(Frame* curFrame, uint32_t row, RateControlEntry* rce, double& qpVbv, uint32_t* m_sliceBaseRow, uint32_t sliceId, uint32_t wppFlag);
    void hrdFullness(SEIBufferingPeriod* sei);

    double forwardMasking(Frame* curFrame, double q);
    double backwardMasking(Frame* curFrame, double q);

protected:

    static const int   s_slidingWindowFrames;

    double m_amortizeFraction;
    int    m_amortizeFrames;
    int    m_residualFrames;
    int    m_partialResidualFrames;
    int    m_residualCost;
    int    m_partialResidualCost;
    double getQScale(RateControlEntry *rce, double rateFactor);
    double rateEstimateQscale(Frame* pic, RateControlEntry *rce); // main logic for calculating QP based on ABR
    double tuneAbrQScaleFromFeedback(double qScale);
    void   accumPQpUpdate();

    int    getPredictorType(int lowresSliceType, int sliceType);
    int    updateVbv(int64_t bits, RateControlEntry* rce);
    void   updatePredictor(Predictor *p, double q, double var, double bits);
    void   calculateFrameQ(double frameQ[], double q, int predType);
    double clipQscale(Frame* pic, RateControlEntry* rce, double q);
    void   updateVbvPlan(Encoder* enc);
    double predictSize(Predictor *p, double q, double var);
    void   checkAndResetABR(RateControlEntry* rce, bool isFrameDone);
    double predictRowsSizeSum(Frame* pic, RateControlEntry* rce, double qpm, int32_t& encodedBits);
    void   initFramePredictors();
    double tuneQScaleForGrain(double rcOverflow);
};
}
#endif // ifndef S265_RATECONTROL_H
