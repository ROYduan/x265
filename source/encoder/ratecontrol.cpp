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

#if _MSC_VER
#pragma warning(disable: 4127) // conditional expression is constant, yes I know
#endif

#include "common.h"
#include "param.h"
#include "frame.h"
#include "framedata.h"
#include "picyuv.h"

#include "encoder.h"
#include "slicetype.h"
#include "ratecontrol.h"
#include "sei.h"

#define BR_SHIFT  6
#define CPB_SHIFT 4

#define SHARED_DATA_ALIGNMENT      4 ///< 4btye, 32bit
#define CUTREE_SHARED_MEM_NAME     "cutree"
#define GOP_CNT_CU_TREE            3

using namespace S265_NS;

/* Amortize the partial cost of I frames over the next N frames */

const int RateControl::s_slidingWindowFrames = 20;

namespace {
#define CMP_OPT_FIRST_PASS(opt, param_val)\
{\
    bErr = 0;\
    p = strstr(opts, opt "=");\
    char* q = strstr(opts, "no-" opt " ");\
    if (p && sscanf(p, opt "=%d" , &i) && param_val != i)\
        bErr = 1;\
    else if (!param_val && !q && !p)\
        bErr = 1;\
    else if (param_val && (q || !strstr(opts, opt)))\
        bErr = 1;\
    if (bErr)\
    {\
        s265_log(m_param, S265_LOG_ERROR, "different " opt " setting than first pass (%d vs %d)\n", param_val, i);\
        return false;\
    }\
}

inline int calcScale(uint32_t x)
{
    static uint8_t lut[16] = {4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0};
    int y, z = (((x & 0xffff) - 1) >> 27) & 16;
    x >>= z;
    z += y = (((x & 0xff) - 1) >> 28) & 8;
    x >>= y;
    z += y = (((x & 0xf) - 1) >> 29) & 4;
    x >>= y;
    return z + lut[x&0xf];
}

inline int calcLength(uint32_t x)
{
    static uint8_t lut[16] = {4, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0};
    int y, z = (((x >> 16) - 1) >> 27) & 16;
    x >>= z ^ 16;
    z += y = ((x - 0x100) >> 28) & 8;
    x >>= y ^ 8;
    z += y = ((x - 0x10) >> 29) & 4;
    x >>= y ^ 4;
    return z + lut[x];
}

}  // end anonymous namespace
/* Returns the zone for the current frame */
s265_zone* RateControl::getZone()
{
    for (int i = m_param->rc.zoneCount - 1; i >= 0; i--)
    {
        s265_zone *z = &m_param->rc.zones[i];
        if (m_framesDone + 1 >= z->startFrame && m_framesDone < z->endFrame)
            return z;
    }
    return NULL;
}

RateControl::RateControl(s265_param& p, Encoder *top)
{
    m_param = &p;
    m_top = top;
    int lowresCuWidth = ((m_param->sourceWidth / 2) + S265_LOWRES_CU_SIZE - 1) >> S265_LOWRES_CU_BITS;
    int lowresCuHeight = ((m_param->sourceHeight / 2) + S265_LOWRES_CU_SIZE - 1) >> S265_LOWRES_CU_BITS;
    m_ncu = lowresCuWidth * lowresCuHeight;

    m_qCompress = (m_param->rc.cuTree && !m_param->rc.hevcAq) ? 1 : m_param->rc.qCompress;

    // validate for param->rc, maybe it is need to add a function like s265_parameters_valiate()
    m_zoneBufferIdx = 0;
    m_residualFrames = 0;
    m_partialResidualFrames = 0;
    m_residualCost = 0;
    m_partialResidualCost = 0;
    m_rateFactorMaxIncrement = 0;
    m_rateFactorMaxDecrement = 0;
    m_fps = (double)m_param->fpsNum / m_param->fpsDenom;
    m_startEndOrder.set(0);// 初始值 0
    m_bTerminated = false;
    m_finalFrameCount = 0;
    m_numEntries = 0;
    m_isSceneTransition = false;
    m_lastPredictorReset = 0;
    m_avgPFrameQp = 0;
    m_isFirstMiniGop = false;
    m_lastScenecut = -1;
    m_lastScenecutAwareIFrame = -1;
    if (m_param->rc.rateControlMode == S265_RC_CRF)
    {
        m_param->rc.qp = (int)m_param->rc.rfConstant;
        m_param->rc.bitrate = 0;

        double baseCplx = m_ncu * (m_param->bframes ? 120 : 80);
        double mbtree_offset = m_param->rc.cuTree ? (1.0 - m_param->rc.qCompress) * 13.5 : 0;
        m_rateFactorConstant = pow(baseCplx, 1 - m_qCompress) /
            s265_qp2qScale(m_param->rc.rfConstant + mbtree_offset);
        if (m_param->rc.rfConstantMax)
        {
            m_rateFactorMaxIncrement = m_param->rc.rfConstantMax - m_param->rc.rfConstant;
            if (m_rateFactorMaxIncrement <= 0)
            {
                s265_log(m_param, S265_LOG_WARNING, "CRF max must be greater than CRF\n");
                m_rateFactorMaxIncrement = 0;
            }
        }
        if (m_param->rc.rfConstantMin)
            m_rateFactorMaxDecrement = m_param->rc.rfConstant - m_param->rc.rfConstantMin;
    }
    // 类似S264 只要非cqp 都算abr 
    m_isAbr = m_param->rc.rateControlMode != S265_RC_CQP;
    m_bitrate = m_param->rc.bitrate * 1000;
    m_frameDuration = (double)m_param->fpsDenom / m_param->fpsNum;
    m_qp = m_param->rc.qp;
    m_lastRceq = 1; /* handles the cmplxrsum when the previous frame cost is zero */
    m_shortTermCplxSum = 0;
    m_shortTermCplxCount = 0;
    m_lastNonBPictType = I_SLICE;
    m_isAbrReset = false;
    m_lastAbrResetPoc = -1;
    m_lastBsliceSatdCost = 0;
    m_movingAvgSum = 0.0;
    m_isNextGop = false;
    m_relativeComplexity = NULL;

    // vbv initialization
    m_param->rc.vbvBufferSize = s265_clip3(0, 2000000, m_param->rc.vbvBufferSize);
    m_param->rc.vbvMaxBitrate = s265_clip3(0, 2000000, m_param->rc.vbvMaxBitrate);
    m_param->rc.vbvBufferInit = s265_clip3(0.0, 2000000.0, m_param->rc.vbvBufferInit);
    m_param->vbvBufferEnd = s265_clip3(0.0, 2000000.0, m_param->vbvBufferEnd);
    m_initVbv = false;
    m_singleFrameVbv = 0;
    m_rateTolerance = 1.0;

    if (m_param->rc.vbvBufferSize)
    {
        if (m_param->rc.rateControlMode == S265_RC_CQP)
        {
            s265_log(m_param, S265_LOG_WARNING, "VBV is incompatible with constant QP, ignored.\n");
            m_param->rc.vbvBufferSize = 0;
            m_param->rc.vbvMaxBitrate = 0;
        }
        else if (m_param->rc.vbvMaxBitrate == 0)
        {
            if (m_param->rc.rateControlMode == S265_RC_ABR)
            {
                s265_log(m_param, S265_LOG_WARNING, "VBV maxrate unspecified, assuming CBR\n");
                m_param->rc.vbvMaxBitrate = m_param->rc.bitrate;
            }
            else
            {
                s265_log(m_param, S265_LOG_WARNING, "VBV bufsize set but maxrate unspecified, ignored\n");
                m_param->rc.vbvBufferSize = 0;
            }
        }
        else if (m_param->rc.vbvMaxBitrate < m_param->rc.bitrate &&
                 m_param->rc.rateControlMode == S265_RC_ABR)
        {
            s265_log(m_param, S265_LOG_WARNING, "max bitrate less than average bitrate, assuming CBR\n");
            m_param->rc.bitrate = m_param->rc.vbvMaxBitrate;
        }
    }
    else if (m_param->rc.vbvMaxBitrate)
    {
        s265_log(m_param, S265_LOG_WARNING, "VBV maxrate specified, but no bufsize, ignored\n");
        m_param->rc.vbvMaxBitrate = 0;
    }
    m_isVbv = m_param->rc.vbvMaxBitrate > 0 && m_param->rc.vbvBufferSize > 0;
    if (m_param->vbvBufferEnd && !m_isVbv)
    {
        s265_log(m_param, S265_LOG_WARNING, "vbv-end requires VBV parameters, ignored\n");
        m_param->vbvBufferEnd = 0;
    }
    if (m_param->bEmitHRDSEI && !m_isVbv)
    {
        s265_log(m_param, S265_LOG_WARNING, "NAL HRD parameters require VBV parameters, ignored\n");
        m_param->bEmitHRDSEI = 0;
    }
    m_isCbr = m_param->rc.rateControlMode == S265_RC_ABR && m_isVbv && m_param->rc.vbvMaxBitrate <= m_param->rc.bitrate;
    if (m_param->rc.bStrictCbr && !m_isCbr)
    {
        s265_log(m_param, S265_LOG_WARNING, "strict CBR set without CBR mode, ignored\n");
        m_param->rc.bStrictCbr = 0;
    }
    if(m_param->rc.bStrictCbr)
        m_rateTolerance = 0.7;

    m_bframeBits = 0;
    m_leadingNoBSatd = 0;
    m_ipOffset = 6.0 * S265_LOG2(m_param->rc.ipFactor);
    m_pbOffset = 6.0 * S265_LOG2(m_param->rc.pbFactor);

    for (int i = 0; i < QP_MAX_MAX; i++)
        m_qpToEncodedBits[i] = 0;

    /* Adjust the first frame in order to stabilize the quality level compared to the rest */
#define ABR_INIT_QP_MIN (24)
#define ABR_INIT_QP_MAX (37)
#define ABR_INIT_QP_GRAIN_MAX (33)
#define ABR_SCENECUT_INIT_QP_MIN (12)
#define CRF_INIT_QP (int)m_param->rc.rfConstant
    for (int i = 0; i < 3; i++)
    {
        m_lastQScaleFor[i] = s265_qp2qScale(m_param->rc.rateControlMode == S265_RC_CRF ? CRF_INIT_QP : ABR_INIT_QP_MIN);
        m_lmin[i] = s265_qp2qScale(m_param->rc.qpMin);
        m_lmax[i] = s265_qp2qScale(m_param->rc.qpMax);
    }

    if (m_param->rc.rateControlMode == S265_RC_CQP)
    {
        if (m_qp && !m_param->bLossless)
        {
            m_qpConstant[P_SLICE] = m_qp;
            m_qpConstant[I_SLICE] = s265_clip3(QP_MIN, QP_MAX_MAX, (int)(m_qp - m_ipOffset + 0.5));
            m_qpConstant[B_SLICE] = s265_clip3(QP_MIN, QP_MAX_MAX, (int)(m_qp + m_pbOffset + 0.5));
        }
        else
        {
            m_qpConstant[P_SLICE] = m_qpConstant[I_SLICE] = m_qpConstant[B_SLICE] = m_qp;
        }
    }

    /* qpstep - value set as encoder specific */
    m_lstep = pow(2, m_param->rc.qpStep / 6.0);
}
//一个encoder 对应一个有个RateControl 来把控全局码率
bool RateControl::init(const SPS& sps)
{
    if (m_isVbv && !m_initVbv)
    {
        /* We don't support changing the ABR bitrate right now,
         * so if the stream starts as CBR, keep it CBR. */
        if (m_param->rc.vbvBufferSize < (int)(m_param->rc.vbvMaxBitrate / m_fps))
        {
            m_param->rc.vbvBufferSize = (int)(m_param->rc.vbvMaxBitrate / m_fps);
            s265_log(m_param, S265_LOG_WARNING, "VBV buffer size cannot be smaller than one frame, using %d kbit\n",
                     m_param->rc.vbvBufferSize);
        }
        int vbvBufferSize = m_param->rc.vbvBufferSize * 1000;
        int vbvMaxBitrate = m_param->rc.vbvMaxBitrate * 1000;

        if (m_param->bEmitHRDSEI && !m_param->decoderVbvMaxRate)
        {
            const HRDInfo* hrd = &sps.vuiParameters.hrdParameters;
            vbvBufferSize = hrd->cpbSizeValue << (hrd->cpbSizeScale + CPB_SHIFT);
            vbvMaxBitrate = hrd->bitRateValue << (hrd->bitRateScale + BR_SHIFT);
        }
        m_bufferRate = vbvMaxBitrate / m_fps;
        m_vbvMaxRate = vbvMaxBitrate;
        m_bufferSize = vbvBufferSize;
        m_singleFrameVbv = m_bufferRate * 1.1 > m_bufferSize;

        if (m_param->rc.vbvBufferInit > 1.)
            m_param->rc.vbvBufferInit = s265_clip3(0.0, 1.0, m_param->rc.vbvBufferInit / m_param->rc.vbvBufferSize);
        if (m_param->vbvBufferEnd > 1.)
            m_param->vbvBufferEnd = s265_clip3(0.0, 1.0, m_param->vbvBufferEnd / m_param->rc.vbvBufferSize);
        if (m_param->vbvEndFrameAdjust > 1.)
            m_param->vbvEndFrameAdjust = s265_clip3(0.0, 1.0, m_param->vbvEndFrameAdjust);
        m_param->rc.vbvBufferInit = s265_clip3(0.0, 1.0, S265_MAX(m_param->rc.vbvBufferInit, m_bufferRate / m_bufferSize));
        m_bufferFillFinal = m_bufferSize * m_param->rc.vbvBufferInit;
        m_bufferFillActual = m_bufferFillFinal;
        m_bufferExcess = 0;
        m_minBufferFill = m_param->minVbvFullness / 100;
        m_maxBufferFill = 1 - (m_param->maxVbvFullness / 100);
        m_initVbv = true;
    }

    if (!m_param->bResetZoneConfig && (m_relativeComplexity == NULL))
    {
        m_relativeComplexity = S265_MALLOC(double, m_param->reconfigWindowSize);
        if (m_relativeComplexity == NULL)
        {
            s265_log(m_param, S265_LOG_ERROR, "Failed to allocate memory for m_relativeComplexity\n");
            return false;
        }
    }

    m_totalBits = 0;
    m_encodedBits = 0;
    m_framesDone = 0;
    m_residualCost = 0;
    m_partialResidualCost = 0;
    m_amortizeFraction = 0.85;
    m_amortizeFrames = 75;
    if (m_param->totalFrames && m_param->totalFrames <= 2 * m_fps && m_param->rc.bStrictCbr) /* Strict CBR segment encode */
    {
        m_amortizeFraction = 0.85;
        m_amortizeFrames = m_param->totalFrames / 2;
    }

    for (int i = 0; i < s_slidingWindowFrames; i++)
    {
        m_satdCostWindow[i] = 0;
        m_encodedBitsWindow[i] = 0;
    }
    m_sliderPos = 0;
    m_isPatternPresent = false;
    m_numBframesInPattern = 0;

    m_isGrainEnabled = false;
    if(m_param->rc.bEnableGrain) // tune for grainy content OR equal p-b frame sizes
        m_isGrainEnabled = true;
    for (int i = 0; i < 3; i++)
        m_lastQScaleFor[i] = s265_qp2qScale(m_param->rc.rateControlMode == S265_RC_CRF ? CRF_INIT_QP : ABR_INIT_QP_MIN);
    m_avgPFrameQp = 0 ;

    /* 720p videos seem to be a good cutoff for cplxrSum */
    double tuneCplxFactor = (m_ncu > 3600 && m_param->rc.cuTree && !m_param->rc.hevcAq) ? 2.5 : m_param->rc.hevcAq ? 1.5 : m_isGrainEnabled ? 1.9 : 1.0;

    /* estimated ratio that produces a reasonable QP for the first I-frame */
    m_cplxrSum = .01 * pow(7.0e5, m_qCompress) * pow(m_ncu, 0.5) * tuneCplxFactor;
    m_wantedBitsWindow = m_bitrate * m_frameDuration;
    m_accumPNorm = .01;
    m_accumPQp = (m_param->rc.rateControlMode == S265_RC_CRF ? CRF_INIT_QP : ABR_INIT_QP_MIN) * m_accumPNorm;


    /* Frame Predictors used in vbv */
    initFramePredictors();// 首次init predictor 在此处
    return true;
}
void RateControl::reconfigureRC()
{
    if (m_isVbv)
    {
        m_param->rc.vbvBufferSize = s265_clip3(0, 2000000, m_param->rc.vbvBufferSize);
        m_param->rc.vbvMaxBitrate = s265_clip3(0, 2000000, m_param->rc.vbvMaxBitrate);
        if (m_param->reconfigWindowSize)
            m_param->rc.vbvMaxBitrate = (int)(m_param->rc.vbvMaxBitrate * (double)(m_fps / m_param->reconfigWindowSize));
        if (m_param->rc.vbvMaxBitrate < m_param->rc.bitrate &&
            m_param->rc.rateControlMode == S265_RC_ABR)
        {
            s265_log(m_param, S265_LOG_WARNING, "max bitrate less than average bitrate, assuming CBR\n");
            m_param->rc.bitrate = m_param->rc.vbvMaxBitrate;
        }

        if (m_param->rc.vbvBufferSize < (int)(m_param->rc.vbvMaxBitrate / m_fps))
        {
            m_param->rc.vbvBufferSize = (int)(m_param->rc.vbvMaxBitrate / m_fps);
            s265_log(m_param, S265_LOG_WARNING, "VBV buffer size cannot be smaller than one frame, using %d kbit\n",
                m_param->rc.vbvBufferSize);
        }
        int vbvBufferSize = m_param->rc.vbvBufferSize * 1000;
        int vbvMaxBitrate = m_param->rc.vbvMaxBitrate * 1000;
        m_bufferRate = vbvMaxBitrate / m_fps;
        m_vbvMaxRate = vbvMaxBitrate;
        m_bufferSize = vbvBufferSize;
        m_singleFrameVbv = m_bufferRate * 1.1 > m_bufferSize;
    }
    if (m_param->rc.rateControlMode == S265_RC_CRF)
    {
        #define CRF_INIT_QP (int)m_param->rc.rfConstant
        m_param->rc.bitrate = 0;
        double baseCplx = m_ncu * (m_param->bframes ? 120 : 80);
        double mbtree_offset = m_param->rc.cuTree ? (1.0 - m_param->rc.qCompress) * 13.5 : 0;
        m_rateFactorConstant = pow(baseCplx, 1 - m_qCompress) /
            s265_qp2qScale(m_param->rc.rfConstant + mbtree_offset);
        if (m_param->rc.rfConstantMax)
        {
            m_rateFactorMaxIncrement = m_param->rc.rfConstantMax - m_param->rc.rfConstant;
            if (m_rateFactorMaxIncrement <= 0)
            {
                s265_log(m_param, S265_LOG_WARNING, "CRF max must be greater than CRF\n");
                m_rateFactorMaxIncrement = 0;
            }
        }
        if (m_param->rc.rfConstantMin)
            m_rateFactorMaxDecrement = m_param->rc.rfConstant - m_param->rc.rfConstantMin;
    }
    if (m_param->rc.rateControlMode == S265_RC_CQP)
    {
        m_qp = m_param->rc.qp;
        if (m_qp && !m_param->bLossless)
        {
            m_qpConstant[P_SLICE] = m_qp;
            m_qpConstant[I_SLICE] = s265_clip3(QP_MIN, QP_MAX_MAX, (int)(m_qp - m_ipOffset + 0.5));
            m_qpConstant[B_SLICE] = s265_clip3(QP_MIN, QP_MAX_MAX, (int)(m_qp + m_pbOffset + 0.5));
        }
        else
        {
            m_qpConstant[P_SLICE] = m_qpConstant[I_SLICE] = m_qpConstant[B_SLICE] = m_qp;
        }
    }
    m_bitrate = (double)m_param->rc.bitrate * 1000;
}

void RateControl::initHRD(SPS& sps)
{
    int vbvBufferSize = m_param->rc.vbvBufferSize * 1000;
    int vbvMaxBitrate = m_param->rc.vbvMaxBitrate * 1000;

    // Init HRD
    HRDInfo* hrd = &sps.vuiParameters.hrdParameters;
    hrd->cbrFlag = m_isCbr;
    if (m_param->reconfigWindowSize) {
        hrd->cbrFlag = 0;
        vbvMaxBitrate = m_param->decoderVbvMaxRate * 1000;
    }

    // normalize HRD size and rate to the value / scale notation
    hrd->bitRateScale = s265_clip3(0, 15, calcScale(vbvMaxBitrate) - BR_SHIFT);
    hrd->bitRateValue = (vbvMaxBitrate >> (hrd->bitRateScale + BR_SHIFT));

    hrd->cpbSizeScale = s265_clip3(0, 15, calcScale(vbvBufferSize) - CPB_SHIFT);
    hrd->cpbSizeValue = (vbvBufferSize >> (hrd->cpbSizeScale + CPB_SHIFT));
    int bitRateUnscale = hrd->bitRateValue << (hrd->bitRateScale + BR_SHIFT);
    int cpbSizeUnscale = hrd->cpbSizeValue << (hrd->cpbSizeScale + CPB_SHIFT);

    // arbitrary
    #define MAX_DURATION 0.5

    TimingInfo *time = &sps.vuiParameters.timingInfo;
    int maxCpbOutputDelay = (int)(S265_MIN(m_param->keyframeMax * MAX_DURATION * time->timeScale / time->numUnitsInTick, INT_MAX));
    int maxDpbOutputDelay = (int)(sps.maxDecPicBuffering * MAX_DURATION * time->timeScale / time->numUnitsInTick);
    int maxDelay = (int)(90000.0 * cpbSizeUnscale / bitRateUnscale + 0.5);

    hrd->initialCpbRemovalDelayLength = 2 + s265_clip3(4, 22, 32 - calcLength(maxDelay));
    hrd->cpbRemovalDelayLength = s265_clip3(4, 31, 32 - calcLength(maxCpbOutputDelay));
    hrd->dpbOutputDelayLength = s265_clip3(4, 31, 32 - calcLength(maxDpbOutputDelay));

    #undef MAX_DURATION
}

void RateControl::initFramePredictors()
{
    /* Frame Predictors used in vbv */
    for (int i = 0; i < 4; i++)
    {
        m_pred[i].coeffMin = 1.0 / 4;
        m_pred[i].coeff = 1.0;
        m_pred[i].count = 1.0;
        m_pred[i].decay = 0.5;
        m_pred[i].offset = 0.0;
    }
    //预测器类型 0位b  1为p  2为I  3为Bref
    //这里修改 b/Bref 帧使用的预测器系数
    m_pred[0].coeff = m_pred[3].coeff = 0.75;
    m_pred[0].coeffMin = m_pred[3].coeffMin = 0.75 / 4;
    if (m_isGrainEnabled) // when tuned for grain 
    {
        m_pred[1].coeffMin = 0.75 / 4;
        m_pred[1].coeff = 0.75;
        m_pred[0].coeff = m_pred[3].coeff = 0.75;
        m_pred[0].coeffMin = m_pred[3].coeffMin = 0.75 / 4;
    }
}
// 每个帧编码线程在编码一帧前调用 called by frameEncode threads
int RateControl::rateControlStart(Frame* curFrame, RateControlEntry* rce, Encoder* enc)
{
    int orderValue = m_startEndOrder.get();// 从主线程获取start end order值
    int startOrdinal = rce->encodeOrder * 2;
    // 主线程 当前的 orderValue 比当前线程中rce编码的帧 的两倍 值要小，
    // 执行当前帧的start 之前需要等并行帧中最老的帧完成 end的调用
    while (orderValue < startOrdinal && !m_bTerminated) // m_bTerminated 由 s265_encoder_close 进行设置
        orderValue = m_startEndOrder.waitForChange(orderValue);//等待 m_startEndOrder 的value 发生变化

    if (!curFrame)// 传进来的frame 为空,表示encoder 在 flushing
    {
        // faked rateControlStart calls when the encoder is flushing
        m_startEndOrder.incr();// flushing return 相当于update+1
        return 0;
    }

    FrameData& curEncData = *curFrame->m_encData;
    m_curSlice = curEncData.m_slice;
    m_sliceType = m_curSlice->m_sliceType;
    rce->sliceType = m_sliceType;
    rce->keptAsRef = IS_REFERENCED(curFrame);
    //预测器类型 0位b  1为p 2为I  3为Bref   
    m_predType = getPredictorType(curFrame->m_lowres.sliceType, m_sliceType);
    rce->poc = m_curSlice->m_poc;// 输入帧的序号

    if (!m_param->bResetZoneConfig && (rce->encodeOrder % m_param->reconfigWindowSize == 0))
    {
        int index = m_zoneBufferIdx % m_param->rc.zonefileCount;
        int read = m_top->zoneReadCount[index].get();
        int write = m_top->zoneWriteCount[index].get();
        if (write <= read)
            write = m_top->zoneWriteCount[index].waitForChange(write);
        m_zoneBufferIdx++;

        for (int i = 0; i < m_param->rc.zonefileCount; i++)
        {
            if (m_param->rc.zones[i].startFrame == rce->encodeOrder)
            {
                m_param->rc.bitrate = m_param->rc.zones[i].zoneParam->rc.bitrate;
                m_param->rc.vbvMaxBitrate = m_param->rc.zones[i].zoneParam->rc.vbvMaxBitrate;
                memcpy(m_relativeComplexity, m_param->rc.zones[i].relativeComplexity, sizeof(double) * m_param->reconfigWindowSize);
                reconfigureRC();
                m_isCbr = 1; /* Always vbvmaxrate == bitrate here*/
                m_top->zoneReadCount[i].incr();
            }
        }
    }
    
    
    if (m_param->bResetZoneConfig)
    {
        /* change ratecontrol stats for next zone if specified */
        for (int i = 0; i < m_param->rc.zonefileCount; i++)
        {
            if (m_param->rc.zones[i].startFrame == curFrame->m_encodeOrder)
            {
                m_param = m_param->rc.zones[i].zoneParam;
                reconfigureRC();
                init(*m_curSlice->m_sps);
            }
        }
    }

    rce->isActive = true;// 标记状态
    rce->scenecut = false;
    rce->isFadeEnd = curFrame->m_lowres.bIsFadeEnd;
    // list0 的首个ref 是否是一个场景切换帧
    bool isRefFrameScenecut = m_sliceType!= I_SLICE && m_curSlice->m_refFrameList[0][0]->m_lowres.bScenecut;
    // 是否是I帧开始的首个minigop
    m_isFirstMiniGop = m_sliceType == I_SLICE ? true : m_isFirstMiniGop;
    if (curFrame->m_lowres.bScenecut)
    {
        m_isSceneTransition = true;//场景变换
        rce->scenecut = true;// 场景切换帧
        m_lastPredictorReset = rce->encodeOrder;// 因为场景变化了更新
        // 遇到场景切换 重新initPredictor
        initFramePredictors();
    }
    else if (m_sliceType != B_SLICE && !isRefFrameScenecut)
        m_isSceneTransition = false;
    //
    if (rce->encodeOrder < m_lastPredictorReset + m_param->frameNumThreads)
    {// 这里理解为:在遇到场景变换后,后面紧随的共frameNumThreads 个frameEncode 在他们编码帧前都要重置该rowPreds
    //即:所有的frameEncode 的rowPreds 都要重置，但是他们重置的时间点不一样
        // 这里将B帧使用的rowpred中的用于pred_s 的count 置零
        rce->rowPreds[0][0].count = 0;
    }

    rce->bLastMiniGopBFrame = curFrame->m_lowres.bLastMiniGopBFrame;
    rce->bufferRate = m_bufferRate;
    rce->rowCplxrSum = 0.0;
    rce->rowTotalBits = 0;
    if (m_isVbv)
    {
        if (rce->rowPreds[0][0].count == 0)//发现B帧使用的 用于pred_s 的预测器参数重置，注意第一次也在这里初始化
        {
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    rce->rowPreds[i][j].coeffMin = 0.25 / 4;
                    rce->rowPreds[i][j].coeff = 0.25;
                    rce->rowPreds[i][j].count = 1.0;
                    rce->rowPreds[i][j].decay = 0.5;
                    rce->rowPreds[i][j].offset = 0.0;
                }
            }
        }
        rce->rowPred[0] = &rce->rowPreds[m_sliceType][0];
        rce->rowPred[1] = &rce->rowPreds[m_sliceType][1];
        updateVbvPlan(enc);//根据所有在活frameEncoder 的预估消耗bits与整段时间内应注入bits 更新总体m_bufferFill
        rce->bufferFill = m_bufferFill;//总体m_bufferFill 赋给该帧的rce的bufferFill
        rce->vbvEndAdj = false;
        if (m_param->vbvBufferEnd && rce->encodeOrder >= m_param->vbvEndFrameAdjust * m_param->totalFrames)
        {//如果指定了 最终要求vbvbufferEnd的水平，并指定了从哪一帧开始调节
            rce->vbvEndAdj = true;
            rce->targetFill = 0;
        }

        // 根据min compress rate 计算单帧的maxSize
        // 这里可以不用放到rce 每一帧编码前都调用, 可以放到ratecontrol里面最开始计算好
        int mincr = enc->m_vps.ptl.minCrForLevel;
        /* Profiles above Main10 don't require maxAU size check, so just set the maximum to a large value. */
        if (enc->m_vps.ptl.profileIdc > Profile::MAIN10 || enc->m_vps.ptl.levelIdc == Level::NONE)
            rce->frameSizeMaximum = 1e9;
        else
        {
            /* The spec has a special case for the first frame. */
            if (curFrame->m_lowres.bKeyframe)
            {
                /* 1.5 * (Max( PicSizeInSamplesY, fR * MaxLumaSr) + MaxLumaSr * (AuCpbRemovalTime[ 0 ] -AuNominalRemovalTime[ 0 ])) ? MinCr */
                double fr = 1. / 300;
                int picSizeInSamplesY = m_param->sourceWidth * m_param->sourceHeight;
                rce->frameSizeMaximum = 8 * 1.5 * S265_MAX(picSizeInSamplesY, fr * enc->m_vps.ptl.maxLumaSrForLevel) / mincr;
            }
            else
            {
                /* 1.5 * MaxLumaSr * (AuCpbRemovalTime[ n ] - AuCpbRemovalTime[ n - 1 ]) / MinCr */
                rce->frameSizeMaximum = 8 * 1.5 * enc->m_vps.ptl.maxLumaSrForLevel * m_frameDuration / mincr;
            }
            rce->frameSizeMaximum *= m_param->maxAUSizeFactor;
        }
    }


    if (m_isAbr) // ABR,CRF
    {
        if (m_isAbr || m_isVbv)
        {   
            //curFrame->m_lowres.satdCost 在编码前已经算好了 （代码中主线程计算，可以放到与分析中计算）
            m_currentSatd = curFrame->m_lowres.satdCost >> (S265_DEPTH - 8);
            /* Update rce for use in rate control VBV later */
            rce->lastSatd = m_currentSatd;
            S265_CHECK(rce->lastSatd, "satdcost cannot be zero\n");
            /* Detect a pattern for B frames with same SATDcost to identify a series of static frames
             * and the P frame at the end of the series marks a possible case for ABR reset logic */
            if (m_param->bframes)
            {
                if (m_sliceType != B_SLICE && m_numBframesInPattern > m_param->bframes)
                {
                    m_isPatternPresent = true;
                }
                else if (m_sliceType == B_SLICE && !IS_REFERENCED(curFrame))
                {
                    if (m_currentSatd != m_lastBsliceSatdCost && !rce->bLastMiniGopBFrame)
                    {
                        m_isPatternPresent = false;
                        m_lastBsliceSatdCost = m_currentSatd;
                        m_numBframesInPattern = 0;
                    }
                    else if (m_currentSatd == m_lastBsliceSatdCost)
                        m_numBframesInPattern++;
                }
            }
            if (rce->isFadeEnd)
                m_isPatternPresent = true;
        }
        /* For a scenecut that occurs within the mini-gop, enable scene transition
         * switch until the next mini-gop to ensure a min qp for all the frames within 
         * the scene-transition mini-gop */
        // 获取当前帧编码使用的qp
        double q = s265_qScale2qp(rateEstimateQscale(curFrame, rce));

        q = s265_clip3((double)m_param->rc.qpMin, (double)m_param->rc.qpMax, q);
        m_qp = int(q + 0.5);
        q = m_isGrainEnabled ? m_qp : q;
        //赋值给当前编码帧使用的 rce->qpaRC
        rce->qpaRc = curEncData.m_avgQpRc = curEncData.m_avgQpAq = q;
        /* copy value of lastRceq into thread local rce struct *to be used in RateControlEnd() */
        rce->qRceq = m_lastRceq;//使用更新后的Rceq 给thread local 
        accumPQpUpdate();//qp累加衰减
    }
    else // CQP
    {
        if (m_sliceType == B_SLICE && IS_REFERENCED(curFrame))
            m_qp = (m_qpConstant[B_SLICE] + m_qpConstant[P_SLICE]) / 2;
        else
            m_qp = m_qpConstant[m_sliceType];
        curEncData.m_avgQpAq = curEncData.m_avgQpRc = m_qp;
        
        s265_zone* zone = getZone();
        if (zone)
        {
            if (zone->bForceQp)
                m_qp += zone->qp - m_qpConstant[P_SLICE];
            else
                m_qp -= (int)(6.0 * S265_LOG2(zone->bitrateFactor));
        }
    }
    if (m_sliceType != B_SLICE)
    {
        m_lastNonBPictType = m_sliceType;
        m_leadingNoBSatd = m_currentSatd;
    }
    rce->leadingNoBSatd = m_leadingNoBSatd;
    if (curFrame->m_forceqp)
    {
        m_qp = (int32_t)(curFrame->m_forceqp + 0.5) - 1;
        m_qp = s265_clip3(m_param->rc.qpMin, m_param->rc.qpMax, m_qp);
        rce->qpaRc = curEncData.m_avgQpRc = curEncData.m_avgQpAq = m_qp;
        if (m_isAbr)
        {
            rce->qpNoVbv = rce->qpaRc;
            m_lastQScaleFor[m_sliceType] = s265_qp2qScale(rce->qpaRc);
            if (rce->poc == 0)
                 m_lastQScaleFor[P_SLICE] = m_lastQScaleFor[m_sliceType] * fabs(m_param->rc.ipFactor);
            rce->frameSizePlanned = predictSize(&m_pred[m_predType], m_qp, (double)m_currentSatd);
        }
    }
    m_framesDone++;

    return m_qp;
}

void RateControl::accumPQpUpdate()
{
    m_accumPQp   *= .95;
    m_accumPNorm *= .95;
    m_accumPNorm += 1;
    if (m_sliceType == I_SLICE)
        m_accumPQp += m_qp + m_ipOffset;
    else//PB帧 直接相加
        m_accumPQp += m_qp;
}

int RateControl::getPredictorType(int lowresSliceType, int sliceType)
{
    /* Use a different predictor for B Ref and B frames for vbv frame size predictions */
    if (lowresSliceType == S265_TYPE_BREF)// 对于B ref帧 使用 第3类型的predictor
        return 3;
    return sliceType;// 0/1/2 分别为 B帧/P帧/I帧
}

double RateControl::tuneAbrQScaleFromFeedback(double qScale)
{
    double abrBuffer = 2 * m_rateTolerance * m_bitrate;
    /* use framesDone instead of POC as poc count is not serial with bframes enabled */
    double overflow = 1.0;
    double timeDone = (double)(m_framesDone - m_param->frameNumThreads + 1) * m_frameDuration;
    double wantedBits = timeDone * m_bitrate;
    int64_t encodedBits = m_totalBits;
    if (m_param->totalFrames && m_param->totalFrames <= 2 * m_fps)
    {
        abrBuffer = m_param->totalFrames * (m_bitrate / m_fps);
        encodedBits = m_encodedBits;
    }

    if (wantedBits > 0 && encodedBits > 0 && (!m_partialResidualFrames || 
        m_param->rc.bStrictCbr || m_isGrainEnabled))
    {
        abrBuffer *= S265_MAX(1, sqrt(timeDone));
        overflow = s265_clip3(.5, 2.0, 1.0 + (encodedBits - wantedBits) / abrBuffer);
        qScale *= overflow;
    }
    return qScale;
}

double RateControl::tuneQScaleForZone(RateControlEntry *rce, double qScale)
{
    rce->frameSizePlanned = predictSize(&m_pred[m_predType], qScale, (double)m_currentSatd);
    int loop = 0;

    double availableBits = (double)m_param->rc.bitrate * 1000 * m_relativeComplexity[rce->encodeOrder % m_param->reconfigWindowSize];

    // Tune qScale to adhere to the available frame bits.
    for (int i = 0; i < 1000 && loop != 3; i++)
    {
        if (rce->frameSizePlanned < availableBits)
        {
            qScale = qScale / 1.01;
            loop = loop | 1;
        }
        else if (rce->frameSizePlanned > availableBits)
        {
            qScale = qScale * 1.01;
            loop = loop | 2;
        }
        rce->frameSizePlanned = predictSize(&m_pred[m_predType], qScale, (double)m_currentSatd);
    }
    return qScale;
}

double RateControl::tuneQScaleForGrain(double rcOverflow)
{
    double qpstep = rcOverflow > 1.1 ? rcOverflow : m_lstep;
    double qScaleAvg = s265_qp2qScale(m_avgPFrameQp);
    double  q = m_lastQScaleFor[P_SLICE];
    int curQp = int (s265_qScale2qp(m_lastQScaleFor[P_SLICE]) + 0.5);
    double curBitrate = m_qpToEncodedBits[curQp] * int(m_fps + 0.5);
    int newQp = rcOverflow > 1.1 ? curQp + 2 : rcOverflow > 1 ? curQp + 1 : curQp - 1 ;
    double projectedBitrate =  int(m_fps + 0.5) * m_qpToEncodedBits[newQp];
    if (curBitrate > 0 && projectedBitrate > 0)
        q =  abs(projectedBitrate - m_bitrate) < abs (curBitrate - m_bitrate) ? s265_qp2qScale(newQp) : m_lastQScaleFor[P_SLICE];
    else
        q = rcOverflow > 1 ? qScaleAvg * qpstep : rcOverflow < 1 ?  qScaleAvg / qpstep : m_lastQScaleFor[P_SLICE];
    return q;
}
// 每个帧编码线程在编码一帧前 由 rateControlStart调用
double RateControl::rateEstimateQscale(Frame* curFrame, RateControlEntry *rce)
{
    double q;

    // 计算 rce->movingAvgSum
    if ((m_param->bliveVBV2pass && m_param->rc.rateControlMode == S265_RC_ABR) || m_isAbr)
    {
        int pos = m_sliderPos % s_slidingWindowFrames;
        // 可以改成 addpos = pos? pos -1 : s_slidingWindowFrames -1;
        int addPos = (pos + s_slidingWindowFrames - 1) % s_slidingWindowFrames;
        if (m_sliderPos > s_slidingWindowFrames)//如果一段时间 超过窗口大小后
        {
            const static double base = pow(0.5, s_slidingWindowFrames - 1);
            m_movingAvgSum -= m_lastRemovedSatdCost * base;
            m_movingAvgSum *= 0.5;
            m_movingAvgSum += m_satdCostWindow[addPos];
        }
        else if (m_sliderPos == s_slidingWindowFrames)
        {
            m_movingAvgSum += m_satdCostWindow[addPos];
        }
        else if (m_sliderPos > 0)
        {
            m_movingAvgSum += m_satdCostWindow[addPos];
            m_movingAvgSum *= 0.5;
        }

        rce->movingAvgSum = m_movingAvgSum;//更新滑动窗口相关数据
        m_lastRemovedSatdCost = m_satdCostWindow[pos];
        m_satdCostWindow[pos] = rce->lastSatd;
        m_sliderPos++;
    }

    if (m_sliceType == B_SLICE)
    { // b/Bref
        /* B-frames don't have independent rate control, but rather get the
         * average QP of the two adjacent P-frames + an offset */
        Slice* prevRefSlice = m_curSlice->m_refFrameList[0][0]->m_encData->m_slice;
        Slice* nextRefSlice = m_curSlice->m_refFrameList[1][0]->m_encData->m_slice;
        double q0 = m_curSlice->m_refFrameList[0][0]->m_encData->m_avgQpRc;
        double q1 = m_curSlice->m_refFrameList[1][0]->m_encData->m_avgQpRc;
        bool i0 = prevRefSlice->m_sliceType == I_SLICE;
        bool i1 = nextRefSlice->m_sliceType == I_SLICE;
        // distance
        int dt0 = abs(m_curSlice->m_poc - prevRefSlice->m_poc);
        int dt1 = abs(m_curSlice->m_poc - nextRefSlice->m_poc);

        // Skip taking a reference frame before the Scenecut if ABR has been reset.
        if (m_lastAbrResetPoc >= 0)
        {
            if (prevRefSlice->m_sliceType == P_SLICE && prevRefSlice->m_poc < m_lastAbrResetPoc)
            {
                i0 = i1;
                dt0 = dt1;
                q0 = q1;
            }
        }
        int is_hier = m_param->rc.pyQpMethod&&(m_param->bBPyramid == S265_B_PYRAMID_HIER);
        if(m_param->rc.pyQpMethod == 2 && m_param->rc.rfConstant<30)
        {
            is_hier = false;
        }

        if (is_hier)
        {
            if (m_param->rc.pyQpMethod!=3)
            {
                i0 = m_curSlice->m_refFrameList[0][0]->m_lowres.i_temporal_id == 0;
                i1 = m_curSlice->m_refFrameList[1][0]->m_lowres.i_temporal_id == 0;
            }
        }
        else
        {
            if (prevRefSlice->m_sliceType == B_SLICE && IS_REFERENCED(m_curSlice->m_refFrameList[0][0]))
                q0 -= m_pbOffset / 2;
            if (nextRefSlice->m_sliceType == B_SLICE && IS_REFERENCED(m_curSlice->m_refFrameList[1][0]))
                q1 -= m_pbOffset / 2;
        }

        if (i0 && i1)
            q = (q0 + q1) / 2 + m_ipOffset;
        else if (i0)
            q = q1;
        else if (i1)
            q = q0;
        else if(m_isGrainEnabled)
                q = q1;
            else
            q = (q0 * dt1 + q1 * dt0) / (dt0 + dt1);

        double crf_factor = 0.5;
        if (is_hier && m_param->rc.pyQpMethod==3)
            crf_factor = m_param->rc.rfConstant * 1.1 / 15 - 1.2;

        if (IS_REFERENCED(curFrame) || is_hier)
            //q += m_pbOffset / 2;
            q += m_pbOffset * crf_factor;
        else
            q += m_pbOffset;

                /* Set a min qp at scenechanges and transitions */
        if (m_isSceneTransition)
        {
            q = S265_MAX(ABR_SCENECUT_INIT_QP_MIN, q);
            double minScenecutQscale =s265_qp2qScale(ABR_SCENECUT_INIT_QP_MIN); 
            m_lastQScaleFor[P_SLICE] = S265_MAX(minScenecutQscale, m_lastQScaleFor[P_SLICE]);
        }
        double qScale = s265_qp2qScale(q);
        rce->qpNoVbv = q;
        double lmin = 0, lmax = 0;
        if (m_isGrainEnabled && m_isFirstMiniGop)
        {
            lmin = m_lastQScaleFor[P_SLICE] / m_lstep;
            lmax = m_lastQScaleFor[P_SLICE] * m_lstep;
            double tunedQscale = tuneAbrQScaleFromFeedback(qScale);
            double overflow = tunedQscale / qScale;
            if (!m_isAbrReset)
                qScale = s265_clip3(lmin, lmax, qScale);
            m_avgPFrameQp = m_avgPFrameQp == 0 ? rce->qpNoVbv : m_avgPFrameQp;
            if (overflow != 1)
            {
                qScale = tuneQScaleForGrain(overflow);
                q = s265_qScale2qp(qScale);
            }
            rce->qpNoVbv = q;
        }
        /* Scenecut Aware QP offsets*/
        if (m_param->bEnableSceneCutAwareQp)
        {
            double lqmin = m_lmin[m_sliceType];
            double lqmax = m_lmax[m_sliceType];
            if (m_param->bEnableSceneCutAwareQp & FORWARD)
                qScale = forwardMasking(curFrame, qScale);
            if (m_param->bEnableSceneCutAwareQp & BACKWARD)
                qScale = backwardMasking(curFrame, qScale);
            qScale = s265_clip3(lqmin, lqmax, qScale);
            q = s265_qScale2qp(qScale);
            rce->qpNoVbv = q;
        }

        if (m_isVbv)
        {
            lmin = m_lastQScaleFor[P_SLICE] / m_lstep;
            lmax = m_lastQScaleFor[P_SLICE] * m_lstep;

            if (m_isCbr && !m_isGrainEnabled)
            {
                qScale = tuneAbrQScaleFromFeedback(qScale);
                if (!m_isAbrReset)
                    qScale = s265_clip3(lmin, lmax, qScale);
                q = s265_qScale2qp(qScale);
            }

            if (!m_param->bResetZoneConfig)
            {
                double lqmin = m_lmin[m_sliceType];
                double lqmax = m_lmax[m_sliceType];
                qScale = tuneQScaleForZone(rce, qScale);
                qScale = s265_clip3(lqmin, lqmax, qScale);
            }

            /* clip qp to permissible range after vbv-lookahead estimation to avoid possible 
                * mispredictions by initial frame size predictors */
            qScale = clipQscale(curFrame, rce, qScale);

            if (m_pred[m_predType].count == 1)
                qScale = s265_clip3(lmin, lmax, qScale);
            m_lastQScaleFor[m_sliceType] = qScale;
        }

        rce->frameSizePlanned = predictSize(&m_pred[m_predType], qScale, (double)m_currentSatd);

        /* Limit planned size by MinCR */
        if (m_isVbv)
            rce->frameSizePlanned = S265_MIN(rce->frameSizePlanned, rce->frameSizeMaximum);
        rce->frameSizeEstimated = rce->frameSizePlanned;

        rce->newQScale = qScale;
        if(rce->bLastMiniGopBFrame)
        {
            if (m_isFirstMiniGop && m_isGrainEnabled)
            {
                m_avgPFrameQp = (m_avgPFrameQp + rce->qpNoVbv) / 2;
                m_lastQScaleFor[P_SLICE] = s265_qp2qScale(m_avgPFrameQp);
            }
            m_isFirstMiniGop = false;
        }
        return qScale;
    }
    else
    {
        {
            /* 1pass ABR */

            /* Calculate the quantizer which would have produced the desired
             * average bitrate if it had been applied to all frames so far.
             * Then modulate that quant based on the current frame's complexity
             * relative to the average complexity so far (using the 2pass RCEQ).
             * Then bias the quant up or down if total size so far was far from
             * the target.
             * Result: Depending on the value of rate_tolerance, there is a
             * trade-off between quality and bitrate precision. But at large
             * tolerances, the bit distribution approaches that of 2pass. */

            double overflow = 1;
            double lqmin = m_lmin[m_sliceType];
            double lqmax = m_lmax[m_sliceType];
            m_shortTermCplxSum *= 0.5;
            m_shortTermCplxCount *= 0.5;
            m_shortTermCplxSum += m_currentSatd / (CLIP_DURATION(m_frameDuration) / BASE_FRAME_DURATION);
            m_shortTermCplxCount++;
            /* coeffBits to be used in 2-pass */
            rce->coeffBits = (int)m_currentSatd;
            rce->blurredComplexity = m_shortTermCplxSum / m_shortTermCplxCount;
            rce->mvBits = 0;
            rce->sliceType = m_sliceType;

            if (m_param->rc.rateControlMode == S265_RC_CRF)
            {
                q = getQScale(rce, m_rateFactorConstant);
                s265_zone* zone = getZone();
                if (zone)
                {
                    if (zone->bForceQp)
                        q = s265_qp2qScale(zone->qp);
                    else
                        q /= zone->bitrateFactor;
                }
            }
            else
            {
                checkAndResetABR(rce, false);
                double initialQScale = getQScale(rce, m_wantedBitsWindow / m_cplxrSum);
                s265_zone* zone = getZone();
                if (zone)
                {
                    if (zone->bForceQp)
                        initialQScale = s265_qp2qScale(zone->qp);
                    else
                        initialQScale /= zone->bitrateFactor;
                }
                double tunedQScale = tuneAbrQScaleFromFeedback(initialQScale);
                overflow = tunedQScale / initialQScale;
                q = !m_partialResidualFrames? tunedQScale : initialQScale;
                bool isEncodeEnd = (m_param->totalFrames && 
                    m_framesDone > 0.75 * m_param->totalFrames) ? 1 : 0;
                bool isEncodeBeg = m_framesDone < (int)(m_fps + 0.5);
                if (m_isGrainEnabled)
                {
                    if(m_sliceType!= I_SLICE && m_framesDone && !isEncodeEnd &&
                        ((overflow < 1.05 && overflow > 0.95) || isEncodeBeg))
                    {
                        q = tuneQScaleForGrain(overflow);
                    }
                }
            }
            if ((m_sliceType == I_SLICE && m_param->keyframeMax > 1
                && m_lastNonBPictType != I_SLICE && !m_isAbrReset) || (m_isNextGop && !m_framesDone))
            {
                if (!m_param->rc.bStrictCbr)
                    q = s265_qp2qScale(m_accumPQp / m_accumPNorm);
                q /= fabs(m_param->rc.ipFactor);
                m_avgPFrameQp = 0;
            }
            else if (m_framesDone > 0)
            {
                if (m_param->rc.rateControlMode != S265_RC_CRF)
                {
                    lqmin = m_lastQScaleFor[m_sliceType] / m_lstep;
                    lqmax = m_lastQScaleFor[m_sliceType] * m_lstep;
                    if (!m_partialResidualFrames || m_isGrainEnabled)
                    {
                        if (overflow > 1.1 && m_framesDone > 3)
                            lqmax *= m_lstep;
                        else if (overflow < 0.9)
                            lqmin /= m_lstep;
                    }
                    q = s265_clip3(lqmin, lqmax, q);
                }
            }
            else if (m_qCompress != 1 && m_param->rc.rateControlMode == S265_RC_CRF)
            {
                q = s265_qp2qScale(CRF_INIT_QP) / fabs(m_param->rc.ipFactor);
            }
            else if (m_framesDone == 0 && !m_isVbv && m_param->rc.rateControlMode == S265_RC_ABR)
            {
                /* for ABR alone, clip the first I frame qp */
                lqmax = (m_isGrainEnabled && m_lstep) ? s265_qp2qScale(ABR_INIT_QP_GRAIN_MAX) :
                        s265_qp2qScale(ABR_INIT_QP_MAX);
                q = S265_MIN(lqmax, q);
            }
            q = s265_clip3(lqmin, lqmax, q);
            /* Set a min qp at scenechanges and transitions */
            if (m_isSceneTransition)
            {
                double minScenecutQscale = s265_qp2qScale(ABR_SCENECUT_INIT_QP_MIN);
                q = S265_MAX(minScenecutQscale, q);
                m_lastQScaleFor[P_SLICE] = S265_MAX(minScenecutQscale, m_lastQScaleFor[P_SLICE]);
            }
            rce->qpNoVbv = s265_qScale2qp(q);
            if (m_sliceType == P_SLICE)
            {
                m_avgPFrameQp = m_avgPFrameQp == 0 ? rce->qpNoVbv : m_avgPFrameQp;
                m_avgPFrameQp = (m_avgPFrameQp + rce->qpNoVbv) / 2;
            }

            if (!m_param->bResetZoneConfig)
            {
                q = tuneQScaleForZone(rce, q);
                q = s265_clip3(lqmin, lqmax, q);
            }
            /* Scenecut Aware QP offsets*/
            if (m_param->bEnableSceneCutAwareQp)
            {
                double qmin = m_lmin[m_sliceType];
                double qmax = m_lmax[m_sliceType];

                if (m_param->bEnableSceneCutAwareQp & FORWARD)
                    q = forwardMasking(curFrame, q);
                if (m_param->bEnableSceneCutAwareQp & BACKWARD)
                    q = backwardMasking(curFrame, q);

                q = s265_clip3(qmin, qmax, q);
                rce->qpNoVbv = s265_qScale2qp(q);
            }
            q = clipQscale(curFrame, rce, q);

            rce->frameSizePlanned = predictSize(&m_pred[m_predType], q, (double)m_currentSatd);

            /*  clip qp to permissible range after vbv-lookahead estimation to avoid possible
             * mispredictions by initial frame size predictors, after each scenecut */
            bool isFrameAfterScenecut = m_sliceType!= I_SLICE && m_curSlice->m_refFrameList[0][0]->m_lowres.bScenecut;
            if (m_isVbv && isFrameAfterScenecut)
                q = s265_clip3(lqmin, lqmax, q);
        }
        m_lastQScaleFor[m_sliceType] = q;
        if ((m_curSlice->m_poc == 0 || m_lastQScaleFor[P_SLICE] < q))
            m_lastQScaleFor[P_SLICE] = q * fabs(m_param->rc.ipFactor);

        rce->frameSizePlanned = predictSize(&m_pred[m_predType], q, (double)m_currentSatd);

        /* Always use up the whole VBV in this case. */
        if (m_singleFrameVbv)
            rce->frameSizePlanned = m_bufferRate;
        /* Limit planned size by MinCR */
        if (m_isVbv)
            rce->frameSizePlanned = S265_MIN(rce->frameSizePlanned, rce->frameSizeMaximum);
        rce->frameSizeEstimated = rce->frameSizePlanned;
        rce->newQScale = q;
        return q;
    }
}
// 开启码空或者vbv 下，在一帧的最后一个slice完成若干需要等待的CTU行后进行调用
// called by workthreads
void RateControl::rateControlUpdateStats(RateControlEntry* rce)
{
        if (rce->sliceType == I_SLICE)
        {
            /* previous I still had a residual; roll it into the new loan */
            if (m_partialResidualFrames)
                rce->rowTotalBits += m_partialResidualCost * m_partialResidualFrames;
            if ((m_param->totalFrames != 0) && (m_amortizeFrames > (m_param->totalFrames - m_framesDone)))
            {
                m_amortizeFrames = 0;
                m_amortizeFraction = 0;
            }
            else
            {
                double depreciateRate = 1.1;
                m_amortizeFrames = (int)(m_amortizeFrames / depreciateRate);
                m_amortizeFraction /= depreciateRate;
                m_amortizeFrames = S265_MAX(m_amortizeFrames, MIN_AMORTIZE_FRAME);
                m_amortizeFraction = S265_MAX(m_amortizeFraction, MIN_AMORTIZE_FRACTION);
            }
            rce->amortizeFrames = m_amortizeFrames;
            rce->amortizeFraction = m_amortizeFraction;
            m_partialResidualFrames = S265_MIN((int)rce->amortizeFrames, m_param->keyframeMax);
            m_partialResidualCost = (int)((rce->rowTotalBits * rce->amortizeFraction) / m_partialResidualFrames);
            rce->rowTotalBits -= m_partialResidualCost * m_partialResidualFrames;
        }
        else if (m_partialResidualFrames)
        {
             rce->rowTotalBits += m_partialResidualCost;
             m_partialResidualFrames--;
        }
    if (rce->sliceType != B_SLICE)
        rce->rowCplxrSum = rce->rowTotalBits * s265_qp2qScale(rce->qpaRc) / rce->qRceq; // I/P slice
    else
        rce->rowCplxrSum = rce->rowTotalBits * s265_qp2qScale(rce->qpaRc) / (rce->qRceq * fabs(m_param->rc.pbFactor));

    m_cplxrSum += rce->rowCplxrSum;
    m_totalBits += rce->rowTotalBits;

    /* do not allow the next frame to enter rateControlStart() until this
     * frame has updated its mid-frame statistics */
    if (m_param->rc.rateControlMode == S265_RC_ABR || m_isVbv)
    {
        m_startEndOrder.incr();//  码空 update+1 by workthreads

        if (rce->encodeOrder < m_param->frameNumThreads - 1)
            m_startEndOrder.incr(); // 码空提前 end+1 by workthreads   Start faked rateControlEnd calls for negative frames
    }
}

void RateControl::checkAndResetABR(RateControlEntry* rce, bool isFrameDone)
{
    double abrBuffer = 2 * m_rateTolerance * m_bitrate;

    // Check if current Slice is a scene cut that follows low detailed/blank frames
    if (rce->lastSatd > 4 * rce->movingAvgSum || rce->scenecut || rce->isFadeEnd)
    {
        if (!m_isAbrReset && rce->movingAvgSum > 0
            && (m_isPatternPresent || !m_param->bframes))
        {
            int pos = S265_MAX(m_sliderPos - m_param->frameNumThreads, 0);
            int64_t shrtTermWantedBits = (int64_t) (S265_MIN(pos, s_slidingWindowFrames) * m_bitrate * m_frameDuration);
            int64_t shrtTermTotalBitsSum = 0;
            // Reset ABR if prev frames are blank to prevent further sudden overflows/ high bit rate spikes.
            for (int i = 0; i < s_slidingWindowFrames ; i++)
                shrtTermTotalBitsSum += m_encodedBitsWindow[i];
            double underflow = (shrtTermTotalBitsSum - shrtTermWantedBits) / abrBuffer;
            const double epsilon = 0.0001f;
            if ((underflow < epsilon || rce->isFadeEnd) && !isFrameDone)
            {
                init(*m_curSlice->m_sps);
                // Reduce tune complexity factor for scenes that follow blank frames
                double tuneCplxFactor = (m_ncu > 3600 && m_param->rc.cuTree && !m_param->rc.hevcAq) ? 2.5 : m_param->rc.hevcAq ? 1.5 : m_isGrainEnabled ? 1.9 : 1.0;
                m_cplxrSum /= tuneCplxFactor;
                m_shortTermCplxSum = rce->lastSatd / (CLIP_DURATION(m_frameDuration) / BASE_FRAME_DURATION);
                m_shortTermCplxCount = 1;
                m_isAbrReset = true;
                m_lastAbrResetPoc = rce->poc;
            }
        }
        else if (m_isAbrReset && isFrameDone)
        {
            // Clear flag to reset ABR and continue as usual.
            m_isAbrReset = false;
        }
    }
}

void RateControl::hrdFullness(SEIBufferingPeriod *seiBP)
{
    const VUI* vui = &m_curSlice->m_sps->vuiParameters;
    const HRDInfo* hrd = &vui->hrdParameters;
    int num = 90000;
    int denom = hrd->bitRateValue << (hrd->bitRateScale + BR_SHIFT);
    int64_t cpbState = (int64_t)m_bufferFillFinal;
    int64_t cpbSize = (int64_t)hrd->cpbSizeValue << (hrd->cpbSizeScale + CPB_SHIFT);

    if (cpbState < 0 || cpbState > cpbSize)
    {
        s265_log(m_param, S265_LOG_WARNING, "CPB %s: %.0lf bits in a %.0lf-bit buffer\n",
                 cpbState < 0 ? "underflow" : "overflow", (float)cpbState, (float)cpbSize);
    }

    seiBP->m_initialCpbRemovalDelay = (uint32_t)(num * cpbState / denom);
    seiBP->m_initialCpbRemovalDelayOffset = (uint32_t)(num * cpbSize / denom - seiBP->m_initialCpbRemovalDelay);
}

void RateControl::updateVbvPlan(Encoder* enc)
{
    m_bufferFill = m_bufferFillFinal;
    enc->updateVbvPlan(this);
}

double RateControl::predictSize(Predictor *p, double q, double var)
{
    return (p->coeff * var + p->offset) / (q * p->count);
}

double RateControl::clipQscale(Frame* curFrame, RateControlEntry* rce, double q)
{
    // B-frames are not directly subject to VBV,
    // since they are controlled by referenced P-frames' QPs.
    double lmin = m_lmin[rce->sliceType];
    double lmax = m_lmax[rce->sliceType];
    double q0 = q;
    if (m_isVbv && m_currentSatd > 0 && curFrame)
    {
        if (m_param->lookaheadDepth || m_param->rc.cuTree ||
            (m_param->scenecutThreshold || m_param->bHistBasedSceneCut) ||
            (m_param->bFrameAdaptive && m_param->bframes))
        {
           /* Lookahead VBV: If lookahead is done, raise the quantizer as necessary
            * such that no frames in the lookahead overflow and such that the buffer
            * is in a reasonable state by the end of the lookahead. */
            int loopTerminate = 0;
            /* Avoid an infinite loop. */
            for (int iterations = 0; iterations < 1000 && loopTerminate != 3; iterations++)
            {
                double frameQ[3];
                double curBits;
                curBits = predictSize(&m_pred[m_predType], q, (double)m_currentSatd);
                double bufferFillCur = m_bufferFill - curBits;
                double targetFill;
                double totalDuration = m_frameDuration;
                frameQ[P_SLICE] = m_sliceType == I_SLICE ? q * m_param->rc.ipFactor : (m_sliceType == B_SLICE ? q / m_param->rc.pbFactor : q);
                frameQ[B_SLICE] = frameQ[P_SLICE] * m_param->rc.pbFactor;
                frameQ[I_SLICE] = frameQ[P_SLICE] / m_param->rc.ipFactor;
                /* Loop over the planned future frames. */
                bool iter = true;
                for (int j = 0; bufferFillCur >= 0 && iter ; j++)
                {
                    int type = curFrame->m_lowres.plannedType[j];
                    if (type == S265_TYPE_AUTO || totalDuration >= 1.0)
                        break;
                    totalDuration += m_frameDuration;
                    double wantedFrameSize = m_vbvMaxRate * m_frameDuration;
                    if (bufferFillCur + wantedFrameSize <= m_bufferSize)
                        bufferFillCur += wantedFrameSize;
                    int64_t satd = curFrame->m_lowres.plannedSatd[j] >> (S265_DEPTH - 8);
                    type = IS_S265_TYPE_I(type) ? I_SLICE : IS_S265_TYPE_B(type) ? B_SLICE : P_SLICE;
                    int predType = getPredictorType(curFrame->m_lowres.plannedType[j], type);
                    curBits = predictSize(&m_pred[predType], frameQ[type], (double)satd);
                    bufferFillCur -= curBits;
                    if (!m_param->bResetZoneConfig && ((uint64_t)j == (m_param->reconfigWindowSize - 1)))
                        iter = false;
                }
                if (rce->vbvEndAdj)
                {
                    bool loopBreak = false;
                    double bufferDiff = m_param->vbvBufferEnd - (m_bufferFill / m_bufferSize);
                    rce->targetFill = m_bufferFill + m_bufferSize * (bufferDiff / (m_param->totalFrames - rce->encodeOrder));
                    if (bufferFillCur < rce->targetFill)
                    {
                        q *= 1.01;
                        loopTerminate |= 1;
                        loopBreak = true;
                    }
                    if (bufferFillCur > m_param->vbvBufferEnd * m_bufferSize)
                    {
                        q /= 1.01;
                        loopTerminate |= 2;
                        loopBreak = true;
                    }
                    if (!loopBreak)
                        break;
                }
                else
                {
                    /* Try to get the buffer at least 50% filled, but don't set an impossible goal. */
                    double finalDur = 1;
                    if (m_param->rc.bStrictCbr)
                    {
                        finalDur = s265_clip3(0.4, 1.0, totalDuration);
                    }
                    targetFill = S265_MIN(m_bufferFill + totalDuration * m_vbvMaxRate * 0.5, m_bufferSize * (1 - m_minBufferFill * finalDur));
                    if (bufferFillCur < targetFill)
                    {
                        q *= 1.01;
                        loopTerminate |= 1;
                        continue;
                    }
                    /* Try to get the buffer not more than 80% filled, but don't set an impossible goal. */

                    targetFill = s265_clip3(m_bufferSize * (1 - m_maxBufferFill * finalDur), m_bufferSize, m_bufferFill - totalDuration * m_vbvMaxRate * 0.5);
                    if ((m_isCbr) && bufferFillCur > targetFill && !m_isSceneTransition)
                    {
                        q /= 1.01;
                        loopTerminate |= 2;
                        continue;
                    }
                    break;
                }
            }
            q = S265_MAX(q0 / 2, q);
        }
        else
        {
            /* Fallback to old purely-reactive algorithm: no lookahead. */
            if ((m_sliceType == P_SLICE || m_sliceType == B_SLICE ||
                    (m_sliceType == I_SLICE && m_lastNonBPictType == I_SLICE)) &&
                m_bufferFill / m_bufferSize < m_minBufferFill)
            {
                q /= s265_clip3(0.5, 1.0, 2.0 * m_bufferFill / m_bufferSize);
            }
            // Now a hard threshold to make sure the frame fits in VBV.
            // This one is mostly for I-frames.
            double bits = predictSize(&m_pred[m_predType], q, (double)m_currentSatd);

            // For small VBVs, allow the frame to use up the entire VBV.
            double maxFillFactor;
            maxFillFactor = m_bufferSize >= 5 * m_bufferRate ? 2 : 1;
            // For single-frame VBVs, request that the frame use up the entire VBV.
            double minFillFactor = m_singleFrameVbv ? 1 : 2;

            for (int iterations = 0; iterations < 10; iterations++)
            {
                double qf = 1.0;
                if (bits > m_bufferFill / maxFillFactor)
                    qf = s265_clip3(0.2, 1.0, m_bufferFill / (maxFillFactor * bits));
                q /= qf;
                bits *= qf;
                if (bits < m_bufferRate / minFillFactor)
                    q *= bits * minFillFactor / m_bufferRate;
                bits = predictSize(&m_pred[m_predType], q, (double)m_currentSatd);
            }

            q = S265_MAX(q0, q);
        }

        /* Apply MinCR restrictions */
        double pbits = predictSize(&m_pred[m_predType], q, (double)m_currentSatd);
        if (pbits > rce->frameSizeMaximum)
            q *= pbits / rce->frameSizeMaximum;
        /* To detect frames that are more complex in SATD costs compared to prev window, yet 
         * lookahead vbv reduces its qscale by half its value. Be on safer side and avoid drastic 
         * qscale reductions for frames high in complexity */
        bool mispredCheck = rce->movingAvgSum && m_currentSatd >= rce->movingAvgSum && q <= q0 / 2;
        if (!m_isCbr || ((m_isAbr) && mispredCheck))
            q = S265_MAX(q0, q);

        if (m_rateFactorMaxIncrement)
        {
            double qpNoVbv = s265_qScale2qp(q0);
            double qmax = S265_MIN(lmax,s265_qp2qScale(qpNoVbv + m_rateFactorMaxIncrement));
            return s265_clip3(lmin, qmax, q);
        }
    }
    return s265_clip3(lmin, lmax, q);
}

double RateControl::predictRowsSizeSum(Frame* curFrame, RateControlEntry* rce, double qpVbv, int32_t& encodedBitsSoFar)
{
    uint32_t rowSatdCostSoFar = 0, totalSatdBits = 0;
    encodedBitsSoFar = 0;

    double qScale = s265_qp2qScale(qpVbv);
    FrameData& curEncData = *curFrame->m_encData;
    int picType = curEncData.m_slice->m_sliceType;
    Frame* refFrame = curEncData.m_slice->m_refFrameList[0][0];

    uint32_t maxRows = curEncData.m_slice->m_sps->numCuInHeight;
    uint32_t maxCols = curEncData.m_slice->m_sps->numCuInWidth;

    for (uint32_t row = 0; row < maxRows; row++)
    {
        encodedBitsSoFar += curEncData.m_rowStat[row].encodedBits;
        rowSatdCostSoFar = curEncData.m_rowStat[row].rowSatd;
        uint32_t satdCostForPendingCus = curEncData.m_rowStat[row].satdForVbv - rowSatdCostSoFar;
        satdCostForPendingCus >>= S265_DEPTH - 8;
        if (satdCostForPendingCus  > 0)
        {
            double pred_s = predictSize(rce->rowPred[0], qScale, satdCostForPendingCus);
            uint32_t refRowSatdCost = 0, refRowBits = 0, intraCostForPendingCus = 0;
            double refQScale = 0;

            if (picType != I_SLICE && !m_param->rc.bEnableConstVbv)
            {
                FrameData& refEncData = *refFrame->m_encData;
                uint32_t endCuAddr = maxCols * (row + 1);
                uint32_t startCuAddr = curEncData.m_rowStat[row].numEncodedCUs;
                if (startCuAddr)
                {
                    for (uint32_t cuAddr = startCuAddr + 1 ; cuAddr < endCuAddr; cuAddr++)
                    {
                        refRowSatdCost += refEncData.m_cuStat[cuAddr].vbvCost;
                        refRowBits += refEncData.m_cuStat[cuAddr].totalBits;
                    }
                }
                else
                {
                    refRowBits = refEncData.m_rowStat[row].encodedBits;
                    refRowSatdCost = refEncData.m_rowStat[row].satdForVbv;
                }

                refRowSatdCost >>= S265_DEPTH - 8;
                refQScale = refEncData.m_rowStat[row].rowQpScale;
            }

            if (picType == I_SLICE || qScale >= refQScale)
            {
                if (picType == P_SLICE 
                    && refFrame 
                    && refFrame->m_encData->m_slice->m_sliceType == picType
                    && refQScale > 0
                    && refRowBits > 0
                    && !m_param->rc.bEnableConstVbv)
                {
                    if (abs((int32_t)(refRowSatdCost - satdCostForPendingCus)) < (int32_t)satdCostForPendingCus / 2)
                    {
                        double predTotal = refRowBits * satdCostForPendingCus / refRowSatdCost * refQScale / qScale;
                        totalSatdBits += (int32_t)((pred_s + predTotal) * 0.5);
                        continue;
                    }
                }
                totalSatdBits += (int32_t)pred_s;
            }
            else if (picType == P_SLICE)
            {
                intraCostForPendingCus = curEncData.m_rowStat[row].intraSatdForVbv - curEncData.m_rowStat[row].rowIntraSatd;
                intraCostForPendingCus >>= S265_DEPTH - 8;
                /* Our QP is lower than the reference! */
                double pred_intra = predictSize(rce->rowPred[1], qScale, intraCostForPendingCus);
                /* Sum: better to overestimate than underestimate by using only one of the two predictors. */
                totalSatdBits += (int32_t)(pred_intra + pred_s);
            }
            else
                totalSatdBits += (int32_t)pred_s;
        }
    }

    return totalSatdBits + encodedBitsSoFar;
}

int RateControl::rowVbvRateControl(Frame* curFrame, uint32_t row, RateControlEntry* rce, double& qpVbv, uint32_t* m_sliceBaseRow, uint32_t sliceId)
{
    FrameData& curEncData = *curFrame->m_encData;
    double qScaleVbv = s265_qp2qScale(qpVbv);
    uint64_t rowSatdCost = curEncData.m_rowStat[row].rowSatd;
    double encodedBits = curEncData.m_rowStat[row].encodedBits;

    rowSatdCost >>= S265_DEPTH - 8;
    updatePredictor(rce->rowPred[0], qScaleVbv, (double)rowSatdCost, encodedBits);
    if (curEncData.m_slice->m_sliceType != I_SLICE && !m_param->rc.bEnableConstVbv)
    {
        Frame* refFrame = curEncData.m_slice->m_refFrameList[0][0];
        if (qpVbv < refFrame->m_encData->m_rowStat[row].rowQp)
        {
            uint64_t intraRowSatdCost = curEncData.m_rowStat[row].rowIntraSatd;
            intraRowSatdCost >>= S265_DEPTH - 8;
            updatePredictor(rce->rowPred[1], qScaleVbv, (double)intraRowSatdCost, encodedBits);
        }
    }

    int canReencodeRow = 1;
    /* tweak quality based on difference from predicted size */
    double prevRowQp = qpVbv;
    double qpAbsoluteMax = m_param->rc.qpMax;
    double qpAbsoluteMin = m_param->rc.qpMin;
    if (m_rateFactorMaxIncrement)
        qpAbsoluteMax = S265_MIN(qpAbsoluteMax, rce->qpNoVbv + m_rateFactorMaxIncrement);

    if (m_rateFactorMaxDecrement)
        qpAbsoluteMin = S265_MAX(qpAbsoluteMin, rce->qpNoVbv - m_rateFactorMaxDecrement);

    double qpMax = S265_MIN(prevRowQp + m_param->rc.qpStep, qpAbsoluteMax);
    double qpMin = S265_MAX(prevRowQp - m_param->rc.qpStep, qpAbsoluteMin);
    double stepSize = 0.5;
    double bufferLeftPlanned = rce->bufferFill - rce->frameSizePlanned;

    const SPS& sps = *curEncData.m_slice->m_sps;
    double maxFrameError = S265_MAX(0.05, 1.0 / sps.numCuInHeight);

    if (row < m_sliceBaseRow[sliceId + 1] - 1)
    {
        /* More threads means we have to be more cautious in letting ratecontrol use up extra bits. */
        double rcTol = bufferLeftPlanned / m_param->frameNumThreads * m_rateTolerance;
        int32_t encodedBitsSoFar = 0;
        double accFrameBits = predictRowsSizeSum(curFrame, rce, qpVbv, encodedBitsSoFar);
        double vbvEndBias = 0.95;

        /* * Don't increase the row QPs until a sufficent amount of the bits of
         * the frame have been processed, in case a flat area at the top of the
         * frame was measured inaccurately. */
        if (encodedBitsSoFar < 0.05f * rce->frameSizePlanned)
            qpMax = qpAbsoluteMax = prevRowQp;

        if (rce->sliceType != I_SLICE || (m_param->rc.bStrictCbr && rce->poc > 0))
            rcTol *= 0.5;

        if (!m_isCbr)
            qpMin = S265_MAX(qpMin, rce->qpNoVbv);

        double totalBitsNeeded = m_wantedBitsWindow;
        if (m_param->totalFrames)
            totalBitsNeeded = (m_param->totalFrames * m_bitrate) / m_fps;
        double abrOvershoot = (accFrameBits + m_totalBits - m_wantedBitsWindow) / totalBitsNeeded;

        while (qpVbv < qpMax
               && (((accFrameBits > rce->frameSizePlanned + rcTol) ||
                   (rce->bufferFill - accFrameBits < bufferLeftPlanned * 0.5) ||
                   (accFrameBits > rce->frameSizePlanned && qpVbv < rce->qpNoVbv) ||
                   (rce->vbvEndAdj && ((rce->bufferFill - accFrameBits) < (rce->targetFill * vbvEndBias))))
                   && (!m_param->rc.bStrictCbr ? 1 : abrOvershoot > 0.1)))
        {
            qpVbv += stepSize;
            accFrameBits = predictRowsSizeSum(curFrame, rce, qpVbv, encodedBitsSoFar);
            abrOvershoot = (accFrameBits + m_totalBits - m_wantedBitsWindow) / totalBitsNeeded;
        }

        while (qpVbv > qpMin
               && (qpVbv > curEncData.m_rowStat[0].rowQp || m_singleFrameVbv)
               && (((accFrameBits < rce->frameSizePlanned * 0.8f && qpVbv <= prevRowQp)
                   || accFrameBits < (rce->bufferFill - m_bufferSize + m_bufferRate) * 1.1
                   || (rce->vbvEndAdj && ((rce->bufferFill - accFrameBits) > (rce->targetFill * vbvEndBias))))
                   && (!m_param->rc.bStrictCbr ? 1 : abrOvershoot < 0)))
        {
            qpVbv -= stepSize;
            accFrameBits = predictRowsSizeSum(curFrame, rce, qpVbv, encodedBitsSoFar);
            abrOvershoot = (accFrameBits + m_totalBits - m_wantedBitsWindow) / totalBitsNeeded;
        }

        if (m_param->rc.bStrictCbr && m_param->totalFrames)
        {
            double timeDone = (double)(m_framesDone) / m_param->totalFrames;
            while (qpVbv < qpMax && (qpVbv < rce->qpNoVbv + (m_param->rc.qpStep * timeDone)) &&
                   (timeDone > 0.75 && abrOvershoot > 0))
            {
                qpVbv += stepSize;
                accFrameBits = predictRowsSizeSum(curFrame, rce, qpVbv, encodedBitsSoFar);
                abrOvershoot = (accFrameBits + m_totalBits - m_wantedBitsWindow) / totalBitsNeeded;
            }
            if (qpVbv > curEncData.m_rowStat[0].rowQp &&
                abrOvershoot < -0.1 && timeDone > 0.5 && accFrameBits < rce->frameSizePlanned - rcTol)
            {
                qpVbv -= stepSize;
                accFrameBits = predictRowsSizeSum(curFrame, rce, qpVbv, encodedBitsSoFar);
            }
        }

        /* avoid VBV underflow or MinCr violation */
        while ((qpVbv < qpAbsoluteMax)
               && ((rce->bufferFill - accFrameBits < m_bufferRate * maxFrameError) ||
                   (rce->frameSizeMaximum - accFrameBits < rce->frameSizeMaximum * maxFrameError)))
        {
            qpVbv += stepSize;
            accFrameBits = predictRowsSizeSum(curFrame, rce, qpVbv, encodedBitsSoFar);
        }

        rce->frameSizeEstimated = accFrameBits;

        /* If the current row was large enough to cause a large QP jump, try re-encoding it. */
        if (qpVbv > qpMax && prevRowQp < qpMax && canReencodeRow)
        {
            /* Bump QP to halfway in between... close enough. */
            qpVbv = s265_clip3(prevRowQp + 1.0f, qpMax, (prevRowQp + qpVbv) * 0.5);
            return -1;
        }

        if (m_param->rc.rfConstantMin)
        {
            if (qpVbv < qpMin && prevRowQp > qpMin && canReencodeRow)
            {
                qpVbv = s265_clip3(qpMin, prevRowQp, (prevRowQp + qpVbv) * 0.5);
                return -1;
            }
        }
    }
    else
    {
        int32_t encodedBitsSoFar = 0;
        rce->frameSizeEstimated = predictRowsSizeSum(curFrame, rce, qpVbv, encodedBitsSoFar);

        /* Last-ditch attempt: if the last row of the frame underflowed the VBV,
         * try again. */
        if ((rce->frameSizeEstimated > (rce->bufferFill - m_bufferRate * maxFrameError) &&
             qpVbv < qpMax && canReencodeRow))
        {
            qpVbv = qpMax;
            return -1;
        }
    }
    return 0;
}

/* modify the bitrate curve from pass1 for one frame */
double RateControl::getQScale(RateControlEntry *rce, double rateFactor)
{
    double q;

    if (m_param->rc.cuTree && !m_param->rc.hevcAq)
    {
        // Scale and units are obtained from rateNum and rateDenom for videos with fixed frame rates.
        double timescale = (double)m_param->fpsDenom / (2 * m_param->fpsNum);
        q = pow(BASE_FRAME_DURATION / CLIP_DURATION(2 * timescale), 1 - m_param->rc.qCompress);
    }
    else
        q = pow(rce->blurredComplexity, 1 - m_param->rc.qCompress);

    // avoid NaN's in the Rceq
    if (rce->coeffBits + rce->mvBits == 0)
        q = m_lastQScaleFor[rce->sliceType];
    else
    {
        m_lastRceq = q;
        q /= rateFactor;
    }

    return q;
}

void RateControl::updatePredictor(Predictor *p, double q, double var, double bits)
{
    if (var < 10)
        return;
    const double range = 2;
    double old_coeff = p->coeff / p->count;
    double old_offset = p->offset / p->count;
    double new_coeff = S265_MAX((bits * q - old_offset) / var, p->coeffMin );
    double new_coeff_clipped = s265_clip3(old_coeff / range, old_coeff * range, new_coeff);
    double new_offset = bits * q - new_coeff_clipped * var;
    if (new_offset >= 0)
        new_coeff = new_coeff_clipped;
    else
        new_offset = 0;
    p->count  *= p->decay;
    p->coeff  *= p->decay;
    p->offset *= p->decay;
    p->count++;
    p->coeff  += new_coeff;
    p->offset += new_offset;
}
// called by frameEncoder threads
// bits: 当前帧编码实际产生的bits
// updateVbv 和 rateControlStart rateControlEnd 一样 按照严格的顺序执行
// 虽然会被不同的线程执行，但不会有竞争的现象
int RateControl::updateVbv(int64_t bits, RateControlEntry* rce)
{
    int predType = rce->sliceType;
    int filler = 0;
    double bufferBits;
    predType = rce->sliceType == B_SLICE && rce->keptAsRef ? 3 : predType;
    if (rce->lastSatd >= m_ncu && rce->encodeOrder >= m_lastPredictorReset)
        updatePredictor(&m_pred[predType], s265_qp2qScale(rce->qpaRc), (double)rce->lastSatd, (double)bits);w

    m_bufferFillFinal -= bits;

    if (m_bufferFillFinal < 0)
        s265_log(m_param, S265_LOG_WARNING, "poc:%d, VBV underflow (%.0f bits)\n", rce->poc, m_bufferFillFinal);

    m_bufferFillFinal = S265_MAX(m_bufferFillFinal, 0);
    m_bufferFillFinal += rce->bufferRate;//加上应流入的bits
    if (m_param->csvLogLevel >= 2)
        m_unclippedBufferFillFinal = m_bufferFillFinal;

    if (m_param->rc.bStrictCbr)
    {
        if (m_bufferFillFinal > m_bufferSize)
        {   // 可用bits量有多，填充filler
            filler = (int)(m_bufferFillFinal - m_bufferSize);
            filler += FILLER_OVERHEAD * 8;
        }
        m_bufferFillFinal -= filler;
        bufferBits = S265_MIN(bits + filler + m_bufferExcess, rce->bufferRate);
        m_bufferExcess = S265_MAX(m_bufferExcess - bufferBits + bits + filler, 0);
        m_bufferFillActual += bufferBits - bits - filler;
    }
    else
    {
        m_bufferFillFinal = S265_MIN(m_bufferFillFinal, m_bufferSize);//fillFinal 肯定要小于bufferSize
        // 当实际使用掉了的bits一直< bufferRate时，m_bufferExcess 一直为0,bufferBits 则一直等于实际消耗的bits，m_bufferFillActual一直为m_bufferFillFinal
        // 当实际使用掉了的bits 超过 bufferRate时，bufferBits 取去上界bufferRate，m_bufferExcess 更新本次多用了的bits,m_bufferFillActual 会下降(用多了)
        // 如果后面用掉的bits仍然太多了，bufferBits 取去上界bufferRate，m_bufferExcess 在之前多用的基础上继续累加，m_bufferFillActual 会继续下降(用多了)
        // 如果后面用掉的bits < bufferRate了:
            //如果 bits + 之前整个多用了的 < bufferRate了 ,则 bufferBits 取值为 实际消耗的+之前整个多用了的，m_bufferExcess 跟新为0 m_bufferFillActual 向上调整
            //如果 bits + 之前整个多用了的 > bufferRate了,则 bufferBits 取值为 bufferRate， m_bufferExcess将变小留给后面继续调整，m_bufferFillActual 向上调整
            //直到 m_bufferExcess 跟新为0 为了 m_bufferFillActual 不在调整
        bufferBits = S265_MIN(bits + m_bufferExcess, rce->bufferRate);//本次实际用掉的bits+之前多用的bits（如果之前还有多用了的） 最多不超过bufferRate,表示实际流入vbv 的bits
        m_bufferExcess = S265_MAX(m_bufferExcess - bufferBits + bits, 0);// 下次进来时 还需要考虑的剩余了的多用的bits
        m_bufferFillActual += bufferBits - bits;//alctual fill + 应流入 - 实际输出的bits
        m_bufferFillActual = S265_MIN(m_bufferFillActual, m_bufferSize);
    }
    return filler;// 返回需要填充的bits
}

/* After encoding one frame, update rate control state */
// 每个帧编码线程在编码一帧全部完成后 called by frameEncode threads
int RateControl::rateControlEnd(Frame* curFrame, int64_t bits, RateControlEntry* rce, int *filler)
{
    int orderValue = m_startEndOrder.get();//从主线程获取start end order
    // (当前编码帧序号 + 一圈的帧级线程） 需要等一圈的帧编码都完成了rateControlEnd 
    int endOrdinal = (rce->encodeOrder + m_param->frameNumThreads) * 2 - 1;
    //如果主线程当前统计到 order 值小于 （当前线程编码帧号+再加其他正在并行帧）* 2 -1
    // 执行当前帧的End 时 之前需要等并行帧中最新的帧完成update的调用
    while (orderValue < endOrdinal && !m_bTerminated)
    {
        /* no more frames are being encoded, so fake the start event if we would
         * have blocked on it. Note that this does not enforce rateControlEnd()
         * ordering during flush, but this has no impact on the outputs */
        // 遇到编码器flush了
        if (m_finalFrameCount && orderValue >= 2 * m_finalFrameCount)
            break;
        orderValue = m_startEndOrder.waitForChange(orderValue);
    }

    FrameData& curEncData = *curFrame->m_encData;
    int64_t actualBits = bits;
    Slice *slice = curEncData.m_slice;

    if (m_param->rc.aqMode || m_isVbv || m_param->bAQMotion)
    {
        if (m_isVbv)
        {
            double avgQpRc = 0;
            /* determine avg QP decided by VBV rate control */
            for (uint32_t i = 0; i < slice->m_sps->numCuInHeight; i++)
                avgQpRc += curEncData.m_rowStat[i].sumQpRc;

            avgQpRc /= slice->m_sps->numCUsInFrame;//计算所有CTU的平均QPRC
            curEncData.m_avgQpRc = s265_clip3((double)m_param->rc.qpMin, (double)m_param->rc.qpMax, avgQpRc);
            rce->qpaRc = curEncData.m_avgQpRc;
        }

        if (m_param->rc.aqMode || m_param->bAQMotion)
        {
            double avgQpAq = 0;
            /* determine actual avg encoded QP, after AQ/cutree/distortion adjustments */
            for (uint32_t i = 0; i < slice->m_sps->numCuInHeight; i++)
                avgQpAq += curEncData.m_rowStat[i].sumQpAq;

            avgQpAq /= (slice->m_sps->numCUsInFrame * m_param->num4x4Partitions);//计算所有4x4的平均qp
            curEncData.m_avgQpAq = avgQpAq;
        }
        else
            curEncData.m_avgQpAq = curEncData.m_avgQpRc;
    }

    if (m_isAbr)
    {
        if (m_param->rc.rateControlMode == S265_RC_ABR)// abr 码空先不看
            checkAndResetABR(rce, true);
    }
    if (m_param->rc.rateControlMode == S265_RC_CRF)
    {
        double crfVal, qpRef = curEncData.m_avgQpRc;

        if (fabs(qpRef - rce->qpNoVbv) > 0.5)
        {
            double crfFactor = rce->qRceq /s265_qp2qScale(qpRef);
            double baseCplx = m_ncu * (m_param->bframes ? 120 : 80);
            double mbtree_offset = m_param->rc.cuTree ? (1.0 - m_param->rc.qCompress) * 13.5 : 0;
            crfVal = s265_qScale2qp(pow(baseCplx, 1 - m_qCompress) / crfFactor) - mbtree_offset;
        }
        else
            crfVal = rce->sliceType == I_SLICE ? m_param->rc.rfConstant - m_ipOffset : 
            (rce->sliceType == B_SLICE ? m_param->rc.rfConstant + m_pbOffset : m_param->rc.rfConstant);

        curEncData.m_rateFactor = crfVal;
    }

    if (m_isAbr && !m_isAbrReset)
    {
        if (rce->sliceType == I_SLICE)
        {
            /* previous I still had a residual; roll it into the new loan */
            if (m_residualFrames)
                bits += m_residualCost * m_residualFrames;
            m_residualFrames = S265_MIN((int)rce->amortizeFrames, m_param->keyframeMax);
            m_residualCost = (int)((bits * rce->amortizeFraction) / m_residualFrames);
            bits -= m_residualCost * m_residualFrames;
        }
        else if (m_residualFrames)
        {
            bits += m_residualCost;
            m_residualFrames--;
        }
        if (rce->sliceType != B_SLICE)
        {
            /* The factor 1.5 is to tune up the actual bits, otherwise the cplxrSum is scaled too low
                * to improve short term compensation for next frame. */
            m_cplxrSum += (bits * s265_qp2qScale(rce->qpaRc) / rce->qRceq) - (rce->rowCplxrSum);
        }
        else
        {
            /* Depends on the fact that B-frame's QP is an offset from the following P-frame's.
                * Not perfectly accurate with B-refs, but good enough. */
            m_cplxrSum += (bits * s265_qp2qScale(rce->qpaRc) / (rce->qRceq * fabs(m_param->rc.pbFactor))) - (rce->rowCplxrSum);
        }
        m_wantedBitsWindow += m_frameDuration * m_bitrate;
        m_totalBits += bits - rce->rowTotalBits;
        m_encodedBits += actualBits;
        int pos = m_sliderPos - m_param->frameNumThreads;
        if (pos >= 0)
            m_encodedBitsWindow[pos % s_slidingWindowFrames] = actualBits;
        if(rce->sliceType != I_SLICE)
        {
            int qp = int (rce->qpaRc + 0.5);
            m_qpToEncodedBits[qp] =  m_qpToEncodedBits[qp] == 0 ? actualBits : (m_qpToEncodedBits[qp] + actualBits) * 0.5;
        }
    }

    if (m_isVbv)
    {
        *filler = updateVbv(actualBits, rce);
        if (m_param->bEmitHRDSEI)
        {
            const VUI *vui = &curEncData.m_slice->m_sps->vuiParameters;
            const HRDInfo *hrd = &vui->hrdParameters;
            const TimingInfo *time = &vui->timingInfo;
            if (!curFrame->m_poc)
            {
                // first access unit initializes the HRD
                rce->hrdTiming->cpbInitialAT = 0;
                rce->hrdTiming->cpbRemovalTime = m_nominalRemovalTime = (double)m_bufPeriodSEI.m_initialCpbRemovalDelay / 90000;
            }
            else
            {
                rce->hrdTiming->cpbRemovalTime = m_nominalRemovalTime + (double)rce->picTimingSEI->m_auCpbRemovalDelay * time->numUnitsInTick / time->timeScale;
                double cpbEarliestAT = rce->hrdTiming->cpbRemovalTime - (double)m_bufPeriodSEI.m_initialCpbRemovalDelay / 90000;
                if (!curFrame->m_lowres.bKeyframe)
                    cpbEarliestAT -= (double)m_bufPeriodSEI.m_initialCpbRemovalDelayOffset / 90000;

                rce->hrdTiming->cpbInitialAT = hrd->cbrFlag ? m_prevCpbFinalAT : S265_MAX(m_prevCpbFinalAT, cpbEarliestAT);
            }
            int filler_bits = *filler ? (*filler - START_CODE_OVERHEAD * 8)  : 0; 
            uint32_t cpbsizeUnscale = hrd->cpbSizeValue << (hrd->cpbSizeScale + CPB_SHIFT);
            rce->hrdTiming->cpbFinalAT = m_prevCpbFinalAT = rce->hrdTiming->cpbInitialAT + (actualBits + filler_bits)/ cpbsizeUnscale;
            rce->hrdTiming->dpbOutputTime = (double)rce->picTimingSEI->m_picDpbOutputDelay * time->numUnitsInTick / time->timeScale + rce->hrdTiming->cpbRemovalTime;
        }
    }
    rce->isActive = false;
    // Allow rateControlStart of next frame only when rateControlEnd of previous frame is over
    m_startEndOrder.incr(); // 码控/非码控 都end +1
    return 0;
}

#if defined(_MSC_VER)
#pragma warning(disable: 4996) // POSIX function names are just fine, thank you
#endif

/* called when the encoder is flushing, and thus the final frame count is
 * unambiguously known */
void RateControl::setFinalFrameCount(int count)
{
    m_finalFrameCount = count;
    /* unblock waiting threads */
    m_startEndOrder.poke();
}

/* called when the encoder is closing, and no more frames will be output.
 * all blocked functions must finish so the frame encoder threads can be
 * closed */
void RateControl::terminate()
{
    m_bTerminated = true;
    /* unblock waiting threads */
    m_startEndOrder.poke();
}

void RateControl::destroy()
{   
    if (m_relativeComplexity)
        S265_FREE(m_relativeComplexity);

}

void RateControl::splitdeltaPOC(char deltapoc[], RateControlEntry *rce)
{
    int idx = 0, length = 0;
    char tmpStr[128];
    char* src = deltapoc;
    char* buf = strstr(src, "~");
    while (buf)
    {
        memset(tmpStr, 0, sizeof(tmpStr));
        length = (int)(buf - src);
        if (length != 0)
        {
            strncpy(tmpStr, src, length);
            rce->rpsData.deltaPOC[idx] = atoi(tmpStr);
            idx++;
            if (idx == rce->rpsData.numberOfPictures)
                break;
        }
        src += (length + 1);
        buf = strstr(src, "~");
    }
}

void RateControl::splitbUsed(char bused[], RateControlEntry *rce)
{
    int idx = 0, length = 0;
    char tmpStr[128];
    char* src = bused;
    char* buf = strstr(src, "~");
    while (buf)
    {
        memset(tmpStr, 0, sizeof(tmpStr));
        length = (int)(buf - src);
        if (length != 0)
        {
            strncpy(tmpStr, src, length);
            rce->rpsData.bUsed[idx] = atoi(tmpStr) > 0;
            idx++;
            if (idx == rce->rpsData.numberOfPictures)
                break;
        }
        src += (length + 1);
        buf = strstr(src, "~");
    }
}

double RateControl::forwardMasking(Frame* curFrame, double q)
{
    double qp = s265_qScale2qp(q);
    uint32_t maxWindowSize = uint32_t((m_param->fwdScenecutWindow / 1000.0) * (m_param->fpsNum / m_param->fpsDenom) + 0.5);
    uint32_t windowSize = maxWindowSize / 3;
    int lastScenecut = m_top->m_rateControl->m_lastScenecut;
    int lastIFrame = m_top->m_rateControl->m_lastScenecutAwareIFrame;
    double fwdRefQpDelta = double(m_param->fwdRefQpDelta);
    double fwdNonRefQpDelta = double(m_param->fwdNonRefQpDelta);
    double sliceTypeDelta = SLICE_TYPE_DELTA * fwdRefQpDelta;

    //Check whether the current frame is within the forward window
    if (curFrame->m_poc > lastScenecut && curFrame->m_poc <= (lastScenecut + int(maxWindowSize)))
        curFrame->m_isInsideWindow = FORWARD_WINDOW;
    if (curFrame->m_isInsideWindow == FORWARD_WINDOW)
    {
        if (IS_S265_TYPE_I(curFrame->m_lowres.sliceType) || curFrame->m_lowres.bScenecut)
        {
            m_top->m_rateControl->m_lastScenecutAwareIFrame = curFrame->m_poc;
        }
        else if (curFrame->m_lowres.sliceType == S265_TYPE_P)
        {
            if (!(lastIFrame > lastScenecut && lastIFrame <= (lastScenecut + int(maxWindowSize))
                && curFrame->m_poc >= lastIFrame))
            {
                //Add offsets corresponding to the window in which the P-frame occurs
                if (curFrame->m_poc <= (lastScenecut + int(windowSize)))
                    qp += WINDOW1_DELTA * (fwdRefQpDelta - sliceTypeDelta);
                else if (((curFrame->m_poc) > (lastScenecut + int(windowSize))) && ((curFrame->m_poc) <= (lastScenecut + 2 * int(windowSize))))
                    qp += WINDOW2_DELTA * (fwdRefQpDelta - sliceTypeDelta);
                else if (curFrame->m_poc > lastScenecut + 2 * int(windowSize))
                    qp += WINDOW3_DELTA * (fwdRefQpDelta - sliceTypeDelta);
            }
        }
        else if (curFrame->m_lowres.sliceType == S265_TYPE_BREF)
        {
            if (!(lastIFrame > lastScenecut && lastIFrame <= (lastScenecut + int(maxWindowSize))
                && curFrame->m_poc >= lastIFrame))
            {
                //Add offsets corresponding to the window in which the B-frame occurs
                if (curFrame->m_poc <= (lastScenecut + int(windowSize)))
                    qp += WINDOW1_DELTA * fwdRefQpDelta;
                else if (((curFrame->m_poc) > (lastScenecut + int(windowSize))) && ((curFrame->m_poc) <= (lastScenecut + 2 * int(windowSize))))
                    qp += WINDOW2_DELTA * fwdRefQpDelta;
                else if (curFrame->m_poc > lastScenecut + 2 * int(windowSize))
                    qp += WINDOW3_DELTA * fwdRefQpDelta;
            }
        }
        else if (curFrame->m_lowres.sliceType == S265_TYPE_B)
        {
            if (!(lastIFrame > lastScenecut && lastIFrame <= (lastScenecut + int(maxWindowSize))
                && curFrame->m_poc >= lastIFrame))
            {
                //Add offsets corresponding to the window in which the b-frame occurs
                if (curFrame->m_poc <= (lastScenecut + int(windowSize)))
                    qp += WINDOW1_DELTA * fwdNonRefQpDelta;
                else if (((curFrame->m_poc) > (lastScenecut + int(windowSize))) && ((curFrame->m_poc) <= (lastScenecut + 2 * int(windowSize))))
                    qp += WINDOW2_DELTA * fwdNonRefQpDelta;
                else if (curFrame->m_poc > lastScenecut + 2 * int(windowSize))
                    qp += WINDOW3_DELTA * fwdNonRefQpDelta;
            }
        }
    }

    return s265_qp2qScale(qp);
}
double RateControl::backwardMasking(Frame* curFrame, double q)
{
    double qp = s265_qScale2qp(q);
    double fwdRefQpDelta = double(m_param->fwdRefQpDelta);
    double bwdRefQpDelta = double(m_param->bwdRefQpDelta);
    double bwdNonRefQpDelta = double(m_param->bwdNonRefQpDelta);

    if (curFrame->m_isInsideWindow == BACKWARD_WINDOW)
    {
        if (bwdRefQpDelta < 0)
            bwdRefQpDelta = WINDOW3_DELTA * fwdRefQpDelta;
        double sliceTypeDelta = SLICE_TYPE_DELTA * bwdRefQpDelta;
        if (bwdNonRefQpDelta < 0)
            bwdNonRefQpDelta = bwdRefQpDelta + sliceTypeDelta;

        if (curFrame->m_lowres.sliceType == S265_TYPE_P)
            qp += bwdRefQpDelta - sliceTypeDelta;
        else if (curFrame->m_lowres.sliceType == S265_TYPE_BREF)
            qp += bwdRefQpDelta;
        else if (curFrame->m_lowres.sliceType == S265_TYPE_B)
            qp += bwdNonRefQpDelta;
    }

    return s265_qp2qScale(qp);
}
