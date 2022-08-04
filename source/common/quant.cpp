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
#include "primitives.h"
#include "quant.h"
#include "framedata.h"
#include "entropy.h"
#include "yuv.h"
#include "cudata.h"
#include "contexts.h"

using namespace S265_NS;

#define SIGN(x,y) ((x^(y >> 31))-(y >> 31))

namespace {

struct coeffGroupRDStats
{
    int     nnzBeforePos0;     /* indicates coeff other than pos 0 are coded */
    int64_t codedLevelAndDist; /* distortion and level cost of coded coefficients */
    int64_t uncodedDist;       /* uncoded distortion cost of coded coefficients */
    int64_t sigCost;           /* cost of signaling significant coeff bitmap */
    int64_t sigCost0;          /* cost of signaling sig coeff bit of coeff 0 */
};

inline int fastMin(int x, int y)
{
    return y + ((x - y) & ((x - y) >> (sizeof(int) * CHAR_BIT - 1))); // min(x, y)
}

inline int getICRate(uint32_t absLevel, int32_t diffLevel, const int* greaterOneBits, const int* levelAbsBits, const uint32_t absGoRice, const uint32_t maxVlc, const uint32_t c1c2Rate)
{
    S265_CHECK(absGoRice <= 4, "absGoRice check failure\n");
    if (!absLevel)
    {
        S265_CHECK(diffLevel < 0, "diffLevel check failure\n");
        return 0;
    }
    int rate = 0;

    if (diffLevel < 0)
    {
        S265_CHECK(absLevel <= 2, "absLevel check failure\n");
        rate += greaterOneBits[(absLevel == 2)];

        if (absLevel == 2)
            rate += levelAbsBits[0];
    }
    else
    {
        uint32_t symbol = diffLevel;
        bool expGolomb = (symbol > maxVlc);

        if (expGolomb)
        {
            absLevel = symbol - maxVlc;

            // NOTE: mapping to x86 hardware instruction BSR
            unsigned long size;
            CLZ(size, absLevel);
            int egs = size * 2 + 1;

            rate += egs << 15;

            // NOTE: in here, expGolomb=true means (symbol >= maxVlc + 1)
            S265_CHECK(fastMin(symbol, (maxVlc + 1)) == (int)maxVlc + 1, "min check failure\n");
            symbol = maxVlc + 1;
        }

        uint32_t prefLen = (symbol >> absGoRice) + 1;
        uint32_t numBins = fastMin(prefLen + absGoRice, 8 /* g_goRicePrefixLen[absGoRice] + absGoRice */);

        rate += numBins << 15;
        rate += c1c2Rate;
    }
    return rate;
}

#if CHECKED_BUILD || _DEBUG
inline int getICRateNegDiff(uint32_t absLevel, const int* greaterOneBits, const int* levelAbsBits)
{
    S265_CHECK(absLevel <= 2, "absLevel check failure\n");

    int rate;
    if (absLevel == 0)
        rate = 0;
    else if (absLevel == 2)
        rate = greaterOneBits[1] + levelAbsBits[0];
    else
        rate = greaterOneBits[0];
    return rate;
}
#endif

inline int getICRateLessVlc(uint32_t absLevel, int32_t diffLevel, const uint32_t absGoRice)
{
    S265_CHECK(absGoRice <= 4, "absGoRice check failure\n");
    if (!absLevel)
    {
        S265_CHECK(diffLevel < 0, "diffLevel check failure\n");
        return 0;
    }
    int rate;

    uint32_t symbol = diffLevel;
    uint32_t prefLen = (symbol >> absGoRice) + 1;
    uint32_t numBins = fastMin(prefLen + absGoRice, 8 /* g_goRicePrefixLen[absGoRice] + absGoRice */);

    rate = numBins << 15;

    return rate;
}

/* Calculates the cost for specific absolute transform level */
inline uint32_t getICRateCost(uint32_t absLevel, int32_t diffLevel, const int* greaterOneBits, const int* levelAbsBits, uint32_t absGoRice, const uint32_t c1c2Rate)
{
    S265_CHECK(absLevel, "absLevel should not be zero\n");

    if (diffLevel < 0)
    {
        S265_CHECK((absLevel == 1) || (absLevel == 2), "absLevel range check failure\n");

        uint32_t rate = greaterOneBits[(absLevel == 2)];
        if (absLevel == 2)
            rate += levelAbsBits[0];
        return rate;
    }
    else
    {
        uint32_t rate;
        uint32_t symbol = diffLevel;
        if ((symbol >> absGoRice) < COEF_REMAIN_BIN_REDUCTION)
        {
            uint32_t length = symbol >> absGoRice;
            rate = (length + 1 + absGoRice) << 15;
        }
        else
        {
            uint32_t length = 0;
            symbol = (symbol >> absGoRice) - COEF_REMAIN_BIN_REDUCTION;
            if (symbol)
            {
                unsigned long idx;
                CLZ(idx, symbol + 1);
                length = idx;
            }

            rate = (COEF_REMAIN_BIN_REDUCTION + length + absGoRice + 1 + length) << 15;
        }
        rate += c1c2Rate;
        return rate;
    }
}

}

Quant::rdoQuant_t Quant::rdoQuant_func[NUM_CU_DEPTH] = {&Quant::rdoQuant<2>, &Quant::rdoQuant<3>, &Quant::rdoQuant<4>, &Quant::rdoQuant<5>};

Quant::Quant()
{
    m_resiDctCoeff = NULL;
    m_fencDctCoeff = NULL;
    m_fencShortBuf = NULL;
    m_frameNr      = NULL;
    m_nr           = NULL;
}

bool Quant::init(double psyScale, const ScalingList& scalingList, Entropy& entropy)
{
    m_entropyCoder = &entropy;// 初始化熵编码器
    m_psyRdoqScale = (int32_t)(psyScale * 256.0);// 针对RDOQ的心理视觉优化
    S265_CHECK((psyScale * 256.0) < (double)MAX_INT, "psyScale value too large\n");
    m_scalingList  = &scalingList;
    m_resiDctCoeff = S265_MALLOC(int16_t, MAX_TR_SIZE * MAX_TR_SIZE * 2);
    m_fencDctCoeff = m_resiDctCoeff + (MAX_TR_SIZE * MAX_TR_SIZE);
    m_fencShortBuf = S265_MALLOC(int16_t, MAX_TR_SIZE * MAX_TR_SIZE);

    return m_resiDctCoeff && m_fencShortBuf;
}

bool Quant::allocNoiseReduction(const s265_param& param)
{
    m_frameNr = S265_MALLOC(NoiseReduction, param.frameNumThreads);
    if (m_frameNr)
        memset(m_frameNr, 0, sizeof(NoiseReduction) * param.frameNumThreads);
    else
        return false;
    return true;
}

Quant::~Quant()
{
    S265_FREE(m_frameNr);
    S265_FREE(m_resiDctCoeff);
    S265_FREE(m_fencShortBuf);
}

void Quant::setQPforQuant(const CUData& ctu, int qp)
{
    m_nr = m_frameNr ? &m_frameNr[ctu.m_encData->m_frameEncoderID] : NULL;
    m_qpParam[TEXT_LUMA].setQpParam(qp + QP_BD_OFFSET);
    m_rdoqLevel = ctu.m_encData->m_param->rdoqLevel;
    if (ctu.m_chromaFormat != S265_CSP_I400)
    {
        setChromaQP(qp + ctu.m_slice->m_pps->chromaQpOffset[0] + ctu.m_slice->m_chromaQpOffset[0], TEXT_CHROMA_U, ctu.m_chromaFormat);
        setChromaQP(qp + ctu.m_slice->m_pps->chromaQpOffset[1] + ctu.m_slice->m_chromaQpOffset[1], TEXT_CHROMA_V, ctu.m_chromaFormat);
    }
}

void Quant::setChromaQP(int qpin, TextType ttype, int chFmt)
{
    int qp = s265_clip3(-QP_BD_OFFSET, 57, qpin);
    if (qp >= 30)
    {
        if (chFmt == S265_CSP_I420)
            qp = g_chromaScale[qp];
        else
            qp = S265_MIN(qp, QP_MAX_SPEC);
    }
    m_qpParam[ttype].setQpParam(qp + QP_BD_OFFSET);
}

/* To minimize the distortion only. No rate is considered */
uint32_t Quant::signBitHidingHDQ(int16_t* coeff, int32_t* deltaU, uint32_t numSig, const TUEntropyCodingParameters &codeParams, uint32_t log2TrSize)
{
    uint32_t trSize = 1 << log2TrSize;
    const uint16_t* scan = codeParams.scan;

    uint8_t coeffNum[MLS_GRP_NUM];      // value range[0, 16]，每个cg 有多少个非零系数
    uint16_t coeffSign[MLS_GRP_NUM];    // bit mask map for non-zero coeff sign 每个cg 中对应各个非零系数的符号
    uint16_t coeffFlag[MLS_GRP_NUM];    // bit mask map for non-zero coeff 每个cg 中对应哪些位置的系数非零，

#if CHECKED_BUILD || _DEBUG
    // clean output buffer, the asm version of scanPosLast Never output anything after latest non-zero coeff group
    memset(coeffNum, 0, sizeof(coeffNum));
    memset(coeffSign, 0, sizeof(coeffNum));
    memset(coeffFlag, 0, sizeof(coeffNum));
#endif
    // 从dc位置开始扫描，返回扫描到的最后一个非零系数的位置 
    const int lastScanPos = primitives.scanPosLast(codeParams.scan, coeff, coeffSign, coeffFlag, coeffNum, numSig, g_scan4x4[codeParams.scanType], trSize);
    const int cgLastScanPos = (lastScanPos >> LOG2_SCAN_SET_SIZE);//  除16 得倒 最后一个非零系数所在 cg（4x4）的位置
    unsigned long tmp;

    // first CG need specially processing
    // 最后一个CG(逆扫描过程中的最后一个CG),由于可能没有扫描完全部的pos，所以coeffFlag 对应的各个bit 任需要左几个（对应几个没有扫描到的pos）bit，以达到各个cg 的coeffFlag 的各个bit可以与pos 对应起来
    const uint32_t correctOffset = 0x0F & (lastScanPos ^ 0xF);
    coeffFlag[cgLastScanPos] <<= correctOffset;

    for (int cg = cgLastScanPos; cg >= 0; cg--)//从最后一个cg 开始遍历
    {
        int cgStartPos = cg << LOG2_SCAN_SET_SIZE;// x16 得倒 对应的系数的起始位置
        int n;

#if CHECKED_BUILD || _DEBUG
        for (n = SCAN_SET_SIZE - 1; n >= 0; --n)
            if (coeff[scan[n + cgStartPos]])
                break;
        int lastNZPosInCG0 = n;// 只在debug下打开，用于重新寻找一个cg 里面的最后一个非零系数
#endif

        if (coeffNum[cg] == 0)//当前cg 无非零系数
        {
            S265_CHECK(lastNZPosInCG0 < 0, "all zero block check failure\n");
            continue;
        }

#if CHECKED_BUILD || _DEBUG
        for (n = 0;; n++)
            if (coeff[scan[n + cgStartPos]])// 只在debug下打开，用于重新寻找一个cg 里面的第一一个非零系数
                break;

        int firstNZPosInCG0 = n;
#endif

        CLZ(tmp, coeffFlag[cg]);
        const int firstNZPosInCG = (15 ^ tmp);//cg 中第一个非零系数位置

        CTZ(tmp, coeffFlag[cg]);
        const int lastNZPosInCG = (15 ^ tmp);// cg 中最后一个非零系数位置

        S265_CHECK(firstNZPosInCG0 == firstNZPosInCG, "firstNZPosInCG0 check failure\n");
        S265_CHECK(lastNZPosInCG0 == lastNZPosInCG, "lastNZPosInCG0 check failure\n");

        if (lastNZPosInCG - firstNZPosInCG >= SBH_THRESHOLD)// 如果最后一个非零系数的位置与第一个非零系数的位置相隔4 以上 
        {
            uint32_t signbit = coeff[scan[cgStartPos + firstNZPosInCG]] > 0 ? 0 : 1;//获取cg里面对应的第一系数的符号（也就是cg内最后被编码的系数）
            uint32_t absSum = 0;

            for (n = firstNZPosInCG; n <= lastNZPosInCG; n++)//获取整个cg内所有非零系数的和
                absSum += coeff[scan[n + cgStartPos]];
// 对标注规定，如果cg内所有非零系数的和是偶数，则最后一个非零系数的符号位为 0；正
// 否则，如果是奇数，则最后一个非零系数的符号位为 1；负
            //如果最后被编码的系数的符号与cg内所有系数的和的奇偶性不满足要求
            if (signbit != (absSum & 0x1)) // compare signbit with sum_parity
            {
                int minCostInc = MAX_INT,  minPos = -1, curCost = MAX_INT;
                int32_t finalChange = 0, curChange = 0;
                uint32_t cgFlags = coeffFlag[cg];//获取当前cg 内的非零系数位置，每个bit 对应一个位置
                if (cg == cgLastScanPos)// 只有最后一个cg 的扫描可能没有扫描完全部就退出了，所以cgFlags需要丢弃对应没有扫描到的那些pos对应的bit，从而与lastNZPosInCG 对应起来
                    cgFlags >>= correctOffset;

                for (n = (cg == cgLastScanPos ? lastNZPosInCG : SCAN_SET_SIZE - 1); n >= 0; --n)//从cg内最后的非零系数位置往前扫描
                {
                    uint32_t blkPos = scan[n + cgStartPos];
                    S265_CHECK(!!coeff[blkPos] == !!(cgFlags & 1), "non zero coeff check failure\n");

                    if (cgFlags & 1)// 当前blkpos 对应的cgFlags bit 位为1，即: 有非零系数
                    {
                        if (deltaU[blkPos] > 0)// 如果量化前的系数- 反量化回来的系数 >0
                        {
                            curCost = -deltaU[blkPos];// curcost 取负值
                            curChange = 1;// 因为反量化回来的系数小于量化前的系数，故而 对量化后的系数加1
                        }
                        else //否则 如果量化前的系数- 反量化后的系数 < 0
                        {
                            if ((cgFlags == 1) && (abs(coeff[blkPos]) == 1))
                            {// 遇到了逆扫描过程中的最后一个非零系数 (cg里面的第一个非零系数) 且其绝对值为 1
                                S265_CHECK(n == firstNZPosInCG, "firstNZPosInCG position check failure\n");
                                curCost = MAX_INT;//此位置不调整
                            }
                            else
                            {
                                curCost = deltaU[blkPos];// curcost 取负值
                                curChange = -1;
                            }
                        }
                    }
                    else // 当前blkpos 无非零系数,对应deltaU[blkPos] 一定 > 0
                    {
                        if (cgFlags == 0)
                        {   // 所有系数都已经扫描完成
                            S265_CHECK(n < firstNZPosInCG, "firstNZPosInCG position check failure\n");
                            uint32_t thisSignBit = m_resiDctCoeff[blkPos] >= 0 ? 0 : 1;//当逆扫描所有的非零系数都已经完成，直到扫描到0level系数，则判断对应位置的dct系数 并取其符号
                            if (thisSignBit != signbit)//如果这两的符号位不一致
                                curCost = MAX_INT;
                            else
                            {
                                curCost = -deltaU[blkPos];// curcost 取负值 - （m_resiDctCoeff[blkPos]-0）
                                curChange = 1;
                            }
                        }
                        else //前面还有非零系数
                        {
                            curCost = -deltaU[blkPos];
                            curChange = 1;
                        }
                    }
// 原理：对于每一个cg，最后一个非零系数的符号位，如果和所有非零系数的和不满足极性要求
// 则寻找当前cg 内量化失真最大位置的系数，对其量化后的系数调整 +1(反量化回来的系数 绝对值小于量化前的系数的绝对值 )
// or -1（反量化回来的系数 绝对值大于量化前的系数的绝对值）;
                    if (curCost < minCostInc)// 注意 minCostinC 对应失真最大的量化失真
                    {
                        minCostInc = curCost;//保持量化矢真最大的pos 已经最后的change 信息
                        finalChange = curChange;
                        minPos = blkPos;
                    }
                    cgFlags>>=1;
                }

                /* do not allow change to violate coeff clamp */
                // 量化后的系数 由 level 与符号一起组成
                // 如果量化后的系数取到了边界处，则 符号不变，abs（level） -1;
                if (coeff[minPos] == 32767 || coeff[minPos] == -32768)
                    finalChange = -1;

                if (!coeff[minPos])// 如果当前需调整的位置原来的量化后系数为0, 调整后 abs level 为 1 不在为0，则需要将非零系数+1
                    numSig++;
                else if (finalChange == -1 && abs(coeff[minPos]) == 1)//如果当前需要调整的位置的量化后系数的绝对值为1，并且调整值为 -1 ，则调整后 变成0,此时，非零系数需要 -1 
                    numSig--;

                {
                    const int16_t sigMask = ((int16_t)m_resiDctCoeff[minPos]) >> 15;// 原系数为正，全0，否则全1
                    coeff[minPos] += ((int16_t)finalChange ^ sigMask) - sigMask;
                }
// if (m_resiDctCoeff[minPos] >= 0)
//{
//    coeff[minPos] += finalChange; // 加1 or 加-1;
//}
//else
//{
//   if( finalChange < 0)//需要将abslevel 往0方向调整
//    {
//        //coeff[minPos] += 1;
//        coeff[minPos] -=finalChange
//    }
//    else// 需要将 abslevel 往反0方向调整
//    {
//        //coeff[minPos] -= 1;
//        coeff[minPos] -=finalChange
//    }
//}
            }
        }
    }

    return numSig;
}

/** 函数功能       ： 对残差块进行变换、量化
* \参数 cu         ：CUData对象
* \参数 fenc       ：原始图像
* \参数 fencStride ：原始图像块的stride
* \参数 residual   ：残差数据
* \参数 resiStride ：残差数据的stride
* \参数 coeff      ：存储残差经过变换、量化后的系数
* \参数 log2TrSize ：TU尺寸
* \参数 ttype      ：数据分量类型（亮度/色度）
* \参数 absPartIdx ：CU地址
* \参数 useTransformSkip ：是否使用变换跳过模式
* \返回            ：量化后非零系数的个数
**/
uint32_t Quant::transformNxN(const CUData& cu, const pixel* fenc, uint32_t fencStride, const int16_t* residual, uint32_t resiStride,
                             coeff_t* coeff, uint32_t log2TrSize, TextType ttype, uint32_t absPartIdx, bool useTransformSkip)
{
    const uint32_t sizeIdx = log2TrSize - 2;

    if (cu.m_tqBypass[0])
    {// 如果使用 变换/量化的bypass模式，即跳过变换/量化，则直接将残差块拷贝到变换系数块
        S265_CHECK(log2TrSize >= 2 && log2TrSize <= 5, "Block size mistake!\n");
        return primitives.cu[sizeIdx].copy_cnt(coeff, residual, resiStride);// 拷贝残差块到变换系数块coeff，返回非零系数的个数
    }

    bool isLuma  = ttype == TEXT_LUMA;
    bool usePsy  = m_psyRdoqScale && isLuma && !useTransformSkip;
    int transformShift = MAX_TR_DYNAMIC_RANGE - S265_DEPTH - log2TrSize; // Represents scaling through forward transform

    S265_CHECK((cu.m_slice->m_sps->quadtreeTULog2MaxSize >= log2TrSize), "transform size too large\n");
    if (useTransformSkip)// 如果应用"跳过变换"模式，则只需将残差进行相应的移位，无需进行其他操作
    {
#if S265_DEPTH <= 10
        S265_CHECK(transformShift >= 0, "invalid transformShift\n");
        primitives.cu[sizeIdx].cpy2Dto1D_shl(m_resiDctCoeff, residual, resiStride, transformShift);// 将残差数据进行左移操作然后写入m_resiDctCoeff
#else
        if (transformShift >= 0)
            primitives.cu[sizeIdx].cpy2Dto1D_shl(m_resiDctCoeff, residual, resiStride, transformShift);
        else
            primitives.cu[sizeIdx].cpy2Dto1D_shr(m_resiDctCoeff, residual, resiStride, -transformShift);
#endif
    }
    else// 进行常规变换
    {
        bool isIntra = cu.isIntra(absPartIdx);

        if (!sizeIdx && isLuma && isIntra) //sizeIdx 为 0 表示的是4x4  如果变换块是4x4(sizeIdx=0)，且是亮度块、intra预测模式，则使用4x4的dst变换
            primitives.dst4x4(residual, m_resiDctCoeff, resiStride);// 进行dst变换后数据结果存放在m_resiDctCoeff
        else // 其他size 对residual进行dct 结果写入m_resiDctCoeff
            primitives.cu[sizeIdx].dct(residual, m_resiDctCoeff, resiStride); //否则dct 变换

        /* NOTE: if RDOQ is disabled globally, psy-rdoq is also disabled, so
         * there is no risk of performing this DCT unnecessarily */
        if (usePsy)
        {
            int trSize = 1 << log2TrSize;
            /* perform DCT on source pixels for psy-rdoq */
            primitives.cu[sizeIdx].copy_ps(m_fencShortBuf, trSize, fenc, fencStride);// 将fenc pix数据 拷贝到 m_fencShortBuf 8bit 数据到16bit数据
            primitives.cu[sizeIdx].dct(m_fencShortBuf, m_fencDctCoeff, trSize);// 对fenc org数据进行dct 结果存放到m_fencDctCoeff
        }

        if (m_nr && m_nr->offset)
        {
            /* denoise is not applied to intra residual, so DST can be ignored */
            int cat = sizeIdx + 4 * !isLuma + 8 * !isIntra;
            int numCoeff = 1 << (log2TrSize * 2);
            primitives.denoiseDct(m_resiDctCoeff, m_nr->residualSum[cat], m_nr->offset[cat], numCoeff);
            m_nr->count[cat]++;
        }
    }
     // 以上完成dct变换 dct 后的结果放在 m_resiDctCoeff

    if (m_rdoqLevel)// 如果RDOQ的级别大于0，才进行RDOQ量化，否则使用常规（均匀）量化
        return (this->*rdoQuant_func[log2TrSize - 2])(cu, coeff, ttype, absPartIdx, usePsy);
    else
    {
        int deltaU[32 * 32];// 用于存储量化误差矩阵，在常规量化中，deltaU只用于进行符号位隐藏的操作

        int scalingListType = (cu.isIntra(absPartIdx) ? 0 : 3) + ttype;// 根据预测模式和亮度/色度分量得到 前向量化表的类型，用于选择不同的前向量化表
        int rem = m_qpParam[ttype].rem;// qp%6
        int per = m_qpParam[ttype].per;// qp/6
        const int32_t* quantCoeff = m_scalingList->m_quantCoef[log2TrSize - 2][scalingListType][rem];// 根据TU的尺寸、前向量化类型和Qp余数部分，选择对应的量化表

        int qbits = QUANT_SHIFT + per + transformShift;// 量化右移的位数，由3部分组成：1.量化带来的位数增加 2.Qp/6部分带来的位数增加 3.前向变换所带来的位数增加
        int add = (cu.m_slice->m_sliceType == I_SLICE ? 171 : 85) << (qbits - 9);// 量化后右移可能会带来低位上的损失，这里对右移可能带来的损失进行补偿，HEVC规定I_SLICE补偿1/3，其他类型SLICE补偿1/6
                                                                                 // 结合量化公式，I_SLICE中的add实际相当于： add >> qbits = (171 << (qbits-9))>>qbits = 171>>9 = 171/512 = 1/3
                                                                                 // 非I_SLICE中add实际相当于： add >> qbits = (85 << (qbits-9))>>qbits = 85>>9 = 85/512 = 1/6
        int numCoeff = 1 << (log2TrSize * 2);// 总共有多少个coefficients
        // 使用量化表quantCoeff 对dct变换后的系数m_resiDctCoeff进行 量化，量化结果level写入coeff(带符号)，返回量化后的非零系数的个数
        uint32_t numSig = primitives.quant(m_resiDctCoeff, quantCoeff, deltaU, coeff, qbits, add, numCoeff);// 进行常规量化，参看C版本函数 quant_c，返回值为量化后非零系数的个数

        if (numSig >= 2 && cu.m_slice->m_pps->bSignHideEnabled)// 假如非零系数的个数大于等于2，并且使能符号位隐藏，则进行符号位隐藏的操作
        {
            TUEntropyCodingParameters codeParams;
            cu.getTUEntropyCodingParameters(codeParams, absPartIdx, log2TrSize, isLuma);
            return signBitHidingHDQ(coeff, deltaU, numSig, codeParams, log2TrSize);
        }
        else
            return numSig;
    }
}

uint64_t Quant::ssimDistortion(const CUData& cu, const pixel* fenc, uint32_t fStride, const pixel* recon, intptr_t rstride, uint32_t log2TrSize, TextType ttype, uint32_t absPartIdx)
{
    static const int ssim_c1 = (int)(.01 * .01 * PIXEL_MAX * PIXEL_MAX * 64 + .5); // 416
    static const int ssim_c2 = (int)(.03 * .03 * PIXEL_MAX * PIXEL_MAX * 64 * 63 + .5); // 235963
    int shift = (S265_DEPTH - 8);

    int trSize = 1 << log2TrSize;
    uint64_t ssDc = 0, ssBlock = 0, ssAc = 0;

    // Calculation of (X(0) - Y(0)) * (X(0) - Y(0)), DC
    ssDc = 0;
    for (int y = 0; y < trSize; y += 4)
    {
        for (int x = 0; x < trSize; x += 4)
        {
            int temp = fenc[y * fStride + x] - recon[y * rstride + x]; // copy of residual coeff
            ssDc += temp * temp;
        }
    }

    // Calculation of (X(k) - Y(k)) * (X(k) - Y(k)), AC
    ssBlock = 0;
    uint64_t ac_k = 0;
    primitives.cu[log2TrSize - 2].ssimDist(fenc, fStride, recon, rstride, &ssBlock, shift, &ac_k);
    ssAc = ssBlock - ssDc;

    // 1. Calculation of fdc'
    // Calculate numerator of dc normalization factor
    uint64_t fDc_num = 0;

    // 2. Calculate dc component
    uint64_t dc_k = 0;
    for (int block_yy = 0; block_yy < trSize; block_yy += 4)
    {
        for (int block_xx = 0; block_xx < trSize; block_xx += 4)
        {
            uint32_t temp = fenc[block_yy * fStride + block_xx] >> shift;
            dc_k += temp * temp;
        }
    }

    fDc_num = (2 * dc_k)  + (trSize * trSize * ssim_c1); // 16 pixels -> for each 4x4 block
    fDc_num /= ((trSize >> 2) * (trSize >> 2));

    // 1. Calculation of fac'
    // Calculate numerator of ac normalization factor
    uint64_t fAc_num = 0;

    // 2. Calculate ac component
    ac_k -= dc_k;

    double s = 1 + 0.005 * cu.m_qp[absPartIdx];

    fAc_num = ac_k + uint64_t(s * ac_k) + ssim_c2;
    fAc_num /= ((trSize >> 2) * (trSize >> 2));

    // Calculate dc and ac normalization factor
    uint64_t ssim_distortion = ((ssDc * cu.m_fDc_den[ttype]) / fDc_num) + ((ssAc * cu.m_fAc_den[ttype]) / fAc_num);
    return ssim_distortion;
}
// 反量化反变换
void Quant::invtransformNxN(const CUData& cu, int16_t* residual, uint32_t resiStride, const coeff_t* coeff,
                            uint32_t log2TrSize, TextType ttype, bool bIntra, bool useTransformSkip, uint32_t numSig)
{
    const uint32_t sizeIdx = log2TrSize - 2;
    if (cu.m_tqBypass[0])
    {   //将coeff数据一个一个写入residual（会覆盖原有数据）
        primitives.cu[sizeIdx].cpy1Dto2D_shl[resiStride % 64 == 0](residual, coeff, resiStride, 0);
        return;
    }
    // Values need to pass as input parameter in dequant
    int rem = m_qpParam[ttype].rem;
    int per = m_qpParam[ttype].per;
    int transformShift = MAX_TR_DYNAMIC_RANGE - S265_DEPTH - log2TrSize;
    int shift = QUANT_IQUANT_SHIFT - QUANT_SHIFT - transformShift;
    int numCoeff = 1 << (log2TrSize * 2);

    if (m_scalingList->m_bEnabled)
    {
        int scalingListType = (bIntra ? 0 : 3) + ttype;
        const int32_t* dequantCoef = m_scalingList->m_dequantCoef[sizeIdx][scalingListType][rem];
        primitives.dequant_scaling(coeff, dequantCoef, m_resiDctCoeff, numCoeff, per, shift);
    }
    else
    {
        int scale = m_scalingList->s_invQuantScales[rem] << per;
        //反量化过程 将coeff 反量化后写入 m_resiDctCoeff
        primitives.dequant_normal(coeff, m_resiDctCoeff, numCoeff, scale, shift);
    }

    if (useTransformSkip)
    {
#if S265_DEPTH <= 10
        S265_CHECK(transformShift > 0, "invalid transformShift\n");
        primitives.cu[sizeIdx].cpy1Dto2D_shr(residual, m_resiDctCoeff, resiStride, transformShift);
#else
        if (transformShift > 0)
            primitives.cu[sizeIdx].cpy1Dto2D_shr(residual, m_resiDctCoeff, resiStride, transformShift);
        else
            primitives.cu[sizeIdx].cpy1Dto2D_shl[resiStride % 64 == 0](residual, m_resiDctCoeff, resiStride, -transformShift);
#endif
    }
    else
    {
        int useDST = !sizeIdx && ttype == TEXT_LUMA && bIntra;
        S265_CHECK((int)numSig == primitives.cu[log2TrSize - 2].count_nonzero(coeff), "numSig differ\n");
        // DC only
        if (numSig == 1 && coeff[0] != 0 && !useDST)
        {
            const int shift_1st = 7 - 6;
            const int add_1st = 1 << (shift_1st - 1);
            const int shift_2nd = 12 - (S265_DEPTH - 8) - 3;
            const int add_2nd = 1 << (shift_2nd - 1);

            int dc_val = (((m_resiDctCoeff[0] * (64 >> 6) + add_1st) >> shift_1st) * (64 >> 3) + add_2nd) >> shift_2nd;
            primitives.cu[sizeIdx].blockfill_s[resiStride % 64 == 0](residual, resiStride, (int16_t)dc_val);
            return;
        }

        if (useDST)
            primitives.idst4x4(m_resiDctCoeff, residual, resiStride);
        else // 逆dct变换 反量化后的结果反dct变换后写入 residual
            primitives.cu[sizeIdx].idct(m_resiDctCoeff, residual, resiStride);
    }
}


/** 函数功能       ： 使用RDO（率失真优化）技术对变换后的系数进行量化
 **                  RDOQ主要可以分成三步：
 **                      step1. 对每个系数单独做RDO优化，找到率失真意义上的最优量化值
 **                      step2. 对每一个系数组（Coefficient Group，下面都缩写为CG）进行优化，试图将整个CG都设置为0
 **                      step3. 找到最优的最后一个非零系数的位置，尝试从最后一个非零位置开始将量化后的系数设置为0
 **  调用范围       ：只在Quant::transformNxN函数中被调用
* \参数 cu         ：CUData对象
* \参数 dstCoeff   ：存放RDOQ量化后的系数（变换后的系数存储在m_resiDctCoeff中）
* \参数 log2TrSize ：TU尺寸
* \参数 ttype      ：数据分量类型（亮度/色度）
* \参数 absPartIdx ：CU地址
* \参数 usePsy     ：是否使用心理视觉量化
* \返回            ：非零系数的个数
**/
/* Rate distortion optimized quantization for entropy coding engines using
 * probability models like CABAC */
template<uint32_t log2TrSize>
uint32_t Quant::rdoQuant(const CUData& cu, int16_t* dstCoeff, TextType ttype, uint32_t absPartIdx, bool usePsy)
{
    const int transformShift = MAX_TR_DYNAMIC_RANGE - S265_DEPTH - log2TrSize; // 前变换的需要的右移位数，需要在量化中完成 /* Represents scaling through forward transform */
    int scalingListType = (cu.isIntra(absPartIdx) ? 0 : 3) + ttype;// 根据预测类型(Intra/Inter)和当前分量(Y/U/V)判断使用的扫描类型
    const uint32_t usePsyMask = usePsy ? -1 : 0;// 是否使用心理视觉量化

    S265_CHECK(scalingListType < 6, "scaling list type out of range\n");//scalingListType 最大为5

    int rem = m_qpParam[ttype].rem; // 得到Qp的余数部分，=Qp%6
    int per = m_qpParam[ttype].per; // 得到Qp的整数部分，=Qp/6
    int qbits = QUANT_SHIFT + per + transformShift; // 常规量化中需要右移的位数 /* Right shift of non-RDOQ quantizer level = (coeff*Q + offset)>>q_bits */
    int add = (1 << (qbits - 1));// 常规量化中右移前需要补偿的加数
    const int32_t* qCoef = m_scalingList->m_quantCoef[log2TrSize - 2][scalingListType][rem]; // 得到常规量化使用的量化乘数

    const int numCoeff = 1 << (log2TrSize * 2); // 当前TU中系数的个数
    // 注意: 这里的 m_resiDctCoeff qCoef dstCoeff 存储方式一致 采用1为光栅扫描的方式
    uint32_t numSig = primitives.nquant(m_resiDctCoeff, qCoef, dstCoeff, qbits, add, numCoeff);// 对变换系数进行常规量化，参考C语言版本的函数 nquant_c
    S265_CHECK((int)numSig == primitives.cu[log2TrSize - 2].count_nonzero(dstCoeff), "numSig differ\n");// 再次统计量化后的非零系数个数，并判断与常规量化的结果是否一致
    if (!numSig)// 如果常规量化后系数为全零，则跳过RDOQ过程（这是RDOQ提前终止的快速算法）
        return 0;
    const uint32_t trSize = 1 << log2TrSize;// 得到TU大小
    int64_t lambda2 = m_qpParam[ttype].lambda2; // 得到RDO中的lambda
    int64_t psyScale = ((int64_t)m_psyRdoqScale * m_qpParam[ttype].lambda);// 得到心理视觉量化系数
    /* unquant constants for measuring distortion. Scaling list quant coefficients have a (1 << 4)
     * scale applied that must be removed during unquant. Note that in real dequant there is clipping
     * at several stages. We skip the clipping for simplicity when measuring RD cost */
    const int32_t* unquantScale = m_scalingList->m_dequantCoef[log2TrSize - 2][scalingListType][rem];// 得到反量化乘数
    int unquantShift = QUANT_IQUANT_SHIFT - QUANT_SHIFT - transformShift + (m_scalingList->m_bEnabled ? 4 : 0);// 得到反量化右移位数
    int unquantRound = (unquantShift > per) ? 1 << (unquantShift - per - 1) : 0;// 反量化右移时补偿加数
    const int scaleBits = SCALE_BITS - 2 * transformShift;

#define UNQUANT(lvl)    (((lvl) * (unquantScale[blkPos] << per) + unquantRound) >> unquantShift)
#define SIGCOST(bits)   ((lambda2 * (bits)) >> 8)
#define RDCOST(d, bits) ((((int64_t)d * d) << scaleBits) + SIGCOST(bits))
#define PSYVALUE(rec)   ((psyScale * (rec)) >> S265_MAX(0, (2 * transformShift + 1)))
    // 以下是系数级别的变量，即每个系数都占用一个存储单元
    int64_t costCoeff[trSize * trSize];   // 每一个系数花费  /* d*d + lambda * bits */
    int64_t costUncoded[trSize * trSize]; // 每一个系数被量化为0的花费（Z型顺序存储 /* d*d + lambda * 0    */
    int64_t costSig[trSize * trSize];     // 每一个系数的是否为0标记(sig_coeff_flag)的花费 /* lambda * bits       */

    int rateIncUp[trSize * trSize];      /* signal overhead of increasing level */
    int rateIncDown[trSize * trSize];    /* signal overhead of decreasing level */
    int sigRateDelta[trSize * trSize];   // 将系数量化为0和量化为非0，系数标记（sig_coeff_flag）的花费差异 /* signal difference between zero and non-zero */
    // 以下是系数组（CG）级别的变量
    int64_t costCoeffGroupSig[MLS_GRP_NUM];  // 每一个系数组CG的花费 //* lambda * bits of group coding cost */
    uint64_t sigCoeffGroupFlag64 = 0; // CG的非零标记，每一位代表一个CG，某一位为1代表对应的CG不是全0，反之代表对应的CG为全0

    const uint32_t cgSize = (1 << MLS_CG_SIZE); // 一个系数组CG中的系数个数 /* 4x4 num coef = 16 *//* 4x4 num coef = 16 */
    bool bIsLuma = ttype == TEXT_LUMA;// 当前是否是亮度分量

    /* total rate distortion cost of transform block, as CBF=0 */
    int64_t totalUncodedCost = 0;// 当前块都被量化为0时的cost

    /* Total rate distortion cost of this transform block, counting te distortion of uncoded blocks,
     * the distortion and signal cost of coded blocks, and the coding cost of significant
     * coefficient and coefficient group bitmaps */
    int64_t totalRdCost = 0;

    TUEntropyCodingParameters codeParams;
    cu.getTUEntropyCodingParameters(codeParams, absPartIdx, log2TrSize, bIsLuma);// 得到熵编码参数
    const uint32_t log2TrSizeCG = log2TrSize - 2;
    const uint32_t cgNum = 1 << (log2TrSizeCG * 2); // TU中的系数组CG的个数
    const uint32_t cgStride = (trSize >> MLS_CG_LOG2_SIZE);// TU中CG的stride

    uint8_t coeffNum[MLS_GRP_NUM];      // 每个CG中的非零系数的个数 // value range[0, 16]
    uint16_t coeffSign[MLS_GRP_NUM];    // 每个CG中的非零系数的符号 // bit mask map for non-zero coeff sign
    uint16_t coeffFlag[MLS_GRP_NUM];    // 每个CG中的非零系数的标记 // bit mask map for non-zero coeff

#if CHECKED_BUILD || _DEBUG
    // clean output buffer, the asm version of scanPosLast Never output anything after latest non-zero coeff group
    memset(coeffNum, 0, sizeof(coeffNum));
    memset(coeffSign, 0, sizeof(coeffNum));// 这里size应该是写错了，应该改为 sizeof(coeffSign)，下面也是一样
    memset(coeffFlag, 0, sizeof(coeffNum));
#endif
    // 统计每个CG中的非零系数的符号、非零系数的标志（是否是非零系数）、非零系数个数，以及最后一个非零系数的扫描位置（lastScanPos）
    const int lastScanPos = primitives.scanPosLast(codeParams.scan, dstCoeff, coeffSign, coeffFlag, coeffNum, numSig, g_scan4x4[codeParams.scanType], trSize);// 参看 scanPosLast_c
    const int cgLastScanPos = (lastScanPos >> LOG2_SCAN_SET_SIZE);// 得到最后一个非零系数的扫描位置（lastScanPos）所对应的系数组CG的位置
                                                                  // 但是如果 lastScanPos%(2^LOG2_SCAN_SET_SIZE) != 0，即如果最后一个非零系数位置并不能被16整除，这里得到的实际上倒数第二个非零CG的位置


    /* TODO: update bit estimates if dirty */
    EstBitsSbac& estBitsSbac = m_entropyCoder->m_estBitsSbac;

    uint32_t scanPos = 0;
    uint32_t c1 = 1;

    // process trail all zero Coeff Group

    /* coefficients after lastNZ have no distortion signal cost */
    const int zeroCG = cgNum - 1 - cgLastScanPos;// 得到全零CG的个数
    memset(&costCoeff[(cgLastScanPos + 1) << MLS_CG_SIZE], 0, zeroCG * MLS_CG_BLK_SIZE * sizeof(int64_t));// 将全零CG中的每个系数花费都设置为0
    memset(&costSig[(cgLastScanPos + 1) << MLS_CG_SIZE], 0, zeroCG * MLS_CG_BLK_SIZE * sizeof(int64_t));// 将全零CG中的每个系数的标记花费设置为0

    /* sum zero coeff (uncodec) cost */

    // TODO: does we need these cost?
    if (usePsyMask)// 如果使用心理视觉量化
    {
        for (int cgScanPos = cgLastScanPos + 1; cgScanPos < (int)cgNum ; cgScanPos++)
        {
            S265_CHECK(coeffNum[cgScanPos] == 0, "count of coeff failure\n");
            uint32_t scanPosBase = (cgScanPos << MLS_CG_SIZE);
            uint32_t blkPos      = codeParams.scan[scanPosBase];
#if S265_ARCH_X86
            bool enable512 = detect512();
            if (enable512)
                primitives.cu[log2TrSize - 2].psyRdoQuant(m_resiDctCoeff, m_fencDctCoeff, costUncoded, &totalUncodedCost, &totalRdCost, &psyScale, blkPos);
            else
            {
                primitives.cu[log2TrSize - 2].psyRdoQuant_1p(m_resiDctCoeff,  costUncoded, &totalUncodedCost, &totalRdCost,blkPos);
                primitives.cu[log2TrSize - 2].psyRdoQuant_2p(m_resiDctCoeff, m_fencDctCoeff, costUncoded, &totalUncodedCost, &totalRdCost, &psyScale, blkPos);
            }
#else
            primitives.cu[log2TrSize - 2].psyRdoQuant_1p(m_resiDctCoeff, costUncoded, &totalUncodedCost, &totalRdCost, blkPos);
            primitives.cu[log2TrSize - 2].psyRdoQuant_2p(m_resiDctCoeff, m_fencDctCoeff, costUncoded, &totalUncodedCost, &totalRdCost, &psyScale, blkPos);
#endif
        }
    }
    else// 不使用心理视觉量化，对量化为全零的CG计算每个系数的cost
    {
        // non-psy path
        for (int cgScanPos = cgLastScanPos + 1; cgScanPos < (int)cgNum ; cgScanPos++)// 遍历每一个全零的CG
        {
            S265_CHECK(coeffNum[cgScanPos] == 0, "count of coeff failure\n");// 再次确认是否该CG中非零系数的个数是0
            uint32_t scanPosBase = (cgScanPos << MLS_CG_SIZE);// 得到每个CG的首地址（这里的CG顺序是Z型扫描，而不是按照选择的scan模式扫描）
            uint32_t blkPos      = codeParams.scan[scanPosBase];// 找到一个CG首地址对应的扫描位置
            primitives.cu[log2TrSize - 2].nonPsyRdoQuant(m_resiDctCoeff, costUncoded, &totalUncodedCost, &totalRdCost, blkPos);
        }
    }
    static const uint8_t table_cnt[5][SCAN_SET_SIZE] =
    {
        // patternSigCtx = 0
        {
            2, 1, 1, 0,
            1, 1, 0, 0,
            1, 0, 0, 0,
            0, 0, 0, 0,
        },
        // patternSigCtx = 1
        {
            2, 2, 2, 2,
            1, 1, 1, 1,
            0, 0, 0, 0,
            0, 0, 0, 0,
        },
        // patternSigCtx = 2
        {
            2, 1, 0, 0,
            2, 1, 0, 0,
            2, 1, 0, 0,
            2, 1, 0, 0,
        },
        // patternSigCtx = 3
        {
            2, 2, 2, 2,
            2, 2, 2, 2,
            2, 2, 2, 2,
            2, 2, 2, 2,
        },
        // 4x4
        {
            0, 1, 4, 5,
            2, 3, 4, 5,
            6, 6, 8, 8,
            7, 7, 8, 8
        }
    };

    /* iterate over coding groups in reverse scan order */
    // step1. 对每个系数单独做RDO优化，找到率失真意义上的最优量化值
    for (int cgScanPos = cgLastScanPos; cgScanPos >= 0; cgScanPos--)// 从最后一非零CG开始遍历每个CG，从最后一个到第一个
    {
        uint32_t ctxSet = (cgScanPos && bIsLuma) ? 2 : 0;
        const uint32_t cgBlkPos = codeParams.scanCG[cgScanPos];// 得到CG的扫描位置
        const uint32_t cgPosY   = cgBlkPos >> log2TrSizeCG;// CG扫描位置的Y坐标
        const uint32_t cgPosX   = cgBlkPos & ((1 << log2TrSizeCG) - 1);// CG扫描位置的X坐标
        const uint64_t cgBlkPosMask = ((uint64_t)1 << cgBlkPos);// 当前CG的位置，使用cgBlkPosMask中1的位置来表示
        const int patternSigCtx = calcPatternSigCtx(sigCoeffGroupFlag64, cgPosX, cgPosY, cgBlkPos, cgStride);
        const int ctxSigOffset = codeParams.firstSignificanceMapContext + (cgScanPos && bIsLuma ? 3 : 0);

        if (c1 == 0)
            ctxSet++;
        c1 = 1;

        if (cgScanPos && (coeffNum[cgScanPos] == 0))// 如果当前CG不是第一个CG，并且CG系数为全零，即在最后一个非全零CG可能还会存在全零的CG
        {                                           // 如果发现全零的CG，处理方法与上文中，最后一个非零CG之后的CG的处理方法相同；但是对这些系数又计算了量化为0和非零时，系数标记的花费，这是为了之后进行step2、step3的优化
            // TODO: does we need zero-coeff cost?
            const uint32_t scanPosBase = (cgScanPos << MLS_CG_SIZE);
            uint32_t blkPos = codeParams.scan[scanPosBase];
            if (usePsyMask)// 如果使用心理视觉量化
            {
#if S265_ARCH_X86
                bool enable512 = detect512();
                if (enable512)
                    primitives.cu[log2TrSize - 2].psyRdoQuant(m_resiDctCoeff, m_fencDctCoeff, costUncoded, &totalUncodedCost, &totalRdCost, &psyScale, blkPos);
                else
                {
                    primitives.cu[log2TrSize - 2].psyRdoQuant_1p(m_resiDctCoeff, costUncoded, &totalUncodedCost, &totalRdCost, blkPos);
                    primitives.cu[log2TrSize - 2].psyRdoQuant_2p(m_resiDctCoeff, m_fencDctCoeff, costUncoded, &totalUncodedCost, &totalRdCost, &psyScale, blkPos);
                }
#else
                primitives.cu[log2TrSize - 2].psyRdoQuant_1p(m_resiDctCoeff, costUncoded, &totalUncodedCost, &totalRdCost, blkPos);
                primitives.cu[log2TrSize - 2].psyRdoQuant_2p(m_resiDctCoeff, m_fencDctCoeff, costUncoded, &totalUncodedCost, &totalRdCost, &psyScale, blkPos);
#endif
                blkPos = codeParams.scan[scanPosBase];
                for (int y = 0; y < MLS_CG_SIZE; y++)
                {
                    for (int x = 0; x < MLS_CG_SIZE; x++)
                    {
                        const uint32_t scanPosOffset =  y * MLS_CG_SIZE + x; // 得到变换系数
                        const uint32_t ctxSig = table_cnt[patternSigCtx][g_scan4x4[codeParams.scanType][scanPosOffset]] + ctxSigOffset;
                        S265_CHECK(trSize > 4, "trSize check failure\n");
                        S265_CHECK(ctxSig == getSigCtxInc(patternSigCtx, log2TrSize, trSize, codeParams.scan[scanPosBase + scanPosOffset], bIsLuma, codeParams.firstSignificanceMapContext), "sigCtx check failure\n");

                        costSig[scanPosBase + scanPosOffset] = SIGCOST(estBitsSbac.significantBits[0][ctxSig]);
                        costCoeff[scanPosBase + scanPosOffset] = costUncoded[blkPos + x];
                        sigRateDelta[blkPos + x] = estBitsSbac.significantBits[1][ctxSig] - estBitsSbac.significantBits[0][ctxSig];
                    }
                    blkPos += trSize;
                }
            }
            else// 不使用心理视觉量化，对量化为全零的CG计算每个系数的cost
            {
                // non-psy path
                primitives.cu[log2TrSize - 2].nonPsyRdoQuant(m_resiDctCoeff, costUncoded, &totalUncodedCost, &totalRdCost, blkPos);
                blkPos = codeParams.scan[scanPosBase];
                for (int y = 0; y < MLS_CG_SIZE; y++)// 对整个CG进行遍历
                {
                    for (int x = 0; x < MLS_CG_SIZE; x++)
                    {
                        const uint32_t scanPosOffset =  y * MLS_CG_SIZE + x;
                        const uint32_t ctxSig = table_cnt[patternSigCtx][g_scan4x4[codeParams.scanType][scanPosOffset]] + ctxSigOffset;
                        S265_CHECK(trSize > 4, "trSize check failure\n");
                        S265_CHECK(ctxSig == getSigCtxInc(patternSigCtx, log2TrSize, trSize, codeParams.scan[scanPosBase + scanPosOffset], bIsLuma, codeParams.firstSignificanceMapContext), "sigCtx check failure\n");

                        costSig[scanPosBase + scanPosOffset] = SIGCOST(estBitsSbac.significantBits[0][ctxSig]);// 计算系数标记的cost，实际上全零的CG没有这部分的花费
                        costCoeff[scanPosBase + scanPosOffset] = costUncoded[blkPos + x]; // 对于全零的CG，每个系数的花费就是量化为0的distortion部分
                        sigRateDelta[blkPos + x] = estBitsSbac.significantBits[1][ctxSig] - estBitsSbac.significantBits[0][ctxSig];// 计算系数被量化为0和非0，系数标记的bit花费差异
                    }
                    blkPos += trSize;
                }
            }

            /* there were no coded coefficients in this coefficient group */
            {
                uint32_t ctxSig = getSigCoeffGroupCtxInc(sigCoeffGroupFlag64, cgPosX, cgPosY, cgBlkPos, cgStride);
                costCoeffGroupSig[cgScanPos] = SIGCOST(estBitsSbac.significantCoeffGroupBits[ctxSig][0]);// 得到CG的非零标记的cos
                totalRdCost += costCoeffGroupSig[cgScanPos];  /* add cost of 0 bit in significant CG bitmap */
            }
            continue;
        }

        coeffGroupRDStats cgRdStats;
        memset(&cgRdStats, 0, sizeof(coeffGroupRDStats));

        uint32_t subFlagMask = coeffFlag[cgScanPos];// 得到一个CG内系数的标记
        int    c2            = 0;
        uint32_t goRiceParam = 0;
        uint32_t levelThreshold = 3;
        uint32_t c1Idx       = 0;
        uint32_t c2Idx       = 0;
        /* iterate over coefficients in each group in reverse scan order */
        for (int scanPosinCG = cgSize - 1; scanPosinCG >= 0; scanPosinCG--)// 扫描CG内部的每一个系数
        {
            scanPos              = (cgScanPos << MLS_CG_SIZE) + scanPosinCG;// 找到每个系数的Z型扫描顺序
            uint32_t blkPos      = codeParams.scan[scanPos];// 找到对应的扫描位置
            uint32_t maxAbsLevel = dstCoeff[blkPos];// 得到常规量化后的值      /* abs(quantized coeff) */
            int signCoef         = m_resiDctCoeff[blkPos];// 得到DCT系数            /* pre-quantization DCT coeff */
            int predictedCoef    = m_fencDctCoeff[blkPos] - signCoef;   /* predicted DCT = source DCT - residual DCT*/

            /* RDOQ measures distortion as the squared difference between the unquantized coded level
             * and the original DCT coefficient. The result is shifted scaleBits to account for the
             * FIX15 nature of the CABAC cost tables minus the forward transform scale */

            /* cost of not coding this coefficient (all distortion, no signal bits) */
            costUncoded[blkPos] = ((int64_t)signCoef * signCoef) << scaleBits;// 得到量化为0的cost
            S265_CHECK((!!scanPos ^ !!blkPos) == 0, "failed on (blkPos=0 && scanPos!=0)\n");// ^表示按位异或，若参加运算的两个二进制位值相同则为0，否则为1。这里是保证scanPos和blkPos同时为0或者同时不为0
            if (usePsyMask & scanPos)
                /* when no residual coefficient is coded, predicted coef == recon coef */
                costUncoded[blkPos] -= PSYVALUE(predictedCoef);

            totalUncodedCost += costUncoded[blkPos];// 累加量化为0的cost

            // coefficient level estimation
            const int* greaterOneBits = estBitsSbac.greaterOneBits[4 * ctxSet + c1];
            //const uint32_t ctxSig = (blkPos == 0) ? 0 : table_cnt[(trSize == 4) ? 4 : patternSigCtx][g_scan4x4[codeParams.scanType][scanPosinCG]] + ctxSigOffset;
            static const uint64_t table_cnt64[4] = {0x0000000100110112ULL, 0x0000000011112222ULL, 0x0012001200120012ULL, 0x2222222222222222ULL};
            uint64_t ctxCnt = (trSize == 4) ? 0x8877886654325410ULL : table_cnt64[patternSigCtx];
            const uint32_t ctxSig = (blkPos == 0) ? 0 : ((ctxCnt >> (4 * g_scan4x4[codeParams.scanType][scanPosinCG])) & 0xF) + ctxSigOffset;
            // NOTE: above equal to 'table_cnt[(trSize == 4) ? 4 : patternSigCtx][g_scan4x4[codeParams.scanType][scanPosinCG]] + ctxSigOffset'
            S265_CHECK(ctxSig == getSigCtxInc(patternSigCtx, log2TrSize, trSize, blkPos, bIsLuma, codeParams.firstSignificanceMapContext), "sigCtx check failure\n");

            // before find lastest non-zero coeff
            if (scanPos > (uint32_t)lastScanPos)// 如果当前扫描位置在最后一个非零系数之后，则将这些系数的cost都设置为0
            {                                   // 这种情况出现是由于最后一个非零系数可能出现在最后一个非零CG中的任何位置，所以对最后一个CG扫描就会出现这种情况
                /* coefficients after lastNZ have no distortion signal cost */
                costCoeff[scanPos] = 0;
                costSig[scanPos] = 0;

                /* No non-zero coefficient yet found, but this does not mean
                 * there is no uncoded-cost for this coefficient. Pre-
                 * quantization the coefficient may have been non-zero */
                totalRdCost += costUncoded[blkPos];
            }
            else if (!(subFlagMask & 1)) // 如果当前位置位于最后一个非零系数之前，并且该系数为0，计算量化为0的cost
            {
                // fast zero coeff path
                /* set default costs to uncoded costs */
                costSig[scanPos] = SIGCOST(estBitsSbac.significantBits[0][ctxSig]); // 计算系数标记sig_coeff_flag为0的cost
                costCoeff[scanPos] = costUncoded[blkPos] + costSig[scanPos]; // 得到一个系数的总花费， = 量化为0的distortion + 系数标记为0的cost
                sigRateDelta[blkPos] = estBitsSbac.significantBits[1][ctxSig] - estBitsSbac.significantBits[0][ctxSig];// 将系数量化为0和量化为非0，系数标记（sig_coeff_flag）的花费差异
                totalRdCost += costCoeff[scanPos];// 累加系数总花费
                rateIncUp[blkPos] = greaterOneBits[0];// 得到比当前量化系数大1所需要花费的bit ???

                subFlagMask >>= 1;// 系数非零标记右移移位，下次取到CG中前一个系数的非零标记
            }
            else// 如果当前位置位于最后一个非零系数之前，并且该系数为1，估计每一个系数的最优量化值
            {
                subFlagMask >>= 1;// 系数非零标记右移移位，下次取到CG中前一个系数的非零标记

                const uint32_t c1c2idx = ((c1Idx - 8) >> (sizeof(int) * CHAR_BIT - 1)) + (((-(int)c2Idx) >> (sizeof(int) * CHAR_BIT - 1)) + 1) * 2;
                const uint32_t baseLevel = ((uint32_t)0xD9 >> (c1c2idx * 2)) & 3;  // {1, 2, 1, 3}

                S265_CHECK(!!((int)c1Idx < C1FLAG_NUMBER) == (int)((c1Idx - 8) >> (sizeof(int) * CHAR_BIT - 1)), "scan validation 1\n");
                S265_CHECK(!!(c2Idx == 0) == ((-(int)c2Idx) >> (sizeof(int) * CHAR_BIT - 1)) + 1, "scan validation 2\n");
                S265_CHECK((int)baseLevel == ((c1Idx < C1FLAG_NUMBER) ? (2 + (c2Idx == 0)) : 1), "scan validation 3\n");
                S265_CHECK(c1c2idx <= 3, "c1c2Idx check failure\n");

                // coefficient level estimation
                const int* levelAbsBits = estBitsSbac.levelAbsBits[ctxSet + c2];
                const uint32_t c1c2Rate = ((c1c2idx & 1) ?  greaterOneBits[1] : 0) + ((c1c2idx == 3) ? levelAbsBits[1] : 0);

                uint32_t level = 0;
                uint32_t sigCoefBits = 0;
                costCoeff[scanPos] = MAX_INT64;

                if ((int)scanPos == lastScanPos)// 如果当前位置是最后一个非零系数的位置，则将量化为0和非0的系数标记（sig_coeff_flag）的花费差异设为0
                    sigRateDelta[blkPos] = 0;
                else
                {
                    if (maxAbsLevel < 3)// 如果该系数的常规量化值很小，则尝试将它量化为0
                    {
                        /* set default costs to uncoded costs */
                        costSig[scanPos] = SIGCOST(estBitsSbac.significantBits[0][ctxSig]); // 计算系数标记sig_coeff_flag为0的cost
                        costCoeff[scanPos] = costUncoded[blkPos] + costSig[scanPos]; // 得到一个系数的总花费， = 量化为0的distortion + 系数标记为0的cost
                    }
                    sigRateDelta[blkPos] = estBitsSbac.significantBits[1][ctxSig] - estBitsSbac.significantBits[0][ctxSig];// 将系数量化为0和量化为非0，系数标记（sig_coeff_flag）的花费差异
                    sigCoefBits = estBitsSbac.significantBits[1][ctxSig];// 将系数量化为非0，系数标记（sig_coeff_flag）的花费
                }

                const uint32_t unQuantLevel = (maxAbsLevel * (unquantScale[blkPos] << per) + unquantRound);
                // NOTE: S265_MAX(maxAbsLevel - 1, 1) ==> (X>=2 -> X-1), (X<2 -> 1)  | (0 < X < 2 ==> X=1)
                if (maxAbsLevel == 1)// 如果常规量化值为1，则与量化为0相比较
                {
                    uint32_t levelBits = (c1c2idx & 1) ? greaterOneBits[0] + IEP_RATE : ((1 + goRiceParam) << 15) + IEP_RATE;// 计算量化为1时，系数幅值的bit消耗
                    S265_CHECK(levelBits == getICRateCost(1, 1 - baseLevel, greaterOneBits, levelAbsBits, goRiceParam, c1c2Rate) + IEP_RATE, "levelBits mistake\n");

                    int unquantAbsLevel = unQuantLevel >> unquantShift;// 得到1的反量化值
                    S265_CHECK(UNQUANT(1) == unquantAbsLevel, "DQuant check failed\n");
                    int d = abs(signCoef) - unquantAbsLevel;// 得到distortion
                    int64_t curCost = RDCOST(d, sigCoefBits + levelBits);// 计算量化为1的RDcost

                    /* Psy RDOQ: bias in favor of higher AC coefficients in the reconstructed frame */
                    if (usePsyMask & scanPos)
                    {
                        int reconCoef = abs(unquantAbsLevel + SIGN(predictedCoef, signCoef));
                        curCost -= PSYVALUE(reconCoef);
                    }

                    if (curCost < costCoeff[scanPos])// 将量化为1的cost与量化为0相比较，如果量化为1更好，则将其量化为1
                    {
                        level = 1;// 更新量化值为1
                        costCoeff[scanPos] = curCost;// 更新量化当前系数的cost
                        costSig[scanPos] = SIGCOST(sigCoefBits);// 更新sig_coeff_flag的cost
                    }
                }
                else if (maxAbsLevel)// 如果常规量化值大于1
                {
                    // 计算量化为当前常规量化值所花费的比特数，并计算量化为常规量化值减1所花费的比特数
                    uint32_t levelBits0 = getICRateCost(maxAbsLevel,     maxAbsLevel     - baseLevel, greaterOneBits, levelAbsBits, goRiceParam, c1c2Rate) + IEP_RATE;
                    uint32_t levelBits1 = getICRateCost(maxAbsLevel - 1, maxAbsLevel - 1 - baseLevel, greaterOneBits, levelAbsBits, goRiceParam, c1c2Rate) + IEP_RATE;

                    const uint32_t preDQuantLevelDiff = (unquantScale[blkPos] << per);
                    // 计算量化为当前常规量化值所产生的cost
                    const int unquantAbsLevel0 = unQuantLevel >> unquantShift;// 得到反量化值
                    S265_CHECK(UNQUANT(maxAbsLevel) == (uint32_t)unquantAbsLevel0, "DQuant check failed\n");
                    int d0 = abs(signCoef) - unquantAbsLevel0;// 得到distortion
                    int64_t curCost0 = RDCOST(d0, sigCoefBits + levelBits0);// 计算得到RDcost
                    // 计算量化为当前常规量化值减1所产生的cost
                    const int unquantAbsLevel1 = (unQuantLevel - preDQuantLevelDiff) >> unquantShift;// 得到反量化值
                    S265_CHECK(UNQUANT(maxAbsLevel - 1) == (uint32_t)unquantAbsLevel1, "DQuant check failed\n");
                    int d1 = abs(signCoef) - unquantAbsLevel1;// 得到distortion
                    int64_t curCost1 = RDCOST(d1, sigCoefBits + levelBits1);// 计算得到RDcost

                    /* Psy RDOQ: bias in favor of higher AC coefficients in the reconstructed frame */
                    if (usePsyMask & scanPos)
                    {
                        int reconCoef;
                        reconCoef = abs(unquantAbsLevel0 + SIGN(predictedCoef, signCoef));
                        curCost0 -= PSYVALUE(reconCoef);

                        reconCoef = abs(unquantAbsLevel1 + SIGN(predictedCoef, signCoef));
                        curCost1 -= PSYVALUE(reconCoef);
                    }
                    if (curCost0 < costCoeff[scanPos])// 如果量化为当前常规量化值的cost更小，则更新量化值和cost
                    {
                        level = maxAbsLevel;// 更新量化值为1
                        costCoeff[scanPos] = curCost0;// 更新量化当前系数的cost
                        costSig[scanPos] = SIGCOST(sigCoefBits);// 更新sig_coeff_flag的cost
                    }
                    if (curCost1 < costCoeff[scanPos])// 如果量化为当前常规量化值减1的cost更小，则更新量化值和cost
                    {
                        level = maxAbsLevel - 1;// 更新量化值为1
                        costCoeff[scanPos] = curCost1;// 更新量化当前系数的cost
                        costSig[scanPos] = SIGCOST(sigCoefBits); // 更新sig_coeff_flag的cost
                    }
                }

                dstCoeff[blkPos] = (int16_t)level;// 在对当前量化系数进行完RDO优化后，更新目标量化值
                totalRdCost += costCoeff[scanPos];// 累加总的RDcost

                /* record costs for sign-hiding performed at the end */
                if ((cu.m_slice->m_pps->bSignHideEnabled ? ~0 : 0) & level)
                {
                    const int32_t diff0 = level - 1 - baseLevel;
                    const int32_t diff2 = level + 1 - baseLevel;
                    const int32_t maxVlc = g_goRiceRange[goRiceParam];
                    int rate0, rate1, rate2;

                    if (diff0 < -2)  // prob (92.9, 86.5, 74.5)%
                    {
                        // NOTE: Min: L - 1 - {1,2,1,3} < -2 ==> L < {0,1,0,2}
                        //            additional L > 0, so I got (L > 0 && L < 2) ==> L = 1
                        S265_CHECK(level == 1, "absLevel check failure\n");

                        const int rateEqual2 = greaterOneBits[1] + levelAbsBits[0];;
                        const int rateNotEqual2 = greaterOneBits[0];

                        rate0 = 0;
                        rate2 = rateEqual2;
                        rate1 = rateNotEqual2;

                        S265_CHECK(rate1 == getICRateNegDiff(level + 0, greaterOneBits, levelAbsBits), "rate1 check failure!\n");
                        S265_CHECK(rate2 == getICRateNegDiff(level + 1, greaterOneBits, levelAbsBits), "rate1 check failure!\n");
                        S265_CHECK(rate0 == getICRateNegDiff(level - 1, greaterOneBits, levelAbsBits), "rate1 check failure!\n");
                    }
                    else if (diff0 >= 0 && diff2 <= maxVlc)     // prob except from above path (98.6, 97.9, 96.9)%
                    {
                        // NOTE: no c1c2 correct rate since all of rate include this factor
                        rate1 = getICRateLessVlc(level + 0, diff0 + 1, goRiceParam);
                        rate2 = getICRateLessVlc(level + 1, diff0 + 2, goRiceParam);
                        rate0 = getICRateLessVlc(level - 1, diff0 + 0, goRiceParam);
                    }
                    else
                    {
                        rate1 = getICRate(level + 0, diff0 + 1, greaterOneBits, levelAbsBits, goRiceParam, maxVlc, c1c2Rate);
                        rate2 = getICRate(level + 1, diff0 + 2, greaterOneBits, levelAbsBits, goRiceParam, maxVlc, c1c2Rate);
                        rate0 = getICRate(level - 1, diff0 + 0, greaterOneBits, levelAbsBits, goRiceParam, maxVlc, c1c2Rate);
                    }
                    rateIncUp[blkPos] = rate2 - rate1;
                    rateIncDown[blkPos] = rate0 - rate1;
                }
                else
                {
                    rateIncUp[blkPos] = greaterOneBits[0];
                    rateIncDown[blkPos] = 0;
                }

                /* Update CABAC estimation state */
                if ((level >= baseLevel) && (goRiceParam < 4) && (level > levelThreshold))
                {
                    goRiceParam++;
                    levelThreshold <<= 1;
                }

                const uint32_t isNonZero = (uint32_t)(-(int32_t)level) >> 31;
                c1Idx += isNonZero;

                /* update bin model */
                if (level > 1)
                {
                    c1 = 0;
                    c2 += (uint32_t)(c2 - 2) >> 31;
                    c2Idx++;
                }
                else if (((c1 == 1) | (c1 == 2)) & isNonZero)
                    c1++;

                if (dstCoeff[blkPos])// 如果当前量化系数为非零
                {
                    sigCoeffGroupFlag64 |= cgBlkPosMask;// 与当前CG位置的Mask相或，标志该CG不是全0
                    cgRdStats.codedLevelAndDist += costCoeff[scanPos] - costSig[scanPos];
                    cgRdStats.uncodedDist += costUncoded[blkPos];
                    cgRdStats.nnzBeforePos0 += scanPosinCG;
                }
            }

            cgRdStats.sigCost += costSig[scanPos];
        } /* end for (scanPosinCG) */

        S265_CHECK((cgScanPos << MLS_CG_SIZE) == (int)scanPos, "scanPos mistake\n");
        cgRdStats.sigCost0 = costSig[scanPos];

        costCoeffGroupSig[cgScanPos] = 0;

        /* nothing to do at this case */
        S265_CHECK(cgLastScanPos >= 0, "cgLastScanPos check failure\n");

        if (!cgScanPos || cgScanPos == cgLastScanPos)
        {
            /* coeff group 0 is implied to be present, no signal cost */
            /* coeff group with last NZ is implied to be present, handled below */
        }
        else if (sigCoeffGroupFlag64 & cgBlkPosMask)
        {
            if (!cgRdStats.nnzBeforePos0)
            {
                /* if only coeff 0 in this CG is coded, its significant coeff bit is implied */
                totalRdCost -= cgRdStats.sigCost0;
                cgRdStats.sigCost -= cgRdStats.sigCost0;
            }

            /* there are coded coefficients in this group, but now we include the signaling cost
             * of the significant coefficient group flag and evaluate whether the RD cost of the
             * coded group is more than the RD cost of the uncoded group */

            uint32_t sigCtx = getSigCoeffGroupCtxInc(sigCoeffGroupFlag64, cgPosX, cgPosY, cgBlkPos, cgStride);

            int64_t costZeroCG = totalRdCost + SIGCOST(estBitsSbac.significantCoeffGroupBits[sigCtx][0]);
            costZeroCG += cgRdStats.uncodedDist;       /* add distortion for resetting non-zero levels to zero levels */
            costZeroCG -= cgRdStats.codedLevelAndDist; /* remove distortion and level cost of coded coefficients */
            costZeroCG -= cgRdStats.sigCost;           /* remove signaling cost of significant coeff bitmap */

            costCoeffGroupSig[cgScanPos] = SIGCOST(estBitsSbac.significantCoeffGroupBits[sigCtx][1]);
            totalRdCost += costCoeffGroupSig[cgScanPos];  /* add the cost of 1 bit in significant CG bitmap */

            if (costZeroCG < totalRdCost && m_rdoqLevel > 1)
            {
                sigCoeffGroupFlag64 &= ~cgBlkPosMask;
                totalRdCost = costZeroCG;
                costCoeffGroupSig[cgScanPos] = SIGCOST(estBitsSbac.significantCoeffGroupBits[sigCtx][0]);

                /* reset all coeffs to 0. UNCODE THIS COEFF GROUP! */
                const uint32_t blkPos = codeParams.scan[cgScanPos * cgSize];
                memset(&dstCoeff[blkPos + 0 * trSize], 0, 4 * sizeof(*dstCoeff));
                memset(&dstCoeff[blkPos + 1 * trSize], 0, 4 * sizeof(*dstCoeff));
                memset(&dstCoeff[blkPos + 2 * trSize], 0, 4 * sizeof(*dstCoeff));
                memset(&dstCoeff[blkPos + 3 * trSize], 0, 4 * sizeof(*dstCoeff));
            }
        }
        else
        {
            /* there were no coded coefficients in this coefficient group */
            uint32_t ctxSig = getSigCoeffGroupCtxInc(sigCoeffGroupFlag64, cgPosX, cgPosY, cgBlkPos, cgStride);
            costCoeffGroupSig[cgScanPos] = SIGCOST(estBitsSbac.significantCoeffGroupBits[ctxSig][0]);
            totalRdCost += costCoeffGroupSig[cgScanPos];  /* add cost of 0 bit in significant CG bitmap */
            totalRdCost -= cgRdStats.sigCost;             /* remove cost of significant coefficient bitmap */
        }
    } /* end for (cgScanPos) */

    S265_CHECK(lastScanPos >= 0, "numSig non zero, but no coded CG\n");

    /* calculate RD cost of uncoded block CBF=0, and add cost of CBF=1 to total */
    int64_t bestCost;
    if (!cu.isIntra(absPartIdx) && bIsLuma && !cu.m_tuDepth[absPartIdx])
    {
        bestCost = totalUncodedCost + SIGCOST(estBitsSbac.blockRootCbpBits[0]);
        totalRdCost += SIGCOST(estBitsSbac.blockRootCbpBits[1]);
    }
    else
    {
        int ctx = ctxCbf[ttype][cu.m_tuDepth[absPartIdx]];
        bestCost = totalUncodedCost + SIGCOST(estBitsSbac.blockCbpBits[ctx][0]);
        totalRdCost += SIGCOST(estBitsSbac.blockCbpBits[ctx][1]);
    }

    /* This loop starts with the last non-zero found in the first loop and then refines this last
     * non-zero by measuring the true RD cost of the last NZ at this position, and then the RD costs
     * at all previous coefficients until a coefficient greater than 1 is encountered or we run out
     * of coefficients to evaluate.  This will factor in the cost of coding empty groups and empty
     * coeff prior to the last NZ. The base best cost is the RD cost of CBF=0 */
    int  bestLastIdx = 0;
    bool foundLast = false;
    for (int cgScanPos = cgLastScanPos; cgScanPos >= 0 && !foundLast; cgScanPos--)
    {
        if (!cgScanPos || cgScanPos == cgLastScanPos)
        {
            /* the presence of these coefficient groups are inferred, they have no bit in
             * sigCoeffGroupFlag64 and no saved costCoeffGroupSig[] cost */
        }
        else if (sigCoeffGroupFlag64 & (1ULL << codeParams.scanCG[cgScanPos]))
        {
            /* remove cost of significant coeff group flag, the group's presence would be inferred
             * from lastNZ if it were present in this group */
            totalRdCost -= costCoeffGroupSig[cgScanPos];
        }
        else
        {
            /* remove cost of signaling this empty group as not present */
            totalRdCost -= costCoeffGroupSig[cgScanPos];
            continue;
        }

        for (int scanPosinCG = cgSize - 1; scanPosinCG >= 0; scanPosinCG--)
        {
            scanPos = cgScanPos * cgSize + scanPosinCG;
            if ((int)scanPos > lastScanPos)
                continue;

            /* if the coefficient was coded, measure the RD cost of it as the last non-zero and then
             * continue as if it were uncoded. If the coefficient was already uncoded, remove the
             * cost of signaling it as not-significant */
            uint32_t blkPos = codeParams.scan[scanPos];
            if (dstCoeff[blkPos])// 如果目标量化系数不是0，则试图将该系数设置为0
            {
                // Calculates the cost of signaling the last significant coefficient in the block 
                uint32_t pos[2] = { (blkPos & (trSize - 1)), (blkPos >> log2TrSize) };// 得到该系数所在位置的X/Y坐标，X = blkPos&(trSize-1); Y = blkPos>>log2TrSize
                if (codeParams.scanType == SCAN_VER)// 如果当前的扫描类型是竖直扫描，则调换X和Y坐标
                    std::swap(pos[0], pos[1]);
                uint32_t bitsLastNZ = 0;

                for (int i = 0; i < 2; i++)
                {
                    int temp = g_lastCoeffTable[pos[i]];// 估计该位置的X/Y坐标所需要的bit花费，X=pos[0]，Y=pos[1]
                    int prefixOnes = temp & 15;// 得到该系数位置的前缀和后缀
                    int suffixLen = temp >> 4;

                    bitsLastNZ += m_entropyCoder->m_estBitsSbac.lastBits[i][prefixOnes];// 估计对前缀熵编码所消耗的bits
                    bitsLastNZ += IEP_RATE * suffixLen;// 加上后缀所消耗的bits
                }

                int64_t costAsLast = totalRdCost - costSig[scanPos] + SIGCOST(bitsLastNZ);

                if (costAsLast < bestCost)
                {
                    bestLastIdx = scanPos + 1;
                    bestCost = costAsLast;
                }
                if (dstCoeff[blkPos] > 1 || m_rdoqLevel == 1)
                {
                    foundLast = true;
                    break;
                }

                totalRdCost -= costCoeff[scanPos];
                totalRdCost += costUncoded[blkPos];
            }
            else
                totalRdCost -= costSig[scanPos];
        }
    }

    /* recount non-zero coefficients and re-apply sign of DCT coef */
    numSig = 0;
    for (int pos = 0; pos < bestLastIdx; pos++)
    {
        int blkPos = codeParams.scan[pos];
        int level  = dstCoeff[blkPos];
        numSig += (level != 0);

        uint32_t mask = (int32_t)m_resiDctCoeff[blkPos] >> 31;
        dstCoeff[blkPos] = (int16_t)((level ^ mask) - mask);
    }

    // Average 49.62 pixels
    /* clean uncoded coefficients */
    S265_CHECK((uint32_t)(fastMin(lastScanPos, bestLastIdx) | (SCAN_SET_SIZE - 1)) < trSize * trSize, "array beyond bound\n");
    for (int pos = bestLastIdx; pos <= (fastMin(lastScanPos, bestLastIdx) | (SCAN_SET_SIZE - 1)); pos++)
    {
        dstCoeff[codeParams.scan[pos]] = 0;
    }
    for (int pos = (bestLastIdx & ~(SCAN_SET_SIZE - 1)) + SCAN_SET_SIZE; pos <= lastScanPos; pos += SCAN_SET_SIZE)
    {
        const uint32_t blkPos = codeParams.scan[pos];
        memset(&dstCoeff[blkPos + 0 * trSize], 0, 4 * sizeof(*dstCoeff));
        memset(&dstCoeff[blkPos + 1 * trSize], 0, 4 * sizeof(*dstCoeff));
        memset(&dstCoeff[blkPos + 2 * trSize], 0, 4 * sizeof(*dstCoeff));
        memset(&dstCoeff[blkPos + 3 * trSize], 0, 4 * sizeof(*dstCoeff));
    }

    /* rate-distortion based sign-hiding */
    if (cu.m_slice->m_pps->bSignHideEnabled && numSig >= 2)
    {
        const int realLastScanPos = (bestLastIdx - 1) >> LOG2_SCAN_SET_SIZE;
        int lastCG = 1;

        for (int subSet = realLastScanPos; subSet >= 0; subSet--)
        {
            int subPos = subSet << LOG2_SCAN_SET_SIZE;
            int n;

            if (!(sigCoeffGroupFlag64 & (1ULL << codeParams.scanCG[subSet])))
                continue;

            /* measure distance between first and last non-zero coef in this
             * coding group */
            const uint32_t posFirstLast = primitives.findPosFirstLast(&dstCoeff[codeParams.scan[subPos]], trSize, g_scan4x4[codeParams.scanType]);
            const int firstNZPosInCG = (uint8_t)posFirstLast;
            const int lastNZPosInCG = (int8_t)(posFirstLast >> 8);
            const uint32_t absSumSign = posFirstLast;

            if (lastNZPosInCG - firstNZPosInCG >= SBH_THRESHOLD)
            {
                const int32_t signbit = ((int32_t)dstCoeff[codeParams.scan[subPos + firstNZPosInCG]]);

#if CHECKED_BUILD || _DEBUG
                int32_t absSum_dummy = 0;
                for (n = firstNZPosInCG; n <= lastNZPosInCG; n++)
                    absSum_dummy += dstCoeff[codeParams.scan[n + subPos]];
                S265_CHECK(((uint32_t)absSum_dummy & 1) == (absSumSign >> 31), "absSumSign check failure\n");
#endif

                //if (signbit != absSumSign)
                if (((int32_t)(signbit ^ absSumSign)) < 0)
                {
                    /* We must find a coeff to toggle up or down so the sign bit of the first non-zero coeff
                     * is properly implied. Note dstCoeff[] are signed by this point but curChange and
                     * finalChange imply absolute levels (+1 is away from zero, -1 is towards zero) */

                    int64_t minCostInc = MAX_INT64, curCost = MAX_INT64;
                    uint32_t minPos = 0;
                    int8_t finalChange = 0;
                    int curChange = 0;
                    uint32_t lastCoeffAdjust = (lastCG & (abs(dstCoeff[codeParams.scan[lastNZPosInCG + subPos]]) == 1)) * 4 * IEP_RATE;

                    for (n = (lastCG ? lastNZPosInCG : SCAN_SET_SIZE - 1); n >= 0; --n)
                    {
                        const uint32_t blkPos = codeParams.scan[n + subPos];
                        const int32_t signCoef = m_resiDctCoeff[blkPos]; /* pre-quantization DCT coeff */
                        const int absLevel = abs(dstCoeff[blkPos]);
                        // TODO: this is constant in non-scaling mode
                        const uint32_t preDQuantLevelDiff = (unquantScale[blkPos] << per);
                        const uint32_t unQuantLevel = (absLevel * (unquantScale[blkPos] << per) + unquantRound);

                        int d = abs(signCoef) - (unQuantLevel >> unquantShift);
                        S265_CHECK((uint32_t)UNQUANT(absLevel) == (unQuantLevel >> unquantShift), "dquant check failed\n");

                        const int64_t origDist = (((int64_t)d * d));

#define DELTARDCOST(d0, d, deltabits) ((((int64_t)d * d - d0) << scaleBits) + ((lambda2 * (int64_t)(deltabits)) >> 8))

                        const uint32_t isOne = (absLevel == 1);
                        if (dstCoeff[blkPos])
                        {
                            d = abs(signCoef) - ((unQuantLevel + preDQuantLevelDiff) >> unquantShift);
                            S265_CHECK((uint32_t)UNQUANT(absLevel + 1) == ((unQuantLevel + preDQuantLevelDiff) >> unquantShift), "dquant check failed\n");
                            int64_t costUp = DELTARDCOST(origDist, d, rateIncUp[blkPos]);

                            /* if decrementing would make the coeff 0, we can include the
                             * significant coeff flag cost savings */
                            d = abs(signCoef) - ((unQuantLevel - preDQuantLevelDiff) >> unquantShift);
                            S265_CHECK((uint32_t)UNQUANT(absLevel - 1) == ((unQuantLevel - preDQuantLevelDiff) >> unquantShift), "dquant check failed\n");
                            int downBits = rateIncDown[blkPos] - (isOne ? (IEP_RATE + sigRateDelta[blkPos]) : 0);
                            int64_t costDown = DELTARDCOST(origDist, d, downBits);

                            costDown -= lastCoeffAdjust;
                            curCost = ((n == firstNZPosInCG) & isOne) ? MAX_INT64 : costDown;

                            curChange = 2 * (costUp < costDown) - 1;
                            curCost = (costUp < costDown) ? costUp : curCost;
                        }
                        //else if ((n < firstNZPosInCG) & (signbit != ((uint32_t)signCoef >> 31)))
                        else if ((n < firstNZPosInCG) & ((signbit ^ signCoef) < 0))
                        {
                            /* don't try to make a new coded coeff before the first coeff if its
                             * sign would be different than the first coeff, the inferred sign would
                             * still be wrong and we'd have to do this again. */
                            curCost = MAX_INT64;
                        }
                        else
                        {
                            /* evaluate changing an uncoded coeff 0 to a coded coeff +/-1 */
                            d = abs(signCoef) - ((preDQuantLevelDiff + unquantRound) >> unquantShift);
                            S265_CHECK((uint32_t)UNQUANT(1) == ((preDQuantLevelDiff + unquantRound) >> unquantShift), "dquant check failed\n");
                            curCost = DELTARDCOST(origDist, d, rateIncUp[blkPos] + IEP_RATE + sigRateDelta[blkPos]);
                            curChange = 1;
                        }

                        if (curCost < minCostInc)
                        {
                            minCostInc = curCost;
                            finalChange = (int8_t)curChange;
                            minPos = blkPos + (absLevel << 16);
                        }
                        lastCoeffAdjust = 0;
                    }

                    const int absInMinPos = (minPos >> 16);
                    minPos = (uint16_t)minPos;

                    // if (dstCoeff[minPos] == 32767 || dstCoeff[minPos] == -32768)
                    if (absInMinPos >= 32767)
                        /* don't allow sign hiding to violate the SPEC range */
                        finalChange = -1;

                    // NOTE: Reference code
                    //if (dstCoeff[minPos] == 0)
                    //    numSig++;
                    //else if (finalChange == -1 && abs(dstCoeff[minPos]) == 1)
                    //    numSig--;
                    numSig += (absInMinPos == 0) - ((finalChange == -1) & (absInMinPos == 1));


                    // NOTE: Reference code
                    //if (m_resiDctCoeff[minPos] >= 0)
                    //    dstCoeff[minPos] += finalChange;
                    //else
                    //    dstCoeff[minPos] -= finalChange;
                    const int16_t resiCoeffSign = ((int16_t)m_resiDctCoeff[minPos] >> 16);
                    dstCoeff[minPos] += (((int16_t)finalChange ^ resiCoeffSign) - resiCoeffSign);
                }
            }

            lastCG = 0;
        }
    }

    return numSig;
}

/* Context derivation process of coeff_abs_significant_flag */
uint32_t Quant::getSigCtxInc(uint32_t patternSigCtx, uint32_t log2TrSize, uint32_t trSize, uint32_t blkPos, bool bIsLuma,
                             uint32_t firstSignificanceMapContext)
{
    static const uint8_t ctxIndMap[16] =
    {
        0, 1, 4, 5,
        2, 3, 4, 5,
        6, 6, 8, 8,
        7, 7, 8, 8
    };

    if (!blkPos) // special case for the DC context variable
        return 0;

    if (log2TrSize == 2) // 4x4
        return ctxIndMap[blkPos];

    const uint32_t posY = blkPos >> log2TrSize;
    const uint32_t posX = blkPos & (trSize - 1);
    S265_CHECK((blkPos - (posY << log2TrSize)) == posX, "block pos check failed\n");

    int posXinSubset = blkPos & 3;
    S265_CHECK((posX & 3) == (blkPos & 3), "pos alignment fail\n");
    int posYinSubset = posY & 3;

    // NOTE: [patternSigCtx][posXinSubset][posYinSubset]
    static const uint8_t table_cnt[4][4][4] =
    {
        // patternSigCtx = 0
        {
            { 2, 1, 1, 0 },
            { 1, 1, 0, 0 },
            { 1, 0, 0, 0 },
            { 0, 0, 0, 0 },
        },
        // patternSigCtx = 1
        {
            { 2, 1, 0, 0 },
            { 2, 1, 0, 0 },
            { 2, 1, 0, 0 },
            { 2, 1, 0, 0 },
        },
        // patternSigCtx = 2
        {
            { 2, 2, 2, 2 },
            { 1, 1, 1, 1 },
            { 0, 0, 0, 0 },
            { 0, 0, 0, 0 },
        },
        // patternSigCtx = 3
        {
            { 2, 2, 2, 2 },
            { 2, 2, 2, 2 },
            { 2, 2, 2, 2 },
            { 2, 2, 2, 2 },
        }
    };

    int cnt = table_cnt[patternSigCtx][posXinSubset][posYinSubset];
    int offset = firstSignificanceMapContext;

    offset += cnt;

    return (bIsLuma && (posX | posY) >= 4) ? 3 + offset : offset;
}

