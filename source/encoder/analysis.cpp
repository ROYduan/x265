/*****************************************************************************
* Copyright (C) 2013-2020 MulticoreWare, Inc
*
* Authors: Deepthi Nandakumar <deepthi@multicorewareinc.com>
*          Steve Borho <steve@borho.org>
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
#include "primitives.h"
#include "threading.h"

#include "analysis.h"
#include "rdcost.h"
#include "encoder.h"

using namespace S265_NS;

/* An explanation of rate distortion levels (--rd-level)
 *
 * rd-level 0 generates no recon per CU (NO RDO or Quant)
 *
 *   sa8d selection between merge / skip / inter / intra and split
 *   no recon pixels generated until CTU analysis is complete, requiring
 *   intra predictions to use source pixels
 *
 * rd-level 1 uses RDO for merge and skip, sa8d for all else
 *
 *   RDO selection between merge and skip
 *   sa8d selection between (merge/skip) / inter modes / intra and split
 *   intra prediction uses reconstructed pixels
 *
 * rd-level 2 uses RDO for merge/skip and split
 *
 *   RDO selection between merge and skip
 *   sa8d selection between (merge/skip) / inter modes / intra
 *   RDO split decisions
 *
 * rd-level 3 uses RDO for merge/skip/best inter/intra
 *
 *   RDO selection between merge and skip
 *   sa8d selection of best inter mode
 *   sa8d decisions include chroma residual cost
 *   RDO selection between (merge/skip) / best inter mode / intra / split
 *
 * rd-level 4 enables RDOQuant
 *   chroma residual cost included in satd decisions, including subpel refine
 *    (as a result of --subme 3 being used by preset slow)
 *
 * rd-level 5,6 does RDO for each inter mode
 */

Analysis::Analysis()
{
    m_reuseRef = NULL;
    m_bHD = false;
    m_modeFlag[0] = false;
    m_modeFlag[1] = false;
    m_checkMergeAndSkipOnly[0] = false;
    m_checkMergeAndSkipOnly[1] = false;
    m_evaluateInter = 0;
}

bool Analysis::create(ThreadLocalData *tld)
{
    m_tld = tld;
    m_bTryLossless = m_param->bCULossless && !m_param->bLossless && m_param->rdLevel >= 2;

    int costArrSize = 1;
    uint32_t maxDQPDepth = g_log2Size[m_param->maxCUSize] - g_log2Size[m_param->rc.qgSize];
    for (uint32_t i = 1; i <= maxDQPDepth; i++)
        costArrSize += (1 << (i * 2));
    cacheCost = S265_MALLOC(uint64_t, costArrSize);

    int csp = m_param->internalCsp;
    uint32_t cuSize = m_param->maxCUSize;// 64/32

    bool ok = true;
    for (uint32_t depth = 0; depth <= m_param->maxCUDepth; depth++, cuSize >>= 1)
    {
        ModeDepth &md = m_modeDepth[depth]; //依次获取64x64/32x32/16x16/8x8 对应的modeDepth
        ok &= md.cuMemPool.create(depth, csp, MAX_PRED_TYPES, *m_param);
        ok &= md.fencYuv.create(cuSize, csp);// 主要是 get 一个 sizexsize 对应的partition 大小的 bufffer m_buf[3]/m_buf[1]/m_buf[2]/
        if (ok)
        {
            for (int j = 0; j < MAX_PRED_TYPES; j++)
            {
                md.pred[j].cu.initialize(md.cuMemPool, depth, *m_param, j);
                ok &= md.pred[j].predYuv.create(cuSize, csp);
                ok &= md.pred[j].reconYuv.create(cuSize, csp);
                md.pred[j].fencYuv = &md.fencYuv;
            }
        }
    }
    if (m_param->sourceHeight >= 1080)
        m_bHD = true;

    return ok;
}

void Analysis::destroy()
{
    for (uint32_t i = 0; i <= m_param->maxCUDepth; i++)
    {
        m_modeDepth[i].cuMemPool.destroy();
        m_modeDepth[i].fencYuv.destroy();

        for (int j = 0; j < MAX_PRED_TYPES; j++)
        {
            m_modeDepth[i].pred[j].predYuv.destroy();
            m_modeDepth[i].pred[j].reconYuv.destroy();
        }
    }
    S265_FREE(cacheCost);
}
// 分析函数入口
Mode& Analysis::compressCTU(CUData& ctu, Frame& frame, const CUGeom& cuGeom, const Entropy& initialContext)
{
    m_slice = ctu.m_slice;//取CTU所在slice
    m_frame = &frame; //取CTU所在frame
    m_bChromaSa8d = m_param->rdLevel >= 3;//若rdlevel>=3则要计算chroma的sa8d
    m_param = m_frame->m_param; //取param

#if _DEBUG || CHECKED_BUILD
    invalidateContexts(0);
#endif

    int qp = setLambdaFromQP(ctu, m_slice->m_pps->bUseDQP ? calculateQpforCuSize(ctu, cuGeom) : m_slice->m_sliceQp);
    ctu.setQPSubParts((int8_t)qp, 0, 0);//从第0个4x4 开始 设置 depth0(256)个4x4 的qp
    //0深度四叉树加载context
    m_rqt[0].cur.load(initialContext);
    ctu.m_meanQP = initialContext.m_meanQP;
    //从m_fencPic中复制YUV数据到0深度的modeDepth中fencYuv
    m_modeDepth[0].fencYuv.copyFromPicYuv(*m_frame->m_fencPic, ctu.m_cuAddr, 0);

    if (m_param->bSsimRd) ////若使用ssim rdo
        calculateNormFactor(ctu, qp);

    ProfileCUScope(ctu, totalCTUTime, totalCTUs);

    if (m_slice->m_sliceType == I_SLICE)
    {
        // intra ctu 分析
        compressIntraCU(ctu, cuGeom, qp);
    }
    else
    {
        if (m_param->bIntraRefresh && m_slice->m_sliceType == P_SLICE &&
            ctu.m_cuPelX / m_param->maxCUSize >= frame.m_encData->m_pir.pirStartCol
            && ctu.m_cuPelX / m_param->maxCUSize < frame.m_encData->m_pir.pirEndCol)
            compressIntraCU(ctu, cuGeom, qp);//帧内刷新波
        else if (!m_param->rdLevel)
        {
            /* In RD Level 0/1, copy source pixels into the reconstructed block so
             * they are available for intra predictions */
            m_modeDepth[0].fencYuv.copyToPicYuv(*m_frame->m_reconPic, ctu.m_cuAddr, 0);

            compressInterCU_rd0_4(ctu, cuGeom, qp);

            /* generate residual for entire CTU at once and copy to reconPic */
            encodeResidue(ctu, cuGeom);
        }// 如果开启了分布式分析
        else if (m_param->bDistributeModeAnalysis && m_param->rdLevel >= 2)
            compressInterCU_dist(ctu, cuGeom, qp);
        else if (m_param->rdLevel <= 4)
            compressInterCU_rd0_4(ctu, cuGeom, qp);
        else
            compressInterCU_rd5_6(ctu, cuGeom, qp);
    }

    if (m_param->bEnableRdRefine || m_param->bOptCUDeltaQP)
        qprdRefine(ctu, cuGeom, qp, qp);// CTU 级别的qp rd 过程

    if (m_param->csvLogLevel >= 2)
        collectPUStatistics(ctu, cuGeom);

    return *m_modeDepth[0].bestMode;
}

void Analysis::collectPUStatistics(const CUData& ctu, const CUGeom& cuGeom)
{
    uint8_t depth = 0;
    uint8_t partSize = 0;
    for (uint32_t absPartIdx = 0; absPartIdx < ctu.m_numPartitions; absPartIdx += ctu.m_numPartitions >> (depth * 2))
    {
        depth = ctu.m_cuDepth[absPartIdx];
        partSize = ctu.m_partSize[absPartIdx];
        uint32_t numPU = nbPartsTable[(int)partSize];
        int shift = 2 * (m_param->maxCUDepth + 1 - depth);
        for (uint32_t puIdx = 0; puIdx < numPU; puIdx++)
        {
            PredictionUnit pu(ctu, cuGeom, puIdx);
            int puabsPartIdx = ctu.getPUOffset(puIdx, absPartIdx);
            int mode = 1;
            if (ctu.m_partSize[puabsPartIdx + absPartIdx] == SIZE_Nx2N || ctu.m_partSize[puabsPartIdx + absPartIdx] == SIZE_2NxN)
                mode = 2;
            else if (ctu.m_partSize[puabsPartIdx + absPartIdx] == SIZE_2NxnU || ctu.m_partSize[puabsPartIdx + absPartIdx] == SIZE_2NxnD || ctu.m_partSize[puabsPartIdx + absPartIdx] == SIZE_nLx2N || ctu.m_partSize[puabsPartIdx + absPartIdx] == SIZE_nRx2N)
                 mode = 3;
            if (ctu.m_predMode[puabsPartIdx + absPartIdx] == MODE_SKIP)
            {
                ctu.m_encData->m_frameStats.cntSkipPu[depth] += 1ULL << shift;
                ctu.m_encData->m_frameStats.totalPu[depth] += 1ULL << shift;
            }
            else if (ctu.m_predMode[puabsPartIdx + absPartIdx] == MODE_INTRA)
            {
                if (ctu.m_partSize[puabsPartIdx + absPartIdx] == SIZE_NxN)
                {
                    ctu.m_encData->m_frameStats.cnt4x4++;
                    ctu.m_encData->m_frameStats.totalPu[4]++;
                }
                else
                {
                    ctu.m_encData->m_frameStats.cntIntraPu[depth] += 1ULL << shift;
                    ctu.m_encData->m_frameStats.totalPu[depth] += 1ULL << shift;
                }
            }
            else if (mode == 3)
            {
                ctu.m_encData->m_frameStats.cntAmp[depth] += 1ULL << shift;
                ctu.m_encData->m_frameStats.totalPu[depth] += 1ULL << shift;
                break;
            }
            else
            {
                if (ctu.m_mergeFlag[puabsPartIdx + absPartIdx])
                    ctu.m_encData->m_frameStats.cntMergePu[depth][ctu.m_partSize[puabsPartIdx + absPartIdx]] += (1 << shift) / mode;
                else
                    ctu.m_encData->m_frameStats.cntInterPu[depth][ctu.m_partSize[puabsPartIdx + absPartIdx]] += (1 << shift) / mode;

                ctu.m_encData->m_frameStats.totalPu[depth] += (1 << shift) / mode;
            }
        }
    }
}
// 预测当前tu的tudepth
int32_t Analysis::loadTUDepth(CUGeom cuGeom, CUData parentCTU)
{
    float predDepth = 0;
    CUData* neighbourCU;
    uint8_t count = 0;
    int32_t maxTUDepth = -1;
    // list0 的第0个参考帧同位置ctu
    neighbourCU = &m_slice->m_refFrameList[0][0]->m_encData->m_picCTU[parentCTU.m_cuAddr];
    // 这里cuGeom.geomRecurId 取值范围为 0～84
    // 但是m_refTuDepth[21]// 0 表示32x32 的tu depth  1:为16x16的tudepth 4:为8x8 de tudepth
    predDepth += neighbourCU->m_refTuDepth[cuGeom.geomRecurId];
    count++;
    if (m_slice->isInterB())
    {
        // list1 的第0个参考帧同位置ctu
        neighbourCU = &m_slice->m_refFrameList[1][0]->m_encData->m_picCTU[parentCTU.m_cuAddr];
        predDepth += neighbourCU->m_refTuDepth[cuGeom.geomRecurId];
        count++;
    }
    if (parentCTU.m_cuAbove)
    {
        predDepth += parentCTU.m_cuAbove->m_refTuDepth[cuGeom.geomRecurId];
        count++;
        if (parentCTU.m_cuAboveLeft)
        {
            predDepth += parentCTU.m_cuAboveLeft->m_refTuDepth[cuGeom.geomRecurId];
            count++;
        }
        if (parentCTU.m_cuAboveRight)
        {
            predDepth += parentCTU.m_cuAboveRight->m_refTuDepth[cuGeom.geomRecurId];
            count++;
        }
    }
    if (parentCTU.m_cuLeft)
    {
        predDepth += parentCTU.m_cuLeft->m_refTuDepth[cuGeom.geomRecurId];
        count++;
    }
    predDepth /= count;

    if (predDepth == 0)
        maxTUDepth = 0;
    else if (predDepth < 1)
        maxTUDepth = 1;
    else if (predDepth >= 1 && predDepth <= 1.5)
        maxTUDepth = 2;
    else if (predDepth > 1.5 && predDepth <= 2.5)
        maxTUDepth = 3;
    else
        maxTUDepth = -1;

    return maxTUDepth;
}

void Analysis::tryLossless(const CUGeom& cuGeom)
{
    ModeDepth& md = m_modeDepth[cuGeom.depth];

    if (!md.bestMode->distortion)
        /* already lossless */
        return;
    else if (md.bestMode->cu.isIntra(0))
    {
        md.pred[PRED_LOSSLESS].initCosts();
        md.pred[PRED_LOSSLESS].cu.initLosslessCU(md.bestMode->cu, cuGeom);
        PartSize size = (PartSize)md.pred[PRED_LOSSLESS].cu.m_partSize[0];
        checkIntra(md.pred[PRED_LOSSLESS], cuGeom, size);
        checkBestMode(md.pred[PRED_LOSSLESS], cuGeom.depth);
    }
    else
    {
        md.pred[PRED_LOSSLESS].initCosts();
        md.pred[PRED_LOSSLESS].cu.initLosslessCU(md.bestMode->cu, cuGeom);
        md.pred[PRED_LOSSLESS].predYuv.copyFromYuv(md.bestMode->predYuv);
        encodeResAndCalcRdInterCU(md.pred[PRED_LOSSLESS], cuGeom);
        checkBestMode(md.pred[PRED_LOSSLESS], cuGeom.depth);
    }
}

void Analysis::qprdRefine(const CUData& parentCTU, const CUGeom& cuGeom, int32_t qp, int32_t lqp)
{
    uint32_t depth = cuGeom.depth;
    ModeDepth& md = m_modeDepth[depth];
    md.bestMode = NULL;
// 如果CTU 中的保存的对应位置的 m_cuDepth与 当前cuGeom 的depth 一致,则表示 深度划分已经结束 
    bool bDecidedDepth = parentCTU.m_cuDepth[cuGeom.absPartIdx] == depth;

    int bestCUQP = qp;
    int lambdaQP = lqp;               //表示cuSize >= qg-size                                            cuSize == qg-size        
    bool doQPRefine = (bDecidedDepth && depth <= m_slice->m_pps->maxCuDQPDepth) || (!bDecidedDepth && depth == m_slice->m_pps->maxCuDQPDepth);

    if (doQPRefine)
    {
        uint64_t bestCUCost, origCUCost, cuCost, cuPrevCost;

        int cuIdx = (cuGeom.childOffset - 1) / 3;//通过当前cuGeom的childOffset 推导出其位于父cu的的几个subcu+1
        bestCUCost = origCUCost = cacheCost[cuIdx];

        int direction = m_param->bOptCUDeltaQP ? 1 : 2;

        for (int dir = direction; dir >= -direction; dir -= (direction * 2))
        {
            // dir 1 or -1  或者 2 or -2
            if (m_param->bOptCUDeltaQP && ((dir != 1) || ((qp + 3) >= (int32_t)parentCTU.m_meanQP)))
                break;

            int threshold = 1;
            int failure = 0;
            cuPrevCost = origCUCost;

            int modCUQP = qp + dir;
            while (modCUQP >= m_param->rc.qpMin && modCUQP <= QP_MAX_SPEC)
            {
                if (m_param->bOptCUDeltaQP && modCUQP > (int32_t)parentCTU.m_meanQP)
                    break;

                recodeCU(parentCTU, cuGeom, modCUQP, qp);
                cuCost = md.bestMode->rdCost;

                COPY2_IF_LT(bestCUCost, cuCost, bestCUQP, modCUQP);
                if (cuCost < cuPrevCost)
                    failure = 0;
                else
                    failure++;

                if (failure > threshold)
                    break;

                cuPrevCost = cuCost;
                modCUQP += dir;
            }
        }
        lambdaQP = bestCUQP;
    }

    recodeCU(parentCTU, cuGeom, bestCUQP, lambdaQP);

    /* Copy best data to encData CTU and recon */
    md.bestMode->cu.copyToPic(depth);
    md.bestMode->reconYuv.copyToPicYuv(*m_frame->m_reconPic, parentCTU.m_cuAddr, cuGeom.absPartIdx);
}

// intraCTU rdo  注意此函数包含递归调用
// 第一个参数 parentCTU 永远为 CTU 根节点 
// cuGeom 随着递归 depth ++
uint64_t Analysis::compressIntraCU(const CUData& parentCTU, const CUGeom& cuGeom, int32_t qp)
{
    uint32_t depth = cuGeom.depth;// 当前cu的depth
    ModeDepth& md = m_modeDepth[depth];//在当前depth下的模式信息 预测数据、原始YUV和best mode
    md.bestMode = NULL;// 最佳模式, 后续checkBestMode会进行检查

    bool mightSplit = !(cuGeom.flags & CUGeom::LEAF);// 0: 叶子结点，不能split 1:  非叶子结点可以split
    // 当前CU不是CTU的叶子节点，设置mightSplit为True，继续分裂
    // 后续根据mightSplit判断是否将split flag添加到RD cost中
    bool mightNotSplit = !(cuGeom.flags & CUGeom::SPLIT_MANDATORY);// 0:强制split了 1:可以不split
    // 注意有递归调用这个判断，是否已经决策了，一开始进来此处为0
    bool bAlreadyDecided = m_param->intraRefine != 4 && parentCTU.m_lumaIntraDir[cuGeom.absPartIdx] != (uint8_t)ALL_IDX;
    // 注意: 一开始 64x64 ctu depth 为0 时 是 bDecidedDepth 为真
    bool bDecidedDepth = m_param->intraRefine != 4 && parentCTU.m_cuDepth[cuGeom.absPartIdx] == depth;
    int split = 0;
    if (m_param->intraRefine && m_param->intraRefine != 4)
    {
        split = 0;
        if (cuGeom.log2CUSize == (uint32_t)(g_log2Size[m_param->minCUSize]) && !bDecidedDepth)
            bAlreadyDecided = false;
    }

    if (bAlreadyDecided)//当存在可用的模式集合时
    {
        if (bDecidedDepth && mightNotSplit)
        {
            Mode& mode = md.pred[0];//初始化模式为PRED_MERGE
            md.bestMode = &mode;
            mode.cu.initSubCU(parentCTU, cuGeom, qp);
            bool reuseModes = !((m_param->intraRefine == 3) ||
                                (m_param->intraRefine == 2 && parentCTU.m_lumaIntraDir[cuGeom.absPartIdx] > DC_IDX));
            if (reuseModes)
            {
                memcpy(mode.cu.m_lumaIntraDir, parentCTU.m_lumaIntraDir + cuGeom.absPartIdx, cuGeom.numPartitions);
                memcpy(mode.cu.m_chromaIntraDir, parentCTU.m_chromaIntraDir + cuGeom.absPartIdx, cuGeom.numPartitions);
            }
            //对可用的帧内模式进行完整的RD search
            checkIntra(mode, cuGeom, (PartSize)parentCTU.m_partSize[cuGeom.absPartIdx]);

            if (m_bTryLossless)
                tryLossless(cuGeom);

            if (mightSplit)//通过mightSplit标识位，判断是否需要添加split flag的率失真。
                addSplitFlagCost(*md.bestMode, cuGeom.depth);
        }
    }
    //当不存在可用的模式集合时
    else if (cuGeom.log2CUSize != MAX_LOG2_CU_SIZE && mightNotSplit)
    {// 当前cu 不为lcu,且可以不用分割 如果是64x64 则intra 一定要分割，如果32x32了 则可以不用分割了
        md.pred[PRED_INTRA].cu.initSubCU(parentCTU, cuGeom, qp);
        checkIntra(md.pred[PRED_INTRA], cuGeom, SIZE_2Nx2N);// 预测数据保存在 md.pred[PRED_INTRA]
        checkBestMode(md.pred[PRED_INTRA], depth); //与bestmode 比较 如果优 则替换

        // 如果当前cu 大小是 8x8,并且允许4x4的tu,这时除了尝试8x8 intra 尝试 4x4intra
        // hevc 中规定只有在cu为允许的最小cu时，才能有part_mode 为 NxN
        if (cuGeom.log2CUSize == 3 && m_slice->m_sps->quadtreeTULog2MinSize < 3)
        {   /* 4x4 intra PU blocks for 8x8 CU */
            md.pred[PRED_INTRA_NxN].cu.initSubCU(parentCTU, cuGeom, qp);
            checkIntra(md.pred[PRED_INTRA_NxN], cuGeom, SIZE_NxN); // 预测数据保存在 md.pred[PRED_INTRA_NxN]
            checkBestMode(md.pred[PRED_INTRA_NxN], depth);
        }

        if (m_bTryLossless)
            tryLossless(cuGeom);

        if (mightSplit)//判断是否将split flag添加到RD cost
            addSplitFlagCost(*md.bestMode, cuGeom.depth);
    }

    // stop recursion if we reach the depth of previous analysis decision
    mightSplit &= !(bAlreadyDecided && bDecidedDepth) || split;
//注：split为True，是因为一般的想法是尽量多试一试；
//bAlreadyDecided为False ，是因为决策模式没有确定， 也要多试一试；
//bDecidedDepth为False，是因为此时的父CU可能位于边界，需要进一步划分。

    if (mightSplit)//因为 通常来说第一次进来从这里入口 x265 禁止了cu64x64 的intra块
    {
        Mode* splitPred = &md.pred[PRED_SPLIT];// 所有数据都得保存在当前depth下的md.pred[PRED_SPLIT]，由四个子CU的返回结果拼接而成
        splitPred->initCosts();
        CUData* splitCU = &splitPred->cu;
        splitCU->initSubCU(parentCTU, cuGeom, qp);

        uint32_t nextDepth = depth + 1;// 深度加1
        ModeDepth& nd = m_modeDepth[nextDepth];// nd是md的下一层深度 next_mode_depth的缩写
        //其中md->bestMode保存最佳模式，而md.pred[...]完成对不同模式的RD search。
        invalidateContexts(nextDepth);
        Entropy* nextContext = &m_rqt[depth].cur;// RQT data 保存了上下文信息
        int32_t nextQP = qp;
        uint64_t curCost = 0;// 当前子CU的累计率失真损失，用于判断是否应该提前终止
        int skipSplitCheck = 0; // 是否提前终止的标识位
        // split 分成4个cu check
        for (uint32_t subPartIdx = 0; subPartIdx < 4; subPartIdx++)
        {
            //在一个64*64CTU中的四叉树表达共有85种（1+4+16+64），在x265中一个CU的四叉树表达抽象成了CUGeom结构体：
            //在底层实现中，85个CUGeom连续分布
            const CUGeom& childGeom = *(&cuGeom + cuGeom.childOffset + subPartIdx);
            //子CU的CUGeom的位置=父CU的CUGeom地址 + 父CU的第一个子CU的偏置+当前子CU的编号。
            if (childGeom.flags & CUGeom::PRESENT)
            {// 该cu 存在
                //copy yuv to nd.fencYuv
                //将64x64的CTU的一部分YUV data复制到nd.fencYuv
                m_modeDepth[0].fencYuv.copyPartToYuv(nd.fencYuv, childGeom.absPartIdx);
                //加载RQT数据，其中包含CABAC context，在四叉树递归过程中，程序利用其跟踪每层的深度和残差和重建数据的临时缓存区；
                m_rqt[nextDepth].cur.load(*nextContext);

                if (m_slice->m_pps->bUseDQP && nextDepth <= m_slice->m_pps->maxCuDQPDepth)
                    nextQP = setLambdaFromQP(parentCTU, calculateQpforCuSize(parentCTU, childGeom));

                if (m_param->bEnableSplitRdSkip)
                {// 看起来可以加速部分intracu analysis
                    curCost += compressIntraCU(parentCTU, childGeom, nextQP);//递归调用
                    //利用curCost判断子CU的累计率失真是否大于划分前的Best mode的率失真损失，
                    //如果大于，则说明划分之后的结果肯定更差劲，所以设置skipSplitCheck为True，并立即跳出循环
                    if (m_modeDepth[depth].bestMode && curCost > m_modeDepth[depth].bestMode->rdCost)
                    {   //如果slipt下的cost 累加超过了 非slipt下的cost了 则可以break掉for循环了
                        skipSplitCheck = 1;
                        break;
                    }
                }
                else
                    compressIntraCU(parentCTU, childGeom, nextQP);// 递归调用

                // Save best CU and pred data for this sub CU
                //将子CU最佳模式的预测结果（m_modeDepth[nextDepth].bestMode->cu）
                //保存到父CU所在深度的PRED_SPLIT中；（m_modeDepth[Depth].pred[PRED_SPLIT]->cu）；
                //将子CU的率失真损失添加到父CU所在深度的PRED_SPLIT中，
                //并且将子CU最佳模式的重建YUV数据保存到父CU所在深度的PRED_SPLIT中
                splitCU->copyPartFrom(nd.bestMode->cu, childGeom, subPartIdx);
                splitPred->addSubCosts(*nd.bestMode);
                nd.bestMode->reconYuv.copyToPartYuv(splitPred->reconYuv, childGeom.numPartitions * subPartIdx);
                nextContext = &nd.bestMode->contexts;// 更新nextContext 为当前子cu的bestMode对应的contexts
            }
            else
            {// 改cu 不在pic 内部
                /* record the depth of this non-present sub-CU */
                splitCU->setEmptyPart(childGeom, subPartIdx);

                /* Set depth of non-present CU to 0 to ensure that correct CU is fetched as reference to code deltaQP */
                // 如果存在可用的模式集合，则将不存在的CU深度设置为0，以确保获取正确的CU作为编码delta QP的参考
                if (bAlreadyDecided)
                    memset(parentCTU.m_cuDepth + childGeom.absPartIdx, 0, childGeom.numPartitions);
            }
        }
        //如果不需要跳过对slpit的check，则进行check
        if (!skipSplitCheck)
        {
            nextContext->store(splitPred->contexts);
            if (mightNotSplit)//可能不需要分割，但是却分割了需要加上flag 的cost,再进行update Mode Cost
                addSplitFlagCost(*splitPred, cuGeom.depth);
            else //需要强制分割,不需要code flag bits,直接进行update mode cost 就可以
                updateModeCost(*splitPred);

            checkDQPForSplitPred(*splitPred, cuGeom);
            checkBestMode(*splitPred, depth);
        }
    }
    
    if (m_param->bEnableRdRefine && depth <= m_slice->m_pps->maxCuDQPDepth)
    {
        int cuIdx = (cuGeom.childOffset - 1) / 3;
        cacheCost[cuIdx] = md.bestMode->rdCost;
    }

    if ((m_limitTU & S265_TU_LIMIT_NEIGH) && cuGeom.log2CUSize >= 4)
    {
        CUData* ctu = md.bestMode->cu.m_encData->getPicCTU(parentCTU.m_cuAddr);
        int8_t maxTUDepth = -1;
        /// 获取当前父CU的最大TU深度
        for (uint32_t i = 0; i < cuGeom.numPartitions; i++)
            maxTUDepth = S265_MAX(maxTUDepth, md.bestMode->cu.m_tuDepth[i]);
        ctu->m_refTuDepth[cuGeom.geomRecurId] = maxTUDepth;
    }

    /* Copy best data to encData CTU and recon */
    md.bestMode->cu.copyToPic(depth);
    // 如果该CU没有划分，则将其重建YUV数据保存到帧级别的重建图像m_frame->m_reconPic中
    if (md.bestMode != &md.pred[PRED_SPLIT])
        md.bestMode->reconYuv.copyToPicYuv(*m_frame->m_reconPic, parentCTU.m_cuAddr, cuGeom.absPartIdx);
    // 返回最佳预测的率失真损失
    return md.bestMode->rdCost;
}

void Analysis::PMODE::processTasks(int workerThreadId)
{
#if DETAILED_CU_STATS
    int fe = master.m_modeDepth[cuGeom.depth].pred[PRED_2Nx2N].cu.m_encData->m_frameEncoderID;
    master.m_stats[fe].countPModeTasks++;
    ScopedElapsedTime pmodeTime(master.m_stats[fe].pmodeTime);
#endif
    ProfileScopeEvent(pmode);
    master.processPmode(*this, master.m_tld[workerThreadId].analysis);
}

/* process pmode jobs until none remain; may be called by the master thread or by
 * a bonded peer (slave) thread via pmodeTasks() */
void Analysis::processPmode(PMODE& pmode, Analysis& slave)
{
    /* acquire a mode task, else exit early */
    int task;
    pmode.m_lock.acquire();
    if (pmode.m_jobTotal > pmode.m_jobAcquired)
    {
        task = pmode.m_jobAcquired++;
        pmode.m_lock.release();
    }
    else
    {
        pmode.m_lock.release();
        return;
    }

    ModeDepth& md = m_modeDepth[pmode.cuGeom.depth];

    /* setup slave Analysis */
    if (&slave != this)
    {
        slave.m_slice = m_slice;
        slave.m_frame = m_frame;
        slave.m_param = m_param;
        slave.m_bChromaSa8d = m_param->rdLevel >= 3;
        slave.setLambdaFromQP(md.pred[PRED_2Nx2N].cu, m_rdCost.m_qp);
        slave.invalidateContexts(0);
        slave.m_rqt[pmode.cuGeom.depth].cur.load(m_rqt[pmode.cuGeom.depth].cur);
    }

    /* perform Mode task, repeat until no more work is available */
    do
    {
        uint32_t refMasks[2] = { 0, 0 };

        if (m_param->rdLevel <= 4)
        {
            switch (pmode.modes[task])
            {
            case PRED_INTRA:
                slave.checkIntraInInter(md.pred[PRED_INTRA], pmode.cuGeom);
                if (m_param->rdLevel > 2)
                    slave.encodeIntraInInter(md.pred[PRED_INTRA], pmode.cuGeom);
                break;

            case PRED_2Nx2N:
                refMasks[0] = m_splitRefIdx[0] | m_splitRefIdx[1] | m_splitRefIdx[2] | m_splitRefIdx[3];

                slave.checkInter_rd0_4(md.pred[PRED_2Nx2N], pmode.cuGeom, SIZE_2Nx2N, refMasks);
                if (m_slice->m_sliceType == B_SLICE)
                    slave.checkBidir2Nx2N(md.pred[PRED_2Nx2N], md.pred[PRED_BIDIR], pmode.cuGeom);
                break;

            case PRED_Nx2N:
                refMasks[0] = m_splitRefIdx[0] | m_splitRefIdx[2]; /* left */
                refMasks[1] = m_splitRefIdx[1] | m_splitRefIdx[3]; /* right */

                slave.checkInter_rd0_4(md.pred[PRED_Nx2N], pmode.cuGeom, SIZE_Nx2N, refMasks);
                break;

            case PRED_2NxN:
                refMasks[0] = m_splitRefIdx[0] | m_splitRefIdx[1]; /* top */
                refMasks[1] = m_splitRefIdx[2] | m_splitRefIdx[3]; /* bot */

                slave.checkInter_rd0_4(md.pred[PRED_2NxN], pmode.cuGeom, SIZE_2NxN, refMasks);
                break;

            case PRED_2NxnU:
                refMasks[0] = m_splitRefIdx[0] | m_splitRefIdx[1]; /* 25% top */
                refMasks[1] = m_splitRefIdx[0] | m_splitRefIdx[1] | m_splitRefIdx[2] | m_splitRefIdx[3]; /* 75% bot */

                slave.checkInter_rd0_4(md.pred[PRED_2NxnU], pmode.cuGeom, SIZE_2NxnU, refMasks);
                break;

            case PRED_2NxnD:
                refMasks[0] = m_splitRefIdx[0] | m_splitRefIdx[1] | m_splitRefIdx[2] | m_splitRefIdx[3]; /* 75% top */
                refMasks[1] = m_splitRefIdx[2] | m_splitRefIdx[3]; /* 25% bot */

                slave.checkInter_rd0_4(md.pred[PRED_2NxnD], pmode.cuGeom, SIZE_2NxnD, refMasks);
                break;

            case PRED_nLx2N:
                refMasks[0] = m_splitRefIdx[0] | m_splitRefIdx[2]; /* 25% left */
                refMasks[1] = m_splitRefIdx[0] | m_splitRefIdx[1] | m_splitRefIdx[2] | m_splitRefIdx[3]; /* 75% right */

                slave.checkInter_rd0_4(md.pred[PRED_nLx2N], pmode.cuGeom, SIZE_nLx2N, refMasks);
                break;

            case PRED_nRx2N:
                refMasks[0] = m_splitRefIdx[0] | m_splitRefIdx[1] | m_splitRefIdx[2] | m_splitRefIdx[3]; /* 75% left */
                refMasks[1] = m_splitRefIdx[1] | m_splitRefIdx[3]; /* 25% right */

                slave.checkInter_rd0_4(md.pred[PRED_nRx2N], pmode.cuGeom, SIZE_nRx2N, refMasks);
                break;

            default:
                S265_CHECK(0, "invalid job ID for parallel mode analysis\n");
                break;
            }
        }
        else
        {
            switch (pmode.modes[task])
            {
            case PRED_INTRA:
                slave.checkIntra(md.pred[PRED_INTRA], pmode.cuGeom, SIZE_2Nx2N);
                if (pmode.cuGeom.log2CUSize == 3 && m_slice->m_sps->quadtreeTULog2MinSize < 3)
                    slave.checkIntra(md.pred[PRED_INTRA_NxN], pmode.cuGeom, SIZE_NxN);
                break;

            case PRED_2Nx2N:
                refMasks[0] = m_splitRefIdx[0] | m_splitRefIdx[1] | m_splitRefIdx[2] | m_splitRefIdx[3];

                slave.checkInter_rd5_6(md.pred[PRED_2Nx2N], pmode.cuGeom, SIZE_2Nx2N, refMasks);
                md.pred[PRED_BIDIR].rdCost = MAX_INT64;
                if (m_slice->m_sliceType == B_SLICE)
                {
                    slave.checkBidir2Nx2N(md.pred[PRED_2Nx2N], md.pred[PRED_BIDIR], pmode.cuGeom);
                    if (md.pred[PRED_BIDIR].sa8dCost < MAX_INT64)
                        slave.encodeResAndCalcRdInterCU(md.pred[PRED_BIDIR], pmode.cuGeom);
                }
                break;

            case PRED_Nx2N:
                refMasks[0] = m_splitRefIdx[0] | m_splitRefIdx[2]; /* left */
                refMasks[1] = m_splitRefIdx[1] | m_splitRefIdx[3]; /* right */

                slave.checkInter_rd5_6(md.pred[PRED_Nx2N], pmode.cuGeom, SIZE_Nx2N, refMasks);
                break;

            case PRED_2NxN:
                refMasks[0] = m_splitRefIdx[0] | m_splitRefIdx[1]; /* top */
                refMasks[1] = m_splitRefIdx[2] | m_splitRefIdx[3]; /* bot */

                slave.checkInter_rd5_6(md.pred[PRED_2NxN], pmode.cuGeom, SIZE_2NxN, refMasks);
                break;

            case PRED_2NxnU:
                refMasks[0] = m_splitRefIdx[0] | m_splitRefIdx[1]; /* 25% top */
                refMasks[1] = m_splitRefIdx[0] | m_splitRefIdx[1] | m_splitRefIdx[2] | m_splitRefIdx[3]; /* 75% bot */

                slave.checkInter_rd5_6(md.pred[PRED_2NxnU], pmode.cuGeom, SIZE_2NxnU, refMasks);
                break;

            case PRED_2NxnD:
                refMasks[0] = m_splitRefIdx[0] | m_splitRefIdx[1] | m_splitRefIdx[2] | m_splitRefIdx[3]; /* 75% top */
                refMasks[1] = m_splitRefIdx[2] | m_splitRefIdx[3]; /* 25% bot */
                slave.checkInter_rd5_6(md.pred[PRED_2NxnD], pmode.cuGeom, SIZE_2NxnD, refMasks);
                break;

            case PRED_nLx2N:
                refMasks[0] = m_splitRefIdx[0] | m_splitRefIdx[2]; /* 25% left */
                refMasks[1] = m_splitRefIdx[0] | m_splitRefIdx[1] | m_splitRefIdx[2] | m_splitRefIdx[3]; /* 75% right */

                slave.checkInter_rd5_6(md.pred[PRED_nLx2N], pmode.cuGeom, SIZE_nLx2N, refMasks);
                break;

            case PRED_nRx2N:
                refMasks[0] = m_splitRefIdx[0] | m_splitRefIdx[1] | m_splitRefIdx[2] | m_splitRefIdx[3]; /* 75% left */
                refMasks[1] = m_splitRefIdx[1] | m_splitRefIdx[3]; /* 25% right */
                slave.checkInter_rd5_6(md.pred[PRED_nRx2N], pmode.cuGeom, SIZE_nRx2N, refMasks);
                break;

            default:
                S265_CHECK(0, "invalid job ID for parallel mode analysis\n");
                break;
            }
        }

        task = -1;
        pmode.m_lock.acquire();
        if (pmode.m_jobTotal > pmode.m_jobAcquired)
            task = pmode.m_jobAcquired++;
        pmode.m_lock.release();
    }
    while (task >= 0);
}

uint32_t Analysis::compressInterCU_dist(const CUData& parentCTU, const CUGeom& cuGeom, int32_t qp)
{
    uint32_t depth = cuGeom.depth;
    uint32_t cuAddr = parentCTU.m_cuAddr;
    ModeDepth& md = m_modeDepth[depth];
    md.bestMode = NULL;

    bool mightSplit = !(cuGeom.flags & CUGeom::LEAF);
    bool mightNotSplit = !(cuGeom.flags & CUGeom::SPLIT_MANDATORY);
    uint32_t minDepth = m_param->rdLevel <= 4 ? topSkipMinDepth(parentCTU, cuGeom) : 0;
    uint32_t splitRefs[4] = { 0, 0, 0, 0 };

    S265_CHECK(m_param->rdLevel >= 2, "compressInterCU_dist does not support RD 0 or 1\n");

    PMODE pmode(*this, cuGeom);

    if (mightNotSplit && depth >= minDepth)
    {
        /* Initialize all prediction CUs based on parentCTU */
        md.pred[PRED_MERGE].cu.initSubCU(parentCTU, cuGeom, qp);
        md.pred[PRED_SKIP].cu.initSubCU(parentCTU, cuGeom, qp);

        if (m_param->rdLevel <= 4)
            checkMerge2Nx2N_rd0_4(md.pred[PRED_SKIP], md.pred[PRED_MERGE], cuGeom);
        else
            checkMerge2Nx2N_rd5_6(md.pred[PRED_SKIP], md.pred[PRED_MERGE], cuGeom);
    }

    bool bNoSplit = false;
    bool splitIntra = true;
    if (md.bestMode)
    {
        bNoSplit = md.bestMode->cu.isSkipped(0);
        // 如果当前bestModecu 是 skip 则 跳过下面的 split 分析
        if (mightSplit && depth && depth >= minDepth && !bNoSplit && m_param->rdLevel <= 4)
            bNoSplit = recursionDepthCheck(parentCTU, cuGeom, *md.bestMode);
    }

    if (mightSplit && !bNoSplit)
    {//如果需要分析split, 则在当前modedepth下取出split mode 进行 splitPred分析
        Mode* splitPred = &md.pred[PRED_SPLIT];
        splitPred->initCosts();
        CUData* splitCU = &splitPred->cu;
        splitCU->initSubCU(parentCTU, cuGeom, qp);//当前 modedepth下 split mode下 cu的初始化

        uint32_t nextDepth = depth + 1;
        //使用nextDepth 对当前depth 下cu 划分为4 给subcu后的子cu 进行分析
        ModeDepth& nd = m_modeDepth[nextDepth];
        invalidateContexts(nextDepth);
        Entropy* nextContext = &m_rqt[depth].cur;
        int nextQP = qp;
        splitIntra = false;

        for (uint32_t subPartIdx = 0; subPartIdx < 4; subPartIdx++)
        {
            const CUGeom& childGeom = *(&cuGeom + cuGeom.childOffset + subPartIdx);
            if (childGeom.flags & CUGeom::PRESENT)
            {
                m_modeDepth[0].fencYuv.copyPartToYuv(nd.fencYuv, childGeom.absPartIdx);
                m_rqt[nextDepth].cur.load(*nextContext);
                // 如果 语训dqp，且当前子cu的大小等于qgsize，则需要求出子cu的delta_qp
                if (m_slice->m_pps->bUseDQP && nextDepth <= m_slice->m_pps->maxCuDQPDepth)
                    nextQP = setLambdaFromQP(parentCTU, calculateQpforCuSize(parentCTU, childGeom));

                splitRefs[subPartIdx] = compressInterCU_dist(parentCTU, childGeom, nextQP);

                // Save best CU and pred data for this sub CU
                splitIntra |= nd.bestMode->cu.isIntra(0);
                splitCU->copyPartFrom(nd.bestMode->cu, childGeom, subPartIdx);
                splitPred->addSubCosts(*nd.bestMode);

                nd.bestMode->reconYuv.copyToPartYuv(splitPred->reconYuv, childGeom.numPartitions * subPartIdx);
                nextContext = &nd.bestMode->contexts;
            }
            else
                splitCU->setEmptyPart(childGeom, subPartIdx);
        }
        nextContext->store(splitPred->contexts);

        if (mightNotSplit)
            addSplitFlagCost(*splitPred, cuGeom.depth);
        else
            updateModeCost(*splitPred);

        checkDQPForSplitPred(*splitPred, cuGeom);
    }

    if (mightNotSplit && depth >= minDepth)
    {
        int bTryAmp = m_slice->m_sps->maxAMPDepth > depth;
        int bTryIntra = (m_slice->m_sliceType != B_SLICE || m_param->bIntraInBFrames) && (!m_param->limitReferences || splitIntra) && (cuGeom.log2CUSize != MAX_LOG2_CU_SIZE);

        if (m_slice->m_pps->bUseDQP && depth <= m_slice->m_pps->maxCuDQPDepth && m_slice->m_pps->maxCuDQPDepth != 0)
            setLambdaFromQP(parentCTU, qp);

        if (bTryIntra)
        {
            md.pred[PRED_INTRA].cu.initSubCU(parentCTU, cuGeom, qp);
            if (cuGeom.log2CUSize == 3 && m_slice->m_sps->quadtreeTULog2MinSize < 3 && m_param->rdLevel >= 5)
                md.pred[PRED_INTRA_NxN].cu.initSubCU(parentCTU, cuGeom, qp);
            pmode.modes[pmode.m_jobTotal++] = PRED_INTRA;
        }
        md.pred[PRED_2Nx2N].cu.initSubCU(parentCTU, cuGeom, qp); pmode.modes[pmode.m_jobTotal++] = PRED_2Nx2N;
        md.pred[PRED_BIDIR].cu.initSubCU(parentCTU, cuGeom, qp);
        if (m_param->bEnableRectInter)
        {
            md.pred[PRED_2NxN].cu.initSubCU(parentCTU, cuGeom, qp); pmode.modes[pmode.m_jobTotal++] = PRED_2NxN;
            md.pred[PRED_Nx2N].cu.initSubCU(parentCTU, cuGeom, qp); pmode.modes[pmode.m_jobTotal++] = PRED_Nx2N;
        }
        if (bTryAmp)
        {
            md.pred[PRED_2NxnU].cu.initSubCU(parentCTU, cuGeom, qp); pmode.modes[pmode.m_jobTotal++] = PRED_2NxnU;
            md.pred[PRED_2NxnD].cu.initSubCU(parentCTU, cuGeom, qp); pmode.modes[pmode.m_jobTotal++] = PRED_2NxnD;
            md.pred[PRED_nLx2N].cu.initSubCU(parentCTU, cuGeom, qp); pmode.modes[pmode.m_jobTotal++] = PRED_nLx2N;
            md.pred[PRED_nRx2N].cu.initSubCU(parentCTU, cuGeom, qp); pmode.modes[pmode.m_jobTotal++] = PRED_nRx2N;
        }

        m_splitRefIdx[0] = splitRefs[0]; m_splitRefIdx[1] = splitRefs[1]; m_splitRefIdx[2] = splitRefs[2]; m_splitRefIdx[3] = splitRefs[3];

        //唤醒线程池里面的多个线程并发执行processTask 从而执行 processPmode
        pmode.tryBondPeers(*m_frame->m_encData->m_jobProvider, pmode.m_jobTotal);

        /* participate in processing jobs, until all are distributed */
        processPmode(pmode, *this);//本线程直接调用,里面会等待所有任务全部完成

        /* the master worker thread (this one) does merge analysis. By doing
         * merge after all the other jobs are at least started, we usually avoid
         * blocking on another thread */

        if (m_param->rdLevel <= 4)
        {
            {
                ProfileCUScope(parentCTU, pmodeBlockTime, countPModeMasters);
                pmode.waitForExit();
            }

            /* select best inter mode based on sa8d cost */
            Mode *bestInter = &md.pred[PRED_2Nx2N];

            if (m_param->bEnableRectInter)
            {
                if (md.pred[PRED_Nx2N].sa8dCost < bestInter->sa8dCost)
                    bestInter = &md.pred[PRED_Nx2N];
                if (md.pred[PRED_2NxN].sa8dCost < bestInter->sa8dCost)
                    bestInter = &md.pred[PRED_2NxN];
            }

            if (bTryAmp)
            {
                if (md.pred[PRED_2NxnU].sa8dCost < bestInter->sa8dCost)
                    bestInter = &md.pred[PRED_2NxnU];
                if (md.pred[PRED_2NxnD].sa8dCost < bestInter->sa8dCost)
                    bestInter = &md.pred[PRED_2NxnD];
                if (md.pred[PRED_nLx2N].sa8dCost < bestInter->sa8dCost)
                    bestInter = &md.pred[PRED_nLx2N];
                if (md.pred[PRED_nRx2N].sa8dCost < bestInter->sa8dCost)
                    bestInter = &md.pred[PRED_nRx2N];
            }

            if (m_param->rdLevel > 2)
            {
                /* RD selection between merge, inter, bidir and intra */
                if (!m_bChromaSa8d && (m_csp != S265_CSP_I400)) /* When m_bChromaSa8d is enabled, chroma MC has already been done */
                {
                    uint32_t numPU = bestInter->cu.getNumPartInter(0);
                    for (uint32_t puIdx = 0; puIdx < numPU; puIdx++)
                    {
                        PredictionUnit pu(bestInter->cu, cuGeom, puIdx);
                        motionCompensation(bestInter->cu, pu, bestInter->predYuv, false, true);
                    }
                }
                encodeResAndCalcRdInterCU(*bestInter, cuGeom);
                checkBestMode(*bestInter, depth);

                /* If BIDIR is available and within 17/16 of best inter option, choose by RDO */
                if (m_slice->m_sliceType == B_SLICE && md.pred[PRED_BIDIR].sa8dCost != MAX_INT64 &&
                    md.pred[PRED_BIDIR].sa8dCost * 16 <= bestInter->sa8dCost * 17)
                {
                    encodeResAndCalcRdInterCU(md.pred[PRED_BIDIR], cuGeom);
                    checkBestMode(md.pred[PRED_BIDIR], depth);
                }

                if (bTryIntra)
                    checkBestMode(md.pred[PRED_INTRA], depth);
            }
            else /* m_param->rdLevel == 2 */
            {
                if (!md.bestMode || bestInter->sa8dCost < md.bestMode->sa8dCost)
                    md.bestMode = bestInter;

                if (m_slice->m_sliceType == B_SLICE && md.pred[PRED_BIDIR].sa8dCost < md.bestMode->sa8dCost)
                    md.bestMode = &md.pred[PRED_BIDIR];

                if (bTryIntra && md.pred[PRED_INTRA].sa8dCost < md.bestMode->sa8dCost)
                {
                    md.bestMode = &md.pred[PRED_INTRA];
                    encodeIntraInInter(*md.bestMode, cuGeom);
                }
                else if (!md.bestMode->cu.m_mergeFlag[0])
                {
                    /* finally code the best mode selected from SA8D costs */
                    uint32_t numPU = md.bestMode->cu.getNumPartInter(0);
                    for (uint32_t puIdx = 0; puIdx < numPU; puIdx++)
                    {
                        PredictionUnit pu(md.bestMode->cu, cuGeom, puIdx);
                        motionCompensation(md.bestMode->cu, pu, md.bestMode->predYuv, false, true);
                    }
                    encodeResAndCalcRdInterCU(*md.bestMode, cuGeom);
                }
            }
        }
        else
        {
            {
                ProfileCUScope(parentCTU, pmodeBlockTime, countPModeMasters);
                pmode.waitForExit();
            }

            checkBestMode(md.pred[PRED_2Nx2N], depth);
            if (m_slice->m_sliceType == B_SLICE && md.pred[PRED_BIDIR].sa8dCost < MAX_INT64)
                checkBestMode(md.pred[PRED_BIDIR], depth);

            if (m_param->bEnableRectInter)//矩形分割允许
            {
                checkBestMode(md.pred[PRED_Nx2N], depth);
                checkBestMode(md.pred[PRED_2NxN], depth);
            }

            if (bTryAmp)// 非对称分割 允许
            {
                checkBestMode(md.pred[PRED_2NxnU], depth);
                checkBestMode(md.pred[PRED_2NxnD], depth);
                checkBestMode(md.pred[PRED_nLx2N], depth);
                checkBestMode(md.pred[PRED_nRx2N], depth);
            }

            if (bTryIntra)
            {
                checkBestMode(md.pred[PRED_INTRA], depth);
                if (cuGeom.log2CUSize == 3 && m_slice->m_sps->quadtreeTULog2MinSize < 3)
                    checkBestMode(md.pred[PRED_INTRA_NxN], depth);
            }
        }

        if (m_bTryLossless)
            tryLossless(cuGeom);

        if (mightSplit)
            addSplitFlagCost(*md.bestMode, cuGeom.depth);
    }

    /* compare split RD cost against best cost */
    if (mightSplit && !bNoSplit)
        checkBestMode(md.pred[PRED_SPLIT], depth);

    /* determine which motion references the parent CU should search */
    uint32_t refMask;
    if (!(m_param->limitReferences & S265_REF_LIMIT_DEPTH))
        refMask = 0;
    else if (md.bestMode == &md.pred[PRED_SPLIT])
        refMask = splitRefs[0] | splitRefs[1] | splitRefs[2] | splitRefs[3];
    else
    {
        /* use best merge/inter mode, in case of intra use 2Nx2N inter references */
        CUData& cu = md.bestMode->cu.isIntra(0) ? md.pred[PRED_2Nx2N].cu : md.bestMode->cu;
        uint32_t numPU = cu.getNumPartInter(0);
        refMask = 0;
        for (uint32_t puIdx = 0, subPartIdx = 0; puIdx < numPU; puIdx++, subPartIdx += cu.getPUOffset(puIdx, 0))
            refMask |= cu.getBestRefIdx(subPartIdx);
    }

    if (mightNotSplit)
    {
        /* early-out statistics */
        FrameData& curEncData = *m_frame->m_encData;
        FrameData::RCStatCU& cuStat = curEncData.m_cuStat[parentCTU.m_cuAddr];
        uint64_t temp = cuStat.avgCost[depth] * cuStat.count[depth];
        cuStat.count[depth] += 1;
        cuStat.avgCost[depth] = (temp + md.bestMode->rdCost) / cuStat.count[depth];
    }

    /* Copy best data to encData CTU and recon */
    md.bestMode->cu.copyToPic(depth);
    md.bestMode->reconYuv.copyToPicYuv(*m_frame->m_reconPic, cuAddr, cuGeom.absPartIdx);

    return refMask;
}

SplitData Analysis::compressInterCU_rd0_4(const CUData& parentCTU, const CUGeom& cuGeom, int32_t qp)
{
    if (parentCTU.m_vbvAffected && calculateQpforCuSize(parentCTU, cuGeom, 1))
        return compressInterCU_rd5_6(parentCTU, cuGeom, qp);

    uint32_t depth = cuGeom.depth;
    uint32_t cuAddr = parentCTU.m_cuAddr;
    ModeDepth& md = m_modeDepth[depth];


    if (m_param->searchMethod == S265_SEA)
    {
        int numPredDir = m_slice->isInterP() ? 1 : 2;
        int offset = (int)(m_frame->m_reconPic->m_cuOffsetY[parentCTU.m_cuAddr] + m_frame->m_reconPic->m_buOffsetY[cuGeom.absPartIdx]);
        for (int list = 0; list < numPredDir; list++)
            for (int i = 0; i < m_frame->m_encData->m_slice->m_numRefIdx[list]; i++)
                for (int planes = 0; planes < INTEGRAL_PLANE_NUM; planes++)
                    m_modeDepth[depth].fencYuv.m_integral[list][i][planes] = m_frame->m_encData->m_slice->m_refFrameList[list][i]->m_encData->m_meIntegral[planes] + offset;
    }

    PicYuv& reconPic = *m_frame->m_reconPic;
    SplitData splitCUData;

    bool bNooffloading = true;

    if (bNooffloading)
    {
        md.bestMode = NULL;
        bool mightSplit = !(cuGeom.flags & CUGeom::LEAF);
        bool mightNotSplit = !(cuGeom.flags & CUGeom::SPLIT_MANDATORY);
        uint32_t minDepth = topSkipMinDepth(parentCTU, cuGeom);
        bool skipModes = false; /* Skip any remaining mode analyses at current depth */
        bool skipRecursion = false; /* Skip recursion */
        bool splitIntra = true;
        bool skipRectAmp = false;
        bool chooseMerge = false;

        if (m_evaluateInter)
        {
            if (m_refineLevel == 2)
            {
                if (parentCTU.m_predMode[cuGeom.absPartIdx] == MODE_SKIP)
                    skipModes = true;
                if (parentCTU.m_partSize[cuGeom.absPartIdx] == SIZE_2Nx2N)
                    skipRectAmp = true;
            }
            mightSplit &= false;
            minDepth = depth;
        }

        if ((m_limitTU & S265_TU_LIMIT_NEIGH) && cuGeom.log2CUSize >= 4)
            m_maxTUDepth = loadTUDepth(cuGeom, parentCTU);

        SplitData splitData[4];
        splitData[0].initSplitCUData();
        splitData[1].initSplitCUData();
        splitData[2].initSplitCUData();
        splitData[3].initSplitCUData();

        // avoid uninitialize value in below reference
        if (m_param->limitModes)
        {
            md.pred[PRED_2Nx2N].bestME[0][0].mvCost = 0; // L0
            md.pred[PRED_2Nx2N].bestME[0][1].mvCost = 0; // L1
            md.pred[PRED_2Nx2N].sa8dCost = 0;
        }


        /* Step 1. Evaluate Merge/Skip candidates for likely early-outs, if skip mode was not set above */
        if ((mightNotSplit && depth >= minDepth && !md.bestMode))
            /* TODO: Re-evaluate if analysis load/save still works */
        {
            /* Compute Merge Cost */
            md.pred[PRED_MERGE].cu.initSubCU(parentCTU, cuGeom, qp);
            md.pred[PRED_SKIP].cu.initSubCU(parentCTU, cuGeom, qp);
            checkMerge2Nx2N_rd0_4(md.pred[PRED_SKIP], md.pred[PRED_MERGE], cuGeom);
            if (m_param->rdLevel)
                skipModes = (m_param->bEnableEarlySkip || m_refineLevel == 2)
                && md.bestMode && md.bestMode->cu.isSkipped(0); // TODO: sa8d threshold per depth
        }
        if (md.bestMode && m_param->recursionSkipMode)
        {
            skipRecursion = md.bestMode->cu.isSkipped(0);
            if (mightSplit && !skipRecursion)
            {
                if (depth >= minDepth && m_param->recursionSkipMode == RDCOST_BASED_RSKIP)
                {
                    if (depth)
                        skipRecursion = recursionDepthCheck(parentCTU, cuGeom, *md.bestMode);
                    if (m_bHD && !skipRecursion && m_param->rdLevel == 2 && md.fencYuv.m_size != MAX_CU_SIZE)
                        skipRecursion = complexityCheckCU(*md.bestMode);
                }
                else if (cuGeom.log2CUSize >= MAX_LOG2_CU_SIZE - 1 && m_param->recursionSkipMode == EDGE_BASED_RSKIP)
                {
                    skipRecursion = complexityCheckCU(*md.bestMode);
                }

            }
        }
        /* Step 2. Evaluate each of the 4 split sub-blocks in series */
        if (mightSplit && !skipRecursion)
        {
            Mode* splitPred = &md.pred[PRED_SPLIT];
            splitPred->initCosts();
            CUData* splitCU = &splitPred->cu;
            splitCU->initSubCU(parentCTU, cuGeom, qp);

            uint32_t nextDepth = depth + 1;
            ModeDepth& nd = m_modeDepth[nextDepth];
            invalidateContexts(nextDepth);
            Entropy* nextContext = &m_rqt[depth].cur;
            int nextQP = qp;
            splitIntra = false;

            for (uint32_t subPartIdx = 0; subPartIdx < 4; subPartIdx++)
            {
                const CUGeom& childGeom = *(&cuGeom + cuGeom.childOffset + subPartIdx);
                if (childGeom.flags & CUGeom::PRESENT)
                {
                    m_modeDepth[0].fencYuv.copyPartToYuv(nd.fencYuv, childGeom.absPartIdx);
                    m_rqt[nextDepth].cur.load(*nextContext);

                    if (m_slice->m_pps->bUseDQP && nextDepth <= m_slice->m_pps->maxCuDQPDepth)
                        nextQP = setLambdaFromQP(parentCTU, calculateQpforCuSize(parentCTU, childGeom));

                    splitData[subPartIdx] = compressInterCU_rd0_4(parentCTU, childGeom, nextQP);

                    // Save best CU and pred data for this sub CU
                    splitIntra |= nd.bestMode->cu.isIntra(0);
                    splitCU->copyPartFrom(nd.bestMode->cu, childGeom, subPartIdx);
                    splitPred->addSubCosts(*nd.bestMode);

                    if (m_param->rdLevel)
                        nd.bestMode->reconYuv.copyToPartYuv(splitPred->reconYuv, childGeom.numPartitions * subPartIdx);
                    else
                        nd.bestMode->predYuv.copyToPartYuv(splitPred->predYuv, childGeom.numPartitions * subPartIdx);
                    if (m_param->rdLevel > 1)
                        nextContext = &nd.bestMode->contexts;
                }
                else
                    splitCU->setEmptyPart(childGeom, subPartIdx);
            }
            nextContext->store(splitPred->contexts);

            if (mightNotSplit)
                addSplitFlagCost(*splitPred, cuGeom.depth);
            else if (m_param->rdLevel > 1)
                updateModeCost(*splitPred);
            else
                splitPred->sa8dCost = m_rdCost.calcRdSADCost((uint32_t)splitPred->distortion, splitPred->sa8dBits);
        }
        /* Split CUs
         *   0  1
         *   2  3 */
        uint32_t allSplitRefs = splitData[0].splitRefs | splitData[1].splitRefs | splitData[2].splitRefs | splitData[3].splitRefs;
        /* Step 3. Evaluate ME (2Nx2N, rect, amp) and intra modes at current depth */
        if (mightNotSplit && (depth >= minDepth))
        {
            if (m_slice->m_pps->bUseDQP && depth <= m_slice->m_pps->maxCuDQPDepth && m_slice->m_pps->maxCuDQPDepth != 0)
                setLambdaFromQP(parentCTU, qp);

            if (!skipModes)
            {
                uint32_t refMasks[2];
                refMasks[0] = allSplitRefs;
                md.pred[PRED_2Nx2N].cu.initSubCU(parentCTU, cuGeom, qp);
                checkInter_rd0_4(md.pred[PRED_2Nx2N], cuGeom, SIZE_2Nx2N, refMasks);

                if (m_param->limitReferences & S265_REF_LIMIT_CU)
                {
                    CUData& cu = md.pred[PRED_2Nx2N].cu;
                    uint32_t refMask = cu.getBestRefIdx(0);
                    allSplitRefs = splitData[0].splitRefs = splitData[1].splitRefs = splitData[2].splitRefs = splitData[3].splitRefs = refMask;
                }

                if (m_slice->m_sliceType == B_SLICE)
                {
                    md.pred[PRED_BIDIR].cu.initSubCU(parentCTU, cuGeom, qp);
                    checkBidir2Nx2N(md.pred[PRED_2Nx2N], md.pred[PRED_BIDIR], cuGeom);
                }

                Mode *bestInter = &md.pred[PRED_2Nx2N];
                if (!skipRectAmp)
                {
                    if (m_param->bEnableRectInter)
                    {
                        uint64_t splitCost = splitData[0].sa8dCost + splitData[1].sa8dCost + splitData[2].sa8dCost + splitData[3].sa8dCost;
                        uint32_t threshold_2NxN, threshold_Nx2N;

                        if (m_slice->m_sliceType == P_SLICE)
                        {
                            threshold_2NxN = splitData[0].mvCost[0] + splitData[1].mvCost[0];
                            threshold_Nx2N = splitData[0].mvCost[0] + splitData[2].mvCost[0];
                        }
                        else
                        {
                            threshold_2NxN = (splitData[0].mvCost[0] + splitData[1].mvCost[0]
                                + splitData[0].mvCost[1] + splitData[1].mvCost[1] + 1) >> 1;
                            threshold_Nx2N = (splitData[0].mvCost[0] + splitData[2].mvCost[0]
                                + splitData[0].mvCost[1] + splitData[2].mvCost[1] + 1) >> 1;
                        }

                        int try_2NxN_first = threshold_2NxN < threshold_Nx2N;
                        if (try_2NxN_first && splitCost < md.pred[PRED_2Nx2N].sa8dCost + threshold_2NxN)
                        {
                            refMasks[0] = splitData[0].splitRefs | splitData[1].splitRefs; /* top */
                            refMasks[1] = splitData[2].splitRefs | splitData[3].splitRefs; /* bot */
                            md.pred[PRED_2NxN].cu.initSubCU(parentCTU, cuGeom, qp);
                            checkInter_rd0_4(md.pred[PRED_2NxN], cuGeom, SIZE_2NxN, refMasks);
                            if (md.pred[PRED_2NxN].sa8dCost < bestInter->sa8dCost)
                                bestInter = &md.pred[PRED_2NxN];
                        }

                        if (splitCost < md.pred[PRED_2Nx2N].sa8dCost + threshold_Nx2N)
                        {
                            refMasks[0] = splitData[0].splitRefs | splitData[2].splitRefs; /* left */
                            refMasks[1] = splitData[1].splitRefs | splitData[3].splitRefs; /* right */
                            md.pred[PRED_Nx2N].cu.initSubCU(parentCTU, cuGeom, qp);
                            checkInter_rd0_4(md.pred[PRED_Nx2N], cuGeom, SIZE_Nx2N, refMasks);
                            if (md.pred[PRED_Nx2N].sa8dCost < bestInter->sa8dCost)
                                bestInter = &md.pred[PRED_Nx2N];
                        }

                        if (!try_2NxN_first && splitCost < md.pred[PRED_2Nx2N].sa8dCost + threshold_2NxN)
                        {
                            refMasks[0] = splitData[0].splitRefs | splitData[1].splitRefs; /* top */
                            refMasks[1] = splitData[2].splitRefs | splitData[3].splitRefs; /* bot */
                            md.pred[PRED_2NxN].cu.initSubCU(parentCTU, cuGeom, qp);
                            checkInter_rd0_4(md.pred[PRED_2NxN], cuGeom, SIZE_2NxN, refMasks);
                            if (md.pred[PRED_2NxN].sa8dCost < bestInter->sa8dCost)
                                bestInter = &md.pred[PRED_2NxN];
                        }
                    }

                    if (m_slice->m_sps->maxAMPDepth > depth)
                    {
                        uint64_t splitCost = splitData[0].sa8dCost + splitData[1].sa8dCost + splitData[2].sa8dCost + splitData[3].sa8dCost;
                        uint32_t threshold_2NxnU, threshold_2NxnD, threshold_nLx2N, threshold_nRx2N;

                        if (m_slice->m_sliceType == P_SLICE)
                        {
                            threshold_2NxnU = splitData[0].mvCost[0] + splitData[1].mvCost[0];
                            threshold_2NxnD = splitData[2].mvCost[0] + splitData[3].mvCost[0];

                            threshold_nLx2N = splitData[0].mvCost[0] + splitData[2].mvCost[0];
                            threshold_nRx2N = splitData[1].mvCost[0] + splitData[3].mvCost[0];
                        }
                        else
                        {
                            threshold_2NxnU = (splitData[0].mvCost[0] + splitData[1].mvCost[0]
                                + splitData[0].mvCost[1] + splitData[1].mvCost[1] + 1) >> 1;
                            threshold_2NxnD = (splitData[2].mvCost[0] + splitData[3].mvCost[0]
                                + splitData[2].mvCost[1] + splitData[3].mvCost[1] + 1) >> 1;

                            threshold_nLx2N = (splitData[0].mvCost[0] + splitData[2].mvCost[0]
                                + splitData[0].mvCost[1] + splitData[2].mvCost[1] + 1) >> 1;
                            threshold_nRx2N = (splitData[1].mvCost[0] + splitData[3].mvCost[0]
                                + splitData[1].mvCost[1] + splitData[3].mvCost[1] + 1) >> 1;
                        }

                        bool bHor = false, bVer = false;
                        if (bestInter->cu.m_partSize[0] == SIZE_2NxN)
                            bHor = true;
                        else if (bestInter->cu.m_partSize[0] == SIZE_Nx2N)
                            bVer = true;
                        else if (bestInter->cu.m_partSize[0] == SIZE_2Nx2N &&
                            md.bestMode && md.bestMode->cu.getQtRootCbf(0))
                        {
                            bHor = true;
                            bVer = true;
                        }

                        if (bHor)
                        {
                            int try_2NxnD_first = threshold_2NxnD < threshold_2NxnU;
                            if (try_2NxnD_first && splitCost < md.pred[PRED_2Nx2N].sa8dCost + threshold_2NxnD)
                            {
                                refMasks[0] = allSplitRefs;                                    /* 75% top */
                                refMasks[1] = splitData[2].splitRefs | splitData[3].splitRefs; /* 25% bot */
                                md.pred[PRED_2NxnD].cu.initSubCU(parentCTU, cuGeom, qp);
                                checkInter_rd0_4(md.pred[PRED_2NxnD], cuGeom, SIZE_2NxnD, refMasks);
                                if (md.pred[PRED_2NxnD].sa8dCost < bestInter->sa8dCost)
                                    bestInter = &md.pred[PRED_2NxnD];
                            }

                            if (splitCost < md.pred[PRED_2Nx2N].sa8dCost + threshold_2NxnU)
                            {
                                refMasks[0] = splitData[0].splitRefs | splitData[1].splitRefs; /* 25% top */
                                refMasks[1] = allSplitRefs;                                    /* 75% bot */
                                md.pred[PRED_2NxnU].cu.initSubCU(parentCTU, cuGeom, qp);
                                checkInter_rd0_4(md.pred[PRED_2NxnU], cuGeom, SIZE_2NxnU, refMasks);
                                if (md.pred[PRED_2NxnU].sa8dCost < bestInter->sa8dCost)
                                    bestInter = &md.pred[PRED_2NxnU];
                            }

                            if (!try_2NxnD_first && splitCost < md.pred[PRED_2Nx2N].sa8dCost + threshold_2NxnD)
                            {
                                refMasks[0] = allSplitRefs;                                    /* 75% top */
                                refMasks[1] = splitData[2].splitRefs | splitData[3].splitRefs; /* 25% bot */
                                md.pred[PRED_2NxnD].cu.initSubCU(parentCTU, cuGeom, qp);
                                checkInter_rd0_4(md.pred[PRED_2NxnD], cuGeom, SIZE_2NxnD, refMasks);
                                if (md.pred[PRED_2NxnD].sa8dCost < bestInter->sa8dCost)
                                    bestInter = &md.pred[PRED_2NxnD];
                            }
                        }
                        if (bVer)
                        {
                            int try_nRx2N_first = threshold_nRx2N < threshold_nLx2N;
                            if (try_nRx2N_first && splitCost < md.pred[PRED_2Nx2N].sa8dCost + threshold_nRx2N)
                            {
                                refMasks[0] = allSplitRefs;                                    /* 75% left  */
                                refMasks[1] = splitData[1].splitRefs | splitData[3].splitRefs; /* 25% right */
                                md.pred[PRED_nRx2N].cu.initSubCU(parentCTU, cuGeom, qp);
                                checkInter_rd0_4(md.pred[PRED_nRx2N], cuGeom, SIZE_nRx2N, refMasks);
                                if (md.pred[PRED_nRx2N].sa8dCost < bestInter->sa8dCost)
                                    bestInter = &md.pred[PRED_nRx2N];
                            }

                            if (splitCost < md.pred[PRED_2Nx2N].sa8dCost + threshold_nLx2N)
                            {
                                refMasks[0] = splitData[0].splitRefs | splitData[2].splitRefs; /* 25% left  */
                                refMasks[1] = allSplitRefs;                                    /* 75% right */
                                md.pred[PRED_nLx2N].cu.initSubCU(parentCTU, cuGeom, qp);
                                checkInter_rd0_4(md.pred[PRED_nLx2N], cuGeom, SIZE_nLx2N, refMasks);
                                if (md.pred[PRED_nLx2N].sa8dCost < bestInter->sa8dCost)
                                    bestInter = &md.pred[PRED_nLx2N];
                            }

                            if (!try_nRx2N_first && splitCost < md.pred[PRED_2Nx2N].sa8dCost + threshold_nRx2N)
                            {
                                refMasks[0] = allSplitRefs;                                    /* 75% left  */
                                refMasks[1] = splitData[1].splitRefs | splitData[3].splitRefs; /* 25% right */
                                md.pred[PRED_nRx2N].cu.initSubCU(parentCTU, cuGeom, qp);
                                checkInter_rd0_4(md.pred[PRED_nRx2N], cuGeom, SIZE_nRx2N, refMasks);
                                if (md.pred[PRED_nRx2N].sa8dCost < bestInter->sa8dCost)
                                    bestInter = &md.pred[PRED_nRx2N];
                            }
                        }
                    }
                }
                bool bTryIntra = (m_slice->m_sliceType != B_SLICE || m_param->bIntraInBFrames) && cuGeom.log2CUSize != MAX_LOG2_CU_SIZE;
                if (m_param->rdLevel >= 3)
                {
                    /* Calculate RD cost of best inter option */
                    if ((!m_bChromaSa8d && (m_csp != S265_CSP_I400)) || (m_frame->m_fencPic->m_picCsp == S265_CSP_I400 && m_csp != S265_CSP_I400)) /* When m_bChromaSa8d is enabled, chroma MC has already been done */
                    {
                        uint32_t numPU = bestInter->cu.getNumPartInter(0);
                        for (uint32_t puIdx = 0; puIdx < numPU; puIdx++)
                        {
                            PredictionUnit pu(bestInter->cu, cuGeom, puIdx);
                            motionCompensation(bestInter->cu, pu, bestInter->predYuv, false, true);
                        }
                    }

                    if (!chooseMerge)
                    {
                        encodeResAndCalcRdInterCU(*bestInter, cuGeom);
                        checkBestMode(*bestInter, depth);

                        /* If BIDIR is available and within 17/16 of best inter option, choose by RDO */
                        if (m_slice->m_sliceType == B_SLICE && md.pred[PRED_BIDIR].sa8dCost != MAX_INT64 &&
                            md.pred[PRED_BIDIR].sa8dCost * 16 <= bestInter->sa8dCost * 17)
                        {
                            uint32_t numPU = md.pred[PRED_BIDIR].cu.getNumPartInter(0);
                            if (m_frame->m_fencPic->m_picCsp == S265_CSP_I400 && m_csp != S265_CSP_I400)
                                for (uint32_t puIdx = 0; puIdx < numPU; puIdx++)
                                {
                                    PredictionUnit pu(md.pred[PRED_BIDIR].cu, cuGeom, puIdx);
                                    motionCompensation(md.pred[PRED_BIDIR].cu, pu, md.pred[PRED_BIDIR].predYuv, true, true);
                                }
                            encodeResAndCalcRdInterCU(md.pred[PRED_BIDIR], cuGeom);
                            checkBestMode(md.pred[PRED_BIDIR], depth);
                        }
                    }

                    if ((bTryIntra && md.bestMode->cu.getQtRootCbf(0)) ||
                        md.bestMode->sa8dCost == MAX_INT64)
                    {
                        if (!m_param->limitReferences || splitIntra)
                        {
                            ProfileCounter(parentCTU, totalIntraCU[cuGeom.depth]);
                            md.pred[PRED_INTRA].cu.initSubCU(parentCTU, cuGeom, qp);
                            checkIntraInInter(md.pred[PRED_INTRA], cuGeom);
                            encodeIntraInInter(md.pred[PRED_INTRA], cuGeom);
                            checkBestMode(md.pred[PRED_INTRA], depth);
                        }
                        else
                        {
                            ProfileCounter(parentCTU, skippedIntraCU[cuGeom.depth]);
                        }
                    }
                }
                else
                {
                    /* SA8D choice between merge/skip, inter, bidir, and intra */
                    if (!md.bestMode || bestInter->sa8dCost < md.bestMode->sa8dCost)
                        md.bestMode = bestInter;

                    if (m_slice->m_sliceType == B_SLICE &&
                        md.pred[PRED_BIDIR].sa8dCost < md.bestMode->sa8dCost)
                        md.bestMode = &md.pred[PRED_BIDIR];

                    if (bTryIntra || md.bestMode->sa8dCost == MAX_INT64)
                    {
                        if (!m_param->limitReferences || splitIntra)
                        {
                            ProfileCounter(parentCTU, totalIntraCU[cuGeom.depth]);
                            md.pred[PRED_INTRA].cu.initSubCU(parentCTU, cuGeom, qp);
                            checkIntraInInter(md.pred[PRED_INTRA], cuGeom);
                            if (md.pred[PRED_INTRA].sa8dCost < md.bestMode->sa8dCost)
                                md.bestMode = &md.pred[PRED_INTRA];
                        }
                        else
                        {
                            ProfileCounter(parentCTU, skippedIntraCU[cuGeom.depth]);
                        }
                    }

                    /* finally code the best mode selected by SA8D costs:
                     * RD level 2 - fully encode the best mode
                     * RD level 1 - generate recon pixels
                     * RD level 0 - generate chroma prediction */
                    if (md.bestMode->cu.m_mergeFlag[0] && md.bestMode->cu.m_partSize[0] == SIZE_2Nx2N)
                    {
                        /* prediction already generated for this CU, and if rd level
                         * is not 0, it is already fully encoded */
                    }
                    else if (md.bestMode->cu.isInter(0))
                    {
                        uint32_t numPU = md.bestMode->cu.getNumPartInter(0);
                        if (m_csp != S265_CSP_I400)
                        {
                            for (uint32_t puIdx = 0; puIdx < numPU; puIdx++)
                            {
                                PredictionUnit pu(md.bestMode->cu, cuGeom, puIdx);
                                motionCompensation(md.bestMode->cu, pu, md.bestMode->predYuv, false, true);
                            }
                        }
                        if (m_param->rdLevel == 2)
                            encodeResAndCalcRdInterCU(*md.bestMode, cuGeom);
                        else if (m_param->rdLevel == 1)
                        {
                            /* generate recon pixels with no rate distortion considerations */
                            CUData& cu = md.bestMode->cu;

                            uint32_t tuDepthRange[2];
                            cu.getInterTUQtDepthRange(tuDepthRange, 0);
                            m_rqt[cuGeom.depth].tmpResiYuv.subtract(*md.bestMode->fencYuv, md.bestMode->predYuv, cuGeom.log2CUSize, m_frame->m_fencPic->m_picCsp);
                            residualTransformQuantInter(*md.bestMode, cuGeom, 0, 0, tuDepthRange);
                            if (cu.getQtRootCbf(0))
                                md.bestMode->reconYuv.addClip(md.bestMode->predYuv, m_rqt[cuGeom.depth].tmpResiYuv, cu.m_log2CUSize[0], m_frame->m_fencPic->m_picCsp);
                            else
                            {
                                md.bestMode->reconYuv.copyFromYuv(md.bestMode->predYuv);
                                if (cu.m_mergeFlag[0] && cu.m_partSize[0] == SIZE_2Nx2N)
                                    cu.setPredModeSubParts(MODE_SKIP);
                            }
                        }
                    }
                    else
                    {
                        if (m_param->rdLevel == 2)
                            encodeIntraInInter(*md.bestMode, cuGeom);
                        else if (m_param->rdLevel == 1)
                        {
                            /* generate recon pixels with no rate distortion considerations */
                            CUData& cu = md.bestMode->cu;

                            uint32_t tuDepthRange[2];
                            cu.getIntraTUQtDepthRange(tuDepthRange, 0);

                            residualTransformQuantIntra(*md.bestMode, cuGeom, 0, 0, tuDepthRange);
                            if (m_csp != S265_CSP_I400)
                            {
                                getBestIntraModeChroma(*md.bestMode, cuGeom);
                                residualQTIntraChroma(*md.bestMode, cuGeom, 0, 0);
                            }
                            md.bestMode->reconYuv.copyFromPicYuv(reconPic, cu.m_cuAddr, cuGeom.absPartIdx); // TODO:
                        }
                    }
                }
            } // !earlyskip

            if (m_bTryLossless)
                tryLossless(cuGeom);

            if (mightSplit)
                addSplitFlagCost(*md.bestMode, cuGeom.depth);
        }

        if (mightSplit && !skipRecursion)
        {
            Mode* splitPred = &md.pred[PRED_SPLIT];
            if (!md.bestMode)
                md.bestMode = splitPred;
            else if (m_param->rdLevel > 1)
                checkBestMode(*splitPred, cuGeom.depth);
            else if (splitPred->sa8dCost < md.bestMode->sa8dCost)
                md.bestMode = splitPred;

            checkDQPForSplitPred(*md.bestMode, cuGeom);
        }

        /* determine which motion references the parent CU should search */
        splitCUData.initSplitCUData();

        if (m_param->limitReferences & S265_REF_LIMIT_DEPTH)
        {
            if (md.bestMode == &md.pred[PRED_SPLIT])
                splitCUData.splitRefs = allSplitRefs;
            else
            {
                /* use best merge/inter mode, in case of intra use 2Nx2N inter references */
                CUData& cu = md.bestMode->cu.isIntra(0) ? md.pred[PRED_2Nx2N].cu : md.bestMode->cu;
                uint32_t numPU = cu.getNumPartInter(0);
                for (uint32_t puIdx = 0, subPartIdx = 0; puIdx < numPU; puIdx++, subPartIdx += cu.getPUOffset(puIdx, 0))
                    splitCUData.splitRefs |= cu.getBestRefIdx(subPartIdx);
            }
        }

        if (m_param->limitModes)
        {
            splitCUData.mvCost[0] = md.pred[PRED_2Nx2N].bestME[0][0].mvCost; // L0
            splitCUData.mvCost[1] = md.pred[PRED_2Nx2N].bestME[0][1].mvCost; // L1
            splitCUData.sa8dCost = md.pred[PRED_2Nx2N].sa8dCost;
        }

        if (mightNotSplit && md.bestMode->cu.isSkipped(0))
        {
            FrameData& curEncData = *m_frame->m_encData;
            FrameData::RCStatCU& cuStat = curEncData.m_cuStat[parentCTU.m_cuAddr];
            uint64_t temp = cuStat.avgCost[depth] * cuStat.count[depth];
            cuStat.count[depth] += 1;
            cuStat.avgCost[depth] = (temp + md.bestMode->rdCost) / cuStat.count[depth];
        }

        /* Copy best data to encData CTU and recon */
        md.bestMode->cu.copyToPic(depth);
        if (m_param->rdLevel)
            md.bestMode->reconYuv.copyToPicYuv(reconPic, cuAddr, cuGeom.absPartIdx);

        if ((m_limitTU & S265_TU_LIMIT_NEIGH) && cuGeom.log2CUSize >= 4)
        {
            if (mightNotSplit)
            {
                CUData* ctu = md.bestMode->cu.m_encData->getPicCTU(parentCTU.m_cuAddr);
                int8_t maxTUDepth = -1;
                for (uint32_t i = 0; i < cuGeom.numPartitions; i++)
                    maxTUDepth = S265_MAX(maxTUDepth, md.bestMode->cu.m_tuDepth[i]);
                ctu->m_refTuDepth[cuGeom.geomRecurId] = maxTUDepth;
            }
        }
    }

    return splitCUData;
}

SplitData Analysis::compressInterCU_rd5_6(const CUData& parentCTU, const CUGeom& cuGeom, int32_t qp)
{
    // 动态rd 先不看
    if (parentCTU.m_vbvAffected && !calculateQpforCuSize(parentCTU, cuGeom, 1))
        return compressInterCU_rd0_4(parentCTU, cuGeom, qp);

    uint32_t depth = cuGeom.depth;
    ModeDepth& md = m_modeDepth[depth];
    md.bestMode = NULL;

    if (m_param->searchMethod == S265_SEA)
    {
        int numPredDir = m_slice->isInterP() ? 1 : 2;
        int offset = (int)(m_frame->m_reconPic->m_cuOffsetY[parentCTU.m_cuAddr] + m_frame->m_reconPic->m_buOffsetY[cuGeom.absPartIdx]);
        for (int list = 0; list < numPredDir; list++)
            for (int i = 0; i < m_frame->m_encData->m_slice->m_numRefIdx[list]; i++)
                for (int planes = 0; planes < INTEGRAL_PLANE_NUM; planes++)
                    m_modeDepth[depth].fencYuv.m_integral[list][i][planes] = m_frame->m_encData->m_slice->m_refFrameList[list][i]->m_encData->m_meIntegral[planes] + offset;
    }

    SplitData splitCUData;
    bool bNooffloading = true;

    if (bNooffloading)
    {
        bool mightSplit = !(cuGeom.flags & CUGeom::LEAF);// 0: 叶子结点，不能split 1:  非叶子结点可以split
        // 当前CU不是CTU的叶子节点，设置mightSplit为True，继续分裂
        // 后续根据mightSplit判断是否将split flag添加到RD cost中
        bool mightNotSplit = !(cuGeom.flags & CUGeom::SPLIT_MANDATORY);
        bool skipRecursion = false;
        bool skipModes = false;
        bool splitIntra = true;
        bool skipRectAmp = false;

        if (m_evaluateInter)// 此处一般情况不会进来，只有在reencode 时
        {
            if (m_refineLevel == 2)
            {
                if (parentCTU.m_predMode[cuGeom.absPartIdx] == MODE_SKIP)
                    skipModes = true;
                if (parentCTU.m_partSize[cuGeom.absPartIdx] == SIZE_2Nx2N)
                    skipRectAmp = true;
            }
            mightSplit &= false;
        }

        // avoid uninitialize value in below reference
        if (m_param->limitModes)
        {
            md.pred[PRED_2Nx2N].bestME[0][0].mvCost = 0; // L0
            md.pred[PRED_2Nx2N].bestME[0][1].mvCost = 0; // L1
            md.pred[PRED_2Nx2N].rdCost = 0;
        }

        if ((m_limitTU & S265_TU_LIMIT_NEIGH) && cuGeom.log2CUSize >= 4)
            m_maxTUDepth = loadTUDepth(cuGeom, parentCTU);

        SplitData splitData[4];
        splitData[0].initSplitCUData();
        splitData[1].initSplitCUData();
        splitData[2].initSplitCUData();
        splitData[3].initSplitCUData();
        uint32_t allSplitRefs = 0;//splitData[0].splitRefs | splitData[1].splitRefs | splitData[2].splitRefs | splitData[3].splitRefs;
        uint32_t refMasks[2];

        /* Step 1. Evaluate Merge/Skip candidates for likely early-outs */
        if (mightNotSplit && !md.bestMode)
        {
            md.pred[PRED_SKIP].cu.initSubCU(parentCTU, cuGeom, qp);
            md.pred[PRED_MERGE].cu.initSubCU(parentCTU, cuGeom, qp);
            checkMerge2Nx2N_rd5_6(md.pred[PRED_SKIP], md.pred[PRED_MERGE], cuGeom);
            skipModes = (m_param->bEnableEarlySkip || m_refineLevel == 2) &&
                md.bestMode && !md.bestMode->cu.getQtRootCbf(0);
            refMasks[0] = allSplitRefs;
            md.pred[PRED_2Nx2N].cu.initSubCU(parentCTU, cuGeom, qp);
            checkInter_rd5_6(md.pred[PRED_2Nx2N], cuGeom, SIZE_2Nx2N, refMasks);
            checkBestMode(md.pred[PRED_2Nx2N], cuGeom.depth);

            if (m_param->recursionSkipMode == RDCOST_BASED_RSKIP && depth && m_modeDepth[depth - 1].bestMode)
                skipRecursion = md.bestMode && !md.bestMode->cu.getQtRootCbf(0);
            else if (cuGeom.log2CUSize >= MAX_LOG2_CU_SIZE - 1 && m_param->recursionSkipMode == EDGE_BASED_RSKIP)
                skipRecursion = md.bestMode && complexityCheckCU(*md.bestMode);
        }
        // estimate split cost
        /* Step 2. Evaluate each of the 4 split sub-blocks in series */
        if (mightSplit && !skipRecursion)
        {
            Mode* splitPred = &md.pred[PRED_SPLIT];
            splitPred->initCosts();
            CUData* splitCU = &splitPred->cu;
            splitCU->initSubCU(parentCTU, cuGeom, qp);

            uint32_t nextDepth = depth + 1;
            ModeDepth& nd = m_modeDepth[nextDepth];
            invalidateContexts(nextDepth);
            Entropy* nextContext = &m_rqt[depth].cur;
            int nextQP = qp;
            splitIntra = false;

            for (uint32_t subPartIdx = 0; subPartIdx < 4; subPartIdx++)
            {
                const CUGeom& childGeom = *(&cuGeom + cuGeom.childOffset + subPartIdx);
                if (childGeom.flags & CUGeom::PRESENT)
                {
                    m_modeDepth[0].fencYuv.copyPartToYuv(nd.fencYuv, childGeom.absPartIdx);
                    m_rqt[nextDepth].cur.load(*nextContext);

                    if (m_slice->m_pps->bUseDQP && nextDepth <= m_slice->m_pps->maxCuDQPDepth)
                        nextQP = setLambdaFromQP(parentCTU, calculateQpforCuSize(parentCTU, childGeom));

                    splitData[subPartIdx] = compressInterCU_rd5_6(parentCTU, childGeom, nextQP);

                    // Save best CU and pred data for this sub CU
                    splitIntra |= nd.bestMode->cu.isIntra(0);
                    splitCU->copyPartFrom(nd.bestMode->cu, childGeom, subPartIdx);
                    splitPred->addSubCosts(*nd.bestMode);
                    nd.bestMode->reconYuv.copyToPartYuv(splitPred->reconYuv, childGeom.numPartitions * subPartIdx);
                    nextContext = &nd.bestMode->contexts;
                    if (m_modeDepth[depth].bestMode && splitPred->rdCost > m_modeDepth[depth].bestMode->rdCost)
                    {   //如果slipt下的cost 累加超过了 非slipt下的cost了 则可以break掉for循环了
                        skipRecursion = true;
                        break;
                    }

                }
                else
                {
                    splitCU->setEmptyPart(childGeom, subPartIdx);
                }
            }
            if(!skipRecursion)
            {
                nextContext->store(splitPred->contexts);
                if (mightNotSplit)
                    addSplitFlagCost(*splitPred, cuGeom.depth);
                else
                    updateModeCost(*splitPred);

                checkDQPForSplitPred(*splitPred, cuGeom);
            }
        }
        /* Split CUs
         *   0  1
         *   2  3 */
        allSplitRefs = splitData[0].splitRefs | splitData[1].splitRefs | splitData[2].splitRefs | splitData[3].splitRefs;
        /* Step 3. Evaluate ME (2Nx2N, rect, amp) and intra modes at current depth */
        if (mightNotSplit)
        {
            if (m_slice->m_pps->bUseDQP && depth <= m_slice->m_pps->maxCuDQPDepth && m_slice->m_pps->maxCuDQPDepth != 0)
                setLambdaFromQP(parentCTU, qp);

            if (!skipModes)
            {
                refMasks[0] = allSplitRefs;

                if (m_param->limitReferences & S265_REF_LIMIT_CU)
                {
                    CUData& cu = md.pred[PRED_2Nx2N].cu;
                    uint32_t refMask = cu.getBestRefIdx(0);
                    allSplitRefs = splitData[0].splitRefs = splitData[1].splitRefs = splitData[2].splitRefs = splitData[3].splitRefs = refMask;
                }

                if (m_slice->m_sliceType == B_SLICE)
                {
                    md.pred[PRED_BIDIR].cu.initSubCU(parentCTU, cuGeom, qp);
                    checkBidir2Nx2N(md.pred[PRED_2Nx2N], md.pred[PRED_BIDIR], cuGeom);
                    if (md.pred[PRED_BIDIR].sa8dCost < MAX_INT64)
                    {
                        uint32_t numPU = md.pred[PRED_BIDIR].cu.getNumPartInter(0);
                        if (m_frame->m_fencPic->m_picCsp == S265_CSP_I400 && m_csp != S265_CSP_I400)
                            for (uint32_t puIdx = 0; puIdx < numPU; puIdx++)
                            {
                                PredictionUnit pu(md.pred[PRED_BIDIR].cu, cuGeom, puIdx);
                                motionCompensation(md.pred[PRED_BIDIR].cu, pu, md.pred[PRED_BIDIR].predYuv, true, true);
                            }
                        encodeResAndCalcRdInterCU(md.pred[PRED_BIDIR], cuGeom);
                        checkBestMode(md.pred[PRED_BIDIR], cuGeom.depth);
                    }
                }

                if (!skipRectAmp)
                {
                    if (m_param->bEnableRectInter)
                    {
                        uint64_t splitCost = splitData[0].sa8dCost + splitData[1].sa8dCost + splitData[2].sa8dCost + splitData[3].sa8dCost;
                        uint32_t threshold_2NxN, threshold_Nx2N;

                        if (m_slice->m_sliceType == P_SLICE)
                        {
                            threshold_2NxN = splitData[0].mvCost[0] + splitData[1].mvCost[0];
                            threshold_Nx2N = splitData[0].mvCost[0] + splitData[2].mvCost[0];
                        }
                        else
                        {
                            threshold_2NxN = (splitData[0].mvCost[0] + splitData[1].mvCost[0]
                                + splitData[0].mvCost[1] + splitData[1].mvCost[1] + 1) >> 1;
                            threshold_Nx2N = (splitData[0].mvCost[0] + splitData[2].mvCost[0]
                                + splitData[0].mvCost[1] + splitData[2].mvCost[1] + 1) >> 1;
                        }

                        int try_2NxN_first = threshold_2NxN < threshold_Nx2N;
                        if (try_2NxN_first && splitCost < md.bestMode->rdCost + threshold_2NxN)
                        {
                            refMasks[0] = splitData[0].splitRefs | splitData[1].splitRefs; /* top */
                            refMasks[1] = splitData[2].splitRefs | splitData[3].splitRefs; /* bot */
                            md.pred[PRED_2NxN].cu.initSubCU(parentCTU, cuGeom, qp);
                            checkInter_rd5_6(md.pred[PRED_2NxN], cuGeom, SIZE_2NxN, refMasks);
                            checkBestMode(md.pred[PRED_2NxN], cuGeom.depth);
                        }

                        if (splitCost < md.bestMode->rdCost + threshold_Nx2N)
                        {
                            refMasks[0] = splitData[0].splitRefs | splitData[2].splitRefs; /* left */
                            refMasks[1] = splitData[1].splitRefs | splitData[3].splitRefs; /* right */
                            md.pred[PRED_Nx2N].cu.initSubCU(parentCTU, cuGeom, qp);
                            checkInter_rd5_6(md.pred[PRED_Nx2N], cuGeom, SIZE_Nx2N, refMasks);
                            checkBestMode(md.pred[PRED_Nx2N], cuGeom.depth);
                        }

                        if (!try_2NxN_first && splitCost < md.bestMode->rdCost + threshold_2NxN)
                        {
                            refMasks[0] = splitData[0].splitRefs | splitData[1].splitRefs; /* top */
                            refMasks[1] = splitData[2].splitRefs | splitData[3].splitRefs; /* bot */
                            md.pred[PRED_2NxN].cu.initSubCU(parentCTU, cuGeom, qp);
                            checkInter_rd5_6(md.pred[PRED_2NxN], cuGeom, SIZE_2NxN, refMasks);
                            checkBestMode(md.pred[PRED_2NxN], cuGeom.depth);
                        }
                    }

                    // Try AMP (SIZE_2NxnU, SIZE_2NxnD, SIZE_nLx2N, SIZE_nRx2N)
                    if (m_slice->m_sps->maxAMPDepth > depth)
                    {
                        uint64_t splitCost = splitData[0].sa8dCost + splitData[1].sa8dCost + splitData[2].sa8dCost + splitData[3].sa8dCost;
                        uint32_t threshold_2NxnU, threshold_2NxnD, threshold_nLx2N, threshold_nRx2N;

                        if (m_slice->m_sliceType == P_SLICE)
                        {
                            threshold_2NxnU = splitData[0].mvCost[0] + splitData[1].mvCost[0];
                            threshold_2NxnD = splitData[2].mvCost[0] + splitData[3].mvCost[0];

                            threshold_nLx2N = splitData[0].mvCost[0] + splitData[2].mvCost[0];
                            threshold_nRx2N = splitData[1].mvCost[0] + splitData[3].mvCost[0];
                        }
                        else
                        {
                            threshold_2NxnU = (splitData[0].mvCost[0] + splitData[1].mvCost[0]
                                + splitData[0].mvCost[1] + splitData[1].mvCost[1] + 1) >> 1;
                            threshold_2NxnD = (splitData[2].mvCost[0] + splitData[3].mvCost[0]
                                + splitData[2].mvCost[1] + splitData[3].mvCost[1] + 1) >> 1;

                            threshold_nLx2N = (splitData[0].mvCost[0] + splitData[2].mvCost[0]
                                + splitData[0].mvCost[1] + splitData[2].mvCost[1] + 1) >> 1;
                            threshold_nRx2N = (splitData[1].mvCost[0] + splitData[3].mvCost[0]
                                + splitData[1].mvCost[1] + splitData[3].mvCost[1] + 1) >> 1;
                        }

                        bool bHor = false, bVer = false;
                        if (md.bestMode->cu.m_partSize[0] == SIZE_2NxN)
                            bHor = true;
                        else if (md.bestMode->cu.m_partSize[0] == SIZE_Nx2N)
                            bVer = true;
                        else if (md.bestMode->cu.m_partSize[0] == SIZE_2Nx2N && !md.bestMode->cu.m_mergeFlag[0])
                        {
                            bHor = true;
                            bVer = true;
                        }

                        if (bHor)
                        {
                            int try_2NxnD_first = threshold_2NxnD < threshold_2NxnU;
                            if (try_2NxnD_first && splitCost < md.bestMode->rdCost + threshold_2NxnD)
                            {
                                refMasks[0] = allSplitRefs;                                    /* 75% top */
                                refMasks[1] = splitData[2].splitRefs | splitData[3].splitRefs; /* 25% bot */
                                md.pred[PRED_2NxnD].cu.initSubCU(parentCTU, cuGeom, qp);
                                checkInter_rd5_6(md.pred[PRED_2NxnD], cuGeom, SIZE_2NxnD, refMasks);
                                checkBestMode(md.pred[PRED_2NxnD], cuGeom.depth);
                            }

                            if (splitCost < md.bestMode->rdCost + threshold_2NxnU)
                            {
                                refMasks[0] = splitData[0].splitRefs | splitData[1].splitRefs; /* 25% top */
                                refMasks[1] = allSplitRefs;                                    /* 75% bot */
                                md.pred[PRED_2NxnU].cu.initSubCU(parentCTU, cuGeom, qp);
                                checkInter_rd5_6(md.pred[PRED_2NxnU], cuGeom, SIZE_2NxnU, refMasks);
                                checkBestMode(md.pred[PRED_2NxnU], cuGeom.depth);
                            }

                            if (!try_2NxnD_first && splitCost < md.bestMode->rdCost + threshold_2NxnD)
                            {
                                refMasks[0] = allSplitRefs;                                    /* 75% top */
                                refMasks[1] = splitData[2].splitRefs | splitData[3].splitRefs; /* 25% bot */
                                md.pred[PRED_2NxnD].cu.initSubCU(parentCTU, cuGeom, qp);
                                checkInter_rd5_6(md.pred[PRED_2NxnD], cuGeom, SIZE_2NxnD, refMasks);
                                checkBestMode(md.pred[PRED_2NxnD], cuGeom.depth);
                            }
                        }

                        if (bVer)
                        {
                            int try_nRx2N_first = threshold_nRx2N < threshold_nLx2N;
                            if (try_nRx2N_first && splitCost < md.bestMode->rdCost + threshold_nRx2N)
                            {
                                refMasks[0] = allSplitRefs;                                    /* 75% left  */
                                refMasks[1] = splitData[1].splitRefs | splitData[3].splitRefs; /* 25% right */
                                md.pred[PRED_nRx2N].cu.initSubCU(parentCTU, cuGeom, qp);
                                checkInter_rd5_6(md.pred[PRED_nRx2N], cuGeom, SIZE_nRx2N, refMasks);
                                checkBestMode(md.pred[PRED_nRx2N], cuGeom.depth);
                            }

                            if (splitCost < md.bestMode->rdCost + threshold_nLx2N)
                            {
                                refMasks[0] = splitData[0].splitRefs | splitData[2].splitRefs; /* 25% left  */
                                refMasks[1] = allSplitRefs;                                    /* 75% right */
                                md.pred[PRED_nLx2N].cu.initSubCU(parentCTU, cuGeom, qp);
                                checkInter_rd5_6(md.pred[PRED_nLx2N], cuGeom, SIZE_nLx2N, refMasks);
                                checkBestMode(md.pred[PRED_nLx2N], cuGeom.depth);
                            }

                            if (!try_nRx2N_first && splitCost < md.bestMode->rdCost + threshold_nRx2N)
                            {
                                refMasks[0] = allSplitRefs;                                    /* 75% left  */
                                refMasks[1] = splitData[1].splitRefs | splitData[3].splitRefs; /* 25% right */
                                md.pred[PRED_nRx2N].cu.initSubCU(parentCTU, cuGeom, qp);
                                checkInter_rd5_6(md.pred[PRED_nRx2N], cuGeom, SIZE_nRx2N, refMasks);
                                checkBestMode(md.pred[PRED_nRx2N], cuGeom.depth);
                            }
                        }
                    }
                }

                if ((m_slice->m_sliceType != B_SLICE || m_param->bIntraInBFrames) && (cuGeom.log2CUSize != MAX_LOG2_CU_SIZE))
                {
                    if (!m_param->limitReferences || splitIntra)
                    {
                        ProfileCounter(parentCTU, totalIntraCU[cuGeom.depth]);
                        md.pred[PRED_INTRA].cu.initSubCU(parentCTU, cuGeom, qp);
                        checkIntra(md.pred[PRED_INTRA], cuGeom, SIZE_2Nx2N);
                        checkBestMode(md.pred[PRED_INTRA], depth);

                        if (cuGeom.log2CUSize == 3 && m_slice->m_sps->quadtreeTULog2MinSize < 3)
                        {
                            md.pred[PRED_INTRA_NxN].cu.initSubCU(parentCTU, cuGeom, qp);
                            checkIntra(md.pred[PRED_INTRA_NxN], cuGeom, SIZE_NxN);
                            checkBestMode(md.pred[PRED_INTRA_NxN], depth);
                        }
                    }
                    else
                    {
                        ProfileCounter(parentCTU, skippedIntraCU[cuGeom.depth]);
                    }
                }
            }

            if ((md.bestMode->cu.isInter(0) && !(md.bestMode->cu.m_mergeFlag[0] && md.bestMode->cu.m_partSize[0] == SIZE_2Nx2N)) && (m_frame->m_fencPic->m_picCsp == S265_CSP_I400 && m_csp != S265_CSP_I400))
            {
                uint32_t numPU = md.bestMode->cu.getNumPartInter(0);

                for (uint32_t puIdx = 0; puIdx < numPU; puIdx++)
                {
                    PredictionUnit pu(md.bestMode->cu, cuGeom, puIdx);
                    motionCompensation(md.bestMode->cu, pu, md.bestMode->predYuv, false, m_csp != S265_CSP_I400);
                }
                encodeResAndCalcRdInterCU(*md.bestMode, cuGeom);
            }
            if (m_bTryLossless)
                tryLossless(cuGeom);

            if (mightSplit)
                addSplitFlagCost(*md.bestMode, cuGeom.depth);
        }

        if ((m_limitTU & S265_TU_LIMIT_NEIGH) && cuGeom.log2CUSize >= 4)
        {
            if (mightNotSplit)
            {
                CUData* ctu = md.bestMode->cu.m_encData->getPicCTU(parentCTU.m_cuAddr);
                int8_t maxTUDepth = -1;
                for (uint32_t i = 0; i < cuGeom.numPartitions; i++)
                    maxTUDepth = S265_MAX(maxTUDepth, md.bestMode->cu.m_tuDepth[i]);
                ctu->m_refTuDepth[cuGeom.geomRecurId] = maxTUDepth;
            }
        }

        /* compare split RD cost against best cost */
        if (mightSplit && !skipRecursion)
            checkBestMode(md.pred[PRED_SPLIT], depth);

        if (m_param->bEnableRdRefine && depth <= m_slice->m_pps->maxCuDQPDepth)
        {
            int cuIdx = (cuGeom.childOffset - 1) / 3;
            cacheCost[cuIdx] = md.bestMode->rdCost;
        }

        /* determine which motion references the parent CU should search */
        splitCUData.initSplitCUData();
        if (m_param->limitReferences & S265_REF_LIMIT_DEPTH)
        {
            if (md.bestMode == &md.pred[PRED_SPLIT])
                splitCUData.splitRefs = allSplitRefs;
            else
            {
                /* use best merge/inter mode, in case of intra use 2Nx2N inter references */
                CUData& cu = md.bestMode->cu.isIntra(0) ? md.pred[PRED_2Nx2N].cu : md.bestMode->cu;
                uint32_t numPU = cu.getNumPartInter(0);
                for (uint32_t puIdx = 0, subPartIdx = 0; puIdx < numPU; puIdx++, subPartIdx += cu.getPUOffset(puIdx, 0))
                    splitCUData.splitRefs |= cu.getBestRefIdx(subPartIdx);
            }
        }

        if (m_param->limitModes)
        {
            splitCUData.mvCost[0] = md.pred[PRED_2Nx2N].bestME[0][0].mvCost; // L0
            splitCUData.mvCost[1] = md.pred[PRED_2Nx2N].bestME[0][1].mvCost; // L1
            splitCUData.sa8dCost = md.pred[PRED_2Nx2N].rdCost;
        }

        /* Copy best data to encData CTU and recon */
        md.bestMode->cu.copyToPic(depth);
        md.bestMode->reconYuv.copyToPicYuv(*m_frame->m_reconPic, parentCTU.m_cuAddr, cuGeom.absPartIdx);
    }

    return splitCUData;
}
// 在qprdRefine 里面 有recodeCU过程
void Analysis::recodeCU(const CUData& parentCTU, const CUGeom& cuGeom, int32_t qp, int32_t lqp)
{
    uint32_t depth = cuGeom.depth;
    ModeDepth& md = m_modeDepth[depth];
    md.bestMode = NULL;

    m_evaluateInter = 0;
    bool mightSplit = !(cuGeom.flags & CUGeom::LEAF);
    bool mightNotSplit = !(cuGeom.flags & CUGeom::SPLIT_MANDATORY);
    bool bDecidedDepth = parentCTU.m_cuDepth[cuGeom.absPartIdx] == depth;

    TrainingData td;
    td.init(parentCTU, cuGeom);

    if (!m_param->bDynamicRefine)
        m_refineLevel = m_param->interRefine;
    else
        m_refineLevel = m_frame->m_classifyFrame ? 1 : 3;

    td.split = 0;

    if ((bDecidedDepth && mightNotSplit))
    {
        setLambdaFromQP(parentCTU, qp, lqp);

        Mode& mode = md.pred[0];
        md.bestMode = &mode;
        mode.cu.initSubCU(parentCTU, cuGeom, qp);
        PartSize size = (PartSize)parentCTU.m_partSize[cuGeom.absPartIdx];
        if (parentCTU.isIntra(cuGeom.absPartIdx) && m_refineLevel < 2)
        {
            if (m_param->intraRefine == 4)
                compressIntraCU(parentCTU, cuGeom, qp);
            else
            {
                bool reuseModes = !((m_param->intraRefine == 3) ||
                    (m_param->intraRefine == 2 && parentCTU.m_lumaIntraDir[cuGeom.absPartIdx] > DC_IDX));
                if (reuseModes)
                {
                    memcpy(mode.cu.m_lumaIntraDir, parentCTU.m_lumaIntraDir + cuGeom.absPartIdx, cuGeom.numPartitions);
                    memcpy(mode.cu.m_chromaIntraDir, parentCTU.m_chromaIntraDir + cuGeom.absPartIdx, cuGeom.numPartitions);
                }
                checkIntra(mode, cuGeom, size);
            }
        }
        else if (!parentCTU.isIntra(cuGeom.absPartIdx) && m_refineLevel < 2)
        {
            mode.cu.copyFromPic(parentCTU, cuGeom, m_csp, false);
            uint32_t numPU = parentCTU.getNumPartInter(cuGeom.absPartIdx);
            for (uint32_t part = 0; part < numPU; part++)
            {
                PredictionUnit pu(mode.cu, cuGeom, part);
                motionCompensation(mode.cu, pu, mode.predYuv, true, (m_csp != S265_CSP_I400 && m_frame->m_fencPic->m_picCsp != S265_CSP_I400));
            }
            if (!m_param->interRefine && !m_param->bDynamicRefine && parentCTU.isSkipped(cuGeom.absPartIdx))
                encodeResAndCalcRdSkipCU(mode);
            else
                encodeResAndCalcRdInterCU(mode, cuGeom);

            /* checkMerge2Nx2N function performs checkDQP after encoding residual, do the same */
            bool mergeInter2Nx2N = size == SIZE_2Nx2N && mode.cu.m_mergeFlag[0];
            if (parentCTU.isSkipped(cuGeom.absPartIdx) || mergeInter2Nx2N)
                checkDQP(mode, cuGeom);
        }

        if (m_refineLevel < 2)
        {
            if (m_bTryLossless)
                tryLossless(cuGeom);

            if (mightSplit)
                addSplitFlagCost(*md.bestMode, cuGeom.depth);

            if (mightSplit && m_param->rdLevel < 5)
                checkDQPForSplitPred(*md.bestMode, cuGeom);
        }

        if (m_param->bDynamicRefine)
            classifyCU(parentCTU,cuGeom, *md.bestMode, td);

        if (m_refineLevel > 1 || (m_refineLevel && parentCTU.m_predMode[cuGeom.absPartIdx] == MODE_SKIP  && !mode.cu.isSkipped(0)))
        {
            if (parentCTU.m_cuDepth[cuGeom.absPartIdx] < 4 && mightNotSplit)
                m_evaluateInter = 1;
            else
                bDecidedDepth = true;
            m_param->rdLevel > 4 ? compressInterCU_rd5_6(parentCTU, cuGeom, qp) : compressInterCU_rd0_4(parentCTU, cuGeom, qp);
            m_evaluateInter = 0;
        }
    }
    if (!bDecidedDepth)
    {
        Mode* splitPred = &md.pred[PRED_SPLIT];
        splitPred->initCosts();
        CUData* splitCU = &splitPred->cu;
        splitCU->initSubCU(parentCTU, cuGeom, qp);

        uint32_t nextDepth = depth + 1;
        ModeDepth& nd = m_modeDepth[nextDepth];
        invalidateContexts(nextDepth);
        Entropy* nextContext = &m_rqt[depth].cur;
        int nextQP = qp;

        for (uint32_t subPartIdx = 0; subPartIdx < 4; subPartIdx++)
        {
            const CUGeom& childGeom = *(&cuGeom + cuGeom.childOffset + subPartIdx);
            if (childGeom.flags & CUGeom::PRESENT)
            {
                m_modeDepth[0].fencYuv.copyPartToYuv(nd.fencYuv, childGeom.absPartIdx);
                m_rqt[nextDepth].cur.load(*nextContext);

                if (m_slice->m_pps->bUseDQP && nextDepth <= m_slice->m_pps->maxCuDQPDepth)
                    nextQP = setLambdaFromQP(parentCTU, calculateQpforCuSize(parentCTU, childGeom));

                int lamdaQP = lqp;

                qprdRefine(parentCTU, childGeom, nextQP, lamdaQP);

                // Save best CU and pred data for this sub CU
                splitCU->copyPartFrom(nd.bestMode->cu, childGeom, subPartIdx);
                splitPred->addSubCosts(*nd.bestMode);
                nd.bestMode->reconYuv.copyToPartYuv(splitPred->reconYuv, childGeom.numPartitions * subPartIdx);
                nextContext = &nd.bestMode->contexts;
            }
            else
            {
                splitCU->setEmptyPart(childGeom, subPartIdx);
                // Set depth of non-present CU to 0 to ensure that correct CU is fetched as reference to code deltaQP
                memset(parentCTU.m_cuDepth + childGeom.absPartIdx, 0, childGeom.numPartitions);
            }
        }
        nextContext->store(splitPred->contexts);
        if (mightNotSplit)
            addSplitFlagCost(*splitPred, cuGeom.depth);
        else
            updateModeCost(*splitPred);

        if (m_refineLevel)
        {
            if (m_param->rdLevel > 1)
                checkBestMode(*splitPred, cuGeom.depth);
            else if (splitPred->sa8dCost < md.bestMode->sa8dCost)
                md.bestMode = splitPred;
        }

        checkDQPForSplitPred(*splitPred, cuGeom);

        /* Copy best data to encData CTU and recon */
        md.bestMode->cu.copyToPic(depth);
        md.bestMode->reconYuv.copyToPicYuv(*m_frame->m_reconPic, parentCTU.m_cuAddr, cuGeom.absPartIdx);
    }
    if (m_param->bDynamicRefine && bDecidedDepth)
        trainCU(parentCTU, cuGeom, *md.bestMode, td);
}

void Analysis::classifyCU(const CUData& ctu, const CUGeom& cuGeom, const Mode& bestMode, TrainingData& trainData)
{
    uint32_t depth = cuGeom.depth;
    trainData.cuVariance = calculateCUVariance(ctu, cuGeom);
    if (m_frame->m_classifyFrame)
    {
        uint64_t diffRefine[S265_REFINE_INTER_LEVELS];
        uint64_t diffRefineRd[S265_REFINE_INTER_LEVELS];
        float probRefine[S265_REFINE_INTER_LEVELS] = { 0 };
        uint8_t varRefineLevel = 1;
        uint8_t rdRefineLevel = 1;
        uint64_t cuCost = bestMode.rdCost;
        int offset = (depth * S265_REFINE_INTER_LEVELS);
        if (cuCost < m_frame->m_classifyRd[offset])
            m_refineLevel = 1;
        else
        {
            uint64_t trainingCount = 0;
            for (uint8_t i = 0; i < S265_REFINE_INTER_LEVELS; i++)
            {
                offset = (depth * S265_REFINE_INTER_LEVELS) + i;
                trainingCount += m_frame->m_classifyCount[offset];
            }
            for (uint8_t i = 0; i < S265_REFINE_INTER_LEVELS; i++)
            {
                offset = (depth * S265_REFINE_INTER_LEVELS) + i;
                /* Calculate distance values */
                diffRefine[i] = abs((int64_t)(trainData.cuVariance - m_frame->m_classifyVariance[offset]));
                diffRefineRd[i] = abs((int64_t)(cuCost - m_frame->m_classifyRd[offset]));

                /* Calculate prior probability - ranges between 0 and 1 */
                if (trainingCount)
                    probRefine[i] = ((float)m_frame->m_classifyCount[offset] / (float)trainingCount);

                /* Bayesian classification - P(c|x)P(x) = P(x|c)P(c)
                P(c|x) is the posterior probability of class given predictor.
                P(c) is the prior probability of class.
                P(x|c) is the likelihood which is the probability of predictor given class.
                P(x) is the prior probability of predictor.*/
                int curRefineLevel = m_refineLevel - 1;
                if ((diffRefine[i] * probRefine[curRefineLevel]) < (diffRefine[curRefineLevel] * probRefine[i]))
                    varRefineLevel = i + 1;
                if ((diffRefineRd[i] * probRefine[curRefineLevel]) < (diffRefineRd[curRefineLevel] * probRefine[i]))
                    rdRefineLevel = i + 1;
            }
            m_refineLevel = S265_MAX(varRefineLevel, rdRefineLevel);
        }
    }
}

void Analysis::trainCU(const CUData& ctu, const CUGeom& cuGeom, const Mode& bestMode, TrainingData& trainData)
{
    uint32_t depth = cuGeom.depth;
    int classify = 1;
    if (!m_frame->m_classifyFrame)
    {
        /* classify = 1 : CUs for which the save data matches with that after encoding with refine-inter 3
                          and CUs that has split.
           classify = 2 : CUs which are encoded as simple modes (Skip/Merge/2Nx2N).
           classify = 3 : CUs encoded as any other mode. */

        bool refineInter0 = (trainData.predMode == ctu.m_predMode[cuGeom.absPartIdx] &&
            trainData.partSize == ctu.m_partSize[cuGeom.absPartIdx] &&
            trainData.mergeFlag == ctu.m_mergeFlag[cuGeom.absPartIdx]);
        bool refineInter1 = (depth == m_param->maxCUDepth - 1) && trainData.split;
        if (refineInter0 || refineInter1)
            classify = 1;
        else if (trainData.partSize == SIZE_2Nx2N && trainData.partSize == ctu.m_partSize[cuGeom.absPartIdx])
            classify = 2;
        else
            classify = 3;
    }
    else
        classify = m_refineLevel;
    uint64_t cuCost = bestMode.rdCost;
    int offset = (depth * S265_REFINE_INTER_LEVELS) + classify - 1;
    ctu.m_collectCURd[offset] += cuCost;
    ctu.m_collectCUVariance[offset] += trainData.cuVariance;
    ctu.m_collectCUCount[offset]++;
}

/* sets md.bestMode if a valid merge candidate is found, else leaves it NULL */
void Analysis::checkMerge2Nx2N_rd0_4(Mode& skip, Mode& merge, const CUGeom& cuGeom)
{
    uint32_t depth = cuGeom.depth;
    ModeDepth& md = m_modeDepth[depth];
    Yuv *fencYuv = &md.fencYuv;

    /* Note that these two Mode instances are named MERGE and SKIP but they may
     * hold the reverse when the function returns. We toggle between the two modes */
    Mode* tempPred = &merge;
    Mode* bestPred = &skip;

    S265_CHECK(m_slice->m_sliceType != I_SLICE, "Evaluating merge in I slice\n");

    tempPred->initCosts();
    tempPred->cu.setPartSizeSubParts(SIZE_2Nx2N);
    tempPred->cu.setPredModeSubParts(MODE_INTER);
    tempPred->cu.m_mergeFlag[0] = true;

    bestPred->initCosts();
    bestPred->cu.setPartSizeSubParts(SIZE_2Nx2N);
    bestPred->cu.setPredModeSubParts(MODE_INTER);
    bestPred->cu.m_mergeFlag[0] = true;

    MVField candMvField[MRG_MAX_NUM_CANDS][2]; // double length for mv of both lists
    uint8_t candDir[MRG_MAX_NUM_CANDS];
    uint32_t numMergeCand = tempPred->cu.getInterMergeCandidates(0, 0, candMvField, candDir);
    PredictionUnit pu(merge.cu, cuGeom, 0);

    bestPred->sa8dCost = MAX_INT64;
    int bestSadCand = -1;
    int sizeIdx = cuGeom.log2CUSize - 2;
    int safeX, maxSafeMv;
    if (m_param->bIntraRefresh && m_slice->m_sliceType == P_SLICE)
    {
        safeX = m_slice->m_refFrameList[0][0]->m_encData->m_pir.pirEndCol * m_param->maxCUSize - 3;
        maxSafeMv = (safeX - tempPred->cu.m_cuPelX) * 4;
    }
    for (uint32_t i = 0; i < numMergeCand; ++i)
    {
        if (m_bFrameParallel)
        {
            // Parallel slices bound check
            if (m_param->maxSlices > 1)
            {
                // NOTE: First row in slice can't negative
                if (S265_MIN(candMvField[i][0].mv.y, candMvField[i][1].mv.y) < m_sliceMinY)
                    continue;

                // Last row in slice can't reference beyond bound since it is another slice area
                // TODO: we may beyond bound in future since these area have a chance to finish because we use parallel slices. Necessary prepare research on load balance
                if (S265_MAX(candMvField[i][0].mv.y, candMvField[i][1].mv.y) > m_sliceMaxY)
                    continue;
            }

            if (candMvField[i][0].mv.y >= (m_param->searchRange + 1) * 4 ||
                candMvField[i][1].mv.y >= (m_param->searchRange + 1) * 4)
                continue;
        }

        if (m_param->bIntraRefresh && m_slice->m_sliceType == P_SLICE &&
            tempPred->cu.m_cuPelX / m_param->maxCUSize < m_frame->m_encData->m_pir.pirEndCol &&
            candMvField[i][0].mv.x > maxSafeMv)
            // skip merge candidates which reference beyond safe reference area
            continue;

        tempPred->cu.m_mvpIdx[0][0] = (uint8_t)i; // merge candidate ID is stored in L0 MVP idx
        S265_CHECK(m_slice->m_sliceType == B_SLICE || !(candDir[i] & 0x10), " invalid merge for P slice\n");
        tempPred->cu.m_interDir[0] = candDir[i];
        tempPred->cu.m_mv[0][0] = candMvField[i][0].mv;
        tempPred->cu.m_mv[1][0] = candMvField[i][1].mv;
        tempPred->cu.m_refIdx[0][0] = (int8_t)candMvField[i][0].refIdx;
        tempPred->cu.m_refIdx[1][0] = (int8_t)candMvField[i][1].refIdx;
        motionCompensation(tempPred->cu, pu, tempPred->predYuv, true, m_bChromaSa8d && (m_csp != S265_CSP_I400 && m_frame->m_fencPic->m_picCsp != S265_CSP_I400));

        tempPred->sa8dBits = getTUBits(i, numMergeCand);
        tempPred->distortion = primitives.cu[sizeIdx].sa8d(fencYuv->m_buf[0], fencYuv->m_size, tempPred->predYuv.m_buf[0], tempPred->predYuv.m_size);
        if (m_bChromaSa8d && (m_csp != S265_CSP_I400 && m_frame->m_fencPic->m_picCsp != S265_CSP_I400))
        {
            tempPred->distortion += primitives.chroma[m_csp].cu[sizeIdx].sa8d(fencYuv->m_buf[1], fencYuv->m_csize, tempPred->predYuv.m_buf[1], tempPred->predYuv.m_csize);
            tempPred->distortion += primitives.chroma[m_csp].cu[sizeIdx].sa8d(fencYuv->m_buf[2], fencYuv->m_csize, tempPred->predYuv.m_buf[2], tempPred->predYuv.m_csize);
        }
        tempPred->sa8dCost = m_rdCost.calcRdSADCost((uint32_t)tempPred->distortion, tempPred->sa8dBits);

        if (tempPred->sa8dCost < bestPred->sa8dCost)
        {
            bestSadCand = i;
            std::swap(tempPred, bestPred);
        }
    }

    /* force mode decision to take inter or intra */
    if (bestSadCand < 0)
        return;

    /* calculate the motion compensation for chroma for the best mode selected */
    if ((!m_bChromaSa8d && (m_csp != S265_CSP_I400)) || (m_frame->m_fencPic->m_picCsp == S265_CSP_I400 && m_csp != S265_CSP_I400)) /* Chroma MC was done above */
        motionCompensation(bestPred->cu, pu, bestPred->predYuv, false, true);

    if (m_param->rdLevel)
    {
        if (m_param->bLossless)
            bestPred->rdCost = MAX_INT64;
        else
            encodeResAndCalcRdSkipCU(*bestPred);

        /* Encode with residual */
        tempPred->cu.m_mvpIdx[0][0] = (uint8_t)bestSadCand;
        tempPred->cu.setPUInterDir(candDir[bestSadCand], 0, 0);
        tempPred->cu.setPUMv(0, candMvField[bestSadCand][0].mv, 0, 0);
        tempPred->cu.setPUMv(1, candMvField[bestSadCand][1].mv, 0, 0);
        tempPred->cu.setPURefIdx(0, (int8_t)candMvField[bestSadCand][0].refIdx, 0, 0);
        tempPred->cu.setPURefIdx(1, (int8_t)candMvField[bestSadCand][1].refIdx, 0, 0);
        tempPred->sa8dCost = bestPred->sa8dCost;
        tempPred->sa8dBits = bestPred->sa8dBits;
        tempPred->predYuv.copyFromYuv(bestPred->predYuv);

        encodeResAndCalcRdInterCU(*tempPred, cuGeom);

        md.bestMode = tempPred->rdCost < bestPred->rdCost ? tempPred : bestPred;
    }
    else
        md.bestMode = bestPred;

    /* broadcast sets of MV field data */
    md.bestMode->cu.setPUInterDir(candDir[bestSadCand], 0, 0);
    md.bestMode->cu.setPUMv(0, candMvField[bestSadCand][0].mv, 0, 0);
    md.bestMode->cu.setPUMv(1, candMvField[bestSadCand][1].mv, 0, 0);
    md.bestMode->cu.setPURefIdx(0, (int8_t)candMvField[bestSadCand][0].refIdx, 0, 0);
    md.bestMode->cu.setPURefIdx(1, (int8_t)candMvField[bestSadCand][1].refIdx, 0, 0);
    checkDQP(*md.bestMode, cuGeom);
}

/* sets md.bestMode if a valid merge candidate is found, else leaves it NULL */
// 注意 skip 和merge 都有candidate 是同一个构建过程
// 不同的是 skip 无没有残差，candidate 是有残差
void Analysis::checkMerge2Nx2N_rd5_6(Mode& skip, Mode& merge, const CUGeom& cuGeom)
{
    uint32_t depth = cuGeom.depth;

    /* Note that these two Mode instances are named MERGE and SKIP but they may
     * hold the reverse when the function returns. We toggle between the two modes */
    Mode* tempPred = &merge;
    Mode* bestPred = &skip;

    merge.initCosts();
    merge.cu.setPredModeSubParts(MODE_INTER);
    merge.cu.setPartSizeSubParts(SIZE_2Nx2N);
    merge.cu.m_mergeFlag[0] = true;

    skip.initCosts();
    skip.cu.setPredModeSubParts(MODE_INTER);
    skip.cu.setPartSizeSubParts(SIZE_2Nx2N);
    skip.cu.m_mergeFlag[0] = true;

    MVField candMvField[MRG_MAX_NUM_CANDS][2]; // double length for mv of both lists
    uint8_t candDir[MRG_MAX_NUM_CANDS];
    //找到可用的mergeCandidates的个数 记录在 candMvField  candDir 
    uint32_t numMergeCand = merge.cu.getInterMergeCandidates(0, 0, candMvField, candDir);
    PredictionUnit pu(merge.cu, cuGeom, 0);// 使用 merge.cu 的信息构造pu

    bool foundCbf0Merge = false;
    bool triedPZero = false, triedBZero = false;
    bestPred->rdCost = MAX_INT64;

    int safeX, maxSafeMv;
    if (m_param->bIntraRefresh && m_slice->m_sliceType == P_SLICE)
    {
        safeX = m_slice->m_refFrameList[0][0]->m_encData->m_pir.pirEndCol * m_param->maxCUSize - 3;
        maxSafeMv = (safeX - tempPred->cu.m_cuPelX) * 4;
    }
    for (uint32_t i = 0; i < numMergeCand; i++)
    {
        if (m_bFrameParallel)
        {
            // Parallel slices bound check
            if (m_param->maxSlices > 1)
            {
                // NOTE: First row in slice can't negative
                if (S265_MIN(candMvField[i][0].mv.y, candMvField[i][1].mv.y) < m_sliceMinY)
                    continue;

                // Last row in slice can't reference beyond bound since it is another slice area
                // TODO: we may beyond bound in future since these area have a chance to finish because we use parallel slices. Necessary prepare research on load balance
                if (S265_MAX(candMvField[i][0].mv.y, candMvField[i][1].mv.y) > m_sliceMaxY)
                    continue;
            }

            if (candMvField[i][0].mv.y >= (m_param->searchRange + 1) * 4 ||
                candMvField[i][1].mv.y >= (m_param->searchRange + 1) * 4)
                continue;
        }

        /* the merge candidate list is packed with MV(0,0) ref 0 when it is not full */
        if (candDir[i] == 1 && !candMvField[i][0].mv.word && !candMvField[i][0].refIdx)
        {// 单前向0mv0ref 检查是否check过了 如果是是 则跳过
            if (triedPZero)
                continue;
            triedPZero = true;
        }
        else if (candDir[i] == 3 &&
            !candMvField[i][0].mv.word && !candMvField[i][0].refIdx &&
            !candMvField[i][1].mv.word && !candMvField[i][1].refIdx)
        {
        //双向0mv 0ref 检查是否check过了 如果是是 则跳过
            if (triedBZero)
                continue;
            triedBZero = true;
        }

        if (m_param->bIntraRefresh && m_slice->m_sliceType == P_SLICE &&
            tempPred->cu.m_cuPelX / m_param->maxCUSize < m_frame->m_encData->m_pir.pirEndCol &&
            candMvField[i][0].mv.x > maxSafeMv)
            // skip merge candidates which reference beyond safe reference area
            continue;
        tempPred->cu.m_mvpIdx[0][0] = (uint8_t)i;    /* merge candidate ID is stored in L0 MVP idx */
        tempPred->cu.m_interDir[0] = candDir[i];
        tempPred->cu.m_mv[0][0] = candMvField[i][0].mv;
        tempPred->cu.m_mv[1][0] = candMvField[i][1].mv;
        tempPred->cu.m_refIdx[0][0] = (int8_t)candMvField[i][0].refIdx;
        tempPred->cu.m_refIdx[1][0] = (int8_t)candMvField[i][1].refIdx;
        tempPred->cu.setPredModeSubParts(MODE_INTER); /* must be cleared between encode iterations */
        //利用第i个mergeCandidates 做mc过程
        motionCompensation(tempPred->cu, pu, tempPred->predYuv, true, m_csp != S265_CSP_I400);

        uint8_t hasCbf = true;
        bool swapped = false;
        if (!foundCbf0Merge)
        {
            /* if the best prediction has CBF (not a skip) then try merge with residual */

            encodeResAndCalcRdInterCU(*tempPred, cuGeom);
            hasCbf = tempPred->cu.getQtRootCbf(0);
            foundCbf0Merge = !hasCbf;

            if (tempPred->rdCost < bestPred->rdCost)
            {
                std::swap(tempPred, bestPred);
                swapped = true;
            }
        }

        if (!m_param->bLossless && hasCbf)
        {
            /* try merge without residual (skip), if not lossless coding */

            if (swapped)
            {
                tempPred->cu.m_mvpIdx[0][0] = (uint8_t)i;
                tempPred->cu.m_interDir[0] = candDir[i];
                tempPred->cu.m_mv[0][0] = candMvField[i][0].mv;
                tempPred->cu.m_mv[1][0] = candMvField[i][1].mv;
                tempPred->cu.m_refIdx[0][0] = (int8_t)candMvField[i][0].refIdx;
                tempPred->cu.m_refIdx[1][0] = (int8_t)candMvField[i][1].refIdx;
                tempPred->cu.setPredModeSubParts(MODE_INTER);
                tempPred->predYuv.copyFromYuv(bestPred->predYuv);
            }

            encodeResAndCalcRdSkipCU(*tempPred);

            if (tempPred->rdCost < bestPred->rdCost)
                std::swap(tempPred, bestPred);
        }
    }

    if (bestPred->rdCost < MAX_INT64)
    {
        m_modeDepth[depth].bestMode = bestPred;

        /* broadcast sets of MV field data */
        uint32_t bestCand = bestPred->cu.m_mvpIdx[0][0];
        bestPred->cu.setPUInterDir(candDir[bestCand], 0, 0);
        bestPred->cu.setPUMv(0, candMvField[bestCand][0].mv, 0, 0);
        bestPred->cu.setPUMv(1, candMvField[bestCand][1].mv, 0, 0);
        bestPred->cu.setPURefIdx(0, (int8_t)candMvField[bestCand][0].refIdx, 0, 0);
        bestPred->cu.setPURefIdx(1, (int8_t)candMvField[bestCand][1].refIdx, 0, 0);
        checkDQP(*bestPred, cuGeom);
    }
}

void Analysis::checkInter_rd0_4(Mode& interMode, const CUGeom& cuGeom, PartSize partSize, uint32_t refMask[2])
{
    interMode.initCosts();
    interMode.cu.setPartSizeSubParts(partSize);
    interMode.cu.setPredModeSubParts(MODE_INTER);

    predInterSearch(interMode, cuGeom, m_bChromaSa8d && (m_csp != S265_CSP_I400 && m_frame->m_fencPic->m_picCsp != S265_CSP_I400), refMask);

    /* predInterSearch sets interMode.sa8dBits */
    const Yuv& fencYuv = *interMode.fencYuv;
    Yuv& predYuv = interMode.predYuv;
    int part = partitionFromLog2Size(cuGeom.log2CUSize);
    interMode.distortion = primitives.cu[part].sa8d(fencYuv.m_buf[0], fencYuv.m_size, predYuv.m_buf[0], predYuv.m_size);
    if (m_bChromaSa8d && (m_csp != S265_CSP_I400 && m_frame->m_fencPic->m_picCsp != S265_CSP_I400))
    {
        interMode.distortion += primitives.chroma[m_csp].cu[part].sa8d(fencYuv.m_buf[1], fencYuv.m_csize, predYuv.m_buf[1], predYuv.m_csize);
        interMode.distortion += primitives.chroma[m_csp].cu[part].sa8d(fencYuv.m_buf[2], fencYuv.m_csize, predYuv.m_buf[2], predYuv.m_csize);
    }
    interMode.sa8dCost = m_rdCost.calcRdSADCost((uint32_t)interMode.distortion, interMode.sa8dBits);
}

void Analysis::checkInter_rd5_6(Mode& interMode, const CUGeom& cuGeom, PartSize partSize, uint32_t refMask[2])
{
    interMode.initCosts();
    interMode.cu.setPartSizeSubParts(partSize);
    interMode.cu.setPredModeSubParts(MODE_INTER);

    predInterSearch(interMode, cuGeom, m_csp != S265_CSP_I400 && m_frame->m_fencPic->m_picCsp != S265_CSP_I400, refMask);

    /* predInterSearch sets interMode.sa8dBits, but this is ignored */
    encodeResAndCalcRdInterCU(interMode, cuGeom);
}

void Analysis::checkBidir2Nx2N(Mode& inter2Nx2N, Mode& bidir2Nx2N, const CUGeom& cuGeom)
{
    CUData& cu = bidir2Nx2N.cu;
    //如果在cu8x8下 partmode 非2Nx2N则不允许 bidir
    if (cu.isBipredRestriction() || inter2Nx2N.bestME[0][0].cost == MAX_UINT || inter2Nx2N.bestME[0][1].cost == MAX_UINT)
    {
        bidir2Nx2N.sa8dCost = MAX_INT64;
        bidir2Nx2N.rdCost = MAX_INT64;
        return;
    }

    const Yuv& fencYuv = *bidir2Nx2N.fencYuv;
    MV   mvzero(0, 0);
    int  partEnum = cuGeom.log2CUSize - 2;

    bidir2Nx2N.bestME[0][0] = inter2Nx2N.bestME[0][0];
    bidir2Nx2N.bestME[0][1] = inter2Nx2N.bestME[0][1];
    MotionData* bestME = bidir2Nx2N.bestME[0];
    int ref0    = bestME[0].ref;
    MV  mvp0    = bestME[0].mvp;
    int mvpIdx0 = bestME[0].mvpIdx;
    int ref1    = bestME[1].ref;
    MV  mvp1    = bestME[1].mvp;
    int mvpIdx1 = bestME[1].mvpIdx;

    bidir2Nx2N.initCosts();
    cu.setPartSizeSubParts(SIZE_2Nx2N);
    cu.setPredModeSubParts(MODE_INTER);
    cu.setPUInterDir(3, 0, 0);
    cu.setPURefIdx(0, (int8_t)ref0, 0, 0);
    cu.setPURefIdx(1, (int8_t)ref1, 0, 0);
    cu.m_mvpIdx[0][0] = (uint8_t)mvpIdx0;
    cu.m_mvpIdx[1][0] = (uint8_t)mvpIdx1;
    cu.m_mergeFlag[0] = 0;

    /* Estimate cost of BIDIR using best 2Nx2N L0 and L1 motion vectors */
    cu.setPUMv(0, bestME[0].mv, 0, 0);
    cu.m_mvd[0][0] = bestME[0].mv - mvp0;

    cu.setPUMv(1, bestME[1].mv, 0, 0);
    cu.m_mvd[1][0] = bestME[1].mv - mvp1;

    PredictionUnit pu(cu, cuGeom, 0);
    motionCompensation(cu, pu, bidir2Nx2N.predYuv, true, m_bChromaSa8d && (m_csp != S265_CSP_I400 && m_frame->m_fencPic->m_picCsp != S265_CSP_I400));

    int sa8d = primitives.cu[partEnum].sa8d(fencYuv.m_buf[0], fencYuv.m_size, bidir2Nx2N.predYuv.m_buf[0], bidir2Nx2N.predYuv.m_size);
    if (m_bChromaSa8d && (m_csp != S265_CSP_I400 && m_frame->m_fencPic->m_picCsp != S265_CSP_I400))
    {
        /* Add in chroma distortion */
        sa8d += primitives.chroma[m_csp].cu[partEnum].sa8d(fencYuv.m_buf[1], fencYuv.m_csize, bidir2Nx2N.predYuv.m_buf[1], bidir2Nx2N.predYuv.m_csize);
        sa8d += primitives.chroma[m_csp].cu[partEnum].sa8d(fencYuv.m_buf[2], fencYuv.m_csize, bidir2Nx2N.predYuv.m_buf[2], bidir2Nx2N.predYuv.m_csize);
    }
    bidir2Nx2N.sa8dBits = bestME[0].bits + bestME[1].bits + m_listSelBits[2] - (m_listSelBits[0] + m_listSelBits[1]);
    bidir2Nx2N.sa8dCost = sa8d + m_rdCost.getCost(bidir2Nx2N.sa8dBits);

    bool bTryZero = bestME[0].mv.notZero() || bestME[1].mv.notZero();
    if (bTryZero)
    {
        /* Do not try zero MV if unidir motion predictors are beyond
         * valid search area */
        MV mvmin, mvmax;
        int merange = S265_MAX(m_param->sourceWidth, m_param->sourceHeight);
        setSearchRange(cu, mvzero, merange, mvmin, mvmax);
        mvmax.y += 2; // there is some pad for subpel refine
        mvmin <<= 2;
        mvmax <<= 2;

        bTryZero &= bestME[0].mvp.checkRange(mvmin, mvmax);
        bTryZero &= bestME[1].mvp.checkRange(mvmin, mvmax);
    }
    if (bTryZero)
    {
        /* Estimate cost of BIDIR using coincident blocks */
        Yuv& tmpPredYuv = m_rqt[cuGeom.depth].tmpPredYuv;

        int zsa8d;

        if (m_bChromaSa8d && (m_csp != S265_CSP_I400 && m_frame->m_fencPic->m_picCsp != S265_CSP_I400))
        {
            cu.m_mv[0][0] = mvzero;
            cu.m_mv[1][0] = mvzero;

            motionCompensation(cu, pu, tmpPredYuv, true, true);
            zsa8d  = primitives.cu[partEnum].sa8d(fencYuv.m_buf[0], fencYuv.m_size, tmpPredYuv.m_buf[0], tmpPredYuv.m_size);
            zsa8d += primitives.chroma[m_csp].cu[partEnum].sa8d(fencYuv.m_buf[1], fencYuv.m_csize, tmpPredYuv.m_buf[1], tmpPredYuv.m_csize);
            zsa8d += primitives.chroma[m_csp].cu[partEnum].sa8d(fencYuv.m_buf[2], fencYuv.m_csize, tmpPredYuv.m_buf[2], tmpPredYuv.m_csize);

        }
        else
        {
            pixel *fref0 = m_slice->m_mref[0][ref0].getLumaAddr(pu.ctuAddr, pu.cuAbsPartIdx);
            pixel *fref1 = m_slice->m_mref[1][ref1].getLumaAddr(pu.ctuAddr, pu.cuAbsPartIdx);
            intptr_t refStride = m_slice->m_mref[0][0].lumaStride;
            primitives.pu[partEnum].pixelavg_pp[(tmpPredYuv.m_size % 64 == 0) && (refStride % 64 == 0)](tmpPredYuv.m_buf[0], tmpPredYuv.m_size, fref0, refStride, fref1, refStride, 32);
            zsa8d = primitives.cu[partEnum].sa8d(fencYuv.m_buf[0], fencYuv.m_size, tmpPredYuv.m_buf[0], tmpPredYuv.m_size);
        }
        uint32_t bits0 = bestME[0].bits - m_me.bitcost(bestME[0].mv, mvp0) + m_me.bitcost(mvzero, mvp0);
        uint32_t bits1 = bestME[1].bits - m_me.bitcost(bestME[1].mv, mvp1) + m_me.bitcost(mvzero, mvp1);
        uint32_t zcost = zsa8d + m_rdCost.getCost(bits0) + m_rdCost.getCost(bits1);

        /* refine MVP selection for zero mv, updates: mvp, mvpidx, bits, cost */
        mvp0 = checkBestMVP(inter2Nx2N.amvpCand[0][ref0], mvzero, mvpIdx0, bits0, zcost);
        mvp1 = checkBestMVP(inter2Nx2N.amvpCand[1][ref1], mvzero, mvpIdx1, bits1, zcost);

        uint32_t zbits = bits0 + bits1 + m_listSelBits[2] - (m_listSelBits[0] + m_listSelBits[1]);
        zcost = zsa8d + m_rdCost.getCost(zbits);

        if (zcost < bidir2Nx2N.sa8dCost)
        {
            bidir2Nx2N.sa8dBits = zbits;
            bidir2Nx2N.sa8dCost = zcost;

            cu.setPUMv(0, mvzero, 0, 0);
            cu.m_mvd[0][0] = mvzero - mvp0;
            cu.m_mvpIdx[0][0] = (uint8_t)mvpIdx0;

            cu.setPUMv(1, mvzero, 0, 0);
            cu.m_mvd[1][0] = mvzero - mvp1;
            cu.m_mvpIdx[1][0] = (uint8_t)mvpIdx1;

            if (m_bChromaSa8d) /* real MC was already performed */
                bidir2Nx2N.predYuv.copyFromYuv(tmpPredYuv);
            else
                motionCompensation(cu, pu, bidir2Nx2N.predYuv, true, m_csp != S265_CSP_I400 && m_frame->m_fencPic->m_picCsp != S265_CSP_I400);
        }
        else if (m_bChromaSa8d && (m_csp != S265_CSP_I400 && m_frame->m_fencPic->m_picCsp != S265_CSP_I400))
        {
            /* recover overwritten motion vectors */
            cu.m_mv[0][0] = bestME[0].mv;
            cu.m_mv[1][0] = bestME[1].mv;
        }
    }
}

void Analysis::encodeResidue(const CUData& ctu, const CUGeom& cuGeom)
{
    if (cuGeom.depth < ctu.m_cuDepth[cuGeom.absPartIdx] && cuGeom.depth < ctu.m_encData->m_param->maxCUDepth)
    {
        for (uint32_t subPartIdx = 0; subPartIdx < 4; subPartIdx++)
        {
            const CUGeom& childGeom = *(&cuGeom + cuGeom.childOffset + subPartIdx);
            if (childGeom.flags & CUGeom::PRESENT)
                encodeResidue(ctu, childGeom);
        }
        return;
    }

    uint32_t absPartIdx = cuGeom.absPartIdx;
    int sizeIdx = cuGeom.log2CUSize - 2;

    /* reuse the bestMode data structures at the current depth */
    Mode *bestMode = m_modeDepth[cuGeom.depth].bestMode;
    CUData& cu = bestMode->cu;

    cu.copyFromPic(ctu, cuGeom, m_csp);

    PicYuv& reconPic = *m_frame->m_reconPic;

    Yuv& fencYuv = m_modeDepth[cuGeom.depth].fencYuv;
    if (cuGeom.depth)
        m_modeDepth[0].fencYuv.copyPartToYuv(fencYuv, absPartIdx);
    S265_CHECK(bestMode->fencYuv == &fencYuv, "invalid fencYuv\n");

    if (cu.isIntra(0))
    {
        ProfileCUScope(ctu, intraRDOElapsedTime[cuGeom.depth], countIntraRDO[cuGeom.depth]); // not really RDO, but close enough
        
        uint32_t tuDepthRange[2];
        cu.getIntraTUQtDepthRange(tuDepthRange, 0);

        residualTransformQuantIntra(*bestMode, cuGeom, 0, 0, tuDepthRange);
        if (m_csp != S265_CSP_I400 && m_frame->m_fencPic->m_picCsp != S265_CSP_I400)
        {
            getBestIntraModeChroma(*bestMode, cuGeom);
            residualQTIntraChroma(*bestMode, cuGeom, 0, 0);
        }
    }
    else // if (cu.isInter(0))
    {
        ProfileCUScope(ctu, interRDOElapsedTime[cuGeom.depth], countInterRDO[cuGeom.depth]); // not really RDO, but close enough

        S265_CHECK(!ctu.isSkipped(absPartIdx), "skip not expected prior to transform\n");

        /* Calculate residual for current CU part into depth sized resiYuv */

        ShortYuv& resiYuv = m_rqt[cuGeom.depth].tmpResiYuv;

        /* at RD 0, the prediction pixels are accumulated into the top depth predYuv */
        Yuv& predYuv = m_modeDepth[0].bestMode->predYuv;
        pixel* predY = predYuv.getLumaAddr(absPartIdx);

        primitives.cu[sizeIdx].sub_ps(resiYuv.m_buf[0], resiYuv.m_size,
                                      fencYuv.m_buf[0], predY,
                                      fencYuv.m_size, predYuv.m_size);

        if (m_csp != S265_CSP_I400 && m_frame->m_fencPic->m_picCsp != S265_CSP_I400)
        {
            pixel* predU = predYuv.getCbAddr(absPartIdx);
            pixel* predV = predYuv.getCrAddr(absPartIdx);
            primitives.chroma[m_csp].cu[sizeIdx].sub_ps(resiYuv.m_buf[1], resiYuv.m_csize,
                                                 fencYuv.m_buf[1], predU,
                                                 fencYuv.m_csize, predYuv.m_csize);

            primitives.chroma[m_csp].cu[sizeIdx].sub_ps(resiYuv.m_buf[2], resiYuv.m_csize,
                                                 fencYuv.m_buf[2], predV,
                                                 fencYuv.m_csize, predYuv.m_csize);
        }

        uint32_t tuDepthRange[2];
        cu.getInterTUQtDepthRange(tuDepthRange, 0);

        residualTransformQuantInter(*bestMode, cuGeom, 0, 0, tuDepthRange);

        if (cu.m_mergeFlag[0] && cu.m_partSize[0] == SIZE_2Nx2N && !cu.getQtRootCbf(0))
            cu.setPredModeSubParts(MODE_SKIP);

        /* residualTransformQuantInter() wrote transformed residual back into
         * resiYuv. Generate the recon pixels by adding it to the prediction */

        if (cu.m_cbf[0][0])
        {
            bool reconPicAlign = (reconPic.m_cuOffsetY[cu.m_cuAddr] + reconPic.m_buOffsetY[absPartIdx]) % 64 == 0;
            bool predYalign = predYuv.getAddrOffset(absPartIdx, predYuv.m_size) % 64 == 0;
            primitives.cu[sizeIdx].add_ps[reconPicAlign && predYalign && (reconPic.m_stride % 64 == 0) && (predYuv.m_size % 64 == 0) &&
                (resiYuv.m_size % 64 == 0)](reconPic.getLumaAddr(cu.m_cuAddr, absPartIdx), reconPic.m_stride, predY, resiYuv.m_buf[0], predYuv.m_size, resiYuv.m_size);
        }
        else
            primitives.cu[sizeIdx].copy_pp(reconPic.getLumaAddr(cu.m_cuAddr, absPartIdx), reconPic.m_stride,
                                           predY, predYuv.m_size);
        if (m_csp != S265_CSP_I400 && m_frame->m_fencPic->m_picCsp != S265_CSP_I400)
        {
             pixel* predU = predYuv.getCbAddr(absPartIdx);
             pixel* predV = predYuv.getCrAddr(absPartIdx);
             if (cu.m_cbf[1][0])
             {
                 bool reconPicAlign = (reconPic.m_cuOffsetC[cu.m_cuAddr] + reconPic.m_buOffsetC[absPartIdx]) % 64 == 0;
                 bool predUalign = predYuv.getChromaAddrOffset(absPartIdx) % 64 == 0;
                 primitives.chroma[m_csp].cu[sizeIdx].add_ps[reconPicAlign && predUalign && (reconPic.m_strideC % 64 == 0) && (predYuv.m_csize % 64 == 0) &&
                     (resiYuv.m_csize % 64 == 0)](reconPic.getCbAddr(cu.m_cuAddr, absPartIdx), reconPic.m_strideC, predU, resiYuv.m_buf[1], predYuv.m_csize, resiYuv.m_csize);
             }
            else
                primitives.chroma[m_csp].cu[sizeIdx].copy_pp(reconPic.getCbAddr(cu.m_cuAddr, absPartIdx), reconPic.m_strideC,
                                                         predU, predYuv.m_csize);

            if (cu.m_cbf[2][0])
            {
                bool reconPicAlign = (reconPic.m_cuOffsetC[cu.m_cuAddr] + reconPic.m_buOffsetC[absPartIdx]) % 64 == 0;
                bool predValign = predYuv.getChromaAddrOffset(absPartIdx) % 64 == 0;
                primitives.chroma[m_csp].cu[sizeIdx].add_ps[reconPicAlign && predValign && (reconPic.m_strideC % 64 == 0) && (predYuv.m_csize % 64 == 0) &&
                    (resiYuv.m_csize % 64 == 0)](reconPic.getCrAddr(cu.m_cuAddr, absPartIdx), reconPic.m_strideC, predV, resiYuv.m_buf[2], predYuv.m_csize, resiYuv.m_csize);
            }
            else
                primitives.chroma[m_csp].cu[sizeIdx].copy_pp(reconPic.getCrAddr(cu.m_cuAddr, absPartIdx), reconPic.m_strideC,
                                                         predV, predYuv.m_csize);
        }
    }

    cu.updatePic(cuGeom.depth, m_frame->m_fencPic->m_picCsp);
}

void Analysis::addSplitFlagCost(Mode& mode, uint32_t depth)
{
    if (m_param->rdLevel >= 3)
    {
        /* code the split flag (0 or 1) and update bit costs */
        mode.contexts.resetBits();
        mode.contexts.codeSplitFlag(mode.cu, 0, depth);
        uint32_t bits = mode.contexts.getNumberOfWrittenBits();
        mode.totalBits += bits;
        updateModeCost(mode);
    }
    else if (m_param->rdLevel <= 1)
    {
        mode.sa8dBits++;
        mode.sa8dCost = m_rdCost.calcRdSADCost((uint32_t)mode.distortion, mode.sa8dBits);
    }
    else
    {
        mode.totalBits++;
        updateModeCost(mode);
    }
}

uint32_t Analysis::topSkipMinDepth(const CUData& parentCTU, const CUGeom& cuGeom)
{
    /* Do not attempt to code a block larger than the largest block in the
     * co-located CTUs in L0 and L1 */
    int currentQP = parentCTU.m_qp[0];
    int previousQP = currentQP;
    uint32_t minDepth0 = 4, minDepth1 = 4;
    uint32_t sum = 0;
    int numRefs = 0;
    if (m_slice->m_numRefIdx[0])
    {
        numRefs++;
        const CUData& cu = *m_slice->m_refFrameList[0][0]->m_encData->getPicCTU(parentCTU.m_cuAddr);
        previousQP = cu.m_qp[0];
        if (!cu.m_cuDepth[cuGeom.absPartIdx])
            return 0;
        for (uint32_t i = 0; i < cuGeom.numPartitions; i += 4)
        {
            uint32_t d = cu.m_cuDepth[cuGeom.absPartIdx + i];
            minDepth0 = S265_MIN(d, minDepth0);
            sum += d;
        }
    }
    if (m_slice->m_numRefIdx[1])
    {
        numRefs++;
        const CUData& cu = *m_slice->m_refFrameList[1][0]->m_encData->getPicCTU(parentCTU.m_cuAddr);
        if (!cu.m_cuDepth[cuGeom.absPartIdx])
            return 0;
        for (uint32_t i = 0; i < cuGeom.numPartitions; i += 4)
        {
            uint32_t d = cu.m_cuDepth[cuGeom.absPartIdx + i];
            minDepth1 = S265_MIN(d, minDepth1);
            sum += d;
        }
    }
    if (!numRefs)
        return 0;

    uint32_t minDepth = S265_MIN(minDepth0, minDepth1);
    uint32_t thresh = minDepth * numRefs * (cuGeom.numPartitions >> 2);

    /* allow block size growth if QP is raising or avg depth is
     * less than 1.5 of min depth */
    if (minDepth && currentQP >= previousQP && (sum <= thresh + (thresh >> 1)))
        minDepth -= 1;

    return minDepth;
}

/* returns true if recursion should be stopped */
bool Analysis::recursionDepthCheck(const CUData& parentCTU, const CUGeom& cuGeom, const Mode& bestMode)
{
    /* early exit when the RD cost of best mode at depth n is less than the sum
     * of average of RD cost of the neighbor CU's(above, aboveleft, aboveright,
     * left, colocated) and avg cost of that CU at depth "n" with weightage for
     * each quantity */

    uint32_t depth = cuGeom.depth;
    FrameData& curEncData = *m_frame->m_encData;
    FrameData::RCStatCU& cuStat = curEncData.m_cuStat[parentCTU.m_cuAddr];
    uint64_t cuCost = cuStat.avgCost[depth] * cuStat.count[depth];
    uint64_t cuCount = cuStat.count[depth];

    uint64_t neighCost = 0, neighCount = 0;
    const CUData* above = parentCTU.m_cuAbove;
    if (above)
    {
        FrameData::RCStatCU& astat = curEncData.m_cuStat[above->m_cuAddr];
        neighCost += astat.avgCost[depth] * astat.count[depth];
        neighCount += astat.count[depth];

        const CUData* aboveLeft = parentCTU.m_cuAboveLeft;
        if (aboveLeft)
        {
            FrameData::RCStatCU& lstat = curEncData.m_cuStat[aboveLeft->m_cuAddr];
            neighCost += lstat.avgCost[depth] * lstat.count[depth];
            neighCount += lstat.count[depth];
        }

        const CUData* aboveRight = parentCTU.m_cuAboveRight;
        if (aboveRight)
        {
            FrameData::RCStatCU& rstat = curEncData.m_cuStat[aboveRight->m_cuAddr];
            neighCost += rstat.avgCost[depth] * rstat.count[depth];
            neighCount += rstat.count[depth];
        }
    }
    const CUData* left = parentCTU.m_cuLeft;
    if (left)
    {
        FrameData::RCStatCU& nstat = curEncData.m_cuStat[left->m_cuAddr];
        neighCost += nstat.avgCost[depth] * nstat.count[depth];
        neighCount += nstat.count[depth];
    }

    // give 60% weight to all CU's and 40% weight to neighbour CU's
    if (neighCount + cuCount)
    {
        uint64_t avgCost = ((3 * cuCost) + (2 * neighCost)) / ((3 * cuCount) + (2 * neighCount));
        uint64_t curCost = m_param->rdLevel > 1 ? bestMode.rdCost : bestMode.sa8dCost;
        if (curCost < avgCost && avgCost)
            return true;
    }

    return false;
}

bool Analysis::complexityCheckCU(const Mode& bestMode)
{
    if (m_param->recursionSkipMode == RDCOST_BASED_RSKIP)
    {
        uint32_t mean = 0;
        uint32_t homo = 0;
        uint32_t cuSize = bestMode.fencYuv->m_size;
        for (uint32_t y = 0; y < cuSize; y++) {
            for (uint32_t x = 0; x < cuSize; x++) {
                mean += (bestMode.fencYuv->m_buf[0][y * cuSize + x]);
            }
        }
        mean = mean / (cuSize * cuSize);
        for (uint32_t y = 0; y < cuSize; y++) {
            for (uint32_t x = 0; x < cuSize; x++) {
                homo += abs(int(bestMode.fencYuv->m_buf[0][y * cuSize + x] - mean));
            }
        }
        homo = homo / (cuSize * cuSize);

        if (homo < (.1 * mean))
            return true;

        return false;
    }
    else
    {
        int blockType = bestMode.cu.m_log2CUSize[0] - LOG2_UNIT_SIZE;
        int shift = bestMode.cu.m_log2CUSize[0] * LOG2_UNIT_SIZE;
        intptr_t stride = m_frame->m_fencPic->m_stride;
        intptr_t blockOffsetLuma = bestMode.cu.m_cuPelX + bestMode.cu.m_cuPelY * stride;
        uint64_t sum_ss = primitives.cu[blockType].var(m_frame->m_edgeBitPic + blockOffsetLuma, stride);
        uint32_t sum = (uint32_t)sum_ss;// 低32bit
        uint32_t ss = (uint32_t)(sum_ss >> 32);// 高32bits
        uint32_t pixelCount = 1 << shift;
        double cuEdgeVariance = (ss - ((double)sum * sum / pixelCount)) / pixelCount;

        if (cuEdgeVariance > (double)m_param->edgeVarThreshold)
            return false;
        else
            return true;
    }
 }

uint32_t Analysis::calculateCUVariance(const CUData& ctu, const CUGeom& cuGeom)
{
    uint32_t cuVariance = 0;
    uint32_t *blockVariance = m_frame->m_lowres.blockVariance;
    int loopIncr = (m_param->rc.qgSize == 8) ? 8 : 16;

    uint32_t width = m_frame->m_fencPic->m_picWidth;
    uint32_t height = m_frame->m_fencPic->m_picHeight;
    uint32_t block_x = ctu.m_cuPelX + g_zscanToPelX[cuGeom.absPartIdx];
    uint32_t block_y = ctu.m_cuPelY + g_zscanToPelY[cuGeom.absPartIdx];
    uint32_t maxCols = (m_frame->m_fencPic->m_picWidth + (loopIncr - 1)) / loopIncr;
    uint32_t blockSize = m_param->maxCUSize >> cuGeom.depth;
    uint32_t cnt = 0; 

    for (uint32_t block_yy = block_y; block_yy < block_y + blockSize && block_yy < height; block_yy += loopIncr)
    {
        for (uint32_t block_xx = block_x; block_xx < block_x + blockSize && block_xx < width; block_xx += loopIncr)
        {
            uint32_t idx = ((block_yy / loopIncr) * (maxCols)) + (block_xx / loopIncr);
            cuVariance += blockVariance[idx];
            cnt++;
        }
    }
    return cuVariance / cnt;
}

double Analysis::aqQPOffset(const CUData& ctu, const CUGeom& cuGeom)
{
    uint32_t aqDepth = S265_MIN(cuGeom.depth, m_frame->m_lowres.maxAQDepth - 1);
    PicQPAdaptationLayer* pQPLayer = &m_frame->m_lowres.pAQLayer[aqDepth];

    uint32_t aqPosX = (ctu.m_cuPelX + g_zscanToPelX[cuGeom.absPartIdx]) / pQPLayer->aqPartWidth;
    uint32_t aqPosY = (ctu.m_cuPelY + g_zscanToPelY[cuGeom.absPartIdx]) / pQPLayer->aqPartHeight;

    uint32_t aqStride = pQPLayer->numAQPartInWidth;

    double dQpOffset = pQPLayer->dQpOffset[aqPosY * aqStride + aqPosX];
    return dQpOffset;
}

double Analysis::cuTreeQPOffset(const CUData& ctu, const CUGeom& cuGeom)
{
    uint32_t aqDepth = S265_MIN(cuGeom.depth, m_frame->m_lowres.maxAQDepth - 1);
    PicQPAdaptationLayer* pcAQLayer = &m_frame->m_lowres.pAQLayer[aqDepth];

    uint32_t aqPosX = (ctu.m_cuPelX + g_zscanToPelX[cuGeom.absPartIdx]) / pcAQLayer->aqPartWidth;
    uint32_t aqPosY = (ctu.m_cuPelY + g_zscanToPelY[cuGeom.absPartIdx]) / pcAQLayer->aqPartHeight;

    uint32_t aqStride = pcAQLayer->numAQPartInWidth;

    double dQpOffset = pcAQLayer->dCuTreeOffset[aqPosY * aqStride + aqPosX];
    return dQpOffset;
}
// 参数缺省默认值 int32_t complexCheck = 0, double baseQP = -1
// 得倒与cusize 对应大小的 baseqp + qpoffset
int Analysis::calculateQpforCuSize(const CUData& ctu, const CUGeom& cuGeom, int32_t complexCheck, double baseQp)
{
    FrameData& curEncData = *m_frame->m_encData;
    double qp = baseQp >= 0 ? baseQp : curEncData.m_cuStat[ctu.m_cuAddr].baseQp;
    bool bCuTreeOffset = IS_REFERENCED(m_frame) && m_param->rc.cuTree && !complexCheck;

    if (m_param->rc.hevcAq)
    {
        /* Use cuTree offsets if cuTree enabled and frame is referenced, else use AQ offsets */
        double dQpOffset = 0;
        if (bCuTreeOffset)
        {
            dQpOffset = cuTreeQPOffset(ctu, cuGeom);// get current ctu_size cutree_qpoffset
        }
        else
        {
            dQpOffset = aqQPOffset(ctu, cuGeom);// get current ctu_size aq_qpoffset
            if (complexCheck)
            {
                int32_t offset = (int32_t)(dQpOffset * 100 + .5);
                double threshold = (1 - ((s265_ADAPT_RD_STRENGTH - m_param->dynamicRd) * 0.5));
                int32_t max_threshold = (int32_t)(threshold * 100 + .5);
                return (offset < max_threshold);
            }
        }
        qp += dQpOffset;
    }
    else
    {
        int loopIncr = (m_param->rc.qgSize == 8) ? 8 : 16;
        /* Use cuTree offsets if cuTree enabled and frame is referenced, else use AQ offsets */
        double *qpoffs = bCuTreeOffset ? m_frame->m_lowres.qpCuTreeOffset : m_frame->m_lowres.qpAqOffset;
        if (qpoffs)
        {
            uint32_t width = m_frame->m_fencPic->m_picWidth;
            uint32_t height = m_frame->m_fencPic->m_picHeight;
            uint32_t block_x = ctu.m_cuPelX + g_zscanToPelX[cuGeom.absPartIdx];
            uint32_t block_y = ctu.m_cuPelY + g_zscanToPelY[cuGeom.absPartIdx];
            uint32_t maxCols = (m_frame->m_fencPic->m_picWidth + (loopIncr - 1)) / loopIncr;
            uint32_t blockSize = m_param->maxCUSize >> cuGeom.depth;
            double dQpOffset = 0;
            uint32_t cnt = 0;
            for (uint32_t block_yy = block_y; block_yy < block_y + blockSize && block_yy < height; block_yy += loopIncr)
            {
                for (uint32_t block_xx = block_x; block_xx < block_x + blockSize && block_xx < width; block_xx += loopIncr)
                {
                    uint32_t idx = ((block_yy / loopIncr) * (maxCols)) + (block_xx / loopIncr);
                    dQpOffset += qpoffs[idx];
                    cnt++;
                }
            }
            dQpOffset /= cnt;
            qp += dQpOffset;
            if (complexCheck)
            {
                int32_t offset = (int32_t)(dQpOffset * 100 + .5);
                double threshold = (1 - ((s265_ADAPT_RD_STRENGTH - m_param->dynamicRd) * 0.5));
                int32_t max_threshold = (int32_t)(threshold * 100 + .5);
                return (offset < max_threshold);
            }
        }
    }

    return s265_clip3(m_param->rc.qpMin, m_param->rc.qpMax, (int)(qp + 0.5));
}

void Analysis::normFactor(const pixel* src, uint32_t blockSize, CUData& ctu, int qp, TextType ttype)
{
    static const int ssim_c1 = (int)(.01 * .01 * PIXEL_MAX * PIXEL_MAX * 64 + .5); // 416
    static const int ssim_c2 = (int)(.03 * .03 * PIXEL_MAX * PIXEL_MAX * 64 * 63 + .5); // 235963
    int shift = (S265_DEPTH - 8);

    double s = 1 + 0.005 * qp;

    // Calculate denominator of normalization factor
    uint64_t fDc_den = 0, fAc_den = 0;

    // 1. Calculate dc component
    uint64_t z_o = 0;
    for (uint32_t block_yy = 0; block_yy < blockSize; block_yy += 4)
    {
        for (uint32_t block_xx = 0; block_xx < blockSize; block_xx += 4)
        {
            uint32_t temp = src[block_yy * blockSize + block_xx] >> shift;
            z_o += temp * temp; // 2 * (Z(0)) pow(2)
        }
    }
    fDc_den = (2 * z_o)  + (blockSize * blockSize * ssim_c1); // 2 * (Z(0)) pow(2) + N * C1
    fDc_den /= ((blockSize >> 2) * (blockSize >> 2));

    // 2. Calculate ac component
    uint64_t z_k = 0;
    int block = (int)(((log(blockSize) / log(2)) - 2) + 0.5);
    primitives.cu[block].normFact(src, blockSize, shift, &z_k);

    // Remove the DC part
    z_k -= z_o;

    fAc_den = z_k + int(s * z_k) + ssim_c2;
    fAc_den /= ((blockSize >> 2) * (blockSize >> 2));

    ctu.m_fAc_den[ttype] = fAc_den;
    ctu.m_fDc_den[ttype] = fDc_den;
}

void Analysis::calculateNormFactor(CUData& ctu, int qp)
{
    const pixel* srcY = m_modeDepth[0].fencYuv.m_buf[0];
    uint32_t blockSize = m_modeDepth[0].fencYuv.m_size;

    normFactor(srcY, blockSize, ctu, qp, TEXT_LUMA);

    if (m_csp != S265_CSP_I400 && m_frame->m_fencPic->m_picCsp != S265_CSP_I400)
    {
        const pixel* srcU = m_modeDepth[0].fencYuv.m_buf[1];
        const pixel* srcV = m_modeDepth[0].fencYuv.m_buf[2];
        uint32_t blockSizeC = m_modeDepth[0].fencYuv.m_csize;

        normFactor(srcU, blockSizeC, ctu, qp, TEXT_CHROMA_U);
        normFactor(srcV, blockSizeC, ctu, qp, TEXT_CHROMA_V);
    }
}
