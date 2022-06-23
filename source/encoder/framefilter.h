/*****************************************************************************
 * Copyright (C) 2013-2020 MulticoreWare, Inc
 *
 * Authors: Chung Shin Yee <shinyee@multicorewareinc.com>
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

#ifndef S265_FRAMEFILTER_H
#define S265_FRAMEFILTER_H

#include "common.h"
#include "frame.h"
#include "deblock.h"
#include "sao.h"
#include "threadpool.h" // class BondedTaskGroup

namespace S265_NS {
// private s265 namespace

class Encoder;
class Entropy;
class FrameEncoder;
struct ThreadLocalData;

// Manages the processing of a single frame loopfilter
class FrameFilter
{
public:

    s265_param*   m_param;
    Frame*        m_frame;
    int           m_useSao;
    FrameEncoder* m_frameEncoder;
    int           m_hChromaShift;
    int           m_vChromaShift;
    int           m_pad[2];

    int           m_numRows;
    int           m_numCols;
    int           m_saoRowDelay;
    int           m_lastHeight;//最后一行的高度可能不足一个ctu 高度
    int           m_lastWidth;//最后一列的宽度可能不足一个ctu宽度
    
    ThreadSafeInteger integralCompleted;     /* check if integral calculation is completed in this row */

    void*         m_ssimBuf;        /* Temp storage for ssim computation */

#define MAX_PFILTER_CUS     (4) /* maximum CUs for every thread */
    class ParallelFilter : public Deblock
    {
    public:
        uint32_t            m_rowHeight;
        int                 m_row;
        uint32_t            m_rowAddr;//对应filter 行的 首个ctu地址
        FrameFilter*        m_frameFilter;
        FrameData*          m_encData;
        ParallelFilter*     m_prevRow;
        SAO                 m_sao;
        ThreadSafeInteger   m_lastCol;          /* The column that next to process */
        ThreadSafeInteger   m_allowedCol;       /* The column that processed from Encode pipeline */
        ThreadSafeInteger   m_lastDeblocked;   /* The column that finished all of Deblock stages  */

        ParallelFilter()
            : m_rowHeight(0)
            , m_row(0)
            , m_rowAddr(0)
            , m_frameFilter(NULL)
            , m_encData(NULL)
            , m_prevRow(NULL)
        {
        }

        ~ParallelFilter()
        { }
        // 普通成员函数
        void processTasks(int workerThreadId);

        // Apply SAO on a CU in current row
        void processSaoCTU(SAOParam *saoParam, int col);

        // Copy and Save SAO reference pixels for SAO Rdo decide
        void copySaoAboveRef(const CUData *ctu, PicYuv* reconPic, uint32_t cuAddr, int col);

        // Post-Process (Border extension)
        void processPostCu(int col) const;

        uint32_t getCUHeight() const
        {
            return m_rowHeight;
        }
    };

    ParallelFilter*     m_parallelFilter;

    FrameFilter()
        : m_param(NULL)
        , m_frame(NULL)
        , m_frameEncoder(NULL)
        , m_ssimBuf(NULL)
        , m_parallelFilter(NULL)
    {
    }

    uint32_t getCUWidth(int colNum) const
    {   //出了最右侧一列ctu的宽度可能不足一个ctu宽度，其他都为64
        return (colNum == (int)m_numCols - 1) ? m_lastWidth : m_param->maxCUSize;
    }

    void init(Encoder *top, FrameEncoder *frame, int numRows, uint32_t numCols);
    void destroy();

    void start(Frame *pic, Entropy& initState);

    void processRow(int row);
    void processPostRow(int row);
    void computeMEIntegral(int row);
};
}

#endif // ifndef S265_FRAMEFILTER_H
