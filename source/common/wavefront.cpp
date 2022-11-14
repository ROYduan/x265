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
 * For more information, contact us at license @ s265.com
 *****************************************************************************/

#include "threadpool.h"
#include "threading.h"
#include "wavefront.h"
#include "common.h"

namespace S265_NS {
// s265 private namespace
// 这里的参数 numRows 为两倍的 ctu 行数
bool WaveFront::init(int numRows)
{
    m_numRows = numRows;
    //numRows 代表总共有多少个任务 （wpp下为 2倍的 ctu 行数）
    m_numWords = (numRows + 31) >> 5;//一个words 有32个bit可以管理32个任务
    m_internalDependencyBitmap = S265_MALLOC(uint32_t, m_numWords);
    if (m_internalDependencyBitmap)
        memset((void*)m_internalDependencyBitmap, 0, sizeof(uint32_t) * m_numWords);

    m_externalDependencyBitmap = S265_MALLOC(uint32_t, m_numWords);
    if (m_externalDependencyBitmap)
        memset((void*)m_externalDependencyBitmap, 0, sizeof(uint32_t) * m_numWords);

    m_row_to_idx = S265_MALLOC(uint32_t, m_numRows);
    m_idx_to_row = S265_MALLOC(uint32_t, m_numRows);

    return m_internalDependencyBitmap && m_externalDependencyBitmap;
}

WaveFront::~WaveFront()
{
    s265_free((void*)m_row_to_idx);
    s265_free((void*)m_idx_to_row);

    s265_free((void*)m_internalDependencyBitmap);
    s265_free((void*)m_externalDependencyBitmap);
}

void WaveFront::clearEnabledRowMask()
{
    memset((void*)m_externalDependencyBitmap, 0, sizeof(uint32_t) * m_numWords);
    memset((void*)m_internalDependencyBitmap, 0, sizeof(uint32_t) * m_numWords);
}

void WaveFront::enqueueRow(int row)
{
    uint32_t bit = 1 << (row & 31);
    ATOMIC_OR(&m_internalDependencyBitmap[row >> 5], bit);
}

void WaveFront::enableRow(int row)
{
    uint32_t bit = 1 << (row & 31);
    ATOMIC_OR(&m_externalDependencyBitmap[row >> 5], bit);
}

void WaveFront::enableAllRows()
{
    memset((void*)m_externalDependencyBitmap, ~0, sizeof(uint32_t) * m_numWords);
}

bool WaveFront::dequeueRow(int row)
{
    uint32_t bit = 1 << (row & 31);
    //先取出 然后在与上
    return !!(ATOMIC_AND(&m_internalDependencyBitmap[row >> 5], ~bit) & bit);
}

void WaveFront::findJob(int threadId)
{
    unsigned long id;

    /* Loop over each word until all available rows are finished */
    //总共有 32bit*m_numWords 个bit，每个bit 对应一个processrow（一个独立的任务 该任务可能是编码 也可能是filter）
    //遍历每一个32bit bitmap
    for (int w = 0; w < m_numWords; w++)
    {
        // 当前32个任务里面哪些任务的内部/外部依赖条件都已经满足
        uint32_t oldval = m_internalDependencyBitmap[w] & m_externalDependencyBitmap[w];
        while (oldval)
        {
            //找到首个bit为1的位置
            CTZ(id, oldval);

            uint32_t bit = 1 << id;
            //如果该bit对应的任务内部依赖条件已经满足,则清除改bit标志,并启动任务（使用当前thread 进行任务的处理与执行）
            if (ATOMIC_AND(&m_internalDependencyBitmap[w], ~bit) & bit)// 先读取再操作
            {
                /* we cleared the bit, we get to process the row */
                // 其中 w*32 +id 为编码row对应的任务id
                processRow(w * 32 + id, threadId);// 通过wavefront的纯虚函数接口调用 FrameEncoder 的processRow 去执行 编码 或者 filter
                m_helpWanted = true;
                return; /* check for a higher priority task */
            }

            oldval = m_internalDependencyBitmap[w] & m_externalDependencyBitmap[w];
        }
    }

    m_helpWanted = false;
}
}
