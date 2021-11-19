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

#ifndef S265_DPB_H
#define S265_DPB_H

#include "piclist.h"

namespace S265_NS {
// private namespace for s265

class Frame;
class FrameData;
class Slice;

class DPB
{
public:

    int                m_lastIDR;
    int                m_pocCRA;
    int                m_bOpenGOP;
    int                m_bhasLeadingPicture;
    int                m_pyramid_type;
    int                m_dpb_method;
    bool               m_bRefreshPending;
    bool               m_bTemporalSublayer;
    PicList            m_picList;
    PicList            m_freeList;
    FrameData*         m_frameDataFreeList;

    DPB(s265_param *param)
    {
        m_lastIDR = 0;
        m_pocCRA = 0;
        m_bhasLeadingPicture = param->radl;
        if (param->bResetZoneConfig)
        {
            for (int i = 0; i < param->rc.zonefileCount ; i++)
            {
                if (param->rc.zones[i].zoneParam->radl)
                {
                    m_bhasLeadingPicture = param->rc.zones[i].zoneParam->radl;
                    break;
                }
            }
        }
        m_pyramid_type = param->bBPyramid;
        m_dpb_method = param->rc.dpbMethod;
        m_bRefreshPending = false;
        m_frameDataFreeList = NULL;
        m_bOpenGOP = param->bOpenGOP;
        m_bTemporalSublayer = !!param->bEnableTemporalSubLayers;
    }

    ~DPB();

    void prepareEncode(Frame*);

    void recycleUnreferenced();

protected:

    void computeRPS(int curPoc, bool isRAP, RPS * rps, unsigned int maxDecPicBuffer);

    void applyReferencePictureSet(RPS *rps, int curPoc);
    void decodingRefreshMarking(int pocCurr, NalUnitType nalUnitType);

    NalUnitType getNalUnitType(int curPoc, bool bIsKeyFrame);
};
}

#endif // S265_DPB_H
