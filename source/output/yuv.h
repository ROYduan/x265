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

#ifndef S265_YUV_H
#define S265_YUV_H

#include "output.h"
#include "common.h"

#include <fstream>

namespace S265_NS {
// private s265 namespace

class YUVOutput : public ReconFile
{
protected:

    int width;

    int height;

    uint32_t depth;

    int colorSpace;

    uint32_t frameSize;

    char *buf;

    std::ofstream ofs;

public:

    YUVOutput(const char *filename, int width, int height, uint32_t bitdepth, int csp);

    virtual ~YUVOutput();

    const char *getName() const                   { return "yuv"; }

    bool isFail() const                           { return ofs.fail(); }

    void release()                                { delete this; }

    bool writePicture(const s265_picture& pic);
};
}

#endif // ifndef S265_YUV_H
