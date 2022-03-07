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

#include "common.h"
#include "output.h"
#include "y4m.h"

using namespace S265_NS;
using namespace std;

Y4MOutput::Y4MOutput(const char *filename, int w, int h, uint32_t fpsNum, uint32_t fpsDenom, int csp)
    : width(w)
    , height(h)
    , colorSpace(csp)
    , frameSize(0)
{
    ofs.open(filename, ios::binary | ios::out);
    buf = new char[width];

    const char *cf = (csp >= S265_CSP_I444) ? "444" : (csp >= S265_CSP_I422) ? "422" : "420";

    if (ofs)
    {
        ofs << "YUV4MPEG2 W" << width << " H" << height << " F" << fpsNum << ":" << fpsDenom << " Ip" << " C" << cf << "\n";
        header = ofs.tellp();
    }

    for (int i = 0; i < s265_cli_csps[colorSpace].planes; i++)
        frameSize += (uint32_t)((width >> s265_cli_csps[colorSpace].width[i]) * (height >> s265_cli_csps[colorSpace].height[i]));
}

Y4MOutput::~Y4MOutput()
{
    ofs.close();
    delete [] buf;
}

bool Y4MOutput::writePicture(const s265_picture& pic)
{
    std::ofstream::pos_type outPicPos = header;
    outPicPos += (uint64_t)pic.poc * (6 + frameSize);
    ofs.seekp(outPicPos);
    ofs << "FRAME\n";

#if HIGH_BIT_DEPTH
    if (pic.bitDepth > 8 && pic.poc == 0)
        s265_log(NULL, S265_LOG_WARNING, "y4m: down-shifting reconstructed pixels to 8 bits\n");
#else
    if (pic.bitDepth > 8 && pic.poc == 0)
        s265_log(NULL, S265_LOG_WARNING, "y4m: forcing reconstructed pixels to 8 bits\n");
#endif

    S265_CHECK(pic.colorSpace == colorSpace, "invalid chroma subsampling\n");

#if HIGH_BIT_DEPTH

    // encoder gave us short pixels, downshift, then write
    S265_CHECK(pic.bitDepth > 8, "invalid bit depth\n");
    int shift = pic.bitDepth - 8;
    for (int i = 0; i < s265_cli_csps[colorSpace].planes; i++)
    {
        uint16_t *src = (uint16_t*)pic.planes[i];
        for (int h = 0; h < height >> s265_cli_csps[colorSpace].height[i]; h++)
        {
            for (int w = 0; w < width >> s265_cli_csps[colorSpace].width[i]; w++)
                buf[w] = (char)(src[w] >> shift);

            ofs.write(buf, width >> s265_cli_csps[colorSpace].width[i]);
            src += pic.stride[i] / sizeof(*src);
        }
    }

#else // if HIGH_BIT_DEPTH

    S265_CHECK(pic.bitDepth == 8, "invalid bit depth\n");
    for (int i = 0; i < s265_cli_csps[colorSpace].planes; i++)
    {
        char *src = (char*)pic.planes[i];
        for (int h = 0; h < height >> s265_cli_csps[colorSpace].height[i]; h++)
        {
            ofs.write(src, width >> s265_cli_csps[colorSpace].width[i]);
            src += pic.stride[i] / sizeof(*src);
        }
    }

#endif // if HIGH_BIT_DEPTH

    return true;
}
