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
#define _FILE_OFFSET_BITS 64
#define _LARGEFILE_SOURCE
#include "yuv.h"
#include "common.h"

#include <iostream>

#define ENABLE_THREADING 1

#if _WIN32
#include <io.h>
#include <fcntl.h>
#if defined(_MSC_VER)
#pragma warning(disable: 4996) // POSIX setmode and fileno deprecated
#endif
#endif

using namespace S265_NS;
using namespace std;

YUVInput::YUVInput(InputFileInfo& info)
{
    for (int i = 0; i < QUEUE_SIZE; i++)
        buf[i] = NULL;

    depth = info.depth;
    width = info.width;
    height = info.height;
    colorSpace = info.csp;
    threadActive = false;
    ifs = NULL;

    uint32_t pixelbytes = depth > 8 ? 2 : 1;
    framesize = 0;
    for (int i = 0; i < s265_cli_csps[colorSpace].planes; i++)
    {
        uint32_t w = width >> s265_cli_csps[colorSpace].width[i];
        uint32_t h = height >> s265_cli_csps[colorSpace].height[i];
        framesize += w * h * pixelbytes;
    }

    if (width == 0 || height == 0 || info.fpsNum == 0 || info.fpsDenom == 0)
    {
        s265_log(NULL, S265_LOG_ERROR, "yuv: width, height, and FPS must be specified\n");
        return;
    }
    if (!strcmp(info.filename, "-"))
    {
        ifs = stdin;
#if _WIN32
        setmode(fileno(stdin), O_BINARY);
#endif
    }
    else
        ifs = s265_fopen(info.filename, "rb");
    if (ifs && !ferror(ifs))
        threadActive = true;
    else
    {
        if (ifs && ifs != stdin)
            fclose(ifs);
        ifs = NULL;
        return;
    }

    for (uint32_t i = 0; i < QUEUE_SIZE; i++)
    {
        buf[i] = S265_MALLOC(char, framesize);
        if (buf[i] == NULL)
        {
            s265_log(NULL, S265_LOG_ERROR, "yuv: buffer allocation failure, aborting\n");
            threadActive = false;
            return;
        }
    }

    info.frameCount = -1;
    /* try to estimate frame count, if this is not stdin */
    if (ifs != stdin)
    {
        int64_t cur = ftello(ifs);
        if (cur >= 0)
        {
            fseeko(ifs, 0, SEEK_END);
            int64_t size = ftello(ifs);
            fseeko(ifs, cur, SEEK_SET);
            if (size > 0)
                info.frameCount = (int)((size - cur) / framesize);
        }
    }
    if (info.skipFrames)
    {
        if (ifs != stdin)
            fseeko(ifs, (int64_t)framesize * info.skipFrames, SEEK_CUR);
        else
            for (int i = 0; i < info.skipFrames; i++)
                if (fread(buf[0], framesize, 1, ifs) != 1)
                    break;
    }
}
YUVInput::~YUVInput()
{
    if (ifs && ifs != stdin)
        fclose(ifs);
    for (int i = 0; i < QUEUE_SIZE; i++)
        S265_FREE(buf[i]);
}

void YUVInput::release()
{
    threadActive = false;
    readCount.poke();
    stop();
    delete this;
}
// 覆盖继承自 InputFile 的 startReader
void YUVInput::startReader()
{
#if ENABLE_THREADING
    if (threadActive)
        start();
#endif
}
// 覆盖继承自thread 的threadMain
void YUVInput::threadMain()
{
    THREAD_NAME("YUVRead", 0);
    while (threadActive)
    {
        if (!populateFrameQueue())
            break;
        //不断从文件一帧一帧读取然后写入queue,如果写入成功，则继续
        //否则，退出循环    
    }

    threadActive = false;
    writeCount.poke();
}
bool YUVInput::populateFrameQueue()
{
    // ifs 为file* 指针句柄
    if (!ifs || ferror(ifs))
        return false;
    /* wait for room in the ring buffer */
    int written = writeCount.get();// 看看已经写了多少个了
    int read = readCount.get();//看看已经读了多少个了
    //如果写了4帧了还没有读（多写了4帧了）
    while (written - read > QUEUE_SIZE - 2)
    {
        //等读走至少一帧
        read = readCount.waitForChange(read);
        if (!threadActive)
            // release() has been called
            return false;
    }
    ProfileScopeEvent(frameRead);
    //从文件读一帧到buf
    if (fread(buf[written % QUEUE_SIZE], framesize, 1, ifs) == 1)
    {
        //写入buf的帧数自加
        writeCount.incr();
        return true;
    }
    else
        return false;
}

bool YUVInput::readPicture(s265_picture& pic)
{
    int read = readCount.get();
    int written = writeCount.get();

#if ENABLE_THREADING

    /* only wait if the read thread is still active */
    while (threadActive && read == written)
        written = writeCount.waitForChange(written);

#else

    populateFrameQueue();

#endif // if ENABLE_THREADING

    if (read < written)
    {
        uint32_t pixelbytes = depth > 8 ? 2 : 1;
        pic.colorSpace = colorSpace;
        pic.bitDepth = depth;
        pic.framesize = framesize;
        pic.height = height;
        pic.width = width;
        pic.stride[0] = width * pixelbytes;
        pic.stride[1] = pic.stride[0] >> s265_cli_csps[colorSpace].width[1];
        pic.stride[2] = pic.stride[0] >> s265_cli_csps[colorSpace].width[2];
        pic.planes[0] = buf[read % QUEUE_SIZE];
        pic.planes[1] = (char*)pic.planes[0] + pic.stride[0] * height;
        pic.planes[2] = (char*)pic.planes[1] + pic.stride[1] * (height >> s265_cli_csps[colorSpace].height[1]);
        readCount.incr();
        return true;
    }
    else
        return false;
}
