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

#if _MSC_VER
#pragma warning(disable: 4127) // conditional expression is constant, yes I know
#endif

#include "s265.h"
#include "s265cli.h"

#if HAVE_VLD
/* Visual Leak Detector */
#include <vld.h>
#endif

#include <signal.h>
#include <errno.h>
#include <fcntl.h>

#include <string>
#include <ostream>
#include <fstream>
#include <queue>

using namespace S265_NS;

#define S265_HEAD_ENTRIES 3
#define CONSOLE_TITLE_SIZE 200

#ifdef _WIN32
#define strdup _strdup
static char orgConsoleTitle[CONSOLE_TITLE_SIZE] = "";
#endif

#ifdef _WIN32
/* Copy of x264 code, which allows for Unicode characters in the command line.
 * Retrieve command line arguments as UTF-8. */
static int get_argv_utf8(int *argc_ptr, char ***argv_ptr)
{
    int ret = 0;
    wchar_t **argv_utf16 = CommandLineToArgvW(GetCommandLineW(), argc_ptr);
    if (argv_utf16)
    {
        int argc = *argc_ptr;
        int offset = (argc + 1) * sizeof(char*);
        int size = offset;

        for (int i = 0; i < argc; i++)
            size += WideCharToMultiByte(CP_UTF8, 0, argv_utf16[i], -1, NULL, 0, NULL, NULL);

        char **argv = *argv_ptr = (char**)malloc(size);
        if (argv)
        {
            for (int i = 0; i < argc; i++)
            {
                argv[i] = (char*)argv + offset;
                offset += WideCharToMultiByte(CP_UTF8, 0, argv_utf16[i], -1, argv[i], size - offset, NULL, NULL);
            }
            argv[argc] = NULL;
            ret = 1;
        }
        LocalFree(argv_utf16);
    }
    return ret;
}
#endif

/* CLI return codes:
 *
 * 0 - encode successful
 * 1 - unable to parse command line
 * 2 - unable to open encoder
 * 3 - unable to generate stream headers
 * 4 - encoder abort */

/* Ctrl-C handler */
static volatile sig_atomic_t b_ctrl_c /* = 0 */;
static void sigint_handler(int)
{
    b_ctrl_c = 1;
}

int main(int argc, char **argv)
{
#if HAVE_VLD
    // This uses Microsoft's proprietary WCHAR type, but this only builds on Windows to start with
    VLDSetReportOptions(VLD_OPT_REPORT_TO_DEBUGGER | VLD_OPT_REPORT_TO_FILE, L"s265_leaks.txt");
#endif
    PROFILE_INIT();
    THREAD_NAME("API", 0);

    GetConsoleTitle(orgConsoleTitle, CONSOLE_TITLE_SIZE);
    SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_AWAYMODE_REQUIRED);
#if _WIN32
    char** orgArgv = argv;
    get_argv_utf8(&argc, &argv);
#endif

    int ret = 0;
    int enc_times = 1;
    while(enc_times--)
    {
    CLIOptions* cliopt = new CLIOptions;
    s265_encoder* encoder = NULL;

    if (cliopt->parse(argc, argv))
    {
        cliopt->destroy();
        if (cliopt->api)
            cliopt->api->param_free(cliopt->param);
        exit(1);
    }

    s265_param*  param = cliopt->param;
    const s265_api* api = cliopt->api;
    if (param)
         encoder = api->encoder_open(param);
    if (!encoder)
    {
        api->encoder_close(encoder);
        s265_log(NULL, S265_LOG_ERROR, "s265_encoder_open() failed for Enc, \n");
        api->param_free(param);
        cliopt->destroy();
        delete cliopt;
        exit(2);
    }

    char* profileName = cliopt->encName ? cliopt->encName : (char *)"s265";
    /* get the encoder parameters post-initialization */
    api->encoder_parameters(encoder, cliopt->param);

    /* This allows muxers to modify bitstream format */
    cliopt->output->setParam(cliopt->param);
    if (signal(SIGINT, sigint_handler) == SIG_ERR)
        s265_log(param, S265_LOG_ERROR, "Unable to register CTRL+C handler: %s in %s\n",
            strerror(errno), profileName);

    s265_picture pic_orig, pic_out;
    s265_picture *pic_in = &pic_orig;
    /* Allocate recon picture if analysis save/load is enabled */
    std::priority_queue<int64_t>* pts_queue = cliopt->output->needPTS() ? new std::priority_queue<int64_t>() : NULL;
    s265_picture *pic_recon = (cliopt->recon || param->analysisSave || param->analysisLoad || pts_queue || param->csvLogLevel) ? &pic_out : NULL;
    uint32_t inFrameCount = 0;
    uint32_t outFrameCount = 0;
    s265_nal *p_nal;
    s265_stats stats;
    uint32_t nal;

    char *opts = s265_param2string(param,0,0);
    if(opts){
        printf("%s\n",opts);
        s265_free(opts);
    }

    if (!param->bRepeatHeaders && !param->bEnableSvtHevc)
    {
        if (api->encoder_headers(encoder, &p_nal, &nal) < 0)
        {
            s265_log(param, S265_LOG_ERROR, "Failure generating stream headers in %s\n", profileName);
            ret = 3;
            goto fail;
        }
        else
            cliopt->totalbytes += cliopt->output->writeHeaders(p_nal, nal);
    }

    api->picture_init(param, &pic_orig);

    // main encoder loop
    while (pic_in && !b_ctrl_c)
    {
        pic_orig.poc = inFrameCount;
        if (cliopt->qpfile)
        {
            if (!cliopt->parseQPFile(pic_orig))
            {
                s265_log(NULL, S265_LOG_ERROR, "can't parse qpfile for frame %d in %s\n",
                    pic_in->poc, profileName);
                fclose(cliopt->qpfile);
                cliopt->qpfile = NULL;
            }
        }

        if (cliopt->framesToBeEncoded && inFrameCount >= cliopt->framesToBeEncoded)
            pic_in = NULL;
        else if (cliopt->input->readPicture(*pic_in))
        {
            inFrameCount++;
        }
        else
            pic_in = NULL;

        if (pic_in)
        {
            /* Overwrite PTS */
            pic_in->pts = pic_in->poc;
        }
        s265_picture *picInput = pic_in;
        int numEncoded = api->encoder_encode(encoder, &p_nal, &nal, picInput, pic_recon);

        if (numEncoded < 0)
        {
            b_ctrl_c = 1;
            ret = 4;
            break;
        }
        outFrameCount += numEncoded;
        if (nal)
        {
            cliopt->totalbytes += cliopt->output->writeFrame(p_nal, nal, pic_out);
            if (pts_queue)
            {
                pts_queue->push(-pic_out.pts);
                if (pts_queue->size() > 2)
                    pts_queue->pop();
            }
        }
        cliopt->printStatus(outFrameCount);
    }

    /* Flush the encoder */
    while (!b_ctrl_c)
    {
        int numEncoded = api->encoder_encode(encoder, &p_nal, &nal, NULL, pic_recon);
        if (numEncoded < 0)
        {
            ret = 4;
            break;
        }

        outFrameCount += numEncoded;
        if (nal)
        {
            cliopt->totalbytes += cliopt->output->writeFrame(p_nal, nal, pic_out);
            if (pts_queue)
            {
                pts_queue->push(-pic_out.pts);
                if (pts_queue->size() > 2)
                    pts_queue->pop();
            }
        }

        cliopt->printStatus(outFrameCount);

        if (!numEncoded)
            break;
    }

    /* clear progress report */
    if (cliopt->bProgress)
        fprintf(stderr, "%*s\r", 80, " ");

fail:
    api->encoder_get_stats(encoder, &stats, sizeof(stats));
    if (param->csvfn && !b_ctrl_c)
#if ENABLE_LIBVMAF
        api->vmaf_encoder_log(m_encoder, m_cliopt.argCnt, m_cliopt.argString, m_cliopt.param, vmafdata);
#else
        api->encoder_log(encoder, cliopt->argCnt, cliopt->argString);
#endif
    api->encoder_close(encoder);

    int64_t second_largest_pts = 0;
    int64_t largest_pts = 0;
    if (pts_queue && pts_queue->size() >= 2)
    {
        second_largest_pts = -pts_queue->top();
        pts_queue->pop();
        largest_pts = -pts_queue->top();
        pts_queue->pop();
        delete pts_queue;
        pts_queue = NULL;
    }
    cliopt->output->closeFile(largest_pts, second_largest_pts);

    if (b_ctrl_c)
        general_log(param, NULL, S265_LOG_INFO, "aborted at input frame %d, output frame %d in %s\n",
            cliopt->seek + inFrameCount, stats.encodedPictureCount, profileName);

    api->param_free(param);
    cliopt->destroy();
    delete cliopt;
    printf("--closed------------------\n");
    sleep(5);
    }
    SetConsoleTitle(orgConsoleTitle);
    SetThreadExecutionState(ES_CONTINUOUS);

#if _WIN32
    if (argv != orgArgv)
    {
        free(argv);
        argv = orgArgv;
    }
#endif

#if HAVE_VLD
    assert(VLDReportLeaks() == 0);
#endif


    return ret;
}
