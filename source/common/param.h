/*****************************************************************************
 * Copyright (C) 2013-2020 MulticoreWare, Inc
 *
 * Authors: Deepthi Nandakumar <deepthi@multicorewareinc.com>
 *          Praveen Kumar Tiwari <praveen@multicorewareinc.com>
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

#ifndef S265_PARAM_H
#define S265_PARAM_H

namespace S265_NS {

int   s265_check_params(s265_param *param);
void  s265_print_params(s265_param *param);
char* s265_param2string(s265_param *param, int padx, int pady);
int   s265_atoi(const char *str, bool& bError);
double s265_atof(const char *str, bool& bError);
int   parseCpuName(const char *value, bool& bError, bool bEnableavx512);
void  setParamAspectRatio(s265_param *p, int width, int height);
void  getParamAspectRatio(s265_param *p, int& width, int& height);
bool  parseLambdaFile(s265_param *param);
void s265_copy_params(s265_param* dst, s265_param* src);

/* this table is kept internal to avoid confusion, since log level indices start at -1 */
static const char * const logLevelNames[] = { "none", "error", "warning", "info", "debug", "full", 0 };

#if EXPORT_C_API
#define PARAM_NS
#else
/* declare param functions within private namespace */
void s265_param_free(s265_param *);
s265_param* s265_param_alloc();
void s265_param_default(s265_param *param);
int s265_param_default_preset(s265_param *, const char *preset, const char *tune);
int s265_param_apply_profile(s265_param *, const char *profile);
int s265_param_parse(s265_param *p, const char *name, const char *value);
#define PARAM_NS S265_NS
#endif
}
#endif // ifndef S265_PARAM_H
