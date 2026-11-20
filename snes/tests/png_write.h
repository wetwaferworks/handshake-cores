/* SPDX-License-Identifier: MIT */
#ifndef HS_PNG_WRITE_H
#define HS_PNG_WRITE_H
#include <stdint.h>
/* writes an XRGB8888 image as an 8-bit RGB PNG (stored deflate). 0 = ok */
int hs_png_write_xrgb(const char *path, const uint32_t *px, int w, int h);
#endif
