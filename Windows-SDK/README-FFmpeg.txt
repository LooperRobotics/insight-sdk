This package contains a precompiled FFmpeg 8.0.1 (x64) with Z16 format support added.
Modifications:
- Added mapping for FourCC 'Z16 ' to AV_PIX_FMT_GRAY16LE in raw_pix_fmt_tags.h
- Added case in dshow_pixfmt() to handle Z16 format

License: LGPL 2.1 or later (see COPYING.LGPLv2.1)