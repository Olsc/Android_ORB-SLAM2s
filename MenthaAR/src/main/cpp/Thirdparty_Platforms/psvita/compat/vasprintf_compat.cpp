/**
 * Copyright (C) 2025-2026 Olsc <OlscStudio@outlook.com>
 *
 * This file is part of the Android ORB-SLAM2s project (a fork of ORB-SLAM2).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "vasprintf_compat.h"

#include <stdio.h>
#include <stdlib.h>

extern "C" int vasprintf(char **strp, const char *fmt, va_list ap)
{
	if (!strp || !fmt)
		return -1;

	va_list ap2;
	va_copy(ap2, ap);
	int needed = vsnprintf(nullptr, 0, fmt, ap2);
	va_end(ap2);

	if (needed < 0)
		return -1;

	char *buf = (char *)malloc((size_t)needed + 1);
	if (!buf)
		return -1;

	int written = vsnprintf(buf, (size_t)needed + 1, fmt, ap);
	if (written < 0) {
		free(buf);
		return -1;
	}

	*strp = buf;
	return written;
}