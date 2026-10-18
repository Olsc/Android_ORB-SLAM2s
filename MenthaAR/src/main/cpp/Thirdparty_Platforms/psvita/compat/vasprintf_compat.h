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

#ifndef MENTHAAR_PSVITA_VASPRINTF_COMPAT_H
#define MENTHAAR_PSVITA_VASPRINTF_COMPAT_H


#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

int vasprintf(char **strp, const char *fmt, va_list ap);

#ifdef __cplusplus
}
#endif

#endif // MENTHAAR_PSVITA_VASPRINTF_COMPAT_H