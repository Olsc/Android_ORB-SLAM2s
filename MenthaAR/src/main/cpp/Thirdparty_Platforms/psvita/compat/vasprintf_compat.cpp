#include "vasprintf_compat.h"

#include <stdio.h>
#include <stdlib.h>

extern "C" int vasprintf(char **strp, const char *fmt, va_list ap)
{
	if (!strp || !fmt)
		return -1;

	va_list ap2;
	va_copy(ap2, ap);
	int needed = vsnprintf(NULL, 0, fmt, ap2);
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
