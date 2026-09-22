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