#ifndef MENTHAAR_PSVITA_VASPRINTF_COMPAT_H
#define MENTHAAR_PSVITA_VASPRINTF_COMPAT_H

/*
 * g2o declares vasprintf() only when compiled with its WINDOWS branch.
 * newlib on VitaSDK does not provide it, so we supply a small implementation
 * and declare it here.  This header is force-included when compiling the
 * bundled g2o sources for the Vita; no project source is modified.
 */

#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

int vasprintf(char **strp, const char *fmt, va_list ap);

#ifdef __cplusplus
}
#endif

#endif /* MENTHAAR_PSVITA_VASPRINTF_COMPAT_H */
