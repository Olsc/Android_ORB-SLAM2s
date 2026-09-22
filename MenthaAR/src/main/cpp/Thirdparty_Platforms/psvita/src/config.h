#ifndef MENTHAAR_PSVITA_CONFIG_H
#define MENTHAAR_PSVITA_CONFIG_H

#include <stdint.h>

/* ------------------------------------------------------------------------ */
/* Application metadata                                                     */
/* ------------------------------------------------------------------------ */
#define APP_NAME        "MenthaAR PSVita"
#define APP_VERSION     "01.00"

/* ------------------------------------------------------------------------ */
/* Display                                                                  */
/* ------------------------------------------------------------------------ */
#define SCR_W 960
#define SCR_H 544

/* ------------------------------------------------------------------------ */
/* Camera                                                                   */
/* ------------------------------------------------------------------------ */
#define CAM_W 640
#define CAM_H 360
#define CAM_FPS 30

/* ------------------------------------------------------------------------ */
/* Point cloud                                                              */
/* ------------------------------------------------------------------------ */
#define MAX_POINTS   150000
#define SAMPLE_STEP  8          /* 640/8 * 360/8 = 80 * 45 = 3600 points/scan */

#define SAVE_DIR     "ux0:data/MenthaAR"
#define SAVE_PATH    SAVE_DIR "/cloud.mpc"

/* RGBA packed the same way the Vita GPU stores A8B8G8R8 textures in memory
 * (little endian byte order: R, G, B, A).                                  */
#define PACK_RGBA(r, g, b, a) \
	( ((uint32_t)(r) & 0xFF)        | (((uint32_t)(g) & 0xFF) << 8)  | \
	  (((uint32_t)(b) & 0xFF) << 16) | (((uint32_t)(a) & 0xFF) << 24) )

#endif /* MENTHAAR_PSVITA_CONFIG_H */
