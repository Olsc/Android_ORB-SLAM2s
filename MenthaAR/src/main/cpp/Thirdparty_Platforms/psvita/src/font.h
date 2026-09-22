#ifndef MENTHAAR_PSVITA_FONT_H
#define MENTHAAR_PSVITA_FONT_H

/*
 * 8x8 bitmap font, taken from the VitaSDK "samples" repository
 * (common/debugScreenFont.c, PSPSDK/BSD licensed). Only the glyph table is
 * reused here; the actual text rendering is done by our software renderer so
 * that no system font / PGF-PVF service is required at runtime.
 */

typedef struct PsvDebugScreenFont {
	unsigned char *glyphs, width, height, first, last, size_w, size_h;
} PsvDebugScreenFont;

extern PsvDebugScreenFont psvDebugScreenFont;

#endif /* MENTHAAR_PSVITA_FONT_H */
