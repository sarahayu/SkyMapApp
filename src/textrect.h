/*
 * textrect.h
 *
 *  Created on: Apr 25, 2022
 *      Author: sarah
 *
 *  Struct and functions for making writing textboxes easier
 */

#ifndef TEXTRECT_H_
#define TEXTRECT_H_

#ifdef    __cplusplus
extern "C" {
#endif

#define MAX_TEXT_RECT_BUF 64

#define GLYPH_HEIGHT 8           // see Adafruit_GFX.h drawChar
#define GLYPH_WIDTH 6           // see Adafruit_GFX.h drawChar

typedef struct TextRect
{
    char text[MAX_TEXT_RECT_BUF];
    int x1, y1, x2, y2;
    int active;         // used for array purposes

    int _boundX, _boundY;
    int _curX, _curY;
    int _lastTextColor;
    int _bgColor;
} TextRect;

void tr_init(TextRect *trect, int x1, int y1, int x2, int y2, unsigned int textColor);                // MAKE SURE TO MALLOC ***BEFORE***
void tr_initBG(TextRect *trect, int x1, int y1, int x2, int y2, unsigned int textColor, unsigned int bgColor);                // MAKE SURE TO MALLOC ***BEFORE***
void tr_setTextColor(TextRect *trect, unsigned int color);
void tr_outputChar(TextRect *trect, const char c);
void tr_outputStr(TextRect *trect, const char *str);
void tr_outputStrTail(TextRect *trect, const char *str);
void tr_replaceLastChar(TextRect *trect, const char c);
void tr_deleteChar(TextRect *trect);
void tr_reset(TextRect *trect);
int tr_isOverflowed(TextRect *trect);

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif /* TEXTRECT_H_ */
