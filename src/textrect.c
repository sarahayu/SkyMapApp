/*
 * textrect.c
 *
 *  Created on: Apr 25, 2022
 *      Author: sarah
 */

#include "textrect.h"
#include "Adafruit_SSD1351.h"
#include "Adafruit_GFX.h"
#include "drawer.h"
#include <string.h>

static char buf[] = "E";

void OutChar(const char c)
{
    buf[0] = c;
    Outstr(buf, -1);
}

void tr_init(TextRect *trect, int x1, int y1, int x2, int y2, unsigned int textColor)
{
    trect->_curX = x1;
    trect->_curY = y1;
    trect->x1 = x1;
    trect->y1 = y1;
    trect->x2 = x2;
    trect->y2 = y2;
    trect->_lastTextColor = textColor;
    trect->_bgColor = WHITE;
    trect->active = 0;
}

void tr_initBG(TextRect *trect, int x1, int y1, int x2, int y2, unsigned int textColor, unsigned int bgColor)
{
    trect->_curX = x1;
    trect->_curY = y1;
    trect->x1 = x1;
    trect->y1 = y1;
    trect->x2 = x2;
    trect->y2 = y2;
    trect->_lastTextColor = textColor;
    trect->_bgColor = bgColor;
    trect->active = 0;
}

void tr_setTextColor(TextRect *trect, unsigned int textColor)
{
    trect->_lastTextColor = textColor;
}

void tr_outputChar(TextRect *trect, const char c)
{
    setCursor(trect->_curX, trect->_curY);
    setTextColor(trect->_lastTextColor, trect->_bgColor);

    // keep track of the max positions chars can be written to
    // can't just use x2 or y2 because it might not be the multiple of glyph dimension
    if (trect->_curX > trect->x2 - GLYPH_WIDTH * 2)
        trect->_boundX = trect->_curX;
    if (trect->_curY > trect->y2 - GLYPH_HEIGHT * 2)
        trect->_boundY = trect->_curY;

    if (!tr_isOverflowed(trect))
    {
        OutChar(c);
        trect->_curX = getCursorX();
        trect->_curY = getCursorY();

        // cursor had wrapped past right edge of OLED screen and to left, possibly past left edge of textrect. Accomodate. (it's ok if y goes past bounds, however)
        if (trect->_curX < trect->x1)
        {
            trect->_curX = trect->x1;
            // _curY is automatically incremented by Adafruit_GFX
        }
        // cursor had might have passed right bounds. Accomodate. (it's ok if y goes past bounds, however)
        if (trect->_curX > trect->x2 - GLYPH_WIDTH)
        {
            trect->_curX = trect->x1;
            trect->_curY = trect->_curY + GLYPH_HEIGHT;
        }
    }

}

void tr_outputStr(TextRect *trect, const char *str)
{
    int len = strlen(str), i, escaped = 0;

    for (i = 0; i < len; i++)
    {
        if (str[i] == '/')
        {
            escaped = 1;
            continue;
        }
//        if (escaped)
//        {
//            tr_setTextColor(trect, getColor(str[i] - '0'));
//            escaped = 0;
//            continue;
//        }
        tr_outputChar(trect, str[i]);
    }
}

void tr_outputStrTail(TextRect *trect, const char *str)
{

}

void tr_replaceLastChar(TextRect *trect, const char c)
{
    tr_deleteChar(trect);
    tr_outputChar(trect, c);
}

void tr_deleteChar(TextRect *trect)
{
    int lastX = trect->_curX - GLYPH_WIDTH, lastY = trect->_curY;

    // we're trying to draw past left of bounds. Go to last line, if there is
    if (lastX < trect->x1)
    {
        if (lastY > trect->y1)
        {
            lastX = trect->_boundX;
            lastY = trect->_curY - GLYPH_HEIGHT;
        }
        // we're trying delete past most top-left position
        else
        {
            lastX = trect->x1;
            lastY = trect->y1;
        }
    }
    fillRect(lastX, lastY, GLYPH_WIDTH, GLYPH_HEIGHT, trect->_bgColor);
    setCursor(lastX, lastY);

//    OutChar(c);
    trect->_curX = getCursorX();
    trect->_curY = getCursorY();
}

void tr_reset(TextRect *trect)
{
    fillRect(trect->x1, trect->y1, trect->x2 - trect->x1,  trect->y2 - trect->y1, trect->_bgColor);
    trect->_curX = trect->x1;
    trect->_curY = trect->y1;
}

int tr_isOverflowed(TextRect *trect)
{
    return trect->_curX > trect->x2 - GLYPH_WIDTH && trect->_curY > trect->y2 - GLYPH_HEIGHT;
}

//void tr_append(TextRect *trect, char c)
//{
//    int i = 0;
//
//    while (str)
//    {
//
//    }
//}
//
//void tr_draw(TextRect *trect)
//{
//    if (trect->lastX != -1)
//    {
//        fillRect(trect->x, trect->y, WIDTH - trect->x, trect->lastY - trect->y, bgColor);
//    }
//
//    setCursor(trect->x, trect->y);
//    Outstr(trect->text, -1);
//}
