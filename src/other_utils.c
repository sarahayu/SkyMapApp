/*
 * utils.c
 *
 *  Created on: Apr 26, 2022
 *      Author: sarah
 */

#include "other_utils.h"
#include "textrect.h"
#include <string.h>
//#include <math.h>

//  function delays 3*ulCount cycles
void delay(unsigned long ulCount){
    int i;

  do{
    ulCount--;
        for (i=0; i< 65535; i++) ;
    }while(ulCount);
}

void clearStr(char *buf)
{
    memset(buf,0,strlen(buf));
}

int numLines(char *text, int availWidth)
{
    int len = strlen(text), curX = 0, lines = 1, escaped = 0, i;

    for (i = 0; i < len; i++)
    {
        if (text[i] == '/')
        {
            escaped = 1;
            continue;
        }


        if (escaped)
        {
            escaped = 0;
            continue;
        }

        if (curX + GLYPH_WIDTH > availWidth)
        {
            lines++;
            curX = 0;
        }
        else
            curX += GLYPH_WIDTH;
    }

    return lines;
}


char* getLastStrtok(char *str, const char *delim) {
    char *token = strtok(str, delim), *lastToken = NULL;

    while (token) {
        lastToken = token;
        token = strtok(NULL, delim);
    }

    return lastToken;
}

unsigned int RGBTo565(int r, int g, int b) {
    int R5 = (r * 31) / 255;
    int G6 = (g * 63) / 255;
    int B5 = (b * 31) / 255;

    if (R5 > 31) R5 = 31;
    if (G6 > 63) G6 = 63;
    if (B5 > 31) B5 = 31;

    return (R5 << 11) + (G6 << 5) + B5;
}

int lerp(int a, int b, int w) {
    return w * (b - a) / 100 + a;
}

void lerpRGB(int r1, int g1, int b1,
             int r2, int g2, int b2,
             int *r_o, int *g_o, int *b_o, int w) {
    *r_o = lerp(r1, r2, w);
    *g_o = lerp(g1, g2, w);
    *b_o = lerp(b1, b2, w);
}

