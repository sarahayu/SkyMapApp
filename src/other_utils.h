/*
 * other_utils.h
 *
 *  Created on: Apr 26, 2022
 *      Author: sarah
 *
 *  Various utility functions including color math
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "drawer.h"

#ifdef    __cplusplus
extern "C" {
#endif

void delay(unsigned long ulCount);
void clearStr(char *buf);
int numLines(char *text, int availWidth);
char* getLastStrtok(char *str, const char *delim);

// all w is range 0-100
Color RGBTo565(int r, int g, int b);
int lerp(int a, int b, int w);
void lerpRGB(int r1, int g1, int b1,
             int r2, int g2, int b2,
             int *r_o, int *g_o, int *b_o, int w);

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif /* UTILS_H_ */
