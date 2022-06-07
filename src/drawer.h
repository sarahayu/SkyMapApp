/*
 * drawer.h
 *
 *  Created on: Apr 7, 2022
 *      Author: sarah
 *
 *  Functions for drawing array of Star and Line data structs to the OLED
 */

#ifndef DRAWER_H_
#define DRAWER_H_


/* These functions are based on the Arduino test program at
*  https://github.com/adafruit/Adafruit-SSD1351-library/blob/master/examples/test/test.ino
*
*  You can use these high-level routines to implement your
*  test program.
*/

// TODO Configure SPI port and use these libraries to implement
// an OLED test program. See SPI example program.


#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "textrect.h"
#include "stars.h"

#ifdef    __cplusplus
extern "C" {
#endif

//extern int cursor_x;
//extern int cursor_y;

// Color definitions
#define BLACK           0x0000
#define GRAY            0x94B2
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

typedef unsigned int Color;

void drawStars(Star *stars, int numStars, int minMag);
void eraseStars(Star *stars, int numStars);

void drawConstellations(Line *lines, int numLines, int bwColor);
void eraseConstellations(Line *lines, int numLines);

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif /* DRAWER_H_ */
