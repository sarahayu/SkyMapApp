#include <drawer.h>
#include <other_utils.h>
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "gpio_if.h"
#include "uart_if.h"
#include "hw_types.h"

void drawStars(Star *stars, int numStars, int minMag) {
    int i;

    int scaleMinMag = minMag * 100 / 8;

    for (i = 0; i < numStars; i++) {
        int scaleMag = stars[i].mag * 100 / scaleMinMag;
        int gray = (100 - scaleMag) * 255 / 100;
        if (gray < 0) gray = 0;
        if (gray > 255) gray = 255;
        Color color = RGBTo565(gray, gray, gray);
        fillRect(stars[i].x * WIDTH, HEIGHT - stars[i].y * HEIGHT, 2, 2, color);
    }
}

void eraseStars(Star *stars, int numStars) {
    int i;
    for (i = 0; i < numStars; i++) {
        fillRect(stars[i].x * WIDTH, HEIGHT - stars[i].y * HEIGHT, 2, 2, BLACK);
    }
}


void drawConstellations(Line *lines, int numLines, int bwColor) {
    Color color = RGBTo565(bwColor, bwColor, bwColor);
    int i;
    for (i = 0; i < numLines; i++) {
        drawLine(
                lines[i].x1 * WIDTH, HEIGHT - lines[i].y1 * HEIGHT,
                lines[i].x2 * WIDTH, HEIGHT - lines[i].y2 * HEIGHT,
                color);
    }
}

void eraseConstellations(Line *lines, int numLines) {
    int i;
    for (i = 0; i < numLines; i++) {
        drawLine(
                lines[i].x1 * WIDTH, HEIGHT - lines[i].y1 * HEIGHT,
                lines[i].x2 * WIDTH, HEIGHT - lines[i].y2 * HEIGHT,
                BLACK);
    }
}
