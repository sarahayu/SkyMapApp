/*
 * stars.h
 *
 *  Created on: May 27, 2022
 *      Author: sarah
 *
 *  Declare structs for holding Star and Line data (the latter for constellations)
 *  as well as parsing functions for them to parse from plain strings as received from our API
 */

#ifndef STARS_H_
#define STARS_H_

#ifdef    __cplusplus
extern "C" {
#endif

typedef struct Star {
    int mag;
    float x, y;
} Star;

int parse_stars(Star *stars, const char *starData, int maxStars);        // returns number of stars read

typedef struct Line {
    float x1, y1, x2, y2;
} Line;

int parse_constellations(Line *lines, const char *constellationData, int maxConstellations);    // returns number of lines read

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif /* STARS_H_ */
