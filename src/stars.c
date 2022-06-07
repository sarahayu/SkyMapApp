/*
 * stars.c
 *
 *  Created on: May 27, 2022
 *      Author: sarah
 */

#include <stdlib.h>
#include <string.h>
#include "stars.h"
#include "uart_if.h"

typedef enum StarToken
{
    ST_MAG = 0,
    ST_X,
    ST_Y,
    ST_COUNT
} StarToken;

typedef enum LineToken
{
    LT_X1 = 0,
    LT_Y1,
    LT_X2,
    LT_Y2,
    LT_COUNT
} LineToken;

int parse_stars(Star *stars, const char *starData, int maxStars) {

    int count = -1;

    char *token = strtok(starData, ",");
    StarToken type = ST_MAG;

    while (token) {

        if (type == ST_MAG) {
            if (count + 1 > maxStars) {
                return count;
            }
            count++;
        }

        switch (type) {
        case ST_MAG:
            stars[count].mag = atoi(token);
            break;
        case ST_X:
            stars[count].x = strtod(token, NULL);
            break;
        case ST_Y:
            stars[count].y = strtod(token, NULL);
            break;
        }

        token = strtok(NULL, ",");
        type = (type + 1) % ST_COUNT;
    }

    return count;
//
//    cJSON *starJSON = cJSON_Parse(starData);
//    if (starJSON == NULL) {
//        Message("starJSON is null\r\n");
//        const char *error_ptr = cJSON_GetErrorPtr();
//        if (error_ptr != NULL)
//        {
//            Report("Error before: %s\r\n", error_ptr);
//        }
//        goto end;
//    }
//
//    cJSON *stars = cJSON_GetObjectItemCaseSensitive(starJSON, "stars"), *star;
//    cJSON_ArrayForEach(star, stars)
//    {
//        Message("In array\r\n");
//        if (count + 1 >= maxStars)
//            goto end;
//
//        cJSON *mag = cJSON_GetObjectItemCaseSensitive(star, "mag");
//        cJSON *x = cJSON_GetObjectItemCaseSensitive(star, "x");
//        cJSON *y = cJSON_GetObjectItemCaseSensitive(star, "y");
//
//        count++;
//        starsArray[count].mag = mag->valueint;
//        starsArray[count].x = x->valuedouble;
//        starsArray[count].y = y->valuedouble;
//    }
//
//end:
//    cJSON_Delete(starJSON);
//    return count;
}


int parse_constellations(Line *lines, const char *constellationData, int maxConstellations) {

    int count = -1;

    char *token = strtok(constellationData, ",");
    LineToken type = LT_X1;

    while (token) {

        if (type == LT_X1) {
            if (count + 1 > maxConstellations) {
                return count;
            }
            count++;
        }

        switch (type) {
        case LT_X1:
            lines[count].x1 = strtod(token, NULL);
            break;
        case LT_Y1:
            lines[count].y1 = strtod(token, NULL);
            break;
        case LT_X2:
            lines[count].x2 = strtod(token, NULL);
            break;
        case LT_Y2:
            lines[count].y2 = strtod(token, NULL);
            break;
        }

        token = strtok(NULL, ",");
        type = (type + 1) % LT_COUNT;
    }

    return count;
}
