/*
 * Filter.c
 *
 *  Created on: Jul 15, 2025
 *      Author: Robert
 */
#include <stdint.h>

typedef struct {
    int16_t x; //input
    int16_t y; //output
} Point;

static int16_t interpolation(int16_t input)
{
    static const Point pts[] = {
        {1180, 100},
        {2400, 200},
        {4000, 330}
    };

    if (input <= pts[0].x) return pts[0].y*input/pts[0].x;
    if (input >= pts[2].x) return pts[2].y;
    if (input <= pts[1].x) {
        return pts[0].y +
            (pts[1].y - pts[0].y) * (input - pts[0].x) / (pts[1].x - pts[0].x);
    } else {

        return pts[1].y +
            (pts[2].y - pts[1].y) * (input - pts[1].x) / (pts[2].x - pts[1].x);
    }
}
