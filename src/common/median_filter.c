#include "median_filter.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static float medianInMedianArrs(int countLow, int countHigh, fbinary_heap_t *lows, fbinary_heap_t *highs) {
    if (countLow > countHigh) {
        return fbinary_heap_top(lows);
    } else if (countHigh > countLow) {
        return fbinary_heap_top(highs);
    }
    return (fbinary_heap_top(lows) + fbinary_heap_top(highs)) / 2;
}

median_filter_t *median_filter_create(int medianN)
{
    if (medianN <= 1) {
        return NULL;
    }

    median_filter_t *f = calloc(1, sizeof(median_filter_t));

    f->medianN = medianN;

    f->medianBuff = circ_buf_create(medianN);
    f->medianLows = fbinary_heap_max_create();
    f->medianHighs = fbinary_heap_min_create();
    return f;
}

void median_filter_destroy(median_filter_t *f)
{
    circ_buf_destroy(f->medianBuff);
    fbinary_heap_destroy(f->medianLows);
    fbinary_heap_destroy(f->medianHighs);
    free(f);
}

// Streaming median as described in https://stackoverflow.com/questions/10930732/c-efficiently-calculating-a-running-median
// but with removing old elements.
float median_filter_march(median_filter_t *f, float newVal) {
    float oldestVal = circ_buf_last(f->medianBuff);
    circ_buf_push(f->medianBuff, newVal);

    int countLow = fbinary_heap_size(f->medianLows);
    int countHigh = fbinary_heap_size(f->medianHighs);
    // base cases
    if (countLow == 0 && countHigh == 0) {
        fbinary_heap_push(f->medianLows, newVal);
        return newVal;
    } else if (countHigh == 0) {
        float lowsMax = fbinary_heap_top(f->medianLows);
        if (lowsMax > newVal) {
            // low value needs to be in low array
            fbinary_heap_pop(f->medianLows);
            fbinary_heap_push(f->medianLows, newVal);
            fbinary_heap_push(f->medianHighs, lowsMax);
        } else {
            fbinary_heap_push(f->medianHighs, newVal);
        }
        return (lowsMax + newVal) / 2;
    }

    // remove the oldest value
    if (countLow + countHigh == f->medianN) {
        if (fbinary_heap_remove(f->medianLows, oldestVal)) {
            countLow--;
        } else {
            fbinary_heap_remove(f->medianHighs, oldestVal);
            countHigh--;
        }
    }

    if (countLow > countHigh + 1) {
        float lowMax = fbinary_heap_top(f->medianLows);
        fbinary_heap_pop(f->medianLows);
        fbinary_heap_push(f->medianHighs, lowMax);
        countLow--;
        countHigh++;
    } else if (countHigh > countLow + 1) {
        float highMin = fbinary_heap_top(f->medianHighs);
        fbinary_heap_pop(f->medianHighs);
        fbinary_heap_push(f->medianLows, highMin);
        countHigh--;
        countLow++;
    }

    float previousMedian = medianInMedianArrs(countLow, countHigh, f->medianLows, f->medianHighs);
    // add new value
    if (newVal < previousMedian) {
        if (countLow == (f->medianN+1)/2) {
            // already full, swap with highs
            float lowMax = fbinary_heap_top(f->medianLows);
            fbinary_heap_pop(f->medianLows);
            fbinary_heap_push(f->medianHighs, lowMax);
            countLow--;
            countHigh++;
        }
        fbinary_heap_push(f->medianLows, newVal);
        countLow++;
    } else if (newVal > previousMedian) {
        if (countHigh == (f->medianN+1)/2) {
            float highMin = fbinary_heap_top(f->medianHighs);
            fbinary_heap_pop(f->medianHighs);
            fbinary_heap_push(f->medianLows, highMin);
            countHigh--;
            countLow++;
        }
        fbinary_heap_push(f->medianHighs, newVal);
        countHigh++;
    } else if (countLow <= countHigh) {
        fbinary_heap_push(f->medianLows, newVal);
        countLow++;
    } else {
        fbinary_heap_push(f->medianHighs, newVal);
        countHigh++;
    }

    return medianInMedianArrs(countLow, countHigh, f->medianLows, f->medianHighs);
}
