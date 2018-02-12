#include "median_filter.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static float minInMedianArr(float *vals, int medianN) {
    float min = NAN;
    for (int i = 0; i < (medianN+1)/2; i++) {
        if (isnan(min) || vals[i] < min) {
            min = vals[i];
        }
    }
    return min;
}

static float maxInMedianArr(float *vals, int medianN) {
    float max = NAN;
    for (int i = 0; i < (medianN+1)/2; i++) {
        if (isnan(max) || vals[i] > max) {
            max = vals[i];
        }
    }
    return max;
}

static int countMedianArr(float *vals, int medianN) {
    int count = 0;
    for (int i = 0; i < (medianN+1)/2; i++) {
        if (!isnan(vals[i])) {
            count++;
        }
    }
    return count;
}

// true if removed false if not present
static bool removeValInMedianArr(float *vals, float val, int medianN) {
    for (int i = 0; i < (medianN+1)/2; i++) {
        if (vals[i] == val) {
            vals[i] = NAN;
            return true;
        }
    }
    return false;
}

static void addValInMedianArr(float *vals, float val, int medianN) {
    for (int i = 0; i < (medianN+1)/2; i++) {
        if (isnan(vals[i])) {
            vals[i] = val;
            return;
        }
    }
    fprintf(stderr, "Median array was already full! Panic!");
    exit(1);
}

static float medianInMedianArrs(int countLow, int countHigh, float *lows, float *highs, int medianN) {
    if (countLow > countHigh) {
        return maxInMedianArr(lows, medianN);
    } else if (countHigh > countLow) {
        return minInMedianArr(highs, medianN);
    }
    return (maxInMedianArr(lows, medianN) + minInMedianArr(highs, medianN)) / 2;
}

median_filter_t *median_filter_create(int medianN)
{
    if (medianN <= 1) {
        return NULL;
    }

    median_filter_t *f = calloc(1, sizeof(median_filter_t));

    f->medianN = medianN;

    f->medianBuff = circ_buf_create(medianN);
    f->medianLows = calloc((medianN+1)/2, sizeof(float));
    f->medianHighs = calloc((medianN+1)/2, sizeof(float));

    // use NAN to mean "empty"
    for (int i = 0; i < (medianN+1)/2; i++) {
        f->medianLows[i] = NAN;
        f->medianHighs[i] = NAN;
    }
    return f;
}

void median_filter_destroy(median_filter_t *f)
{
    circ_buf_destroy(f->medianBuff);

    free(f);
}

// Streaming median as described in https://stackoverflow.com/questions/10930732/c-efficiently-calculating-a-running-median
// but with removing old elements.
float median_filter_march(median_filter_t *f, float newVal) {
    float oldestVal = circ_buf_last(f->medianBuff);
    circ_buf_push(f->medianBuff, newVal);

    int countLow = countMedianArr(f->medianLows, f->medianN);
    int countHigh = countMedianArr(f->medianHighs, f->medianN);
    // base cases
    if (countLow == 0 && countHigh == 0) {
        addValInMedianArr(f->medianLows, newVal, f->medianN);
        return newVal;
    } else if (countHigh == 0) {
        float lowsMax = maxInMedianArr(f->medianLows, f->medianN);
        if (lowsMax > newVal) {
            // low value needs to be in low array
            removeValInMedianArr(f->medianLows, lowsMax, f->medianN);
            addValInMedianArr(f->medianLows, newVal, f->medianN);
            addValInMedianArr(f->medianHighs, lowsMax, f->medianN);
        } else {
            addValInMedianArr(f->medianHighs, newVal, f->medianN);
        }
        return (lowsMax + newVal) / 2;
    }

    // remove the oldest value
    if (countLow + countHigh == f->medianN) {
        if (removeValInMedianArr(f->medianLows, oldestVal, f->medianN)) {
            countLow--;
        } else {
            removeValInMedianArr(f->medianHighs, oldestVal, f->medianN);
            countHigh--;
        }
    }

    if (countLow > countHigh + 1) {
        float lowMax = maxInMedianArr(f->medianLows, f->medianN);
        removeValInMedianArr(f->medianLows, lowMax, f->medianN);
        addValInMedianArr(f->medianHighs, lowMax, f->medianN);
        countLow--;
        countHigh++;
    } else if (countHigh > countLow + 1) {
        float highMin = minInMedianArr(f->medianHighs, f->medianN);
        removeValInMedianArr(f->medianHighs, highMin, f->medianN);
        addValInMedianArr(f->medianLows, highMin, f->medianN);
        countHigh--;
        countLow++;
    }

    float previousMedian = medianInMedianArrs(countLow, countHigh, f->medianLows, f->medianHighs, f->medianN);
    // add new value
    if (newVal < previousMedian) {
        if (countLow == (f->medianN+1)/2) {
            // already full, swap with highs
            float lowMax = maxInMedianArr(f->medianLows, f->medianN);
            removeValInMedianArr(f->medianLows, lowMax, f->medianN);
            addValInMedianArr(f->medianHighs, lowMax, f->medianN);
            countLow--;
            countHigh++;
        }
        addValInMedianArr(f->medianLows, newVal, f->medianN);
        countLow++;
    } else if (newVal > previousMedian) {
        if (countHigh == (f->medianN+1)/2) {
            float highMin = minInMedianArr(f->medianHighs, f->medianN);
            removeValInMedianArr(f->medianHighs, highMin, f->medianN);
            addValInMedianArr(f->medianLows, highMin, f->medianN);
            countHigh--;
            countLow++;
        }
        addValInMedianArr(f->medianHighs, newVal, f->medianN);
        countHigh++;
    } else if (countLow <= countHigh) {
        addValInMedianArr(f->medianLows, newVal, f->medianN);
        countLow++;
    } else {
        addValInMedianArr(f->medianHighs, newVal, f->medianN);
        countHigh++;
    }

    return medianInMedianArrs(countLow, countHigh, f->medianLows, f->medianHighs, f->medianN);
}
