/*
 * utils.h
 *
 * Created: 24.04.2015 22:24:45
 *  Author: Christian
 */ 


#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>

#define utilsMAX(x, y)   ((x) > (y) ? (x) : (y))
#define utilsMIN(x, y)   ((x) < (y) ? (x) : (y))

void vUtilsSwap(int32_t * a, int32_t * b);

#endif /* UTILS_H_ */