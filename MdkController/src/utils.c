#include "utils.h"

void vUtilsSwap(int32_t * a, int32_t * b)
{
    int32_t temp = *a;
    *a = *b;
    *b = temp;
}