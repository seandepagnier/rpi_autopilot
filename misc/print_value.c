#include <stdio.h>
#include <math.h>

void print_value(float v)
{
    printf("%c%05.2f ", v < 0 ? '-' : '+', fabs(v));
}
