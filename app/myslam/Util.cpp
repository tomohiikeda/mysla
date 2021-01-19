#include <cmath>
#include <stdio.h>
#include <iostream>

double to_radian(double degree)
{
    return degree * M_PI / 180;
}

void wait_for_key(void)
{
    char aaa;
    std::cin >> aaa;
}
