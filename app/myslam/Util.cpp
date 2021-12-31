#include <cmath>
#include <stdio.h>
#include <iostream>

double to_radian(double degree)
{
    return degree * M_PI / 180;
}

double to_degree(double radian)
{
    return radian * 180 / M_PI;
}

void wait_for_key(void)
{
    char aaa;
    std::cin >> aaa;
}
