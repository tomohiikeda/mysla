#include "Common.hpp"
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

double elapsedtime(struct timeval starttime)
{
    struct timeval timeval_end;
    gettimeofday(&timeval_end, NULL);
    double start = (double)starttime.tv_sec + (double)starttime.tv_usec / 1000000;
    double end   = (double)timeval_end.tv_sec + (double)timeval_end.tv_usec / 1000000;
    return end - start;
}
