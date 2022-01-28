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

double get_currenttime(void)
{
    struct timeval timeval;
    gettimeofday(&timeval, NULL);
    return (double)timeval.tv_sec + (double)timeval.tv_usec / 1000000;
}

double elapsedtime(double starttime)
{
    return get_currenttime() - starttime;
}

void wait_for_time(double starttime, double sec)
{
    while (elapsedtime(starttime) < sec);
}