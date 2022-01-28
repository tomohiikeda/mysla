#pragma once

double to_radian(double degree);
double to_degree(double radian);
void wait_for_key(void);

double get_currenttime(void);
double elapsedtime(double starttime);
void wait_for_time(double starttime, double sec);
