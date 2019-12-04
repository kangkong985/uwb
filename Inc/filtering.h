#ifndef  __FILTERING_H
#define  __FILTERING_H
#include "main.h"


uint32_t Sum_Average(uint32_t tmp, uint8_t channel);
uint32_t  XiaoDou(uint32_t tmp, uint8_t channel);
double DistAdjustRange(double distance);
double DistAdjustPart1(double distance);
double DistAdjustPart2(double distance);

#endif
