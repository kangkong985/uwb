#include "filtering.h"

#define TAG_MAX     20
#define SUM_AVERAGE_SAMPLE_NUM    10

#define DIS_ADJUST_POINT          28.14


uint32_t SumAverage[TAG_MAX][SUM_AVERAGE_SAMPLE_NUM];

//uint32_t Sum;
uint8_t  Cnt = 0;

uint32_t Sum_Average(uint32_t tmp, uint8_t channel)
{
	uint32_t Sum;
	uint8_t i;
	SumAverage[channel][Cnt] = tmp;
//	Sum += SumAverage[channel][Cnt];
	Cnt++;
	
	if(Cnt == SUM_AVERAGE_SAMPLE_NUM/2)
	{
		for(i = 0; i < Cnt; i++)
		{
			Sum += SumAverage[channel][i];
		}
		return (uint32_t)Sum/Cnt;
	}
	if(!(Cnt < SUM_AVERAGE_SAMPLE_NUM))
	{
		for(i = 0; i < Cnt; i++)
		{
			Sum += SumAverage[channel][i];
		}
		Cnt = 0;
		return (uint32_t)Sum/SUM_AVERAGE_SAMPLE_NUM;
	}
	return SumAverage[channel][0];
}

#define  XIAODOU_NUM      									5
#define  D_VALUE_SAMPLE_NUM									3  //测距采样差值过大后，继续采样次数

#define  XiaoDou_VALUE_IDX									0  //滤波有效值索引，即函数返回值
#define  XiaoDou_CNT_IDX										1  //滤波计数器索引
#define  XiaoDou_BUFFER_BEGIN_IDX						2  //滤波采样值缓存区起始索引

uint32_t  XiaoDouBuff[TAG_MAX][7];
uint8_t		SAMPLE_D_VALUE = 0;

uint32_t  XiaoDou(uint32_t tmp, uint8_t channel)//消抖滤波
{
	if(SAMPLE_D_VALUE == 0)                                            //未出现测距差值大现象
	{
		if(tmp == XiaoDouBuff[channel][XiaoDou_VALUE_IDX])
		{
			XiaoDouBuff[channel][XiaoDou_CNT_IDX] = 0;                     //如果采样测距值与当前有效值相等，计数器清零
		}
		else                                                             //否则
		{
			if(XiaoDouBuff[channel][XiaoDou_CNT_IDX] == XIAODOU_NUM - 1)	 //计数器到达上限值
			{
				if((tmp - XiaoDouBuff[channel][XiaoDou_VALUE_IDX] > 7) || (XiaoDouBuff[channel][XiaoDou_VALUE_IDX] - tmp > 7))  //出现测距值差值大现象
				{
					SAMPLE_D_VALUE = 1;             												   //增加滤波采样次数，该次测距之暂视为无效值
				}
				else                                                         //未出现测距差值大现象
				{
					XiaoDouBuff[channel][XiaoDou_VALUE_IDX] = tmp;             //最后一次测距采样值替换当前有效值
				}
				XiaoDouBuff[channel][XiaoDou_CNT_IDX] = 0;                   //计数器清零
			}
			else
			{
				XiaoDouBuff[channel][XiaoDou_CNT_IDX]++;                     //计数器未达到上限值，计数器加一
//				XiaoDouBuff[channel][XiaoDouBuff[channel][XiaoDou_CNT_IDX]+XiaoDou_BUFFER_BEGIN_IDX] = tmp;
			}
		}
	}
	else                                                               //出现测距值差值大现象，继续进行3次采样
	{
		if((tmp - XiaoDouBuff[channel][XiaoDou_VALUE_IDX] > 7) || (XiaoDouBuff[channel][XiaoDou_VALUE_IDX] - tmp > 7))
		{
			XiaoDouBuff[channel][XiaoDou_CNT_IDX]++;
			if(XiaoDouBuff[channel][XiaoDou_CNT_IDX] >= D_VALUE_SAMPLE_NUM)//连续3次测距值与当前有效值差值均大于允许误差值
			{
				XiaoDouBuff[channel][XiaoDou_VALUE_IDX] = tmp;               //最后一次测距采样值替换当前有效值
				XiaoDouBuff[channel][XiaoDou_CNT_IDX] = 0;                   //计数器清零
				SAMPLE_D_VALUE = 0;                                          //该次测距值有效
			}
		}
		else                                                             //3次测距中至少有一次测距差值在误差范围内
		{
			XiaoDouBuff[channel][XiaoDou_CNT_IDX] = 0;                     //计数器清零  
			SAMPLE_D_VALUE = 0;
		}
	}
	return XiaoDouBuff[channel][XiaoDou_VALUE_IDX];
}


double DistAdjustPart1(double distance)
{
	double DisOffset;
	DisOffset = (-0.0000004563) * pow(distance, 5) + (0.00003567) * pow(distance, 4) - (0.001021) * pow(distance, 3) + (0.01337) * pow(distance, 2) - (0.08282) * distance + 0.1602;
	return DisOffset;
}


double DistAdjustPart2(double distance)
{
	double DisOffset;
	DisOffset = (-0.0000000003766) * pow(distance, 6) + (0.000000128) * pow(distance, 5) - (0.00001732) * pow(distance, 4) + (0.001185) * pow(distance, 3) - (0.04272) * pow(distance, 2) + (0.746) * distance - 4.7804;
	return DisOffset;
}


double DistAdjustRange(double distance)
{
	double DisOffset;
	if(distance <= DIS_ADJUST_POINT)
	{
		DisOffset = DistAdjustPart1(distance);
	}
	else
	{
		DisOffset = DistAdjustPart2(distance);
	}
	return DisOffset;
}






