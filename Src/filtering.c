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
#define  D_VALUE_SAMPLE_NUM									3  //��������ֵ����󣬼�����������

#define  XiaoDou_VALUE_IDX									0  //�˲���Чֵ����������������ֵ
#define  XiaoDou_CNT_IDX										1  //�˲�����������
#define  XiaoDou_BUFFER_BEGIN_IDX						2  //�˲�����ֵ��������ʼ����

uint32_t  XiaoDouBuff[TAG_MAX][7];
uint8_t		SAMPLE_D_VALUE = 0;

uint32_t  XiaoDou(uint32_t tmp, uint8_t channel)//�����˲�
{
	if(SAMPLE_D_VALUE == 0)                                            //δ���ֲ���ֵ������
	{
		if(tmp == XiaoDouBuff[channel][XiaoDou_VALUE_IDX])
		{
			XiaoDouBuff[channel][XiaoDou_CNT_IDX] = 0;                     //����������ֵ�뵱ǰ��Чֵ��ȣ�����������
		}
		else                                                             //����
		{
			if(XiaoDouBuff[channel][XiaoDou_CNT_IDX] == XIAODOU_NUM - 1)	 //��������������ֵ
			{
				if((tmp - XiaoDouBuff[channel][XiaoDou_VALUE_IDX] > 7) || (XiaoDouBuff[channel][XiaoDou_VALUE_IDX] - tmp > 7))  //���ֲ��ֵ��ֵ������
				{
					SAMPLE_D_VALUE = 1;             												   //�����˲������������ôβ��֮����Ϊ��Чֵ
				}
				else                                                         //δ���ֲ���ֵ������
				{
					XiaoDouBuff[channel][XiaoDou_VALUE_IDX] = tmp;             //���һ�β�����ֵ�滻��ǰ��Чֵ
				}
				XiaoDouBuff[channel][XiaoDou_CNT_IDX] = 0;                   //����������
			}
			else
			{
				XiaoDouBuff[channel][XiaoDou_CNT_IDX]++;                     //������δ�ﵽ����ֵ����������һ
//				XiaoDouBuff[channel][XiaoDouBuff[channel][XiaoDou_CNT_IDX]+XiaoDou_BUFFER_BEGIN_IDX] = tmp;
			}
		}
	}
	else                                                               //���ֲ��ֵ��ֵ�����󣬼�������3�β���
	{
		if((tmp - XiaoDouBuff[channel][XiaoDou_VALUE_IDX] > 7) || (XiaoDouBuff[channel][XiaoDou_VALUE_IDX] - tmp > 7))
		{
			XiaoDouBuff[channel][XiaoDou_CNT_IDX]++;
			if(XiaoDouBuff[channel][XiaoDou_CNT_IDX] >= D_VALUE_SAMPLE_NUM)//����3�β��ֵ�뵱ǰ��Чֵ��ֵ�������������ֵ
			{
				XiaoDouBuff[channel][XiaoDou_VALUE_IDX] = tmp;               //���һ�β�����ֵ�滻��ǰ��Чֵ
				XiaoDouBuff[channel][XiaoDou_CNT_IDX] = 0;                   //����������
				SAMPLE_D_VALUE = 0;                                          //�ôβ��ֵ��Ч
			}
		}
		else                                                             //3�β����������һ�β���ֵ����Χ��
		{
			XiaoDouBuff[channel][XiaoDou_CNT_IDX] = 0;                     //����������  
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






