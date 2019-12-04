/**
  ******************************************************************************
  * �ļ�����: main.c
  * ��    ��: ������
	* ��ǰ�汾��V1.0
	* ������ڣ�
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "lcd.h"
#include "port.h"
#include "sleep.h"

#define FRAME_LEN_MAX            127
#define BLINK_FRAME_SN_IDX       1
#define TX_DELAY_MS              1000

#define TX_ANT_DLY               0          //���������ӳ�
#define RX_ANT_DLY               32880      //���������ӳ�

#define ALL_MSG_COMMON_LEN       10

#define ALL_MSG_SN_IDX                        2   //֡����ֵ����
#define FINAL_MSG_POLL_TX_TS_IDX              10  //finally��Ϣ�У�POLL����ʱ�������
#define FINAL_MSG_RESP_RX_TS_IDX              14  //finally��Ϣ�У�RESP����ʱ�������
#define FINAL_MSG_FINAL_TX_TS_IDX             18  //finally��Ϣ�У�FINAL����ʱ�������
#define FINAL_MSG_TS_LEN                      4   //finally��Ϣ�У�ʱ������ȣ�4���ֽ�

#define RX_BUF_LEN            20
#define RX_BUF_LEN2           24

/* UWB microsecond (uus) �� device time unit (dtu, 1/(499.2MHz*128)��15.65ps) ����ϵ��.
 * 1 uus = 512 / 499.2 us 
 * 1 us  = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME       65536

#define POLL_RX_TIMEOUT_UUS									1000

#define POLL_TX_TO_RESP_RX_DLY_UUS          150  //POLL������ɵ���ʼ����RESP�ӳ�ʱ��

/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS         3100

/* Receive response timeout. */
#define RESP_RX_TIMEOUT_UUS                 2700  //RESP���ճ�ʱʱ��
#define POLL_RX_TO_RESP_TX_DLY_UUS          2600  //POLL������ɵ���ʼ����RESP�ӳ�ʱ��

#define RESP_TX_TO_FINAL_RX_DLY_UUS         500   //RESP������ɵ���ʼ����FINAL�ӳ�ʱ��

/* Receive final timeout.  */
#define FINAL_RX_TIMEOUT_UUS                4300  //FINAL���ճ�ʱʱ��
#define SPEED_OF_LIGHT                      299702547  //����

#define RNG_DELAY_MS                        20

static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 8 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};


dwt_txconfig_t  txconfig = 
{
	TC_PGDELAY_CH5,    /*��������*/
	0x1F1F1F1F         /*���͹��ʵ���*/
};
/*֡��ʽ��
 * byte 0--9(����֡ͨ��):
 *     - byte 0/1: ֡������ (0x8841 ��ʾʹ��16λ��ַ).
 *     - byte 2  : ��Ϣ����, ÿ����һ֡��һ(������֤��Ϣ�������ԣ��Ƿ�֡).
 *     - byte 3/4: PAN TAG_ID (0xDECA).
 *     - byte 5/6: Ŀ���ַ.
 *     - byte 7/8: Դ��ַ.
 *     - byte 9  : ֡���.
 * byte 10--21(��֡������ͬ):
 *    Poll ֡:
 *     - no more data
 *    Response ֡:
 *     - byte 10   : RESP״̬λ (0x02 ��ʾ�������ݴ���).
 *     - byte 11/12: RESP����(byte 10Ϊ0x02ʱ�����ֽ�δʹ��).
 *    Final ֡:
 *     - byte 10 -> 13: poll ����ʱ���.
 *     - byte 14 -> 17: response ����ʱ���.
 *     - byte 18 -> 21: final ����ʱ���.
 * ��������֡������Ϻ󶼻���֡ĩ���2�ֽڵ�CRC���ݣ����ڽ���У��(��DW1000�Զ����).
*/
/*��վ*/
static uint8 rx_poll_msg[] = {0x00, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/*��ǩ*/	
static uint8 tx_poll_msg[] = {0x00, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


static uint64_t poll_tx_ts;
static uint64_t resp_rx_ts;
static uint64_t final_tx_ts;

static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;

static double tof;
static double distance,dist,dist2;
int32_t dis;
uint32_t SFDConfig;

double STD_NOISE_Poll;
double STD_NOISE_Final;

double FP_AMPL2_Poll;
double FP_AMPL2_Final;

double NoiseFigure_Poll;
double NoiseFigure_Final;


/* ֡����ֵ, ÿ�η�����ɺ��һ. */
static uint32 frame_seq_nb = 0;
	
static uint32_t TAG_MsgNum = 0;

static uint8 rx_buffer[RX_BUF_LEN+4];
static uint32 status_reg = 0;
	
uint8_t TAG_ID;              //��ǩID
uint8_t ANCHOR_ID;           //��վID
uint8_t jumptime=0;

uint32_t ld[100];            //��ͨ�˲�������

uint16_t DECA_WorkMode = 1;  //UWBģʽ����   0����ǩ   1����վ   2:ң��
OS_EVENT  *WorkModeSem;             //����ģʽ�ź���

/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 *     - byte 10/11: frame check-sum, automatically set by DW1000.  */

uint8_t TX_TimeStamp[5];

void SystemClock_Config(void);
void UWB_Tag(void *pdata);
void UWB_Anchor(void *pdata);
void UWB_RemoteControl(void *pdata);
void TaskStart(void *pdata);

//static uint8 rx_buffer[FRAME_LEN_MAX];
static uint16 frame_len = 0;

extern uint8_t USART1RxBuff[10];

/*���������ջ������256*/
OS_STK      TaskStartStk[OS_TASK_STAT_STK_SIZE];
OS_STK      UWB_ModeChooseStk[OS_TASK_STAT_STK_SIZE];
OS_STK      UWB_TagStk[OS_TASK_STAT_STK_SIZE];
OS_STK      UWB_AnchorStk[OS_TASK_STAT_STK_SIZE];
OS_STK      UWB_RemoteControlStk[OS_TASK_STAT_STK_SIZE];
OS_STK      USART1_RxDataStk[OS_TASK_STAT_STK_SIZE];
OS_STK      USART1_TxDataStk[OS_TASK_STAT_STK_SIZE];
OS_STK      LED_BlinkStk[OS_TASK_STAT_STK_SIZE];

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64_t get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*******************************************************************************************
* �������ƣ�final_msg_get_ts()
* ����������
* ��ڲ�����
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
 static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
    uint32_t i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_set_ts(uint8 *ts_field, uint64_t ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64_t get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*******************************************************************************************
* �������ƣ�LP()
* ����������һ���ͺ��˲�
* ��ڲ�����tmp, channel
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
int LP(uint32_t tmp,uint8_t channel)
{
	uint32_t data;
	data = 0.7*ld[channel]+0.3*tmp;
	ld[channel]=data;
	return data;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	OSInit();           //��ʼ��OSϵͳ
	WorkModeSem=OSSemCreate(1);//����һ���ź���������DECA_WorkMode
	OSTaskCreate(TaskStart,(void *)0,&TaskStartStk[OS_TASK_STAT_STK_SIZE-1],4);          //����TaskStart����,���ȼ�4
	OSStart();          //��ʼ����OSϵͳ
}

/*******************************************************************************************
* �������ƣ�UWB_ModeChoose(void *pdata)
* ����������UWBģ�����ѡ�񣺱�ǩ����վ��ң��
* ��ڲ�����*pdata
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void UWB_ModeChoose(void *pdata)
{
	INT8U err;
	for(;;)
	{
//		OSSemPend(WorkModeSem,0,&err);              //���WorkModeSem�ź�����
		switch(DECA_WorkMode)      //�ж�UWB����ģʽ
		{
			case 0:
				printf("Now Model is work as TAG\r\n");
				OSTaskResume(UWB_TAG_PRIO);           //������ǩ������
				OSTaskSuspend(UWB_ANCHOR_PRIO);       //�����վ������
				break;
			case 1:
				OSTaskResume(UWB_ANCHOR_PRIO);        //������վ������
				OSTaskSuspend(UWB_TAG_PRIO);          //�����ǩ������
				break;
			case 2:
				OSTaskResume(UWB_REMOTECONTROL_PRIO);   //ң����
		}
//		OSSemPost(WorkModeSem);                     //�ͷ��ź���
		OSTaskSuspend(OS_PRIO_SELF);                //ģʽ�ж���ϣ������Լ���ת����Ӧ��ģʽ����ִ��
	}
}

/*******************************************************************************************
* �������ƣ�TaskStart(void *pdata)
* ����������OSϵͳ���еĵ�һ�����񣬸��������ʼ������������Ĵ���
* ��ڲ�����*pdata
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void TaskStart(void *pdata)
{
	/* ��λ�������裬����Systick����1ms */
  HAL_Init();
  /* ����ϵͳʱ��*/
  SystemClock_Config();
  /* ��ʼ������ */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
	
	reset_DW1000();                                  //�����λDW1000
	spi_set_rate_low();                              //����SPIʱ��Ƶ��72/32MHz��Init״̬�£�SPICLK������3MHz��
	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)   //��ʼ��DW1000
	{
			while (1)
			{};
	}
  spi_set_rate_high();	                           //�ָ�SPIʱ��Ƶ��Ϊ72/4MHz��IDLE״̬�£�SPICLK������20MHz��
	dwt_configure(&config);                          //����ͨ�Ų���
	
	dwt_setrxantennadelay(RX_ANT_DLY);		           //���ý��������ӳ�
  dwt_settxantennadelay(TX_ANT_DLY);		           //���÷��������ӳ�
	
	dwt_read32bitreg(TX_POWER_ID);
	SFDConfig = dwt_read32bitreg(USR_SFD_ID);
	
	OSTaskCreate(UWB_ModeChoose,(void *)0,&UWB_ModeChooseStk[OS_TASK_STAT_STK_SIZE-1],UWB_MODECHOOSE_PRIO);
	OSTaskCreate(LED_Blink,(void *)0,&LED_BlinkStk[OS_TASK_STAT_STK_SIZE-1],LED_BLINK_PRIO);
	OSTaskCreateExt(UWB_Tag,
									(void *)0,
									(OS_STK *)&UWB_TagStk[OS_TASK_STAT_STK_SIZE-1],
									UWB_TAG_PRIO,
									UWB_TAG_PRIO,
									(OS_STK *)&UWB_TagStk[0],
									OS_TASK_STAT_STK_SIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);
	OSTaskCreateExt(UWB_Anchor,
									(void *)0,
									(OS_STK *)&UWB_AnchorStk[OS_TASK_STAT_STK_SIZE-1],
									UWB_ANCHOR_PRIO,
									UWB_ANCHOR_PRIO,
									(OS_STK *)&UWB_AnchorStk[0],
									OS_TASK_STAT_STK_SIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);
	OSTaskCreateExt(UWB_RemoteControl,
									(void *)0,
									(OS_STK *)&UWB_RemoteControlStk[OS_TASK_STAT_STK_SIZE-1],
									UWB_REMOTECONTROL_PRIO,
									UWB_REMOTECONTROL_PRIO,
									(OS_STK *)&UWB_RemoteControlStk[0],
									OS_TASK_STAT_STK_SIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);
	OSTaskCreateExt(USART1_RxData,
									(void *)0,
									(OS_STK *)&USART1_RxDataStk[OS_TASK_STAT_STK_SIZE],
									USART1_RXDATA_PRIO,
									USART1_RXDATA_PRIO,
									(OS_STK *)&USART1_RxDataStk[0],
									OS_TASK_STAT_STK_SIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);
	OSTaskCreateExt(USART1_TxData,
									USART1RxBuff,
									(OS_STK *)&USART1_TxDataStk[OS_TASK_STAT_STK_SIZE],
									USART1_TXDATA_PRIO,
									USART1_TXDATA_PRIO,
									(OS_STK *)&USART1_TxDataStk[0],
									OS_TASK_STAT_STK_SIZE,
									(void *)0,
									OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);								
	OSTaskSuspend(UWB_TAG_PRIO);
	OSTaskSuspend(UWB_ANCHOR_PRIO);
	OSTaskSuspend(UWB_REMOTECONTROL_PRIO);
	OSTaskSuspend(USART1_RXDATA_PRIO);
	OSTaskSuspend(USART1_TXDATA_PRIO);
	
	OSTaskDel(OS_PRIO_SELF);
	
}


/*******************************************************************************************
* �������ƣ�UWB_Label(void *pdata)
* ����������UWB��Ϊ��ǩʱ��ִ�к���
* ��ڲ�����*pdata
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void UWB_Tag(void *pdata)
{
	INT8U err;
	float FloatNum = 0;
	for(;;)
	{
//		dwt_rxreset();
		dwt_forcetrxoff();
		OSSemPend(WorkModeSem,0,&err);              //��ѯ�ź���
//		LED_RED_OFF;
		
		dwt_setinterrupt(DWT_INT_ALL, DISABLE);      //�ر��жϣ�����ӻ�վ�л�����ǩʱ������������жϻָ���վ����
		
		/* Set expected response's delay and timeout. See NOTE 4 and 5 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);//���÷��ͺ������գ����趨�ӳ�ʱ�䣬��λ��uus
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);						//���ý��ճ�ʱʱ�䣬��λ��uus
		
		/* Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
		tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
		dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);//��Poll�����ݴ���DW1000�����ڿ�������ʱ����ȥ
		dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 0);//���ó�����������ݳ���
		
		/* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
		 * set by dwt_setrxaftertxdelay() has elapsed. */
		dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);//�������ͣ�������ɺ�ȴ�һ��ʱ�俪�����գ��ȴ�ʱ����dwt_setrxaftertxdelay������
		
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))//���ϲ�ѯоƬ״ֱ̬���������
		{ };
		
		HAL_GPIO_TogglePin(LED_PIN_Port, LED4_PIN);//LED�����˸

		/* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. */
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))//���ϲ�ѯоƬ״ֱ̬���ɹ����ջ��߷���������߽��ճ�ʱ
		{ };

		/* Increment frame sequence number after transmission of the poll message (modulo 256). */
		frame_seq_nb++;

		if (status_reg & SYS_STATUS_RXFCG)//����ɹ�����
		{
			uint32 frame_len;

			/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);//����Ĵ�����־λ

			/* A frame has been received, read it into the local buffer. */
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;	//��ý��յ������ݳ���
			if (frame_len <= RX_BUF_LEN)
			{
				dwt_readrxdata(rx_buffer, frame_len, 0);   //��ȡ��������
			}

			/* Check that the frame is the expected response from the companion "DS TWR responder" example.
			 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
			rx_buffer[ALL_MSG_SN_IDX] = 0;
			if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)//�жϽ��յ��������Ƿ���response����
			{
				uint32 final_tx_time;

			
				/* Retrieve poll transmission and response reception timestamp. */
				poll_tx_ts = get_tx_timestamp_u64();										//���POLL����ʱ��T1
				resp_rx_ts = get_rx_timestamp_u64();										//���RESPONSE����ʱ��T4

				/* Compute final message transmission time. See NOTE 9 below. */
				final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;//����final������ʱ�䣬T5=T4+Treply2
				dwt_setdelayedtrxtime(final_tx_time);//����final������ʱ��T5

				/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
				final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;//final��ʵ�ʷ���ʱ���Ǽ���ʱ����Ϸ�������delay

				/* Write all timestamps in the final message. See NOTE 10 below. */
				final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);//��T1��T4��T5д�뷢������
				final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
				final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

				/* Write and send final message. See NOTE 7 below. */
				tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);//����������д��DW1000
				dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1);//�趨�������ݳ���
				dwt_starttx(DWT_START_TX_DELAYED);//�趨Ϊ�ӳٷ���

				/* Poll DW1000 until TX frame sent event set. See NOTE 8 below. */
				while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))//���ϲ�ѯоƬ״ֱ̬���������
				{ };

				/* Clear TXFRS event. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);//�����־λ

				/* Increment frame sequence number after transmission of the final message (modulo 256). */
				frame_seq_nb++;
//				time32_reset = 0;
				HAL_GPIO_TogglePin(LED_PIN_Port, LED3_PIN);//LED�̵���˸
				jumptime = 0;
			}
			else
			{
				jumptime =5;//���ͨѶʧ�ܣ������ʱ������5ms���ܿ���Ϊ���ǩͬʱ��������ĳ�ͻ��
			}
    }
		else
		{
			/* Clear RX error events in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
			
			dwt_rxreset();
			jumptime = 5;
		}
//		FloatNum += 0.01f;
//		printf("float numble is: %.4f\r\n",FloatNum);
		
		OSSemPost(WorkModeSem);  //�ͷ��ź���
		
//		HAL_GPIO_TogglePin(LED_PIN_Port, LED4_PIN);//LED�����˸
		
		/* Execute a delay between ranging exchanges. */
		OSTimeDly(RNG_DELAY_MS+jumptime);//���߹̶�ʱ��
	}
}

/*******************************************************************************************
* �������ƣ�UWB_BaseStation(void *pdata)
* ����������UWB��Ϊ��վʱ��ִ�к���
* ��ڲ�����*pdata
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void UWB_Anchor(void *pdata)
{

INT8U err;
	
	for(;;)
	{
		
		LED_RED_ON;
		
		/* Clear reception timeout to start next ranging process. */
		dwt_setrxtimeout(0);//�趨���ճ�ʱʱ�䣬0λû�г�ʱʱ��
//		dwt_setrxtimeout(POLL_RX_TIMEOUT_UUS);
		
		dwt_setinterrupt(DWT_INT_ALL, ENABLE);      //�����ж�

		/* Activate reception immediately. */
		dwt_rxenable(DWT_START_RX_IMMEDIATE);//�򿪽���
		
		OSTaskSuspend(OS_PRIO_SELF);         //�������񣬵ȴ������ж�

		/* Poll for reception of a frame or error/timeout. See NOTE 7 below. */
//		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))//���ϲ�ѯоƬ״ֱ̬�����ճɹ����߳��ִ���
//		{ };
		
		OSSemPend(WorkModeSem,0,&err);		              //��ѯ�ź���

		if (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_RXFCG)//�ɹ�����
		{
			/* Clear good RX frame event in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);//�����־λ

			/* A frame has been received, read it into the local buffer. */
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;//��ý������ݳ���
			if (frame_len <= RX_BUFFER_LEN)
			{
					dwt_readrxdata(rx_buffer, frame_len, 0);//��ȡ��������
			}

			/* Check that the frame is a poll sent by "DS TWR initiator" example.
			 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
			rx_buffer[ALL_MSG_SN_IDX] = 0;
			TAG_ID = rx_buffer[7];
			rx_poll_msg[7] = TAG_ID;//Ϊ���ǩͨѶ���񣬷�ֹһ��ͨѶ�н��յ���ͬID��ǩ������
			tx_resp_msg[5] = TAG_ID;
			rx_final_msg[7] = TAG_ID;
			if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)//�ж��Ƿ���poll������
			{
				uint32 resp_tx_time;
				
				dwt_setinterrupt(DWT_INT_ALL, DISABLE);      //�ر��ж�
				
				/* Retrieve poll reception timestamp. */
				poll_rx_ts = get_rx_timestamp_u64();//���Poll������ʱ��T2

				/* Set send time for response. See NOTE 8 below. */
				resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;//����Response����ʱ��T3��
				dwt_setdelayedtrxtime(resp_tx_time);//����Response����ʱ��T3
				
				STD_NOISE_Poll = dwt_read16bitoffsetreg(RX_FQUAL_ID, 0);
				FP_AMPL2_Poll = dwt_read16bitoffsetreg(RX_FQUAL_ID, 2);
				NoiseFigure_Poll = (STD_NOISE_Poll / FP_AMPL2_Poll) * 1000;

				/* Set expected delay and timeout for final message reception. */
				dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);//���÷�����ɺ��������ӳ�ʱ��
				dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);//���ճ�ʱʱ��

				/* Write and send the response message. See NOTE 9 below.*/
		//		memcpy(&tx_resp_msg[11],&dis,4);
				tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);//д�뷢������
				dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);//�趨���ͳ���
				dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);//�ӳٷ��ͣ��ȴ�����

				/* We assume that the transmission is achieved correctly, now poll for reception of expected "final" frame or error/timeout.
				 * See NOTE 7 below. */
				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))///���ϲ�ѯоƬ״ֱ̬�����ճɹ����߳��ִ���
				{ };

				/* Increment frame sequence number after transmission of the response message (modulo 256). */
				frame_seq_nb++;

				if (status_reg & SYS_STATUS_RXFCG)//���ճɹ�
				{
					/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);//�����־λ
					
					/* A frame has been received, read it into the local buffer. */
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;//���ݳ���
					if (frame_len <= RX_BUF_LEN2)
					{
							dwt_readrxdata(rx_buffer, frame_len, 0);//��ȡ��������
					}
					
					TAG_MsgNum = rx_buffer[ALL_MSG_SN_IDX];    //��ȡ֡���к�
					/* Check that the frame is a final message sent by "DS TWR initiator" example.
					 * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)//�ж��Ƿ�ΪFianl��
					{
						uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
						uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
						double Ra, Rb, Da, Db;
						int64_t tof_dtu;
						
						HAL_GPIO_TogglePin(LED_PIN_Port, LED3_PIN);//LED�̵���˸
						
						STD_NOISE_Final = dwt_read16bitoffsetreg(RX_FQUAL_ID, 0);
						FP_AMPL2_Final = dwt_read16bitoffsetreg(RX_FQUAL_ID, 2);
						NoiseFigure_Final = (STD_NOISE_Poll / FP_AMPL2_Poll) * 1000;

						/* Retrieve response transmission and final reception timestamps. */
						resp_tx_ts = get_tx_timestamp_u64();//���response����ʱ��T3
						final_rx_ts = get_rx_timestamp_u64();//���final����ʱ��T6

						/* Get timestamps embedded in the final message. */
						final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);//�ӽ��������ж�ȡT1��T4��T5
						final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
						final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

						/* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 10 below. */
						poll_rx_ts_32 = (uint32)poll_rx_ts;//ʹ��32λ���ݼ���
						resp_tx_ts_32 = (uint32)resp_tx_ts;
						final_rx_ts_32 = (uint32)final_rx_ts;
						Ra = (double)(resp_rx_ts - poll_tx_ts);//Tround1 = T4 - T1  
						Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);//Tround2 = T6 - T3 
						Da = (double)(final_tx_ts - resp_rx_ts);//Treply2 = T5 - T4  
						Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);//Treply1 = T3 - T2  
						tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));//���㹫ʽ

						tof = tof_dtu * DWT_TIME_UNITS;
						distance = tof * SPEED_OF_LIGHT;//����=����*����ʱ��
//						dist2 = distance; //distance - dwt_getrangebias(config.chan,(float)distance, config.prf);//�����ȥ����ϵ��
						dist2 = distance + DistAdjustRange(distance);
						
						dis = dist2 * 100;//dis Ϊ��λΪcm�ľ���
//						dist = (double) dis;
						dist = XiaoDou(dis, 0);
//						if(NoiseFigure_Poll > 15)
//						{
//							printf("NoiseFigure_Poll: %1.2f, dist: %3.0f\r\n", NoiseFigure_Poll, dist);
//						}
//						dist = LP(dis,TAG_ID);//LP Ϊ��ͨ�˲����������ݸ��ȶ�
//						time32_reset = 0;
//						GPIO_Toggle(GPIOA,LED_PIN);
//						if (GPIO_ReadInputDataBit(GPIOA,SW2) != RESET)  //ͨ�����뿪���ж����������ʽ
//						{
//							dID=TAG_ID;
//							printf("TAG_ID: %2.0f		", dID);
//							dID=ANCHOR_ID;
//							printf("ANCHOR_ID: %2.0f		", dID);
//							printf("Distance: %5.0f cm\n", dist);
//						}
//						else
//						{
//							send[2] = ANCHOR_ID;
//							send[3] = TAG_ID;
//							hex_dist=dist;
//							memcpy(&send[4],&hex_dist,4);
//							check=Checksum_u16(&send[2],6);
//							memcpy(&send[8],&check,2);
//							USART_puts(send,10);
//						}
//						printf("TAG Send Message Num: %d, Anchor Receive Message Num: %d\r\n",TAG_MsgNum, frame_seq_nb);
          }
        }
				else
				{
					/* Clear RX error events in the DW1000 status register. */
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
					dwt_rxreset();
				}
      }
    }
		else
		{
			/* Clear RX error events in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
			dwt_rxreset();
		}
//		HAL_GPIO_TogglePin(LED_PIN_Port, LED3_PIN);//LED�̵���˸
		OSSemPost(WorkModeSem);  //�ͷ��ź���
//		OSTaskSuspend(OS_PRIO_SELF);
//		OSTimeDly(RNG_DELAY_MS);
  }
}

/*******************************************************************************************
* �������ƣ�void UWB_RemoteControl(void *pdata)
* ����������UWB��Ϊң��ʱ��ִ�к���
* ��ڲ�����*pdata
* ���ڲ�������
* ʹ��˵������
********************************************************************************************/
void UWB_RemoteControl(void *pdata)
{
	for(;;)
	{
		
	}
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 36;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
