/*! ----------------------------------------------------------------------------
 * @file	port.c
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include "sleep.h"
#include "lcd.h"
#include "port.h"
#include "main.h"


#define EXTI_MODE_OFFSET                    0x08u   /* 0x20: offset between MCU IMR/EMR registers */
#define EXTI_CONFIG_OFFSET                  0x08u   /* 0x20: offset between MCU Rising/Falling configuration registers */

/* DW1000 IRQ handler definition. */
port_DecaISR_f fpport_Deca_ISR = NULL;


int No_Configuration(void)
{
	return -1;
}

unsigned long portGetTickCnt(void)
{
	return uwTick;
}

/**
  * @brief  Checks whether the specified EXTI line is enabled or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The "enable" state of EXTI_Line (SET or RESET).
  */
ITStatus EXTI_GetITEnStatus(uint32_t EXTI_Line)
{
  ITStatus bitstatus = RESET;
  uint32_t enablestatus = 0;
	 __IO uint32_t *regaddr;
  uint32_t regval;
  uint32_t offset;
  /* Check the parameters */
  assert_param(IS_EXTI_LINE(EXTI_Line));
	
	offset = ((EXTI_Line & EXTI_REG_MASK) >> EXTI_REG_SHIFT);
	regaddr = (&(EXTI->IMR1) + (EXTI_CONFIG_OFFSET * offset));
  regval = *regaddr;
  enablestatus =  regval & (EXTI_Line & EXTI_PIN_MASK);
  if (enablestatus != (uint32_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn port_set_deca_isr()
 *
 * @brief This function is used to install the handling function for DW1000 IRQ.
 *
 * NOTE:
 *   - As EXTI9_5_IRQHandler does not check that port_deca_isr is not null, the user application must ensure that a
 *     proper handler is set by calling this function before any DW1000 IRQ occurs!
 *   - This function makes sure the DW1000 IRQ line is deactivated while the handler is installed.
 *
 * @param deca_isr function pointer to DW1000 interrupt handler to install
 *
 * @return none
 */
void port_set_deca_isr(port_DecaISR_f deca_isr)
{
    /* Check DW1000 IRQ activation status. */
    ITStatus en = port_GetEXT_IRQStatus();

    /* If needed, deactivate DW1000 IRQ during the installation of the new handler. */
    if (en)
    {
        port_DisableEXT_IRQ();
    }
    fpport_Deca_ISR = deca_isr;
    if (en)
    {
        port_EnableEXT_IRQ();
    }
}

/*******************************************************************************************
* 函数名称：reset_DW1000()
* 功能描述：DW1000复位函数
* 入口参数：无
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void reset_DW1000(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Enable GPIO used for DW1000 reset
	GPIO_InitStruct.Pin = D_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(D_RST_GPIO_Port, &GPIO_InitStruct);
	
	//drive the RSTn pin low 10ns at least for reset DW1000
	HAL_GPIO_WritePin(D_RST_GPIO_Port,D_RST_Pin,GPIO_PIN_RESET);

	//put the pin back to tri-state ... as input
	GPIO_InitStruct.Pin = D_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(D_RST_GPIO_Port, &GPIO_InitStruct);

  sleep_ms(2);
}

/*******************************************************************************************
* 函数名称：SPI_ChangeRate()
* 功能描述：SPI1波特率更改
* 入口参数：波特率分频系数
* 出口参数：无
* 使用说明：无
********************************************************************************************/
void SPI_ChangeRate(uint16_t ScalingFactor)
{
	uint16_t tmpreg = 0;

	/* Get the SPIx CR1 value */
	tmpreg = SPI1->CR1;

	/*clear the scaling bits*/
	tmpreg &= 0xFFC7;

	/*set the scaling bits*/
	tmpreg |= ScalingFactor;

	/* Write to SPIx CR1 */
	SPI1->CR1 = tmpreg;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_low()
 *
 * @brief Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
 *
 * @param none
 *
 * @return none
 */
void spi_set_rate_low (void)
{
    SPI_ChangeRate(SPI_BAUDRATEPRESCALER_32);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_high()
 *
 * @brief Set SPI rate as close to 20 MHz as possible for optimum performances.
 *
 * @param none
 *
 * @return none
 */
void spi_set_rate_high (void)
{
    SPI_ChangeRate(SPI_BAUDRATEPRESCALER_4);
}









