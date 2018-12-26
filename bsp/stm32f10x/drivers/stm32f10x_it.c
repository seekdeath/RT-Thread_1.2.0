/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "ringbuffer.h"
#include "include.h"
#include <board.h> 

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

//void SysTick_Handler(void)
//{
//    // definition in boarc.c
//}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

#ifdef  RT_USING_LWIP
/*******************************************************************************
* Function Name  : EXTI4_IRQHandler
* Description    : This function handles External lines 9 to 5 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI4_IRQHandler(void)
{
    extern void rt_dm9000_isr(void);

    /* enter interrupt */
    rt_interrupt_enter();

    /* Clear the DM9000A EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line4);

    rt_dm9000_isr();

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_LWIP */

void USART1_IRQHandler(void)
{
      u32 usartxbase = 0x00;
	  u32 bitpos = (u32)0x01 << (USART_IT_RXNE >> 0x08), itmask = (u32)0x01 << (USART_IT_RXNE & 0x001F);//usartreg =(((u8)USART_IT_RXNE) >> 0x05);; 
	  
	  //接收中断的处理	 
	  itmask &= USART1->CR1;  
	  bitpos &= USART1->SR;	
	  if ((itmask != (u16)RESET)&&(bitpos != (u16)RESET))
	  {
			rt_ringbuffer_putchar(&rb_recv,USART1->DR&(u16)0x01FF);
	  }

  	  //发送中断的处理	 	 	  							  
	  itmask = (u32)0x01 << (USART_IT_TXE & ((u16)0x001F));	 	 
	  itmask &= USART1->CR1;  	
	  bitpos = (u32)0x01 << (USART_IT_TXE >> 0x08);	  
	  bitpos &= USART1->SR;	 

	  if ((itmask != (u16)0)&&(bitpos != (u16)0))
	  { 	   
			USART1->DR =USART1_TxBuffer[PTxBufferUSART11++];		
	    	if(PTxBufferUSART11 >= PTxBufferUSART12)
	    	{
	     
		  		PTxBufferUSART11=PTxBufferUSART12=0;
				usartxbase=USART1_BASE+0x0C;
				itmask = (((u32)0x01) << (USART_IT_TXE & 0x001F)); 					 				
				*(vu32*)usartxbase &= ~itmask; 		  
	    	} 
		  
	  } 
}

void USART2_IRQHandler(void)
{
      u32 usartxbase = 0x00;
	  u32 bitpos = (u32)0x01 << (USART_IT_RXNE >> 0x08), itmask = (u32)0x01 << (USART_IT_RXNE & 0x001F);//usartreg =(((u8)USART_IT_RXNE) >> 0x05);; 
	  
	  //接收中断的处理	 
	  itmask &= USART2->CR1;  
	  bitpos &= USART2->SR;	
	  if ((itmask != (u16)RESET)&&(bitpos != (u16)RESET))
	  {
			if(PRxBufferUSART12>=sizeof(USART1_RxBuffer))
			{
				PRxBufferUSART12=0;
			}
			USART1_RxBuffer[USART1_RxCounter] = USART2->DR&(u16)0x01FF;
			USART1_NbrOfDataReceived++;	
			USART1_RxCounter++;  
#if 0
	    USART1_RxBuffer[USART1_RxCounter++] = USART2->DR&(u16)0x01FF;
	    //USART1_NbrOfDataReceived++;	
			if(USART1_RxCounter>1&&(USART1_RxBuffer[USART1_RxCounter-1]==0x0a)&&(USART1_RxBuffer[USART1_RxCounter-2]==0x0d))	 
			{
		 		 USART1_Received_Flag=1;
			}  
#endif 
	  }

  	  //发送中断的处理	 	 	  							  
	  itmask = (u32)0x01 << (USART_IT_TXE & ((u16)0x001F));	 	 
	  itmask &= USART2->CR1;  	
	  bitpos = (u32)0x01 << (USART_IT_TXE >> 0x08);	  
	  bitpos &= USART2->SR;	 

	  if ((itmask != (u16)0)&&(bitpos != (u16)0))
	  { 	   
			USART2->DR =USART1_TxBuffer[PTxBufferUSART11++];		
	    	if(PTxBufferUSART11 >= PTxBufferUSART12)
	    	{
	     
		  		PTxBufferUSART11=PTxBufferUSART12=0;
				usartxbase=USART2_BASE+0x0C;
				itmask = (((u32)0x01) << (USART_IT_TXE & 0x001F)); 					 				
				*(vu32*)usartxbase &= ~itmask; 		  
	    	} 
		  
	  } 
}
/**
  * @}
  */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
