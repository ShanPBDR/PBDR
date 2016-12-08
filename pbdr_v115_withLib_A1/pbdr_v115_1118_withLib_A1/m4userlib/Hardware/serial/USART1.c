#define  BSP_UART2_MODULE
#include <USART1.h>

#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

//#define UART1_DMA_ENABLE
#define  BSP_UART1_PRINTF_STR_BUF_SIZE             80u

static CPU_INT08U   BSP_Uart1RxData[256];
static CPU_INT08U   BSP_Uart1TxData[256];
volatile  CPU_INT08U   BSP_Uart1TxDataLenth=0;
volatile  CPU_INT08U   BSP_Uart1TxDataFlag=0;
volatile  CPU_INT08U   BSP_Uart1RxDataLenth=0;
volatile  CPU_INT08U   BSP_Uart1RxDataFlag=0;

unsigned char USART2_RE_RTKDATA[2048];
unsigned char RTKData_Revc_OK = 0;
volatile uint16_t RTK_RECV_DATA_LENGTH = 0;
/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void        BSP_Uart1_WrByteUnlocked  (CPU_INT08U  c);
static  CPU_INT08U  BSP_Uart1_RdByteUnlocked  (void);

/*
*********************************************************************************************************
*                                      EXTERN FUNCTION PROTOTYPES
*********************************************************************************************************
*/
extern int uart_send_cfgCmd(unsigned char *sendData);
extern void delayMs(uint16_t ms);

/*
*********************************************************************************************************
*                                          BSP_Uar1_Init()
*
* Description : Initialize a serial port for communication.
*
* Argument(s) : baud_rate           The desire RS232 baud rate.
*
* Return(s)   : none.
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_Uart1_Init (CPU_INT32U  baud_rate)
{
  GPIO_InitTypeDef          GPIO_InitStructure;
  USART_InitTypeDef         USART_InitStructure;
  NVIC_InitTypeDef          NVIC_InitStructure;
  /*DMA_InitTypeDef           DMA_InitStructure;*/
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,  ENABLE);
  
  /* ----------------- INIT USART STRUCT ---------------- */
  #ifdef UART1_DMA_ENABLE
  	 		/* DMA1 Stream1 channel1 configuration **************************************/
	 DMA_DeInit(DMA2_Stream5);
	
	 while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE){}
		 
   DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)( &USART1 ->DR);
   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART2_RE_RTKDATA;
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
   DMA_InitStructure.DMA_BufferSize = 2048;
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
   DMA_InitStructure.DMA_Priority = DMA_Priority_High;
   DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;        
   DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
   DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
   DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
   DMA_Init(DMA2_Stream5, &DMA_InitStructure);
	
	 USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE); 
	
	 DMA_Cmd(DMA2_Stream5, ENABLE);
#endif	 
	/* Configure USART1 Tx (PA9) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	
	GPIO_Init( GPIOA, &GPIO_InitStructure );
	
	/* Configure USART1 Rx (PA10) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure );

  GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
  /* 
  UART1的配置:
  1.波特率为调用程序指定的输入 baudrate;
  2. 8位数据			  USART_WordLength_8b;
  3.一个停止位			  USART_StopBits_1;
  4. 无奇偶效验			  USART_Parity_No ;
  5.不使用硬件流控制	  USART_HardwareFlowControl_None;
  6.使能发送和接收功能	  USART_Mode_Rx | USART_Mode_Tx;
  */
  USART_InitStructure.USART_BaudRate = baud_rate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  //应用配置到UART1	
  
  /* ------------------ SETUP USART1 -------------------- */
  USART_Init(USART1, &USART_InitStructure);
  USART_Cmd(USART1, ENABLE);
  USART_ClearFlag(USART1,USART_FLAG_TC);	
  
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; /* Not used as 4 bits are used for the pre-emption priority. */;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );
	
       #ifdef UART1_DMA_ENABLE
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	 #else
	 USART_ITConfig( USART1, USART_IT_RXNE, ENABLE );
	 #endif
	//USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
}


/*
*********************************************************************************************************
*                                         BSP_Uart1_ISR_Handler()
*
* Description : Serial ISR
*
* Argument(s) : none
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  USART1_IRQHandler (void)
{
#ifndef UART1_DMA_ENABLE
    //  int   rtn;
	//CPU_INT08U data=0;
	//CPU_INT08U sendData;
 //接收溢出处理
  if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)//注意！不能使用if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)来判断
  {
    USART_ReceiveData(USART1);
  }
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
  {
		//data=USART_ReceiveData(USART1) & 0xFF; 
    //BSP_Uart1RxData[BSP_Uart1RxDataLenth++]=  data;     /* Read one byte from the receive data register.      */
    //Receive_UbxData_byte(data);
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);       /* Clear the USART1 receive interrupt.                */


  }
  if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)	
  {
    //rtn = uart_send_cfgCmd(&sendData); 
    //USART_SendData(USART1,sendData);
    /* Clear the USART1 transmit interrupt */
    USART_ClearITPendingBit(USART1, USART_IT_TXE); 	
    //if(rtn==1)
    {
      /* Disable the USART1 Transmit interrupt */
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    }  
  }
#else
  unsigned char num = 0;
  uint16_t recvLen = 0; 

    if(USART_GetITStatus(USART1,USART_IT_IDLE) == RESET)
    { 
      num = USART1 -> SR;
      num = USART1 -> DR; 
      recvLen = 2048 -  DMA_GetCurrDataCounter(DMA2_Stream5);

      RTK_RECV_DATA_LENGTH = recvLen;
      RTKData_Revc_OK = 1;

      if(recvLen > 0)
      {
        DMA_Cmd(DMA2_Stream5, DISABLE);
        DMA2_Stream5 ->NDTR = 2048;
      }

      DMA_Cmd(DMA2_Stream5, ENABLE);  
    }
#endif	   
}


/*
*********************************************************************************************************
*                                                BSP_Uart1_RdByte()
*
* Description : Receive a single byte.
*
* Argument(s) : none.
*
* Return(s)   : The received byte
*
* Caller(s)   : Application
*
* Note(s)     : (1) This functions blocks until a data is received.
*
*               (2) It can not be called from an ISR.
*********************************************************************************************************
*/

CPU_INT08U  BSP_Uart1_RdByte (void)
{
  CPU_INT08U  rx_byte;
  rx_byte = BSP_Uart1_RdByteUnlocked();
  return (rx_byte);
}


/*
*********************************************************************************************************
*                                       BSP_Uart1_RdByteUnlocked()
*
* Description : Receive a single byte.
*
* Argument(s) : none.
*
* Return(s)   : The received byte
*
* Caller(s)   : BSP_Uart1_RdByte()
*               BSP_Uart1_RdStr()
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT08U  BSP_Uart1_RdByteUnlocked (void)
{
  
  CPU_INT08U   rx_byte;
  rx_byte = BSP_Uart1RxData[BSP_Uart1RxDataFlag++];/* Read the data form the temporal register          */			
  return (rx_byte);
}



/*
*********************************************************************************************************
*                                          BSP_Uart1_WrByteUnlocked()
*
* Description : Writes a single byte to a serial port.
*
* Argument(s) : c    The character to output.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_Uart1_WrByte()
*               BSP_Uart1_WrByteUnlocked()
*
* Note(s)     : (1) This function blocks until room is available in the UART for the byte to be sent.
*********************************************************************************************************
*/

void  BSP_Uart1_WrByteUnlocked (CPU_INT08U c)
{
  CPU_INT08U i;
  i = 0;
  while((BSP_Uart1TxDataLenth+1) == BSP_Uart1TxDataFlag)
  {			
    delayMs(5);
    i++;
    if(i>3)
	break;
  }
  if(i<4)
  {
  	BSP_Uart1TxData[BSP_Uart1TxDataLenth++] = c;  
  }
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  
	//(void)BSP_Uart1TxData;
}


/*
*********************************************************************************************************
*                                                BSP_Uart1_WrByte()
*
* Description : Writes a single byte to a serial port.
*
* Argument(s) : tx_byte     The character to output.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_Uart1_WrByte(CPU_INT08U  c)
{
  
  BSP_Uart1_WrByteUnlocked(c);
  
}



/*
*********************************************************************************************************
*                                                BSP_Uart1_RdBytelock()
*
* Description : Receive a single byte.
*
* Argument(s) : none.
*
* Return(s)   : The received byte
*
* Caller(s)   : Application
*
* Note(s)     : (1) This functions blocks until Time is OUT.
*
*               (2) It can not be called from an ISR.
*********************************************************************************************************
*/

CPU_INT08U  BSP_Uart1_RdBytelock (CPU_INT32U Time,CPU_INT08U *rx_byte)
{
  CPU_INT08U   Re_i = 1;
    *rx_byte = BSP_Uart1RxData[BSP_Uart1RxDataFlag++];/* Read the data form the temporal register          */	
  return Re_i;	
}

volatile uint32_t usart1_sem = 0;// 信号量
volatile uint32_t usart1_semO = 0;// 信号量
volatile uint8_t usart1_cout = 0;//进环形缓冲区的位置指示

/*
*********************************************************************************************************
*                                                BSP_Uart1_SendCommand()
*
* Description : Send a command and wait Respons
*
* Argument(s) : Comand :Pointer to Comand
*								Length :comand length,if command end with '0xod','0xoa',it should be 0
*								Respons:Pointer to Respons
*								ReLength:Respons length
*								WaitTime:waittime for respons
*
* Return(s)   : 0:fail
*								1:access
*
* Caller(s)   : Application
*
* Note(s)     : (1) This functions blocks until a data is received.
*
*               (2) It can not be called from an ISR.
*********************************************************************************************************
*/

CPU_INT08U	 BSP_Uart1_SendCommand(const CPU_INT08U *Comand,CPU_INT16U Length,CPU_INT08U *Respons,CPU_INT16U *ReLength,CPU_INT16U WaitTime)
{
  CPU_INT16U i=0;
  CPU_INT08U  Re_i=1;
  *ReLength = 0;
  if((Length == 0)&&(Comand!=NULL))
  {
    while((Comand[Length] != 0x0D) && (Comand[Length+1] != 0x0A))
    {
			Length++;
      if(Length >= 512)
      {
        Re_i= 0;
        Length=0;
        break;
      }
    }
    Length+=2;
  }
  for(i=0;i<Length;i++)
  {
    BSP_Uart1_WrByteUnlocked(*(Comand+i));
		//BSP_Uart3_WrByte(*(Comand+i));
  }
  if((Re_i == 1) || (Comand!=NULL))
  {
    Re_i = 0;
    while(BSP_Uart1_RdBytelock(WaitTime,Respons++))
    {
      *ReLength+=1;
      WaitTime = 10; 
      Re_i = 1;
    }
  }
  return Re_i;
}

void BSP_Uart1_EnableTxE(void)
{
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
}
