/*debug*/
#define  BSP_UART3_MODULE

#include <USART3.h>

#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

#define  BSP_UART3_PRINTF_STR_BUF_SIZE             80u

/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

static CPU_INT08U   BSP_Uart3RxData[256];
static CPU_INT08U   BSP_Uart3TxData[256];
volatile  CPU_INT08U   BSP_Uart3TxDataLenth = 0;
volatile  CPU_INT08U   BSP_Uart3TxDataFlag = 0;
volatile  CPU_INT08U   BSP_Uart3RxDataLenth = 0;
volatile  CPU_INT08U   BSP_Uart3RxDataFlag = 0;

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void        BSP_Uart3_WrByteUnlocked  (CPU_INT08U  c);
static  CPU_INT08U  BSP_Uart3_RdByteUnlocked  (void);

/*
*********************************************************************************************************
*                                      EXTERN FUNCTION PROTOTYPES
*********************************************************************************************************
*/
extern int Receive_GpsData_byte(unsigned char data);
extern int uart_send_cfgCmd(unsigned char *sendData);
extern void delayMs(uint16_t ms);


/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*********************************************************************************************************
**                                         GLOBAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          BSP_Uart3_Init()
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

void  BSP_Uart3_Init (CPU_INT32U  baud_rate)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    USART_InitTypeDef       USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);

    /* ----------------- INIT USART STRUCT ---------------- */

    /* Configure USART3 Tx (PB10) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

    GPIO_Init( GPIOB, &GPIO_InitStructure );

    /* Configure USART3 Rx (PB11) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure );

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
    /*
    UART3的配置:
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
    //应用配置到UART3

    /* ------------------ SETUP USART3 -------------------- */
    USART_Init(USART3, &USART_InitStructure);
    USART_Cmd(USART3, ENABLE);
    USART_ClearFlag(USART3, USART_FLAG_TC);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY-1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; /* Not used as 4 bits are used for the pre-emption priority. */;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    USART_ITConfig( USART3, USART_IT_RXNE, ENABLE );
}


/*
*********************************************************************************************************
*                                         BSP_Uart3_ISR_Handler()
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

void  USART3_IRQHandler (void)
{
    int  rtn;
    CPU_INT08U data = 0;
	CPU_INT08U sendData;
	//int syncFlag;
	//  BaseType_t semaphoreStatus = pdFALSE;
    if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET) //note: it does not work by if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART3);
    }
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        data = USART_ReceiveData(USART3) & 0xFF;
        Receive_GpsData_byte(data);
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);         /* Clear the USART3 receive interrupt.                */
        //xSemaphoreGiveFromISR( BSP_Uart3RxWait, &semaphoreStatus); /* Post to the semaphore 		*/
    }
    if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
    {
        //USART_SendData(USART3, BSP_Uart3TxData[BSP_Uart3TxDataFlag++]);
        (void)BSP_Uart3TxData;
        rtn = uart_send_cfgCmd(&sendData); 
        USART_SendData(USART3,sendData);
        /* Clear the USART3 transmit interrupt */
        USART_ClearITPendingBit(USART3, USART_IT_TXE);
        //if(BSP_Uart3TxDataLenth == BSP_Uart3TxDataFlag)
         if(rtn==1)
        {
            /* Disable the USART3 Transmit interrupt */
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
        }
    }
}


/*
*********************************************************************************************************
*                                                BSP_Uart3_RdByte()
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

CPU_INT08U  BSP_Uart3_RdByte (void)
{
    CPU_INT08U  rx_byte;


    rx_byte = BSP_Uart3_RdByteUnlocked();

    return (rx_byte);
}


/*
*********************************************************************************************************
*                                       BSP_Uart3_RdByteUnlocked()
*
* Description : Receive a single byte.
*
* Argument(s) : none.
*
* Return(s)   : The received byte
*
* Caller(s)   : BSP_Uart3_RdByte()
*               BSP_Uart3_RdStr()
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT08U  BSP_Uart3_RdByteUnlocked (void)
{

    CPU_INT08U   rx_byte;
    rx_byte = BSP_Uart3RxData[BSP_Uart3RxDataFlag++];/* Read the data form the temporal register          */
    return (rx_byte);
}
/*
*********************************************************************************************************
*                                          BSP_Uart3_WrByteUnlocked()
*
* Description : Writes a single byte to a serial port.
*
* Argument(s) : c    The character to output.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_Uart3_WrByte()
*               BSP_Uart3_WrByteUnlocked()
*
* Note(s)     : (1) This function blocks until room is available in the UART for the byte to be sent.
*********************************************************************************************************
*/

void  BSP_Uart3_WrByteUnlocked (CPU_INT08U c)
{
    CPU_INT08U i;
    i = 0;
    while((BSP_Uart3TxDataLenth + 1) == BSP_Uart3TxDataFlag)
    {
        delayMs(5);
	  i++;
    if(i>3)
	break;
    }
    if(i<4)
    {
    	BSP_Uart3TxData[BSP_Uart3TxDataLenth++] = c;
    }
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}


/*
*********************************************************************************************************
*                                                BSP_Uart3_WrByte()
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

void  BSP_Uart3_WrByte(CPU_INT08U  c)
{
    BSP_Uart3_WrByteUnlocked(c);

}



/*
*********************************************************************************************************
*                                                BSP_Uart3_RdBytelock()
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

CPU_INT08U  BSP_Uart3_RdBytelock (CPU_INT32U Time, CPU_INT08U *rx_byte)
{
    CPU_INT08U   Re_i = 1;
        *rx_byte = BSP_Uart3RxData[BSP_Uart3RxDataFlag++];/* Read the data form the temporal register          */
    return Re_i;
}


/*
*********************************************************************************************************
*                                                BSP_Uart2_SendCommand()
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

CPU_INT08U	 BSP_Uart3_SendCommand(const CPU_INT08U *Comand, CPU_INT16U Length, CPU_INT08U *Respons, CPU_INT16U *ReLength, CPU_INT16U WaitTime)
{
    CPU_INT16U i = 0;
    CPU_INT08U  Re_i = 1;
    *ReLength = 0;
    if((Length == 0) && (Comand != NULL))
    {
        while((Comand[Length] != 0x0D) && (Comand[Length + 1] != 0x0A))
        {
            Length++;
            if(Length >= 512)
            {
                Re_i = 0;
                Length = 0;
                break;
            }
        }
        Length++;
    }
    for(i = 0; i < Length; i++)
    {
        BSP_Uart3_WrByteUnlocked(*(Comand + i));
//		BSP_UART3_WrByte(*(Comand+i));
    }
    if((Re_i == 1) || (Comand != NULL))
    {
        Re_i = 0;
        while(BSP_Uart3_RdBytelock(WaitTime, Respons++))
        {
            *ReLength += 1;
            WaitTime = 10;
            Re_i = 1;
        }
    }
    return Re_i;
}
void BSP_Uart3_EnableTxE(void)
{
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE); 
}
//Send one byte
int BSP_Uart3_SendByte(CPU_INT08U  data)
{
	CPU_INT16U rtn,u16Loop;
	//check TXE status
	u16Loop = 0;
	rtn = FALSE;
	do
	{
		if (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == SET)
		{			
			rtn = TRUE;
			break;
		}
		u16Loop++;
	}while(u16Loop<10000);
	if(rtn == TRUE)
	{
		USART_SendData(USART3,data);
		return(TRUE);
	}
	else
	{
		return(FALSE);
	}
}
//Send buff
int BSP_Uart3_SendBuff(CPU_INT16U  len,CPU_INT08U *wrPt)
{
	CPU_INT16U  u16Loop,rtn;
	rtn = 0;
	for(u16Loop=0;u16Loop<len;u16Loop++)
	{
		if(BSP_Uart3_SendByte(*(wrPt+u16Loop))==FALSE)
		{
			rtn++;
		}
	}
	if(rtn==0)
	{
		return(TRUE);
	}
	else
	{
		return(FALSE);
	}
}
//Send string


