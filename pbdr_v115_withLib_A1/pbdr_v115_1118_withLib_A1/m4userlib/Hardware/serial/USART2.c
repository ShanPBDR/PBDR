#define  BSP_UART2_MODULE
#include <USART2.h>
/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

#define  BSP_UART2_PRINTF_STR_BUF_SIZE             80u

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
#define UART2_TXBUFF_DEEP   1000
static CPU_INT08U   BSP_Uart2RxData[256];
static CPU_INT08U   BSP_Uart2TxData[UART2_TXBUFF_DEEP];
volatile  CPU_INT16U   BSP_Uart2TxDataLenth=0;
volatile  CPU_INT16U   BSP_Uart2TxDataFlag=0;
volatile  CPU_INT08U   BSP_Uart2RxDataLenth=0;
volatile  CPU_INT08U   BSP_Uart2RxDataFlag=0;

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void        BSP_Uart2_WrByteUnlocked  (CPU_INT08U  c);
static  CPU_INT08U  BSP_Uart2_RdByteUnlocked  (void);
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
*                                          BSP_Uart2_Init()
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

void  BSP_Uart2_Init (CPU_INT32U  baud_rate)
{
  GPIO_InitTypeDef        GPIO_InitStructure;
  USART_InitTypeDef       USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
  /* ----------------- INIT USART STRUCT ---------------- */
  
	/* Configure USART2 Tx (PA2) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	
	GPIO_Init( GPIOD, &GPIO_InitStructure );
	
	/* Configure USART2 Rx (PA3) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOD, &GPIO_InitStructure );

  GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2);
  /* 
  UART2������:
  1.������Ϊ���ó���ָ�������� baudrate;
  2. 8λ����			  USART_WordLength_8b;
  3.һ��ֹͣλ			  USART_StopBits_1;
  4. ����żЧ��			  USART_Parity_No ;
  5.��ʹ��Ӳ��������	  USART_HardwareFlowControl_None;
  6.ʹ�ܷ��ͺͽ��չ���	  USART_Mode_Rx | USART_Mode_Tx;
  */
  USART_InitStructure.USART_BaudRate = baud_rate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  //Ӧ�����õ�UART2	
  
  /* ------------------ SETUP USART2 -------------------- */
  USART_Init(USART2, &USART_InitStructure);
  USART_Cmd(USART2, ENABLE);
  USART_ClearFlag(USART2,USART_FLAG_TC);	
  
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; /* Not used as 4 bits are used for the pre-emption priority. */;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );
	
	USART_ITConfig( USART2, USART_IT_RXNE, ENABLE );
}


/*
*********************************************************************************************************
*                                         BSP_Uart2_ISR_Handler()
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
volatile uint32_t usart2_sem = 0;// �ź���
volatile uint32_t usart2_semO = 0;// �ź���
volatile uint8_t usart2_cin = 0;//�����λ�������λ��ָʾ
volatile uint8_t usart2_cout = 0;//�����λ�������λ��ָʾ

volatile int32_t usart2_semzt1 = 0;// �ź���
volatile int32_t usart2_semzt2 = 0;// �ź���

void  USART2_IRQHandler (void)
{
	CPU_INT08U data=0;
  if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)//ע�⣡����ʹ��if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)���ж�
  {
    USART_ReceiveData(USART2);
  }
  if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
		data=USART_ReceiveData(USART2) & 0xFF; 
    BSP_Uart2RxData[usart2_cin++]=  data;     /* Read one byte from the receive data register.      */
		usart2_sem++;//�ۼ��ź���		
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);         /* Clear the USART2 receive interrupt.                */
		//BSP_Uart3_WrByte(data);  //for wifi
  }
  if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET)	{
    USART_SendData(USART2, BSP_Uart2TxData[BSP_Uart2TxDataFlag]); 
    BSP_Uart2TxDataFlag = (BSP_Uart2TxDataFlag+1)%UART2_TXBUFF_DEEP;
    /* Clear the USART2 transmit interrupt */
    USART_ClearITPendingBit(USART2, USART_IT_TXE); 	
    if(BSP_Uart2TxDataLenth == BSP_Uart2TxDataFlag)
    {
      /* Disable the USART2 Transmit interrupt */
      USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }  
  }
}


/*
*********************************************************************************************************
*                                                BSP_Uart2_RdByte()
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

CPU_INT08U  BSP_Uart2_RdByte (void)
{
  CPU_INT08U  rx_byte;
  
  rx_byte = BSP_Uart2_RdByteUnlocked();
  
  
  return (rx_byte);
}


uint32_t BSP_Uart2_RdBytes(uint8_t *rxbuf, uint32_t size)  //read from USART2 buffer
{
	uint32_t i;
	uint32_t bufsize;
  i = 0;
	if(size <= 0)
		return 1;
	if(usart2_semO <= usart2_sem)
		bufsize = usart2_sem - usart2_semO;
	else
		bufsize = 0xFFFFFFFF - usart2_semO + usart2_sem + 1;
	if(bufsize < size)
		return i;
  while (1)
  {
    *rxbuf++ = BSP_Uart2RxData[usart2_cout++];// �ӽ��ջ�������ȡ����
		usart2_semO ++;
    i++;
	if(i == size)
		break;
  }
  return i;
}

/*
*********************************************************************************************************
*                                       BSP_Uart2_RdByteUnlocked()
*
* Description : Receive a single byte.
*
* Argument(s) : none.
*
* Return(s)   : The received byte
*
* Caller(s)   : BSP_Uart2_RdByte()
*               BSP_Uart2_RdStr()
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT08U  BSP_Uart2_RdByteUnlocked (void)
{
  
  CPU_INT08U   rx_byte;
  rx_byte = BSP_Uart2RxData[BSP_Uart2RxDataFlag++];/* Read the data form the temporal register          */			
  return (rx_byte);
}



/*
*********************************************************************************************************
*                                          BSP_Uart2_WrByteUnlocked()
*
* Description : Writes a single byte to a serial port.
*
* Argument(s) : c    The character to output.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_Uart2_WrByte()
*               BSP_Uart2_WrByteUnlocked()
*
* Note(s)     : (1) This function blocks until room is available in the UART for the byte to be sent.
*********************************************************************************************************
*/

void  BSP_Uart2_WrByteUnlocked (CPU_INT08U c)
{
  CPU_INT08U i;
  i = 0;
  while((BSP_Uart2TxDataLenth+1) == BSP_Uart2TxDataFlag)
  {			
    delayMs(5);
	i++;
    if(i>3)
	break;
  }
  if(i<4)
  {
  	BSP_Uart2TxData[BSP_Uart2TxDataLenth] = c;  
  	BSP_Uart2TxDataLenth = (BSP_Uart2TxDataLenth+1)%UART2_TXBUFF_DEEP;
  }
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);  
}


/*
*********************************************************************************************************
*                                                BSP_Uart2_WrByte()
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

void  BSP_Uart2_WrByte(CPU_INT08U  c)
{
  
  BSP_Uart2_WrByteUnlocked(c);
}



/*
*********************************************************************************************************
*                                                BSP_Uart2_RdBytelock()
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

CPU_INT08U  BSP_Uart2_RdBytelock (CPU_INT32U Time,CPU_INT08U *rx_byte)
{
  CPU_INT08U   Re_i = 1;
    *rx_byte = BSP_Uart2RxData[BSP_Uart2RxDataFlag++];/* Read the data form the temporal register          */	
//   BSP_UART3_WrByte(*rx_byte);  //debug
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

CPU_INT08U	 BSP_Uart2_SendCommand(const CPU_INT08U *Comand,CPU_INT16U Length,CPU_INT08U *Respons,CPU_INT16U *ReLength,CPU_INT16U WaitTime)
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
    Length++;
  }
  for(i=0;i<Length;i++)
  {
    BSP_Uart2_WrByteUnlocked(*(Comand+i));
//		BSP_UART3_WrByte(*(Comand+i));
  }
  if((Re_i == 1) || (Comand!=NULL))
  {
    Re_i = 0;
    while(BSP_Uart2_RdBytelock(WaitTime,Respons++))
    {
      *ReLength+=1;
      WaitTime = 10; 
      Re_i = 1;
    }
  }
  return Re_i;
}

void BSP_Uart2_WrStr(char *data,int length)
{
   int send_i;
   for(send_i=0;send_i < length;send_i ++)
   {
     BSP_Uart2_WrByteUnlocked(*data++);
   }
}
