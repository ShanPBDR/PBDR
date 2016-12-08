#include "stm32f4xx.h"

extern void getDataFromBMI160(void);  
extern void DataHandler_int(void);
extern void Running_LED_int(void);

void TIM3_Int_Init(u16 arr,u16 psc) 
{ 
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; 
  NVIC_InitTypeDef NVIC_InitStructure; 
  
  TIM_DeInit(TIM3);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟 
  
  TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频 
  TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式 
  TIM_TimeBaseInitStructure.TIM_Period=arr;   //自动重装载值 
  TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;  
  TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure); 
   
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断 
 
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);   // 抢占式优先级别 
  NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢占优先级1 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //指定响应子优先级0
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE; 
  NVIC_Init(&NVIC_InitStructure); 
   
  TIM_Cmd(TIM3,ENABLE);
} 

void TIM3_IRQHandler()
{
     
    //print("TIM3_IRQHandler - HongqiTech.inc ! \n");
    if(TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);
   
        getDataFromBMI160();  
        
	  DataHandler_int();
	  Running_LED_int();
	  

    }
}
void TIM4_Init(void) 
{ 
  GPIO_InitTypeDef        GPIO_InitStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure; 
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; 
  //NVIC_InitTypeDef NVIC_InitStructure; 
  
  TIM_DeInit(TIM4);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///使能TIM3时钟 
  /* GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  
  
  //PB9 init input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init( GPIOB, &GPIO_InitStructure );  
	/* Connect TIM pins to AF2 */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);
    
  /* 
  TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //允许定时器4更新中断 
 
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);   // 抢占式优先级别 
  NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //定时器3中断 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //指定响应子优先级3 
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE; 
  NVIC_Init(&NVIC_InitStructure); 
  */
  TIM_TimeBaseInitStructure.TIM_Prescaler=42000-1;  //定时器分频 
  TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式 
  TIM_TimeBaseInitStructure.TIM_Period=0xffff;   //自动重装载值 
  TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;  
  TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure); 
  
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_ICInit(TIM4, &TIM_ICInitStructure);

  TIM_Cmd(TIM4,ENABLE);
} 

//u16 g16TIM4CaptureBuff[50];
//u8 g8CapBuffWrPt=0;
u16 g16TIM4OverCapCnt=0;
int TIM4_capture(u16* captureTimePt)
{
  int rtn;
  rtn = 0;
  if(TIM_GetFlagStatus(TIM4 , TIM_FLAG_CC4) != RESET)
  {
    TIM_ClearFlag(TIM4 , TIM_FLAG_CC4);
    *captureTimePt = TIM_GetCapture4(TIM4);

    rtn = 1;
  }
  if(TIM_GetFlagStatus(TIM4 , TIM_FLAG_CC4OF) != RESET)
  {
    TIM_ClearFlag(TIM4 , TIM_FLAG_CC4OF);
    g16TIM4OverCapCnt++;
  }
  return(rtn);
}
void TIM5_Int_Init(u16 arr,u16 psc) 
{ 
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; 
  //NVIC_InitTypeDef NVIC_InitStructure; 
  
  TIM_DeInit(TIM5);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  ///使能TIM3时钟 
  
  TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频 
  TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式 
  TIM_TimeBaseInitStructure.TIM_Period=arr;   //自动重装载值 
  TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;  
  TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure); 
   
  //TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); //允许定时器3更新中断 
  TIM_Cmd(TIM5,ENABLE); //使能定时器5
 /*
  NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn; //定时器4中断 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //指定响应子优先级3 
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE; 
  NVIC_Init(&NVIC_InitStructure); 
  */

} 

void TIM5_IRQHandler()
{
    //print("TIM5_IRQHandler - HongqiTech.inc ! \n");
    if(TIM_GetITStatus(TIM5 , TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM5 , TIM_FLAG_Update);
//
        if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_12) != 0)
        {
            GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        }
        else
        {
            GPIO_SetBits(GPIOB, GPIO_Pin_12);
        }
//
    }
}

void delayUs(uint16_t u16us)
{
    u16 u16startCnt,u16endCnt;
    u16 u16delayCnt;
    u8 u8flag;
    u32 u32loop;
    u16 u16loop;
    u16delayCnt = u16us;   //Tim5 1Mhz up count
    u16startCnt = TIM_GetCounter(TIM5);
    if((0xffff-u16startCnt)>u16delayCnt)
    {
        u8flag = 0;
        u16endCnt = u16startCnt+u16delayCnt;
    }
    else
    {
        u8flag = 1;
        u16endCnt = u16delayCnt-(0xffff-u16startCnt);
    }
    u32loop = 0;
    u16loop = 0;
    do
    {
        u16delayCnt = TIM_GetCounter(TIM5);
        if(1==u8flag)
        {
            if((u16delayCnt>u16endCnt)&&(u16delayCnt<u16startCnt))
            {
                return;
            }
        }
        else
        {
            if((u16delayCnt>u16endCnt)||(u16delayCnt<u16startCnt))
            {
                return;
            }
        }
        u32loop++;
        if(u32loop>0xfffffff0)
        {
            u32loop = 0;
            u16loop++;
        }
    }while(u16loop<0xfff0);
}

void delayMs(uint16_t u16ms)
{
    u16 u16msCnt;
    u16 u16loop;
    u16msCnt = u16ms;   
   for(u16loop=0;u16loop<u16msCnt;u16loop++)
   {
   	delayUs(1000);
   }
}


