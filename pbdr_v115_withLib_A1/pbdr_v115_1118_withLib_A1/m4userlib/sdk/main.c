
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "hardware.h"
#include "print.h"
#include "pbdrctrlmgr.h"

/** @addtogroup Template_Project
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LED1             GPIO_Pin_6
#define LED2             GPIO_Pin_7
#define LED_PORT        GPIOB
#define LED_GPIO_CLOCK  RCC_AHB1Periph_GPIOB
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

/* Private function prototypes -----------------------------------------------*/
static void Delay(__IO uint32_t nTime);
static void prvSetupHardware( void );
void LED_Configuration(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       files (startup_stm32f40_41xxx.s/startup_stm32f427_437xx.s/
       startup_stm32f429_439xx.s/startup_stm32f401xx.s or startup_stm32f411xe.s)
       before to branch to application main.
       To reconfigure the default setting of SystemInit() function, 
       refer to system_stm32f4xx.c file */
  
  SystemInit();
  prvSetupHardware();      
  TIM5_Int_Init(0xffff,84-1);// 1MHZ cnt
  
  delayMs(2000);  
  LED_Configuration();     /* LED init  */ 
    spi1_Configuration();    /* spi1 init */
    
    sensor_BMI160_init();
    BSP_Uart3_Init(USART3baud_rate);   
	
    DataHandler_init();
  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);

  /* Add your application code here */
  /* Insert 50 ms delay */
  
  //BSP_Uart1_Init(USART1baud_rate);    
  BSP_Uart2_Init(USART2baud_rate);        
     
  TIM3_Int_Init(10000,84-1);// 10ms
  TIM4_Init();
  print("\r\n Start - pbdr BM app !!!!!!!\r\n");
  Delay(5);    
  /* Infinite loop */
  while (1)
  {
  	DataHandler_main();
  }
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  uwTimingDelay = nTime;

  while(uwTimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/** 
  * @ brief   GPIO init, YY comment: have we checked all setting here ; NVIC、GPIO INIT
  * @param Nne
  * @ retval None
  */
  static void prvSetupHardware( void )
{
    /* GPIO, EXTI and NVIC Init structure declaration */
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Essential on STM32 Cortex-M devices. */
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

    /* Systick is fed from HCLK/8. */
    SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK_Div8 );

    /* Enable SYSCFG clocks. */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_SYSCFG , ENABLE );
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    /*PE14: blue pio4 ; PE13: blue pio5 ; PE15: blue pio6****/
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //GPIO_Init( GPIOE, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;  //for gps reset
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init( GPIOE, &GPIO_InitStructure );   
    GPIO_SetBits(GPIOE, GPIO_Pin_1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;  //for gps SAFEBOOT_N
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init( GPIOE, &GPIO_InitStructure );  
    GPIO_SetBits(GPIOE, GPIO_Pin_0);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;  //for gps GEOFENCE_STAT
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //GPIO_Init( GPIOB, &GPIO_InitStructure );  
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //for gps POWER ON
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init( GPIOE, &GPIO_InitStructure ); 
    GPIO_SetBits(GPIOE, GPIO_Pin_12);
    
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //for WIFI_RELOAD
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //GPIO_Init( GPIOE, &GPIO_InitStructure ); 
    //GPIO_SetBits(GPIOE, GPIO_Pin_9);
    
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //for WIFI RESET
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //GPIO_Init( GPIOE, &GPIO_InitStructure ); 
    //GPIO_SetBits(GPIOE, GPIO_Pin_10);
    
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; //for WIFI POWER
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //GPIO_Init( GPIOE, &GPIO_InitStructure ); 
    //GPIO_SetBits(GPIOE, GPIO_Pin_11);

    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //for 2G POWER EN
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //GPIO_Init( GPIOE, &GPIO_InitStructure ); 
    //GPIO_ResetBits(GPIOE, GPIO_Pin_8);

    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //for 2G GPRS_LIGHT
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //GPIO_Init( GPIOB, &GPIO_InitStructure ); 
    //GPIO_ResetBits(GPIOB, GPIO_Pin_0);

    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //for 2G POWERKEY
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //GPIO_Init( GPIOB, &GPIO_InitStructure ); 
    //GPIO_ResetBits(GPIOB, GPIO_Pin_1);

    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //for 2G GPRS_STA
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //GPIO_Init( GPIOA, &GPIO_InitStructure ); 
    //GPIO_ResetBits(GPIOA, GPIO_Pin_4);

    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //for 2G RF_SYNC
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //GPIO_Init( GPIOE, &GPIO_InitStructure ); 
    //GPIO_ResetBits(GPIOE, GPIO_Pin_7);


    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; //for BEEP
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //GPIO_Init( GPIOD, &GPIO_InitStructure ); 
    //GPIO_ResetBits(GPIOD, GPIO_Pin_11);

    
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //BATERY_CHK_CTR
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    //GPIO_Init( GPIOC, &GPIO_InitStructure );

}


/*******************************************************************************
  * name: 	LED_Configuration
  * description:   configure LED
  * input:   NULL
  * output:  NULL
  * return:  NULL
*******************************************************************************/
void LED_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // enable timer on LED GPIO port
    RCC_AHB1PeriphClockCmd(LED_GPIO_CLOCK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = LED1 | LED2 ; //configure LED1-4 pin
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//bandwidth 50Mhz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
    GPIO_InitStructure.GPIO_PuPd =  GPIO_PuPd_UP;  //上拉电阻
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  //输出模式
    //initialize GPIOB
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
    GPIO_SetBits(LED_PORT, LED1);
    GPIO_ResetBits(LED_PORT, LED2);
}

void LED_Toggle(u8 ledNum)
{
    if(ledNum==LED1)
    	GPIO_ToggleBits(LED_PORT, LED1);
    if(ledNum==LED1)
    GPIO_ToggleBits(LED_PORT, LED2);
}

void Running_LED_int(void)
{
	static u16 runningCnt=0;
	runningCnt += 10;
	if(runningCnt > 1000)
	{
		LED_Toggle(LED1);
		runningCnt = 0;
	}
}


