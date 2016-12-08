#include "BSP.h"
#include "SPI.h"
#include "Timer.h"

void spi1_Configuration(void)
{
  int i;
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  
  /* SCK, MISO and MOSI  PA5=CLK,PA6=MISO,PA7=MOSI*/
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /*  CS Init START  */
  for( i = 0 ; i < 4 ; i++)
  {
    GPIO_InitStructure.GPIO_Pin = sensor_cs_seclect[i].pin;    
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(sensor_cs_seclect[i].port , &GPIO_InitStructure);
    GPIO_SetBits(sensor_cs_seclect[i].port , sensor_cs_seclect[i].pin);//
  }  
  
  /*  CS Init STOP  */
  SPI_I2S_DeInit(SPI1);//XIJ ADD
  SPI_Cmd(SPI1, DISABLE); 
  /* SPI1 configuration  */     
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //¨¢???¨¨???1¡è
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;       //?¡Â
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;      //8??
  //SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low/*SPI_CPOL_Low*/;        //CPOL=0 ¨º¡À?¨®D¨¹??¦Ì¨ª
  //SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;       //CPHA=0 ¨ºy?Y2???¦Ì¨²1??
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High/*SPI_CPOL_Low*/;        //CPOL=0 ¨º¡À?¨®D¨¹??¦Ì¨ª
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;       //CPHA=0 ¨ºy?Y2???¦Ì¨²1??
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;        //¨¨¨ª?tNSS
  //SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//  //2¡¤??¦Ì
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//  //2¡¤??¦Ì
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;      //?????¨²?¡ã
  SPI_InitStructure.SPI_CRCPolynomial = 7;        //CRC7
  
  SPI_Init(SPI1, &SPI_InitStructure);	 //¨®|¨®?????¦Ì? SPI1
  SPI_Cmd(SPI1, ENABLE); 
}

uint8_t SPI1_ReadWrite_Byte(uint8_t byte)
{
  uint8_t  i_u8,j_u8;
  /**/
  i_u8 = 0;
  do{
  	i_u8++;
	if(i_u8>100)
		break;
  } while((SPI1->SR & SPI_I2S_FLAG_TXE)==RESET);
  /**/
  SPI1->DR = byte;
  /**/
  j_u8 = 0;
  do{
  	j_u8++;
	if(j_u8>100)
		break;
  } while((SPI1->SR & SPI_I2S_FLAG_RXNE)==RESET);
  return(SPI1->DR);
}

uint8_t SpiReadBmi160DevID(void)
{
	uint8_t Data;
	SECSOR_CS_LOW(BMI160);
	delayMs(1);
	SPI1_ReadWrite_Byte(0x80);///
	Data = SPI1_ReadWrite_Byte(0xFF);
	SECSOR_CS_HIGH(BMI160);
	delayMs(1);
	return(Data);
}

uint8_t SpiReadAk8975DevID(void)
{
	uint8_t Data;
	SECSOR_CS_LOW(AK8975);
	delayMs(1);
	SPI1_ReadWrite_Byte(0x80);///
	Data = SPI1_ReadWrite_Byte(0xFF);
	SECSOR_CS_HIGH(AK8975);
	delayMs(1);
	return(Data);
}

uint8_t SpiReadMpu6500DevID(void)
{
	uint8_t Data;
	SECSOR_CS_LOW(MPU6500);
	delayMs(1);
	SPI1_ReadWrite_Byte(0x80 | 0x75);///
	Data = SPI1_ReadWrite_Byte(0xFF);
	SECSOR_CS_HIGH(MPU6500);
	delayMs(1);
	return(Data);
}

uint8_t SpiReadLsm6ds33DevID(void)
{
	uint8_t Data;
	SECSOR_CS_LOW(LSM6DS3);
	delayMs(1);
	SPI1_ReadWrite_Byte(0xf | 0x80);///
	Data = SPI1_ReadWrite_Byte(0xFF);
	SECSOR_CS_HIGH(LSM6DS3);
	delayMs(1);
	return(Data);
}



uint8_t SpiReadReg(uint8_t cs_sec,uint8_t reg)
{
	uint8_t data = 0;
	SECSOR_CS_LOW(cs_sec);
	delayMs(1);
	SPI1_ReadWrite_Byte(reg| 0x80);
	data = SPI1_ReadWrite_Byte(0xFF);
	SECSOR_CS_HIGH(cs_sec);
	delayMs(1);
	return data;
}


uint8_t SpiReadMultData(uint8_t cs_sec,uint8_t reg,uint8_t length, uint8_t *data)
{
	uint8_t count = 0;
	SECSOR_CS_LOW(cs_sec);
	delayMs(1);
	SPI1_ReadWrite_Byte(reg| 0x80);
	for(count=0;count<length;count++){
	    data[count] = SPI1_ReadWrite_Byte(0xFF);
	}
	SECSOR_CS_HIGH(cs_sec);
	delayMs(1);
	return 0;
}

uint8_t SpiWriteData(uint8_t cs_sec,uint8_t reg,uint8_t data)
{
	uint8_t status = 0;
	SECSOR_CS_LOW(cs_sec);
	delayMs(1);
	SPI1_ReadWrite_Byte(reg);
	SPI1_ReadWrite_Byte(data);
	SECSOR_CS_HIGH(cs_sec);
	delayMs(1);
	return status ;
}

