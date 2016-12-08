#ifndef __SPI_H
#define __SPI_H

#include "stm32f4xx.h"

struct sensor_cs_desc {
    uint16_t pin;
    GPIO_TypeDef *port;
    char *name;	
};

static struct sensor_cs_desc sensor_cs_seclect [] = {
    {GPIO_Pin_6, GPIOC ,  "LSM6DS33"},
    {GPIO_Pin_7, GPIOC ,  "BMI160"},
    {GPIO_Pin_15, GPIOD ,  "MPU6500"},
    {GPIO_Pin_14, GPIOD ,  "AK8975"},
};

#define LSM6DS3     0
#define BMI160     1
#define MPU6500     2
#define AK8975     3

void spi1_Configuration(void);
uint8_t SPI1_ReadWrite_Byte(uint8_t byte);
uint8_t SpiReadBmi160DevID(void);
uint8_t SpiReadAk8975DevID(void);
uint8_t SpiReadMpu6500DevID(void);
uint8_t SpiReadLsm6ds33DevID(void);
uint8_t SpiReadReg(uint8_t cs_sec,uint8_t reg);
uint8_t SpiReadMultData(uint8_t cs_sec,uint8_t reg,uint8_t length, uint8_t *data);
uint8_t SpiWriteData(uint8_t cs_sec,uint8_t reg,uint8_t data);

#define SECSOR_CS_LOW(x) GPIO_ResetBits(sensor_cs_seclect[x].port, sensor_cs_seclect[x].pin)
#define SECSOR_CS_HIGH(x) GPIO_SetBits(sensor_cs_seclect[x].port, sensor_cs_seclect[x].pin)

#endif
