/*
****************************************************************************
* Copyright (C) 2014 Bosch Sensortec GmbH
*
* bmi160_support.c
* Date: 2014/10/27
* Revision: 1.0.6 $
*
* Usage: Sensor Driver support file for BMI160 sensor
*
****************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/
//#include "semphr.h"
#include "stm32f4xx.h"

#include "bmi160_support.h"
#include "bmi160.h"

#include <stdio.h>
#include <string.h>
#include "print.h"
#include "Timer.h"



/* Mapping the structure*/
struct bmi160_t s_bmi160;
/* Read the sensor data of accel, gyro and mag*/
struct bmi160_gyro_t gyroxyz;
struct bmi160_accel_t accelxyz;
struct bmi160_mag_xyz_s32_t magxyz;

s16 bmi160_temperature;

/** @addtogroup BMI160_Private_Variables BMI160_Private_Variables
 * @{
 */
 

/** @addtogroup BMI160_Private_Functions BMI160_Private_Functions
 * @{
 */



/*!
 *	@brief This function used for set the gyro mode
 *
 *   
 *   @param mode : The seclected target sensor mode
 *  value   |  mode
 * -------|-----------
 *   GYRO_MODE_SUSPEND      |  SUSPEND
 *   GYRO_MODE_NORMAL      |  NORMAL
 *   GYRO_MODE_FASTSTARTUP      |  FASTS TARTUP
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_mode(u8 mode)
{
    BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_INIT_VALUE ;
    
        com_rslt = spi_routine(); 
        com_rslt += bmi160_sec(&s_bmi160);
		/*Set the gyro mode  write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(mode);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
    
    return com_rslt;
}

/*!
 *	@brief This function used for set the accel mode
 *
 *   
 *   @param mode : The seclected target sensor accel mode
 *  value   |  mode
 * -------|-----------
 *   ACCEL_MODE_NORMAL      |  NORMAL
 *   ACCEL_LOWPOWER      |  FASTS TARTUP
 *   ACCEL_SUSPEND      |  SUSPEND
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 */

BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_mode(u8 mode)
{
    BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_INIT_VALUE ;
    
        //com_rslt = spi_routine(); 
        //s_bmi160.dev_addr = seclect;
        com_rslt += bmi160_sec(&s_bmi160);
		com_rslt += bmi160_set_command_register(mode);
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
        //com_rslt += bmi160_read_accel_xyz(&accelxyz);
    
    return com_rslt;
}

/*!
 *	@brief This function used for set the gyro rate
 *
 *   @param rate : The seclected target sensor accel mode
 *  value   |  rate
 * -------|-----------
 *
#define BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED		(0x00)
#define BMI160_GYRO_OUTPUT_DATA_RATE_25HZ			(0x06)
#define BMI160_GYRO_OUTPUT_DATA_RATE_50HZ			(0x07)
#define BMI160_GYRO_OUTPUT_DATA_RATE_100HZ			(0x08)
#define BMI160_GYRO_OUTPUT_DATA_RATE_200HZ			(0x09)
#define BMI160_GYRO_OUTPUT_DATA_RATE_400HZ			(0x0A)
#define BMI160_GYRO_OUTPUT_DATA_RATE_800HZ			(0x0B)
#define BMI160_GYRO_OUTPUT_DATA_RATE_1600HZ		(0x0C)
#define BMI160_GYRO_OUTPUT_DATA_RATE_3200HZ		(0x0D)
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_rate(u8 rate)
{
    BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_INIT_VALUE ;
    
        //com_rslt = spi_routine(); 
        //s_bmi160.dev_addr = seclect;
        com_rslt += bmi160_sec(&s_bmi160);
        /* set gyro data rate */
        com_rslt += bmi160_set_gyro_output_data_rate(
            rate);
        s_bmi160.delay_msec(
        BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/

    return com_rslt;
}

/*!
 *	@brief This function used for set the accel rate
 *
 *   @param rate : The seclected target sensor accel mode
 *  value   |  rate
 * -------|-----------
 *
#define BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED       (0x00)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ         (0x01)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_1_56HZ         (0x02)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_3_12HZ         (0x03)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_6_25HZ         (0x04)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_12_5HZ         (0x05)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ           (0x06)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_50HZ           (0x07)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ          (0x08)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ          (0x09)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ          (0x0A)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_800HZ          (0x0B)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ         (0x0C)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED0      (0x0D)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED1      (0x0E)
#define BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED2      (0x0F)

 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_rate(u8 rate)
{
    BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_INIT_VALUE ;
    
        //com_rslt = spi_routine(); 
        //s_bmi160.dev_addr = seclect;
        com_rslt += bmi160_sec(&s_bmi160);
        /* set accel data rate */
        com_rslt += bmi160_set_accel_output_data_rate(
            rate,
            BMI160_ACCEL_OSR4_AVG1);
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/				
    
    return com_rslt;
}

/*!
 *	@brief This function used for set the gyro bandwidth
 *
 *   @param rate : The seclected target sensor accel mode
 *  value   |  rate
 * -------|-----------
 *
#define BMI160_GYRO_OSR4_MODE		(0x00)
#define BMI160_GYRO_OSR2_MODE		(0x01)
#define BMI160_GYRO_NORMAL_MODE		(0x02)
#define BMI160_GYRO_CIC_MODE		(0x03)

 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_bandwidth(u8 bw)
{
    BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_INIT_VALUE ;
    
        //com_rslt = spi_routine(); 
        //s_bmi160.dev_addr = seclect;
        com_rslt += bmi160_sec(&s_bmi160);
        /* Set the gryo bandwidth as Normal */
        com_rslt += bmi160_set_gyro_bw(bw);
        s_bmi160.delay_msec(
        BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
    
    return com_rslt;
}

/*!
 *	@brief This function used for set the accel bandwidth
 *
 *   @param rate : The seclected target sensor accel mode
 *  value   |  rate
 * -------|-----------
 *
#define BMI160_ACCEL_OSR4_AVG1			(0)
#define BMI160_ACCEL_OSR2_AVG2			(1)
#define BMI160_ACCEL_NORMAL_AVG4		(2)
#define BMI160_ACCEL_CIC_AVG8			(3)
#define BMI160_ACCEL_RES_AVG2			(4)
#define BMI160_ACCEL_RES_AVG4			(5)
#define BMI160_ACCEL_RES_AVG8			(6)
#define BMI160_ACCEL_RES_AVG16			(7)
#define BMI160_ACCEL_RES_AVG32			(8)
#define BMI160_ACCEL_RES_AVG64			(9)
#define BMI160_ACCEL_RES_AVG128			(10)

 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_bandwidth(u8 bw)
{
    BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_INIT_VALUE ;
    
        //com_rslt = spi_routine(); 
        //s_bmi160.dev_addr = seclect;
        com_rslt += bmi160_sec(&s_bmi160);
        /* Set the accel bandwidth as Normal */
        com_rslt += bmi160_set_accel_bw(bw);
        s_bmi160.delay_msec(
        BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
    
    return com_rslt;
}

/*!
 *	@brief This function used for read the sensor
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_read_sensor_data(struct bmi160_data_t *dataPt)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_INIT_VALUE ;
    //char SensorTmp[128];
    //get ms
    //need add
    com_rslt += bmi160_sec(&s_bmi160);
	  com_rslt += bmi160_get_sensor_time(&(dataPt->timeTamp));
    com_rslt += bmi160_read_gyro_xyz(&(dataPt->gyro));
    com_rslt += bmi160_read_accel_xyz(&(dataPt->accel));
    //com_rslt += bmi160_get_temp(&bmi160_temperature);
   /*********
    sprintf(SensorTmp, "BMI160 -> %d %d %d %d %d %d\r\n",bmi160_data.gyro.x
        ,bmi160_data.gyro.y,bmi160_data.gyro.z,bmi160_data.accel.x
        ,bmi160_data.accel.y,bmi160_data.accel.z);
    //write bmi160data to queue
    print(SensorTmp);
    *************/

    return com_rslt;
}

/*!
 *	@brief This function used for initialize the sensor
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_initialize_sensor(void)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_INIT_VALUE ;
	u8  v_loop_u8,v_data_u8;
 /*	Based on the user need configure I2C or SPI interface.
  *	It is sample code to explain how to use the bmi160 API*/
  //config 160 use spi as primary interface
      GPIO_ResetBits(sensor_cs_seclect[BMI160].port , sensor_cs_seclect[BMI160].pin);
      bmi160_delay_ms(1);
      GPIO_SetBits(sensor_cs_seclect[BMI160].port , sensor_cs_seclect[BMI160].pin);
	#ifdef INCLUDE_BMI160API
    com_rslt = spi_routine(); 
	#endif

    com_rslt += bmi160_init(&s_bmi160);
    /**** STANDARD UI IMU  output****/
	  com_rslt += bmi160_config_running_mode(STANDARD_UI_IMU);   /* or APPLICATION_NAVIGATION ; original: APPLICATION_HEAD_TRACKING*/
    //output Bmi160 reg value
        print("Bmi160 regVale:");
	v_data_u8 = 0xff;
	print("%x",v_data_u8);
    	   for(v_loop_u8=0;v_loop_u8<0x7c;v_loop_u8++)
    	   {
    	       v_data_u8 = 0xff;
    	       com_rslt += bmi160_read_reg(v_loop_u8,&v_data_u8, 1);
			
			if(0==(v_loop_u8%16))
			{
				print("\r\n ");
			}
			print("%x",v_data_u8);
			print(" ");
			
    	   }
	return com_rslt;
}
/*!
 *	@brief This Function used to read the sensor data using
 *	different running mode
 *	@param v_running_mode_u8 : The value of running mode
 *      Description                |  value
 * --------------------------------|----------
 *  STANDARD_UI_9DOF_FIFO          |   0
 *	STANDARD_UI_IMU_FIFO           |   1
 *	STANDARD_UI_IMU                |   2
 *	STANDARD_UI_ADVANCEPOWERSAVE   |   3
 *	ACCEL_PEDOMETER                |   4
 *	APPLICATION_HEAD_TRACKING      |   5
 *	APPLICATION_NAVIGATION         |   6
 *	APPLICATION_REMOTE_CONTROL     |   7
 *	APPLICATION_INDOOR_NAVIGATION  |   8
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_config_running_mode(
u8 v_running_mode_u8)
{
	struct gyro_sleep_setting gyr_setting;
	/* Variable used for get the status of mag interface*/
	u8 v_mag_interface_u8 = BMI160_INIT_VALUE;
	u8 v_bmm_chip_id_u8 = BMI160_INIT_VALUE;
	BMI160_RETURN_FUNCTION_TYPE com_rslt = ERROR;
		/* Configure the gyro sleep setting based on your need*/
	if (v_running_mode_u8 == STANDARD_UI_ADVANCEPOWERSAVE) {
		gyr_setting.sleep_trigger = BMI160_SLEEP_TRIGGER;
		gyr_setting.wakeup_trigger = BMI160_WAKEUP_TRIGGER;
		gyr_setting.sleep_state = BMI160_SLEEP_STATE;
		gyr_setting.wakeup_int = BMI160_WAKEUP_INTR;
	}
	/* The below code used for enable and
	disable the secondary mag interface*/
	//get interface mode 
	com_rslt = bmi160_get_if_mode(&v_mag_interface_u8);
	if (((v_running_mode_u8 == STANDARD_UI_IMU_FIFO) ||
	(v_running_mode_u8 == STANDARD_UI_IMU) ||
	(v_running_mode_u8 == STANDARD_UI_ADVANCEPOWERSAVE) ||
	(v_running_mode_u8 == APPLICATION_NAVIGATION) ||
	(v_running_mode_u8 == ACCEL_PEDOMETER) ||
	(v_running_mode_u8 == APPLICATION_REMOTE_CONTROL) ||
	(v_running_mode_u8 == APPLICATION_INDOOR_NAVIGATION))
	&& (v_mag_interface_u8 == BMI160_MAG_INTERFACE_ON_PRIMARY_ON)) {
		com_rslt +=
		bmi160_set_bmm150_mag_and_secondary_if_power_mode(
		MAG_SUSPEND_MODE);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		com_rslt += bmi160_set_if_mode(
		BMI160_MAG_INTERFACE_OFF_PRIMARY_ON);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
	}
	if (((v_running_mode_u8 == STANDARD_UI_9DOF_FIFO)
		|| (v_running_mode_u8 == APPLICATION_HEAD_TRACKING) ||
		(v_running_mode_u8 == APPLICATION_NAVIGATION)) &&
		(v_mag_interface_u8 == BMI160_MAG_INTERFACE_OFF_PRIMARY_ON)) {
			/* Init the magnetometer */
			com_rslt += bmi160_bmm150_mag_interface_init(
			&v_bmm_chip_id_u8);
			/* bmi160_delay_ms in ms*/
			s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	}
	switch (v_running_mode_u8) {
	case STANDARD_UI_9DOF_FIFO:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
		/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
		/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
		/* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 100Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
		BMI160_GYRO_OUTPUT_DATA_RATE_100HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 100Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
		BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ, BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/***** read FIFO data based on interrupt*****/
		com_rslt += bmi160_interrupt_configuration();
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO header*/
		com_rslt += bmi160_set_fifo_header_enable(FIFO_HEADER_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO mag*/
		com_rslt += bmi160_set_fifo_mag_enable(FIFO_MAG_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO accel*/
		com_rslt += bmi160_set_fifo_accel_enable(FIFO_ACCEL_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO gyro*/
		com_rslt += bmi160_set_fifo_gyro_enable(FIFO_GYRO_ENABLE);
		/* Enable the FIFO time*/
		com_rslt += bmi160_set_fifo_time_enable(FIFO_TIME_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO water mark interrupt1*/
		com_rslt += bmi160_set_intr_fifo_wm(BMI160_INIT_VALUE,
		FIFO_WM_INTERRUPT_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO water mark interrupt2*/
		com_rslt += bmi160_set_intr_fifo_wm(BMI160_ENABLE,
		FIFO_WM_INTERRUPT_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set the fifo water mark*/
		com_rslt += bmi160_set_fifo_wm(BMI160_ENABLE_FIFO_WM);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read the FIFO data*/
		com_rslt +=  bmi160_read_fifo_header_data(BMI160_SEC_IF_BMM150);
	break;
	case STANDARD_UI_IMU_FIFO:
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
		/* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 100Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_100HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 100Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/***** read FIFO data based on interrupt*****/
		com_rslt += bmi160_interrupt_configuration();
		/* Enable the FIFO header*/
		com_rslt += bmi160_set_fifo_header_enable(FIFO_HEADER_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO accel*/
		com_rslt += bmi160_set_fifo_accel_enable(FIFO_ACCEL_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO gyro*/
		com_rslt += bmi160_set_fifo_gyro_enable(FIFO_GYRO_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO time*/
		com_rslt += bmi160_set_fifo_time_enable(FIFO_TIME_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO water mark interrupt1*/
		com_rslt += bmi160_set_intr_fifo_wm(BMI160_INIT_VALUE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO water mark interrupt2*/
		com_rslt += bmi160_set_intr_fifo_wm(BMI160_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set the fifo water mark as 10*/
		com_rslt += bmi160_set_fifo_wm(BMI160_ENABLE_FIFO_WM);
		/* read the FIFO data*/
		com_rslt +=  bmi160_read_fifo_header_data(BMI160_SEC_IF_BMM150);
	break;
	case STANDARD_UI_IMU:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
		
	  /* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
	
	  /* Set the accel range as 2g */
		com_rslt += bmi160_set_accel_range(BMI160_ACCEL_RANGE_2G);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo range as 125deg/s */
		com_rslt += bmi160_set_gyro_range(BMI160_GYRO_RANGE_125_DEG_SEC);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/	
		
	  /* set gyro data rate as 100Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_100HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 100Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		
		/* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data*/
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
	break;
	case STANDARD_UI_ADVANCEPOWERSAVE:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
		/* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 100Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_1600HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 100Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/

		/* Enable any motion interrupt - x axis*/
		com_rslt += bmi160_set_intr_enable_0(BMI160_ANY_MOTION_X_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable any motion interrupt - y axis*/
		com_rslt += bmi160_set_intr_enable_0(BMI160_ANY_MOTION_Y_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable any motion interrupt - z axis*/
		com_rslt += bmi160_set_intr_enable_0(BMI160_ANY_MOTION_Z_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable no motion interrupt - x axis*/
		com_rslt += bmi160_set_intr_enable_2(BMI160_NOMOTION_X_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable no motion interrupt - y axis*/
		com_rslt += bmi160_set_intr_enable_2(BMI160_NOMOTION_Y_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable no motion interrupt - z axis*/
		com_rslt += bmi160_set_intr_enable_2(BMI160_NOMOTION_Z_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set the gyro sleep trigger*/
		com_rslt += bmi160_set_gyro_sleep_trigger(
		gyr_setting.sleep_trigger);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set the gyro wakeup trigger*/
		com_rslt += bmi160_set_gyro_wakeup_trigger(
		gyr_setting.wakeup_trigger);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set the gyro sleep state*/
		com_rslt += bmi160_set_gyro_sleep_state(
		gyr_setting.sleep_state);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set the gyro wakeup interrupt*/
		com_rslt += bmi160_set_gyro_wakeup_intr(gyr_setting.wakeup_int);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data*/
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
	break;
	case ACCEL_PEDOMETER:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_LOWPOWER);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
		/*Set the gyro mode as SUSPEND write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_SUSPEND);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
		/* Set the accel bandwidth as OSR4 */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 25Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ,
			BMI160_ACCEL_OSR4_AVG1);
		/* 10 not available*/
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read accel data*/
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
	break;
	case APPLICATION_HEAD_TRACKING:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
		/* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 1600Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
		BMI160_GYRO_OUTPUT_DATA_RATE_1600HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 1600Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
		BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ, BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data */
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
		/* read mag data */
		/*com_rslt += bmi160_bmm150_mag_compensate_xyz(&magxyz);*/
	break;
	case APPLICATION_NAVIGATION:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
		/* Set the accel bandwidth as OSRS4 */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 200Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_200HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 200Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data */
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
		/* read mag data*/
		/*com_rslt += bmi160_bmm150_mag_compensate_xyz(&magxyz);*/
	break;
	case APPLICATION_REMOTE_CONTROL:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
		/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
		/* Set the accel bandwidth as OSRS4 */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as OSR4 */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_OSR4_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 200Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_200HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 200Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read gyro data */
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data*/
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
	break;
	case APPLICATION_INDOOR_NAVIGATION:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the accel bandwidth as OSRS4 */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as OSR4 */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_OSR4_MODE);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 200Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_200HZ);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 200Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data */
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
		break;
	case APPLICATION_DRONE_FLIGHTCONTROL:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(C_BMI160_THIRTY_U8X);
		
	  /* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
	
	  /* Set the accel range as 2g */
		com_rslt += bmi160_set_accel_range(BMI160_ACCEL_RANGE_2G);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo range as 250 deg/s */
		com_rslt += bmi160_set_gyro_range(BMI160_GYRO_RANGE_250_DEG_SEC);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/	
		
	  /* set gyro data rate as 400Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_400HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 400Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		
		/* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data*/
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
	break;
  default:
   break;		
	}

	return com_rslt;

}
/*!
 *	@brief This function used for interrupt configuration
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_interrupt_configuration(void)
{
	/* This variable used for provide the communication
	results*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	/* Configure the in/out control of interrupt1*/
	com_rslt = bmi160_set_output_enable(BMI160_INIT_VALUE,
	BMI160_ENABLE);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	/* Configure the in/out control of interrupt2*/
	com_rslt += bmi160_set_output_enable(BMI160_ENABLE,
	BMI160_ENABLE);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	/* Configure the interrupt1 active high
	0x00 -	Active low
	0x01 -	Active high*/
	com_rslt += bmi160_set_intr_level(BMI160_INIT_VALUE,
	BMI160_ENABLE);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	/* Configure the interrupt2 active high
	0x00 -	Active low
	0x01 -	Active high*/
	com_rslt += bmi160_set_intr_level(BMI160_ENABLE,
	BMI160_ENABLE);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	return com_rslt;
}

#ifdef INCLUDE_BMI160API

#define MASK_DATA1	0xFF
#define MASK_DATA2	0x80
#define MASK_DATA3	0x7F

/*!
 *	@brief Used for SPI initialization
 *	@note
 *	The following function is used to map the
 *	SPI bus read, write and bmi160_delay_ms
 *	with global structure bmi160
*/
s8 spi_routine(void)
{
/*--------------------------------------------------------------------------*
 *  By using bmi160 the following structure parameter can be accessed
 *	Bus write function pointer: BMI160_WR_FUNC_PTR
 *	Bus read function pointer: BMI160_RD_FUNC_PTR
 *	bmi160_delay_ms function pointer: bmi160_delay_ms_msec
 *--------------------------------------------------------------------------*/

	s_bmi160.bus_write = bmi160_spi_bus_write;
    s_bmi160.bus_read = bmi160_spi_bus_read;
    s_bmi160.delay_msec = bmi160_delay_ms;
    s_bmi160.dev_addr = BMI160;

	return BMI160_INIT_VALUE;
}

/*!
 *	@brief : The function is used as SPI bus read
 *	@return : Status of the SPI read
 *	@param dev_addr : The device address of the sensor
 *	@param reg_addr : Address of the first register,
 *	will data is going to be read
 *	@param reg_data : This data read from the sensor,
 *	which is hold in an array
 *	@param cnt : The no of byte of data to be read
 */
s8 bmi160_spi_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	uint8_t count = 0;
    SECSOR_CS_LOW(dev_addr);
    //vTaskDelay(1);
    SPI1_ReadWrite_Byte(reg_addr| 0x80);
    for(count=0;count<cnt;count++)
    {       
        reg_data[count] = SPI1_ReadWrite_Byte(0xFF);
    }   
    SECSOR_CS_HIGH(dev_addr);
   // vTaskDelay(1);
    return 0;
}
/*!
 *	@brief : The function is used as SPI bus write
 *	@return : Status of the SPI write
 *	@param dev_addr : The device address of the sensor
 *	@param reg_addr : Address of the first register,
 *	will data is going to be written
 *	@param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	@param cnt : The no of byte of data to be write
 */
s8 bmi160_spi_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 ierror = BMI160_INIT_VALUE;
	#ifdef INCLUDE_BMI160API

	u8 array[SPI_BUFFER_LEN * C_BMI160_BYTE_COUNT];
	u8 stringpos = BMI160_INIT_VALUE;

	for (stringpos = BMI160_INIT_VALUE;
	stringpos < cnt; stringpos++) {
		/* the operation of (reg_addr++)&0x7F done:
		because it ensure the
		   0 and 1 of the given value
		   It is done only for 8bit operation*/
		array[stringpos * C_BMI160_BYTE_COUNT] =
		(reg_addr++) & MASK_DATA3;
		array[stringpos * C_BMI160_BYTE_COUNT +
		BMI160_GEN_READ_WRITE_DATA_LENGTH] =
		*(reg_data + stringpos);
	}
	/* Please take the below function as your reference
	 * for write the data using SPI communication
	 * add your SPI write function here.
	 * "IERROR = SPI_WRITE_STRING(ARRAY, CNT*2)"
	 * ierror is an return value of SPI write function
	 * Please select your valid return value
	 * In the driver SUCCESS defined as 0
     * and FAILURE defined as -1
	 */
	 ierror = SPI_WRITE_STRING(dev_addr,array,cnt*2);
	 #endif
	return (s8)ierror;
}

s8 SPI_WRITE_STRING(u8 dev_addr, u8 *BufferData, u16 length)
{
    s8 count = 0;
    for(count=0;count<length;)
    {
        SpiWriteData(dev_addr ,BufferData[count] ,BufferData[count+1]);
        count = count + 2;
    }
    return 0;
}

#endif
/*!
 *	@brief This function is an example for delay
 *	@param msek: delay in milli seconds
 *	@return : 
 */
void bmi160_delay_ms(u32 msek)
{
 /* user delay*/
 delayMs(msek);
}
