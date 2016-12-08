/**************************************************************************
*                  Copyright(c) 2016, Pinnobot Technology                 *
*                        All rights reserved.                             *
*                                                                         *
*    This Software is protected by United States copyright laws and       *
*    international treaties. You may not reverse engineer, decompile      *
*    or disasssemble this software                                        *
*                                                                         *
*    This Software contains Pinnobot Technology Inc.’s confidential and   *
*    proprietary information. UNAUTHORIZED COPYING, USE, DISTRIBUTION,    *
*    PUBLICATION, TRANSFER, SALE, RENTAL OR DISCLOSURE IS PROHIBITED      *
*    AND MAY RESULT IN SERIOUS LEGAL CONSEQUENCES.                        *
***************************************************************************/

#ifndef __DRSENSOR_CONFIG_H__
#define __DRSENSOR_CONFIG_H__

/* Note:  for automotive or drone platform, 
 * bmi160_config_running_mode in bmi160_support.c should be set correspondingly 
   automotive: STANDARD_UI_IMU at 100Hz or APPLICATION_NAVIGATION at 200Hz 
	 drone:      APPLICATION_DRONE_FLIGHTCONTROL at 400Hz (or 800Hz) */

/*#define DRONE_PLATFORM*/

#ifndef DRONE_PLATFORM
  #define BOSCH160_CONFIG_DATARATE     (BOSCH160_OUTPUT_DATA_RATE_100HZ)              /* 100Hz */
#else
	#define BOSCH160_CONFIG_DATARATE     (BOSCH160_OUTPUT_DATA_RATE_400HZ)          
#endif

/*------- sensor configuration -----*/
/* --- gyro ---*/
#define BOSCH160_GYR_FS_2000              (0x0)         /* range set in regester 0x43 bit 2:0*/
#define BOSCH160_GYR_FS_1000              (0x1)
#define BOSCH160_GYR_FS_500               (0x2)
#define BOSCH160_GYR_FS_250               (0x3)
#define BOSCH160_GYR_FS_125               (0x4)

#ifndef DRONE_PLATFORM
  #define BOSCH160_GYR_FULLSCALE_RANGE_VAL     (BOSCH160_GYR_FS_125)              /* 125 deg/s */
#else
	#define BOSCH160_GYR_FULLSCALE_RANGE_VAL     (BOSCH160_GYR_FS_250)          
#endif


#if (BOSCH160_GYR_FULLSCALE_RANGE_VAL & BOSCH160_GYR_FS_125) == BOSCH160_GYR_FS_125
#define BOSCH160_GYR_SCALE_FACTOR        (262.4) /* LSB/(deg/s) ~3.811 milli-degree per second(mdps) /LSB */
#elif (BOSCH160_GYR_FULLSCALE_RANGE_VAL & BOSCH160_GYR_FS_250) == BOSCH160_GYR_FS_250
#define BOSCH160_GYR_SCALE_FACTOR        (131.2) /* LSB/(deg/s) ~7.622 mdps/LSB */
#elif (BOSCH160_GYR_FULLSCALE_RANGE_VAL & BOSCH160_GYR_FS_500) == BOSCH160_GYR_FS_500
#define BOSCH160_GYR_SCALE_FACTOR        (65.6) /* LSB/(deg/s) ~15.244 mdps/LSB */
#elif (BOSCH160_GYR_FULLSCALE_RANGE_VAL & BOSCH160_GYR_FS_1000) == BOSCH160_GYR_FS_1000
#define BOSCH160_GYR_SCALE_FACTOR        (32.8) /* LSB/(deg/s) ~30.488 mdps/LSB */
#elif (BOSCH160_GYR_FULLSCALE_RANGE_VAL & BOSCH160_GYR_FS_2000) == BOSCH160_GYR_FS_2000
#define BOSCH160_GYR_SCALE_FACTOR        (16.4) /* LSB/(deg/s) ~60.976 mdps/LSB */
#endif

#define BOSCH160_GYR_GYR_DPS_SENSITIVITY      (1.0/BOSCH160_GYR_SCALE_FACTOR) /* dps/LSB */

/*--- accelerometer ---*/
#define BOSCH160_ACC_FS_2                 (0x3)  /*regester 0x41 +/-  2g */
#define BOSCH160_ACC_FS_4                 (0x5)  /* +/-  4g */
#define BOSCH160_ACC_FS_8                 (0x8)  /* +/-  8g */
#define BOSCH160_ACC_FS_16                (0xC)  /* +/- 16g */

#define BOSCH160_ACC_FULLSCALE_RANGE_VAL  (BOSCH160_ACC_FS_2)                /* 2g is enough for both drone and automotive */

#if (BOSCH160_ACC_FULLSCALE_RANGE_VAL & BOSCH160_ACC_FS_2) == BOSCH160_ACC_FS_2
#define BOSCH160_ACC_SCALE_FACTOR        (16384)   /* LSB/g ~0.061 mg/LSB */
#elif (BOSCH160_ACC_FULLSCALE_RANGE_VAL & BOSCH160_ACC_FS_4) == BOSCH160_ACC_FS_4
#define BOSCH160_ACC_SCALE_FACTOR        (8192)    /* LSB/g ~0.122 mg/LSB */
#elif (BOSCH160_ACC_FULLSCALE_RANGE_VAL & BOSCH160_ACC_FS_8) == BOSCH160_ACC_FS_8
#define BOSCH160_ACC_SCALE_FACTOR        (4096)    /* LSB/g ~0.244 mg/LSB */
#elif (BOSCH160_ACC_FULLSCALE_RANGE_VAL & BOSCH160_ACC_FS_16) == BOSCH160_ACC_FS_16
#define BOSCH160_ACC_SCALE_FACTOR        (2048)    /* LSB/g ~0.488 mg/LSB */
#endif

#define BOSCH160_NOMINAL_G              (9.80665f)
#define BOSCH160_ACC_SENSITIVITY        (1.0/BOSCH160_ACC_SCALE_FACTOR*BOSCH160_NOMINAL_G) /* m/s^2/LSB */



/* sensor time config table: bmi160 datasheet page 17 
 * we're using 100Hz for automotive, and 400Hz for drone */
#ifndef DRONE_PLATFORM
/* if data rate @ 100Hz, resolution = 10ms, unit increment from bit 8 */
#define SNSRTIME_DATARATE_BIT      (8)
#define SNSRTIME_RESOLUTION        (10.0)                        /* 10ms */
#else
/* if data rate @ 400Hz, resolution = 2.5ms, unit increment from bit 6 */
#define SNSRTIME_DATARATE_BIT      (6)
#define SNSRTIME_RESOLUTION        (2.5)                         /* 2.5ms */
#endif

/* in ms, timetag in unsigned 24 bits at regester 0x18 to 0x1A  
 * rollover = (2^(24-SNSRTIME_DATARATE_BIT) * SNSRTIME_RESOLUTION), should be always 655360 in ms */
#define SNSRTIME_ROLLOVER          (655360)    


#endif
