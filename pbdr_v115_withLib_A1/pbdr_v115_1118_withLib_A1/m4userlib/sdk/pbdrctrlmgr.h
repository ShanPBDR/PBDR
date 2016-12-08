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

#ifndef __PBDRCTRLMGR_H__
#define __PBDRCTRLMGR_H__

#define FIRMWAREVERSION "FW Version 1.15.A1"                 /* 11/18/2016 */

#define BMI_RAWDATA_RATE       (100)                      /* raw 100Hz, should be consistent with bmi160 configure */
#define BMI_QUEUE_MAX          (BMI_RAWDATA_RATE)         /* BMI160 raw data rate */

#define BMI_NUMSAMPLE_PERCYCLE   (10)                     /* every 10 samples to be down sampled into 1 */

#define BMI_NUMSAMPLE_PERSEC   (10)                       /* 10 samples per second */
#define BMI_MAX_NUM_SNSRSAMPLE (BMI_NUMSAMPLE_PERSEC + 1) /* +1, just a protection */

#define BMI_MAX_DR_NUM_SNSR    (6)                        /* 3 gyros, 3 accels */

/*!
 *	@brief This function is init BMI160 and ram
 *	@param :null
 *	@return : null
 */

void sensor_BMI160_init(void);

/*!
 *	@brief This function is get data from BMI160
 *	@param :null
 *	@return : null
 */
void getDataFromBMI160(void);  

/*!
 *	@brief This function is init gnss and ram
 *	@param pvParameters  
 *	@return : null
 */
void DataHandler_init(void);
/*!
 *	@brief This function is called by 10msInt
 *	@param pvParameters  
 *	@return : null
 */
void DataHandler_int(void);

/*!
 *	@brief This function is deal sensor raw data
 *	@param pvParameters  
 *	@return : null
 */
void DataHandler_main(void);
void DataOutput(void);

void pbDR_Push_DataBuffer( void );

void DRSystemOpen( void );

char* pb_FirmWare_Version_Info(void);

#endif
