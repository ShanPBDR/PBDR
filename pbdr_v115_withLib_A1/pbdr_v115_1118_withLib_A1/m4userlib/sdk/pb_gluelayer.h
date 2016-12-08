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

#ifndef __PB_GLUELAYER_H__
#define __PB_GLUELAYER_H__

#include "public_struct.h"

/* ----- part 1: GNSS struct ----- */

#define MSGSECONDS_PER_WEEK      604800   /* seconds in a week */
#define MSGSECONDS_PER_DAY        86400   /* seconds in a day  */

/* --- bit fields are consistant with internal interface ---*/
#define UBXGNSS_MEASUREMENT_STATE_UNKNOWN                   0
#define UBXGNSS_MEASUREMENT_STATE_CODE_LOCK             (1<<0)
#define UBXGNSS_MEASUREMENT_STATE_BIT_SYNC              (1<<1)
#define UBXGNSS_MEASUREMENT_STATE_SUBFRAME_SYNC         (1<<2)
#define UBXGNSS_MEASUREMENT_STATE_TOW_DECODED           (1<<3)
#define UBXGNSS_MEASUREMENT_STATE_CARRIER_PULLIN        (1<<4)

#define MAX_BMI160_NUM_SNSR     (MAX_DR_NUM_SNSR)
#define MAX_BMI160_DATA_SETS    (2*MAX_DR_DATA_SETS)         /* reserve more room for receiving raw data */

/*-------- BMI 160 sensor struct ---------- */
typedef struct
{
   unsigned int   timeTag;                      /* time in [ms]*/
   int            sensor[MAX_BMI160_NUM_SNSR];   /* 3 gyro in deg/s, 3 accel in m/s^2, 1 odo in m/s, 1 reverse; scaled by 1000 */
} tBMI160_DRSensorData;

typedef struct
{
   unsigned short         numDataSets;
   tBMI160_DRSensorData   data_set[MAX_BMI160_DATA_SETS];
} tBMI160_DRSensor, *tBMI160_DRSensorPtr;


/*----- function declaration -----*/
void pb_Gluelayer_External_Init(void);

tExternalEnrichedGNSSFixPtr Msg_Get_ExternalGnssEnrichedFix(void);
tExternalGNSSGPSTimePtr     Msg_Get_ExternalGnssGpsTime(void);
tExternalGNSS_AuxMeasPtr    Msg_Get_ExternalGnssAuxMeas(void);

tExternalGNSS_MeasPtr Msg_Get_ExternalGNSS_RawChan(void);
tGnssChanCycleSlipPtr Msg_Get_CycleSlip(void);

tBMI160_DRSensorPtr Msg_Get_ExternalDRSensor(void);

void Msg_UTC2GpsTime(unsigned short year, unsigned char month, unsigned short day, unsigned char hour, unsigned char minute, unsigned char seconds, unsigned short *gpsWeek, double *gpsTOW);
unsigned int ConvertGPSToUnixTime(unsigned short wnum, double tow);

#endif


