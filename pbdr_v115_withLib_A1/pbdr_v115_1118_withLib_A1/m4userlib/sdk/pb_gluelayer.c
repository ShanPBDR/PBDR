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

#include <string.h>
#include <math.h>

#include "pb_gluelayer.h"

/* for real time only (in offline the following variables are from log file) */
static tExternalEnrichedGNSSFix   RawExternalGnssFullFix;
static tExternalGNSSGPSTime       RawGpsTime;
static tExternalGNSS_AuxMeas      RawAuxGnssMeas[MAX_GNSS_NUM_CHANS];

static tExternalGNSS_Meas         RawGnssChan[MAX_GNSS_NUM_CHANS];
static tGnssChanCycleSlip         cycleSlip[MAX_GNSS_NUM_CHANS];

static tBMI160_DRSensor           RawDRSensor;

void pb_Gluelayer_External_Init(void)
{
   memset(&RawExternalGnssFullFix, 0, sizeof(RawExternalGnssFullFix));
   memset(&RawGpsTime, 0, sizeof(RawGpsTime));
   memset(&RawAuxGnssMeas, 0, sizeof(RawAuxGnssMeas));

   memset(&RawGnssChan, 0, sizeof(RawGnssChan));
   memset(&cycleSlip, 0, sizeof(cycleSlip));

   memset(&RawDRSensor, 0, sizeof(RawDRSensor));
}

tExternalEnrichedGNSSFixPtr Msg_Get_ExternalGnssEnrichedFix(void)
{
   return (&RawExternalGnssFullFix);
}

tExternalGNSSGPSTimePtr Msg_Get_ExternalGnssGpsTime(void)
{
   return (&RawGpsTime);
}

tExternalGNSS_AuxMeasPtr Msg_Get_ExternalGnssAuxMeas(void)
{
   return (&RawAuxGnssMeas[0]);
}

tExternalGNSS_MeasPtr Msg_Get_ExternalGNSS_RawChan(void)
{
   return (&RawGnssChan[0]);
}

tGnssChanCycleSlipPtr  Msg_Get_CycleSlip(void)
{
   return(&cycleSlip[0]);
}

tBMI160_DRSensorPtr Msg_Get_ExternalDRSensor(void)
{
   return (&RawDRSensor);
}

/*------- some util functions, open to public api ------*/
/* cumulative days */
const short pdaysInMonth_gluelayer[13] = { 0,31, 59, 90,120,151,181,212,243,273,304,334,365 };

void Msg_UTC2GpsTime(unsigned short year, unsigned char month, unsigned short day, unsigned char hour, unsigned char minute, unsigned char seconds, unsigned short *gpsWeek, double *gpsTOW)
{
   float secs_of_day;

   unsigned short   leap_secs, leap_days, day_of_week, day_of_year, delta_year, all_days;

   /* years since to 1980                                                  */
   delta_year = year - 1980;

   /* all leap days since 1980 - includes year 2000                        */
   leap_days = (unsigned short)(1 + floorf(delta_year / 4.f));

   day_of_year = (unsigned short)(pdaysInMonth_gluelayer[month - 1] + day);

   /* whole days since GPS time stated on midnight Jan 5/6, 1980           */
   all_days = delta_year * 365 + leap_days - 6 + day_of_year;

   /* subtract the current leap day under the following conditions         */
   if ((month <= 2) && (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 != 0)))
      leap_days--;


   /* days relative to 1980-01-06-00-00-00  */
   if (all_days > 12960)        leap_secs = 17; /* 2015-07-01-00-00  */
   else if (all_days > 11865)   leap_secs = 16; /* 2012-07-01-00-00-= 00  */
   else if (all_days > 10588)   leap_secs = 15; /* 2009-01-01-00-00-00  */
   else if (all_days >  9492)   leap_secs = 14; /* 2006-01-01-00-00-00  */
   else leap_secs = 0;

   secs_of_day = (float)(leap_secs + seconds + 60 * minute + 3600 * hour);

   /* check to see if the leap seconds cause a day rollover                */
   if (secs_of_day > 86400)
   {
      all_days += 1;
      secs_of_day -= 86400;
   }

   *gpsWeek = (unsigned short)(all_days / 7);

   /* Sunday = 0, Saturday = 6                                             */
   day_of_week = all_days % 7;

   *gpsTOW = (double)(secs_of_day + 86400 * day_of_week);

   return;
}

/* Convert GPS Time to Unix Time */
unsigned int ConvertGPSToUnixTime(unsigned short wnum, double tow)
{
#define LINUX_GPS_EPOCH_OFFSET 315964800
   /* tow in ms */
   double gpsTime = wnum * 604800 + tow;
   double unixTime;
   unsigned char nleaps = 0;

   if (wnum > 1024)
   {
      if (wnum <= 1355)
         nleaps = 13;
      else if (wnum <= 1512)
         nleaps = 14;
      else if (wnum <= 1694)
         nleaps = 15;
      else if (wnum <= 1850)
         nleaps = 16;
      else
         nleaps = 17;
   }

   /* Add offset in seconds */
   unixTime = gpsTime + LINUX_GPS_EPOCH_OFFSET - nleaps;

   return (unsigned int)(unixTime + 0.5);    /* in ms */
}

