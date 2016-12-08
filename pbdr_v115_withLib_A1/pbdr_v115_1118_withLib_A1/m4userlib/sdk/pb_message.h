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

#ifndef __PB_MESSAGE_H__
#define __PB_MESSAGE_H__

#include "public_struct.h"

#define PBMESSAGE_LEVEL_1
#define PBMESSAGE_LEVEL_2
//#define PBMESSAGE_LEVEL_3
#define CONFIG_GPS        (0x01)
#define CONFIG_GLONASS    (0x02)
#define CONFIG_BDS        (0x03)
#define CONFIG_GALILEO    (0x04)

#define CONFIG_GNSS  (CONFIG_GPS | CONFIG_GLONASS | CONFIG_BDS | CONFIG_GALILEO)

#define NMEA_SENTENCE_LENGTH         (102)

#define PB_MSG_SYNC0     (0x50)      /* binary msg sync char 1 */
#define PB_MSG_SYNC1     (0x42)      /* binary msg sync char 1 */

/*-------- NMEA data struct -----*/
typedef struct
{
   /* Status Character (for RMC): 'A' - Active/Data Valid; 'V' - Void/Navigation Receiver Warning */
   char   status;

   /* talker id (for GSA, GLL, RMC) */
   /* 'GN' - For any combined GNSS; 'GP': GPS/SBAS/QZSS only; 'GL': GLONASS only; 'GB': Beidou only; 'GA': Galileo only */
   char   talker_id[3];

   /* Mode indicator String (for GLL, RMC, VTG). 'A':Autonomous; 'D':Differential   */
   char   mode_indicator[3];

   /* Fix Selection Character (for GSA).  'A':Auto selection of 2D/3D fix; 'M': Force to operate in 2D/3D */
   char   mode_1;

   /* Fix Type (for GSA). 1: Fix not available; 2: 2D Fix; 3: 3D Fix */
   unsigned char  mode_2;

   /* Fix Quality Indicator (for GGA) */
   /* 0:No fix; 1:SPS mode fix; 2: DGPS; 3: PPS fix; 4: RTK; 5: float RTK; 6: dead-reckoning; 7: input */
   unsigned char  quality_indicator;

   /* UTC  "hhmmss.sss" (for GNS, GGA, GLL, RMC) */
   char   utc_of_position[11];

   /* Date "ddmmyy" (for RMC) */
   char   date[7];

   /* Latitude String "ddmm.mmmmm,N" (for GNS, GGA, GLL, RMC) */
   char   latitude[13];

   /* Longitude String "dddmm.mmmmm,E" (for GNS, GGA, GLL, RMC) */
   char   longitude[14];

   /* Altitude String "xxxxxx.x,M" (for GNS, GGA) */
   char   altitude[12];

   /* geoid separation string "xxxx.x,M" (for GNS, GGA) */
   char   geoSep[9];

   /* Speed over ground in knots string "xxxx.xx" (for RMC, VTG) */
   char   sog_knots[8];

   /* Speed over ground in km/h String "xxxx.xx" (for VTG) */
   char   sog_km_h[8];

   /* Course over ground in degrees String "xxx.xx" (for RMC, VTG) */
   char   cog_true[7];

   /* Number of GPS satellites in use (for GNS, GGA) */
   unsigned char  number_of_sv_in_use_gp;

   /* Number of GLONASS satellites in use (for GNS, GGA) */
   unsigned char  number_of_sv_in_use_gl;

   /* Number of BDS satellites in use (for GNS, GGA) */
   unsigned char  number_of_sv_in_use_gb;

   unsigned char  number_of_sv_inFix;

   /* Horizontal Dilution of Position string (for GNS, GGA, GSA) */
   float    hdop;

   /* Time in seconds since last DGPS update (for GNS, GGA) */
   char    age_of_diff_data[5];

   /* DGPS Station ID number (for GNS, GGA)  0 - 1023  */
   char  diff_ref_station_id[5];

} tNMEA_CommonData, *tNMEA_CommonDataPtr;

void pb_NMEA_GGA_Output(tNMEA_CommonDataPtr data);
void pb_NMEA_RMC_Output(tNMEA_CommonDataPtr data);

void pb_AddNMEACheckSum(char *pBuf);
/*========= end of NMEA =====*/


/*----- output of message to UART  --------*/
eMOD_STATUS  pb_io_InfoOutput(void);

void pb_NMEA_Output(tOutput_NavStatesPtr NavOut);                /*--- must have: NMEA ---*/

#ifdef PBMESSAGE_LEVEL_1
void pb_SocketMsg_Nav_Output(tOutput_NavStatesPtr NavOut);       /*--- level 1: PBSOL,1 --- navigation solution ---*/
void pb_SocketMsg_BlkEnd_Output(void);                           /*--- level 1: PBLKEND --- epoch end ---*/
#endif

#ifdef PBMESSAGE_LEVEL_2
void pb_SocketMsg_GNSS6_Output(tExternalEnrichedGNSSFixPtr gnssfix);  /*--- level 2: PGNSS,6 --- GNSS enriched geodetic fix ---*/
void pb_SocketMsg_GNSS13_Output(tExternalGNSS_AuxMeasPtr auxChan);    /*--- level 2: PGNSS,13 --- GNSS auxiliary channel measurements ---*/
void pb_SocketMsg_SNSR21_Output(tExternalDRSensorPtr snsr);    /*--- level 2: PSNSR,21 --- DR sensor data ---*/
#endif

#ifdef PBMESSAGE_LEVEL_3
void pb_SocketMsg_GNSS3_Output(tExternalGNSS_MeasPtr gnssMeas);       /*--- level 3: PGNSS,3 --- GNSS raw channel measurements ---*/
#endif

unsigned int pbMsg_ConvertGPSToUnixTime(unsigned short wnum, double tow);

#endif
