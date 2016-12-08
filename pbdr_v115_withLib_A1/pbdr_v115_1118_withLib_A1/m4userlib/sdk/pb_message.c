
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "pb_message.h"
#include "pb_if.h"

#ifndef OFFLINE
#include "Print.h"
#endif

#ifdef OFFLINE
//#define snprintf sprintf_s
extern FILE *fp_output;
#endif

/* ############   final result output ###########*/
/*----- output of message to UART  --------*/
eMOD_STATUS  pb_io_InfoOutput(void)
{
   tOutput_NavStates navout;
   memset(&navout, 0, sizeof(navout));
   if (pb_Query_OutputNavigationState(&navout) != eMOD_SUCCESS)
      return eMOD_FAILURE;

   /*--- final delivery is supposed to be NMEA only---*/
   pb_NMEA_Output(&navout);

#ifdef PBMESSAGE_LEVEL_1
   pb_SocketMsg_Nav_Output(&navout);

#ifdef OFFLINE
	 pb_SocketMsg_BlkEnd_Output();    /* end message runs differently in realtime and offline as offline does not record raw debug data */
#endif

#endif
 
   return eMOD_SUCCESS;
}

/*================= NMEA output ===============*/
/*----- output standard NMEA format -----*/
void pb_NMEA_Output(tOutput_NavStatesPtr NavOut)
{
   unsigned char fix_available = 0;
   short  LatDegree, LonDegree;
   double LatMinute, LonMinute, speed, speedinknots, speedinkmh;
   unsigned char gnssConfig = CONFIG_GNSS;	
   tNMEA_CommonData data;

 	 memset(&data, 0, sizeof(data));

   if ((gnssConfig - CONFIG_GPS) == 0)            
   {
      strncpy(data.talker_id, "GP", sizeof(data.talker_id));
   }
   else if ((gnssConfig - CONFIG_GLONASS) == CONFIG_GLONASS)
   {
      strncpy(data.talker_id, "GL", sizeof(data.talker_id));
   }
   else if ((gnssConfig - CONFIG_BDS) == CONFIG_BDS)
   {
      strncpy(data.talker_id, "GB", sizeof(data.talker_id));
   }
   else if ((gnssConfig - CONFIG_GALILEO) == CONFIG_GALILEO)
   {
      strncpy(data.talker_id, "GA", sizeof(data.talker_id));
   }
   else
   {
      strncpy(data.talker_id, "GN", sizeof(data.talker_id));
   }

   snprintf(data.utc_of_position, sizeof(data.utc_of_position), "%02d%02d%.3f", NavOut->utc_Hour, NavOut->utc_Minute, (double)NavOut->utc_Second);
   
   snprintf(data.date, sizeof(data.date), "%02d%02d%02d", NavOut->utc_Day, NavOut->utc_Month, NavOut->utc_Year % 100);

   data.status = 'V';
   data.quality_indicator = 0;

   fix_available = NavOut->fixValid;

   if (fix_available)
   {
      data.status = 'A';

      if ((NavOut->NavModeInfo & (1 << 10)) == (1 << 10))    /* DGPS used */
         data.quality_indicator = 2;
      else
         data.quality_indicator = 1;

      data.number_of_sv_inFix = NavOut->usedSV;

      data.hdop = 1.00f;  /* comment: for DR loosely couple, no hdop so far*/

      speed = NavOut->GroundSpeed * 0.01 * 3600;   /* in meter per hour */
      speedinknots = speed / (0.3048 * 6076.0);    /* in knots */
      speedinkmh   = speed * 0.001;                /* in km/hr */                        

      /* Convert Latitude to ddmm.mmmm format   -> degrees|minutes.decimal */
      LatDegree = (short)(NavOut->Lat * 1.0e-7);
      LatMinute = fabs((NavOut->Lat*1.0e-7 - LatDegree) * 60.0);

      /* a protection */
      if (abs(LatDegree) >= 90)
      {
         LatMinute = 0.0;
         NavOut->Lon = 0;
      }

      if (LatMinute > 59.99999)
      {
         LatMinute = 0.0;

         if ((LatDegree < 90) && (LatDegree >= 0))
            LatDegree++;
         else if ((LatDegree > -90) && (LatDegree < 0))
            LatDegree--;
      }

      /* Convert Longitude to dddmm.mmmm format   -> degrees|minutes.decimal */
      LonDegree = (short)(NavOut->Lon * 1.0e-7);
      LonMinute = fabs((NavOut->Lon*1.0e-7 - LonDegree) * 60.0);

      /* a protection */
      if (abs(LonDegree) >= 180)
      {
         LonMinute = 0.0;
         if (LonDegree <= -180)
         {
            LonDegree += 360;
         }
      }

      if (LonMinute > 59.99999)
      {
         LonMinute = 0.0;

         if ((LonDegree < 180) && (LonDegree >= 0))
            LonDegree++;
         else if ((LonDegree > -180) && (LonDegree < 0))
            LonDegree--;
      }


      /* latitude in ddmm.mmmmm format */
      snprintf(data.latitude, sizeof(data.latitude), "%.5f,%c", LatDegree * 100 + LatMinute, (NavOut->Lat >= 0) ? ('N') : ('S'));

      /* longitude in ddmm.mmmmm format */
      snprintf(data.longitude, sizeof(data.longitude), "%.5f,%c", LonDegree * 100 + LonMinute, (NavOut->Lon >= 0) ? ('E') : ('W'));

      snprintf(data.altitude, sizeof(data.altitude), "%.1f,%c", (NavOut->Alt*0.01), 'M');
      snprintf(data.geoSep, sizeof(data.geoSep), "%.1f,%c", (NavOut->Alt - NavOut->Msl)*0.01, 'M');

      snprintf(data.sog_knots, sizeof(data.sog_knots), "%.3f", speedinknots);
      snprintf(data.sog_km_h, sizeof(data.sog_km_h), "%.3f", speedinkmh);

      snprintf(data.cog_true, sizeof(data.cog_true), "%.2f", NavOut->Heading*0.01);

      /* no DGPS blank */
      strncpy(data.age_of_diff_data, "", sizeof(data.age_of_diff_data));
      strncpy(data.diff_ref_station_id, "", sizeof(data.diff_ref_station_id));
   }
   else /* fix not available */
   {
      /* empty fields when fix is not available */
      strncpy(data.latitude, ",", sizeof(data.latitude));
      strncpy(data.longitude, ",", sizeof(data.longitude));
      strncpy(data.altitude, "", sizeof(data.altitude));
      strncpy(data.sog_knots, "", sizeof(data.sog_knots));
      strncpy(data.sog_km_h, "", sizeof(data.sog_km_h));
      strncpy(data.cog_true, "", sizeof(data.cog_true));
   }

   pb_NMEA_GGA_Output(&data);

   pb_NMEA_RMC_Output(&data);

   return;
}

void pb_NMEA_GGA_Output(tNMEA_CommonDataPtr data)
{
   char buf[NMEA_SENTENCE_LENGTH];
   memset(buf, 0, sizeof(buf));

   snprintf(buf, NMEA_SENTENCE_LENGTH, "$%sGGA,%s,%s,%s,%hhu,%02d,%.2f,%s,%s,%s,%s", 
      data->talker_id, 
      data->utc_of_position, 
      data->latitude,
      data->longitude,
      data->quality_indicator, 
      data->number_of_sv_inFix,
      data->hdop,
      data->altitude,
      data->geoSep,
      data->age_of_diff_data,
      data->diff_ref_station_id);     

   /* add check sum */
   pb_AddNMEACheckSum(buf);
              
#ifndef OFFLINE
   /* real time to serial port */
   data_Output("%s",buf);
   data_Output("\n");
#else
   /* offline part */
   fprintf(fp_output, "%s", buf);
   fprintf(fp_output, "\n");
#endif

   return;
}

void pb_NMEA_RMC_Output(tNMEA_CommonDataPtr data)
{
   char buf[NMEA_SENTENCE_LENGTH];
   memset(buf, 0, sizeof(buf));

   snprintf(buf, NMEA_SENTENCE_LENGTH, "$%sRMC,%s,%c,%s,%s,%s,%s,%s,,", 
      data->talker_id,
      data->utc_of_position,
      data->status,
      data->latitude,
      data->longitude,
      data->sog_knots,
      data->cog_true,
      data->date);

   /* add check sum */
   pb_AddNMEACheckSum(buf);

#ifndef OFFLINE
   /* real time to serial port */
   data_Output( "%s", buf);
   data_Output( "\n");
#else
   /* offline part */
   fprintf(fp_output, "%s", buf);
   fprintf(fp_output, "\n");
#endif

   return;
}

void pb_AddNMEACheckSum(char *pBuf)
{
   unsigned char ckSum = 0;

   pBuf++; /* skip the $ sign */

   while (*pBuf != '\0')
   {
      ckSum ^= *pBuf++;
   }

   /* using sprintf since the size of pBuf gets modified before reaching here */
   sprintf(pBuf, "*%02X", ckSum);    /* +"*CK"*/
}
/*================= END of NMEA output =============*/


/*================ socket message output ===========*/
/*-------- Level 1 message ---------*/
#ifdef PBMESSAGE_LEVEL_1
/*----- output defined socket protocol format -----*/
void pb_SocketMsg_Nav_Output(tOutput_NavStatesPtr NavOut)
{
	 char MsgBuff[500];
#ifndef OFFLINE
		 #if 0 
	 unsigned char SolBuff[150], checkSum[2], syncChar[2], payloadLength[2], MsgIDSol;
	 unsigned short cnt = 0, i, sum = 0;
	#endif
#endif
	
	 memset(MsgBuff, 0, sizeof(MsgBuff));
	 
	 snprintf( MsgBuff, sizeof(MsgBuff), "$PBSOL,1,%hu,%hhu,%hu,%hhu,%hhu,%hhu,%u,%hu,%u,%u,%d,%d,%d,%d,%d,%d,%d,"
		             "%u,%u,%d,%d,%d,%d,%d,%hu,%hu,%hu,%hu,%hu,%hu,%hu,%hu,%hu,%hd,%hd,%hd",
             NavOut->utc_Year, NavOut->utc_Month, NavOut->utc_Day, NavOut->utc_Hour, NavOut->utc_Minute, NavOut->utc_Second,
	           NavOut->SnsrTime,NavOut->GPSWeek,NavOut->GPSTime,NavOut->NavModeInfo, 
	           NavOut->Lat, NavOut->Lon, NavOut->Alt, NavOut->Msl, NavOut->VelNED[0], NavOut->VelNED[1], NavOut->VelNED[2],
	           NavOut->GroundSpeed, NavOut->Distance, NavOut->Roll, NavOut->Pitch, NavOut->Heading, NavOut->clockbias, NavOut->clockdrift,
	           NavOut->PosErr_Unc[0], NavOut->PosErr_Unc[1], NavOut->PosErr_Unc[2],
	           NavOut->VelErr_Unc[0], NavOut->VelErr_Unc[1], NavOut->VelErr_Unc[2],
	           NavOut->AttErr_Unc[0], NavOut->AttErr_Unc[1], NavOut->AttErr_Unc[2],
	           NavOut->MisAngle[0], NavOut->MisAngle[1], NavOut->MisAngle[2]);	
	 
#ifndef OFFLINE 

   /* real time to serial port */
   /* 1) output string for offline analysis and debug */ 
   
   data_Output( "%s", MsgBuff);    /* do not need checksum */
   data_Output( "\n");
	 
	 #if 0 
   /* 2) output binary to external host 
	  *    little endian 
	  *    **------------------------------------------------
	  *    SyncChar1 | SyncChar2 | ID | payload length 2 bytes (little endian) | payload | CheckSumA | CheckSumB   
	 	*    **------------------------------------------------
	  *    SyncChar1: 0x50 ('P') 0x42 ('B') 
	  *    ID: 0x01 
	  *    payload 
	  *    checksum 2 bytes (little endian) covering payload only */
	 
	 /* prepare payload */
	 memset(SolBuff, 0, sizeof(SolBuff));
	 
	 memcpy(&SolBuff[cnt], &NavOut->utc_Year, 2);   cnt += 2;
	 memcpy(&SolBuff[cnt], &NavOut->utc_Month, 1);  cnt += 1;
     memcpy(&SolBuff[cnt], &NavOut->utc_Day, 2);    cnt += 2;
	 memcpy(&SolBuff[cnt], &NavOut->utc_Hour, 1);   cnt += 1;
	 memcpy(&SolBuff[cnt], &NavOut->utc_Minute, 1); cnt += 1;
	 memcpy(&SolBuff[cnt], &NavOut->utc_Second, 1); cnt += 1;	 
	 memcpy(&SolBuff[cnt], &NavOut->SnsrTime, 4);   cnt += 4;	 
	 memcpy(&SolBuff[cnt], &NavOut->GPSWeek, 2);   cnt += 2;	
	 memcpy(&SolBuff[cnt], &NavOut->GPSTime, 4);   cnt += 4;
	 memcpy(&SolBuff[cnt], &NavOut->NavModeInfo, 4);  cnt += 4;
	 memcpy(&SolBuff[cnt], &NavOut->Lat, 4);   cnt += 4;	 
	 memcpy(&SolBuff[cnt], &NavOut->Lon, 4);   cnt += 4;	
	 memcpy(&SolBuff[cnt], &NavOut->Alt, 4);   cnt += 4;
	 memcpy(&SolBuff[cnt], &NavOut->Msl, 4);  cnt += 4;
	 memcpy(&SolBuff[cnt], &NavOut->VelNED[0], 4);   cnt += 4;	 
	 memcpy(&SolBuff[cnt], &NavOut->VelNED[1], 4);   cnt += 4;		 
	 memcpy(&SolBuff[cnt], &NavOut->VelNED[2], 4);   cnt += 4;	 
	 memcpy(&SolBuff[cnt], &NavOut->GroundSpeed, 4);   cnt += 4;	
	 memcpy(&SolBuff[cnt], &NavOut->Distance, 4);   cnt += 4;
	 memcpy(&SolBuff[cnt], &NavOut->Roll, 4);   cnt += 4;	 
	 memcpy(&SolBuff[cnt], &NavOut->Pitch, 4);   cnt += 4;
	 memcpy(&SolBuff[cnt], &NavOut->Heading, 4);  cnt += 4;	 
	 memcpy(&SolBuff[cnt], &NavOut->PosErr_Unc[0], 2);   cnt += 2;	
	 memcpy(&SolBuff[cnt], &NavOut->PosErr_Unc[1], 2);   cnt += 2;	
	 memcpy(&SolBuff[cnt], &NavOut->PosErr_Unc[2], 2);   cnt += 2;	
	 memcpy(&SolBuff[cnt], &NavOut->VelErr_Unc[0], 2);   cnt += 2;	
	 memcpy(&SolBuff[cnt], &NavOut->VelErr_Unc[1], 2);   cnt += 2;	
	 memcpy(&SolBuff[cnt], &NavOut->VelErr_Unc[2], 2);   cnt += 2;	
	 memcpy(&SolBuff[cnt], &NavOut->AttErr_Unc[0], 2);   cnt += 2;	
	 memcpy(&SolBuff[cnt], &NavOut->AttErr_Unc[1], 2);   cnt += 2;	
	 memcpy(&SolBuff[cnt], &NavOut->AttErr_Unc[2], 2);   cnt += 2;	
	 memcpy(&SolBuff[cnt], &NavOut->MisAngle[0], 2);   cnt += 2;	
	 memcpy(&SolBuff[cnt], &NavOut->MisAngle[1], 2);   cnt += 2;	
	 memcpy(&SolBuff[cnt], &NavOut->MisAngle[2], 2);   cnt += 2;	
	 
	 /* get check sum */
   for ( i=0; i<cnt; i++ )
   {
     sum += (unsigned short)(SolBuff[i]);
   }
   
//	 checkSum[0] = (unsigned char)(sum & 0xFF);
//	 checkSum[1] = (unsigned char)((sum & 0xFF00) >> 8);
	 memcpy(checkSum, &sum, 2);
	 
	 /* binary protocol write */
	 syncChar[0] = PB_MSG_SYNC0;
	 syncChar[1] = PB_MSG_SYNC1;
	 memcpy(payloadLength, &cnt, 2);
	 
	 MsgIDSol = 0x01;

	 data_Output("%c", syncChar[0]);       /* sync header */
	 data_Output("%c", syncChar[1]);
	 data_Output("%c", MsgIDSol);          /* msg ID */
	 data_Output("%c", payloadLength[0]);  /* payload length */
	 data_Output("%c", payloadLength[1]);
	 
	 for(i=0; i<cnt; i++)                  /* payload */
	 {
	   data_Output("%c", SolBuff[i]); 
	 }

	 data_Output("%c", checkSum[0]);       /* check sum */
	 data_Output("%c", checkSum[1]);
	 data_Output("\n");
	 #endif
   
#else
   /* offline part */
   fprintf(fp_output, "%s", MsgBuff);
   fprintf(fp_output, "\n");	 
#endif

   return;
}


/*--- output message block end ---*/
void pb_SocketMsg_BlkEnd_Output(void)
{
   tOutput_NavStates NavOut;
   unsigned short week;
   double gpstime;
   unsigned int unixtime;
   char MsgBuff[50];
	
   memset(&NavOut, 0, sizeof(NavOut));
   pb_Query_OutputNavigationState(&NavOut);
   
   week     = NavOut.GPSWeek;
   gpstime  = (double)(NavOut.GPSTime * 0.001);

   unixtime = (unsigned int)pbMsg_ConvertGPSToUnixTime(week, gpstime);
	 
	 memset(MsgBuff, 0, sizeof(MsgBuff));
	 snprintf(MsgBuff, sizeof(MsgBuff), "$PBLKEND,255,%u", unixtime);
	 
#ifndef OFFLINE
   /* real time to serial port */
	 data_Output("%s",MsgBuff);
	 data_Output("\n");
#else
   /* offline part */
   fprintf(fp_output, "%s", MsgBuff);
   fprintf(fp_output, "\n");	 
#endif

   return;
}

#endif

/*-------- Level 2 message ---------*/
#ifdef PBMESSAGE_LEVEL_2

/* --- socket message PGNSS, 6 ---*/
void pb_SocketMsg_GNSS6_Output(tExternalEnrichedGNSSFixPtr gnssfix)
{
	 char MsgBuff[200];

   memset(MsgBuff, 0, sizeof(MsgBuff));
	 snprintf(MsgBuff, sizeof(MsgBuff), "$PGNSS,6,%hu,%hhu,%hu,%hhu,%hhu,%hhu,%hu,%u,%hhu,%d,%d,%hd,%hd,%d,%hu," 
		         "%hd,%hd,%hd,%d,%d,%hu,%hu,%hu,%hu,%hu,%hu",
            gnssfix->utc_Year, gnssfix->utc_Month, gnssfix->utc_Day,gnssfix->utc_Hour,gnssfix->utc_Minute,gnssfix->utc_Second,
            gnssfix->weekNum, gnssfix->TimeofWeek, gnssfix->numSats, gnssfix->Lat, gnssfix->Lon, gnssfix->Alt, gnssfix->Speed, gnssfix->Heading, gnssfix->Accuracy,
	          gnssfix->VelNED[0], gnssfix->VelNED[1], gnssfix->VelNED[2], gnssfix->ClkBias, gnssfix->ClkDrift,
            gnssfix->uncPosNED[0], gnssfix->uncPosNED[1], gnssfix->uncPosNED[2],
	          gnssfix->uncVelNED[0], gnssfix->uncVelNED[1], gnssfix->uncVelNED[2]);
#ifndef OFFLINE
   /* real time to serial port */
	 data_Output("%s",MsgBuff);
	 data_Output("\n");
#else
   /* offline part */
   fprintf(fp_output, "%s", MsgBuff);
   fprintf(fp_output, "\n");	 
#endif

   return;
}

void pb_SocketMsg_GNSS13_Output(tExternalGNSS_AuxMeasPtr auxChan)
{
   unsigned char chan;
   char MsgBuff[100];

	 /* real time to serial port */
   for (chan = 0; chan < MAX_GNSS_NUM_CHANS; chan++)
   {
      if(auxChan[chan].svid == 0 || auxChan[chan].Cn0 < 5)
         continue;
			
      memset(MsgBuff, 0, sizeof(MsgBuff));
      snprintf(MsgBuff, sizeof(MsgBuff), "$PGNSS,13,%hu,%u,%d,%hu,%hhu,%u,%hhd,%hd,%u", 
               auxChan[chan].svid, auxChan[chan].measTOW, auxChan[chan].prResidual, auxChan[chan].Cn0,
               auxChan[chan].svUsed, auxChan[chan].syncFlag, 
               auxChan[chan].elevation, auxChan[chan].azimuth, auxChan[chan].reserved);

#ifndef OFFLINE			
      data_Output("%s",MsgBuff);
      data_Output( "\n");
#else
      fprintf(fp_output, "%s", MsgBuff);
      fprintf(fp_output, "\n");	 
#endif		
			
   }
   return;
}

void pb_SocketMsg_SNSR21_Output(tExternalDRSensorPtr snsr)
{
   unsigned char i, j;
   int posLocation;
   char MsgBuff[640];

   memset(MsgBuff, 0, sizeof(MsgBuff));
	
   posLocation = snprintf(MsgBuff, sizeof(MsgBuff), "$PSNSR,21,%hu",snsr->numDataSets);   

   for (i = 0; i < snsr->numDataSets; i++)
   {
		posLocation += snprintf(&MsgBuff[posLocation], sizeof(MsgBuff) - posLocation, ",%u", snsr->data_set[i].timeTag);
      
      for (j = 0; j < MAX_DR_NUM_SNSR; j++)
         posLocation += snprintf(&MsgBuff[posLocation], sizeof(MsgBuff) - posLocation, ",%d", snsr->data_set[i].sensor[j]);
   }
	 
#ifndef OFFLINE	 
   data_Output("%s",MsgBuff);
   data_Output( "\n");
#else
   fprintf(fp_output, "%s", MsgBuff);
   fprintf(fp_output, "\n");	 
#endif

   return;
}

#endif   /* end of Level 2 message */


#ifdef PBMESSAGE_LEVEL_3
/* --- socket message PGNSS, 3 ---*/
void pb_SocketMsg_GNSS3_Output(tExternalGNSS_MeasPtr gnssMeas)
{
   unsigned char chan;
	 char MsgBuff[100];

   /* real time to serial port */
   for (chan = 0; chan < MAX_GNSS_NUM_CHANS; chan++)
   {
      if (gnssMeas[chan].svid == 0 || auxChan[chan].Cn0 < 5)
         continue;

			memset(MsgBuff, 0, sizeof(MsgBuff));
			
			snprintf(MsgBuff, sizeof(MsgBuff), "$PGNSS,3,%hu,%u,%.4f,%.4,%.4f,%hhd,%hhu,%hu,%u,%hu", 
               gnssMeas[chan].svid, gnssMeas[chan].measTOW, gnssMeas[chan].Pseudorange, gnssMeas[chan].DeltaRange,
               gnssMeas[chan].CarrierPhase, gnssMeas[chan].CycleSlip, gnssMeas[chan].HalfCycleStatus, 
               gnssMeas[chan].Cn0, gnssMeas[chan].MeasurmentState, gnssMeas[chan].reserved);
#ifndef OFFLINE	
      data_Output("%s",MsgBuff);
      data_Output( "\n");
#else
      fprintf(fp_output, "%s", MsgBuff);
      fprintf(fp_output, "\n");	 
#endif			
			
   }

   return;
}
#endif  /* end of level 3 message */


/*---------- a conversion tool -------- */
unsigned int pbMsg_ConvertGPSToUnixTime(unsigned short wnum, double tow)
{
#define LINUX_GPS_EPOCH_OFFSET 315964800
   /* tow in ms */
   double gpsTime = wnum * 604800 + tow;
   double unixTime;
   double  nleaps = 0;

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
