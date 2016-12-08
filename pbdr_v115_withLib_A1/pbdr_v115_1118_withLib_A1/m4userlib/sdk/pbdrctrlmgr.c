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
#include "print.h"
#include "hardware.h"
#include "bmi160_support.h"
#include "bmi160.h"

#include "drsensor_config.h"

#include "pbdrctrlmgr.h"
#include "public_struct.h"
#include "pb_gluelayer.h"
#include "pb_if.h"
#include "ubx.h"
#include "pb_message.h"

#define SENSOR_PRO_INTERVAL       10     /* in ms, minimum time interval for data proc, to shorten uart response delay */
#define SENSOR_DATA_MAXINTERVAL   6*SENSOR_PRO_INTERVAL    /* in uint of SENSOR_PRO_INTERVAL, max interval of BMI160 data frames                                  */
#define GNSS_DATA_RCVDELAY        50*SENSOR_PRO_INTERVAL    /* in uint of SENSOR_PRO_INTERVAL, timeout of GNSS data waiting                                        */
#define GNSS_EXCEED_BMI_TIME      20*SENSOR_PRO_INTERVAL    /* in uint of SENSOR_PRO_INTERVAL, max time allowed if GNSS data is ahead of BMI 160 processing        */

#define BMI_ONE_MISSEDSAMPLE_BOUNDARY    (u32)(1.5 * SNSRTIME_RESOLUTION + 0.5)    /* round up in ms */ 
#define BMI_TWO_MISSEDSAMPLE_BOUNDARY    (u32)(2.5 * SNSRTIME_RESOLUTION + 0.5) 
#define BMI_THREE_MISSEDSAMPLE_BOUNDARY  (u32)(3.5 * SNSRTIME_RESOLUTION + 0.5) 

static u8  pbDR_Sensor_ready;                   /* flag indicating bmi 160 data proc is complete, and ready for main algorithm call */
static u32 pbDR_PPS_SyncedSensorTimeTag;        /* sensor timetag that alignes with GPS PPS (PPS is used in CPU Timer calibration) */
 
/* a copy of sensor data for uart output purpose in case of conflicts between newly arriving bmi160 data and output task */
static tExternalDRSensor sensorOutputBuffer;   

static void Sensor_Conditioning_DownSampling(struct bmi160_data_t *inputraw,u8 dataNum);
static void pbDR_Sorting_DRData(tBMI160_DRSensorPtr pBmi160Data, tExternalDRSensorPtr pExtDRData );

/*********BMI160 define***/
#define BMI160_DATA_DEEP  20
 u32 g32BmI160LastTimer;
 u32  g32BmI160IntervalMax;
 u32  g32BmI160IntervalMin;
 u32  g32BmI160CostMax;
 u32  g32BmI160CostMin;
 struct bmi160_data_t    bmi160_data[BMI160_DATA_DEEP];
 u8 g8BmiWrPt;
 u8 g8BmiRdPt;
 /*****BMI160 define end********/

/*********handler define***/
u32 g32HandlerLastTimer;
 u32  g32HandlerIntervalMax;
 u32  g32HandlerIntervalMin;
 u32  g32HandlerCostMax;
 u32  g32HandlerCostMin;
	u8  rawBmiDataLostCnt;                   /* number of BMI 160 sample drops during 100ms */ 
	u8  rawBmi160Start;                      /* flag of sampling start for BMI160 since power on, to count number of missing data */
	u32 rawBmi160LastTimer;                  /* in ms, last epoch timetag for bmi 160 */ 
	u16 rawBmi160DelayCnt;                   /* unit in SENSOR_PRO_INTERVAL, timer of Bmi160 data between two adjacent epochs */
	u16 rawGnssGetDataCnt;                   /* counter starting from gnss data arrival to avoid old data consuming */
	u16 rawGnssWaitCnt;                      /* counter starting from valid BMI160, if timeoout, no wait for gnss and proceed with sensor only */
struct bmi160_data_t  rawBmiBuff[BMI_MAX_NUM_SNSRSAMPLE];        
u8  rawBmiWrPt;                          /* write pointer ofBMI160 raw data buff        */	
/*********handler define   end***/
void sensor_BMI160_init(void)
{
    g32BmI160IntervalMax = 0;
    g32BmI160IntervalMin = 0xffff;
    g32BmI160CostMax = 0;
    g32BmI160CostMin = 0xffff;
    g8BmiWrPt = 0;
    g8BmiRdPt = 0;
    bmi160_initialize_sensor();
}
 /*!
 *	@brief This function is get data from BMI160
 *	@param pvParameters  
 *	@return : null
 */
void getDataFromBMI160(void)  
{  
    u32 timerStartCnt,timerEndCnt,timerCnt1,timerCnt2;
	
       timerStartCnt = TIM_GetCounter(TIM5);   
       if(g32BmI160IntervalMin==0xffff)
       {
           g32BmI160LastTimer =  timerStartCnt;
           g32BmI160IntervalMin = 0xfffe;
       }
       else
       {
           if(timerStartCnt>g32BmI160LastTimer)
              timerCnt1 = timerStartCnt-g32BmI160LastTimer;
           else
              timerCnt1 = 0xffff-g32BmI160LastTimer+timerStartCnt;
           if(timerCnt1>g32BmI160IntervalMax)
               g32BmI160IntervalMax = timerCnt1;
           if(timerCnt1<g32BmI160IntervalMin)
               g32BmI160IntervalMin = timerCnt1;    
           g32BmI160LastTimer =  timerStartCnt;
               
       }
       
       bmi160_read_sensor_data(&bmi160_data[g8BmiWrPt]); 
	g8BmiWrPt = (g8BmiWrPt+1)%BMI160_DATA_DEEP;
       timerEndCnt = TIM_GetCounter(TIM5);
       if(timerEndCnt>timerStartCnt)
          timerCnt2 = timerEndCnt - timerStartCnt;
       else
          timerCnt2 = 0xffff-timerStartCnt+timerEndCnt;
       if(timerCnt2>g32BmI160CostMax)
          g32BmI160CostMax = timerCnt2;
       if(timerCnt2<g32BmI160CostMin)
          g32BmI160CostMin = timerCnt2;   
}  
 
/*!
 *	@brief This function is the data processing for BMI160
 *	@param pvParameters  
 *	@return : null
 */
void DataHandler_init(void)
{
    g32HandlerIntervalMax = 0;
    g32HandlerIntervalMin = 0xffff;
    g32HandlerCostMax = 0;
    g32HandlerCostMin = 0xffff;
	//memset(rawBmiBuff, 0, sizeof(rawBmiBuff));
	
	rawBmiDataLostCnt=0;
	rawBmi160Start =0;
	pbDR_Sensor_ready = FALSE;
	rawBmi160DelayCnt = 0;
	rawGnssGetDataCnt = 0;
	rawGnssWaitCnt = 0;
	rawBmiWrPt = 0;
	/* initialize all stuff */
	DRSystemOpen();	
}
void DataHandler_int(void)
{
	 /* counters for delay and timeout */
       if(Flag_Get_UbxDataReady() == TRUE)
       {
          rawGnssGetDataCnt+=10;
       }
	if(pbDR_Sensor_ready == TRUE)
          rawGnssWaitCnt+=10;
       else
          rawGnssWaitCnt = 0;

	rawBmi160DelayCnt+=10;
			
	/* bmi160 is not in working status or unexpected scheduling, reset */ 
	if(rawBmi160DelayCnt > SENSOR_DATA_MAXINTERVAL) /* 160 abnormal£¬reset */
	{
		//print("Bmi160 QUEUE read Fail!\n");
		rawBmiDataLostCnt = 0;
		rawBmi160Start =0;
		rawBmiWrPt = 0;
		rawBmi160DelayCnt = 0;
	}
}
void DataHandler_main(void)  
{
	u32  BmiTimeInterval;
	u8 u8temp;
	u32 timerStartCnt,timerEndCnt,timerCnt1,timerCnt2;

       timerStartCnt = TIM_GetCounter(TIM5);   
       if(g32HandlerIntervalMin==0xffff)
       {
           g32HandlerLastTimer =  timerStartCnt;
           g32HandlerIntervalMin = 0xfffe;
       }
       else
       {
           if(timerStartCnt>g32HandlerLastTimer)
              timerCnt1 = timerStartCnt-g32HandlerLastTimer;
           else
              timerCnt1 = 0xffff-g32HandlerLastTimer+timerStartCnt;
           if(timerCnt1>g32HandlerIntervalMax)
               g32HandlerIntervalMax = timerCnt1;
           if(timerCnt1<g32HandlerIntervalMin)
               g32HandlerIntervalMin = timerCnt1;    
           g32HandlerLastTimer =  timerStartCnt;
               
       }   
  		
       u8temp = 0;
  		/* check for gnss data availability */
  		Process_GpsDataInHandler();   // if new gnss data coming,decode
		
		/* if too much ahead of sensor ready epoch, not successful and reset gnss counter */
			if(rawGnssGetDataCnt > GNSS_EXCEED_BMI_TIME)
			{
				rawGnssGetDataCnt = 0;
				rawGnssWaitCnt = 0;
				Buff_clr_UbxData();
			}
		
		/* when the bmi 160 data is ready along with the acceptable format (defined by the gluelayer side), continously query uart for gnss data */		
		if(pbDR_Sensor_ready == TRUE) 
		{
			/* proceed if gnss on time or with positive delay */
			if((Flag_Get_UbxDataReady() == TRUE) || (rawGnssWaitCnt > GNSS_DATA_RCVDELAY)) 
			{
				if(Flag_Get_UbxDataReady() != TRUE)
                                {
                                    print("Gnss Data Not Ready!\n");
                                }
				pbDR_Push_DataBuffer();
				
        pbDR_Sensor_ready = FALSE;           /* we get all data needed, now clear flag and be ready for the next sensor cycle */

				/* main algorithm entrance */
				pb_DR_Main_external();

				/* queue of output series port */ 
				DataOutput();
				
				rawGnssWaitCnt    = 0;
				rawGnssGetDataCnt = 0;
				Buff_clr_UbxData();
			}
		}
		if(g8BmiWrPt!=g8BmiRdPt)
		{
		      
			rawBmiBuff[rawBmiWrPt].timeTamp = bmi160_data[g8BmiRdPt].timeTamp;
			rawBmiBuff[rawBmiWrPt].gyro.x= bmi160_data[g8BmiRdPt].gyro.x;
			rawBmiBuff[rawBmiWrPt].gyro.y= bmi160_data[g8BmiRdPt].gyro.y;
			rawBmiBuff[rawBmiWrPt].gyro.z= bmi160_data[g8BmiRdPt].gyro.z;
			rawBmiBuff[rawBmiWrPt].accel.x= bmi160_data[g8BmiRdPt].accel.x;
			rawBmiBuff[rawBmiWrPt].accel.y= bmi160_data[g8BmiRdPt].accel.y;
			rawBmiBuff[rawBmiWrPt].accel.z= bmi160_data[g8BmiRdPt].accel.z;
			g8BmiRdPt = (g8BmiRdPt+1)%BMI160_DATA_DEEP;
		
		    /* original timeTamp is in unsigned 32 bits, unitless 
		     * now convert that into ms, if not an integer ms value, then round up */
			rawBmiBuff[rawBmiWrPt].timeTamp = (u32)((double)(rawBmiBuff[rawBmiWrPt].timeTamp >> SNSRTIME_DATARATE_BIT) * 
			                                       SNSRTIME_RESOLUTION + 0.5);
		
			rawBmi160DelayCnt = 0;
			if(0 != rawBmiWrPt)
			{
				/* time interval to determined data missing */
				#if 1 
				if(rawBmiBuff[rawBmiWrPt].timeTamp > rawBmiBuff[rawBmiWrPt-1].timeTamp)
				{
					BmiTimeInterval = rawBmiBuff[rawBmiWrPt].timeTamp - rawBmiBuff[rawBmiWrPt-1].timeTamp;
				}
				else
				{
					BmiTimeInterval = SNSRTIME_ROLLOVER -rawBmiBuff[rawBmiWrPt-1].timeTamp+rawBmiBuff[rawBmiWrPt].timeTamp;    
				} 
				
				if(BmiTimeInterval >= BMI_THREE_MISSEDSAMPLE_BOUNDARY)
				{
					rawBmiDataLostCnt+=3;
					print("Bmi160  data lost Il!\n");  			
				}
				else if(BmiTimeInterval >= BMI_TWO_MISSEDSAMPLE_BOUNDARY)
				{
					rawBmiDataLostCnt+=2;
				}
				else if(BmiTimeInterval >= BMI_ONE_MISSEDSAMPLE_BOUNDARY)
				{
					rawBmiDataLostCnt++;
				}
				#endif
				rawBmi160LastTimer = rawBmiBuff[rawBmiWrPt].timeTamp;
				rawBmiWrPt++;           
				
				/* downsampling when 100ms samples are collected */
				if((rawBmiWrPt+rawBmiDataLostCnt) == BMI_NUMSAMPLE_PERCYCLE)  
				{
					if(rawBmiDataLostCnt>2)   
					{
						print("Bmi160  data lost IIl!\n");  //missed two frames
					}

					/* downsampling */
					Sensor_Conditioning_DownSampling(rawBmiBuff, rawBmiWrPt); 

					rawBmiDataLostCnt = 0;
					rawBmiWrPt = 0;
				}
				else if((rawBmiWrPt+rawBmiDataLostCnt) > BMI_NUMSAMPLE_PERCYCLE)  //  data lost between two 100ms interval
				{
				   print("Bmi160  data lost IIIl!\n");  
					
				   if(BmiTimeInterval >= BMI_THREE_MISSEDSAMPLE_BOUNDARY)
				   {
                      u8temp=3-(rawBmiWrPt+rawBmiDataLostCnt-BMI_NUMSAMPLE_PERCYCLE-1);
                      print("Bmi160  data lost I2!\n"); 		
				   }
				   else if(BmiTimeInterval >= BMI_TWO_MISSEDSAMPLE_BOUNDARY)
				   {
                      u8temp=2-(rawBmiWrPt+rawBmiDataLostCnt-BMI_NUMSAMPLE_PERCYCLE-1);
				   }
				   else if(BmiTimeInterval >= BMI_ONE_MISSEDSAMPLE_BOUNDARY)
                   {
                      u8temp=1;
				   }					

                   Sensor_Conditioning_DownSampling(rawBmiBuff,rawBmiWrPt-u8temp);
					
                   /* move next epoch 100ms data into the first frame of buff */
                   rawBmiDataLostCnt = rawBmiWrPt+rawBmiDataLostCnt-BMI_NUMSAMPLE_PERCYCLE-1;
                   memcpy(&rawBmiBuff[0], &rawBmiBuff[rawBmiWrPt-1], sizeof(struct bmi160_data_t));
										
                   rawBmiDataLostCnt = 0;
                   rawBmiWrPt = 0;
				}
			}  // end of 0 != rawBmiWrPt  
			else   //   first data of 10 samples block 
			{
				if(0!=rawBmi160Start)  
				{
                   /* sample's interval for data missing determination */
                   if(rawBmiBuff[rawBmiWrPt].timeTamp>rawBmi160LastTimer)
                   {
                      BmiTimeInterval = rawBmiBuff[rawBmiWrPt].timeTamp-rawBmi160LastTimer;
                   }
                   else
                   {
                      BmiTimeInterval = SNSRTIME_ROLLOVER -rawBmi160LastTimer+rawBmiBuff[rawBmiWrPt].timeTamp;
                   }
					
                   if(BmiTimeInterval >= BMI_THREE_MISSEDSAMPLE_BOUNDARY)
                   {
                      rawBmiDataLostCnt+=3;
                      print("Bmi160  data lost II2!\n");  //two continuous frames lost 	
                   }
                   else if(BmiTimeInterval >= BMI_TWO_MISSEDSAMPLE_BOUNDARY)
                   {
                      rawBmiDataLostCnt+=2;
                   }
                   else if(BmiTimeInterval >= BMI_ONE_MISSEDSAMPLE_BOUNDARY)
                   {
                      rawBmiDataLostCnt++;
                   }	

                   rawBmi160LastTimer = rawBmiBuff[rawBmiWrPt].timeTamp;
                   rawBmiWrPt++;
				}
				else
				{
					rawBmi160Start = 1;
					rawBmi160LastTimer = rawBmiBuff[rawBmiWrPt].timeTamp;
					rawBmiWrPt++;
				}
			}
		}
        timerEndCnt = TIM_GetCounter(TIM5);
       if(timerEndCnt>timerStartCnt)
          timerCnt2 = timerEndCnt-timerStartCnt;
       else
          timerCnt2 = 0xffff-timerStartCnt+timerEndCnt;
       if(timerCnt2>g32HandlerCostMax)
          g32HandlerCostMax = timerCnt2;
       if(timerCnt2<g32HandlerCostMin)
          g32HandlerCostMin = timerCnt2;   
}
/*!
 *	@brief This function is the data output
 *	@param pvParameters  
 *	@return : null
 */
void DataOutput(void)  
{
#ifdef PBMESSAGE_LEVEL_2
    tExternalEnrichedGNSSFixPtr pExternalGnssEnrichedFix;
    tExternalGNSS_AuxMeasPtr    pExternalAuxChan;    
    tExternalDRSensorPtr        pExternalDRData;
#endif

#ifdef PBMESSAGE_LEVEL_3
    tExternalGNSS_MeasPtr       ExternalGNSS_RawChan;
#endif
	
          /* NMEA and message level 1 output */
          pb_io_InfoOutput();
	 
#ifdef PBMESSAGE_LEVEL_2    /* for current DR project, only keep GNSS 1/6/13 and SNSR 21, whichever is avaliable */
          pExternalGnssEnrichedFix = Msg_Get_ExternalGnssEnrichedFix();
          pExternalAuxChan         = Msg_Get_ExternalGnssAuxMeas();
          pExternalDRData          = &sensorOutputBuffer;      /* load local copy sensor data instead of from gluelayer */

          pb_SocketMsg_GNSS6_Output(pExternalGnssEnrichedFix);
          pb_SocketMsg_GNSS13_Output(pExternalAuxChan);
          pb_SocketMsg_SNSR21_Output(pExternalDRData);
#endif

#ifdef PBMESSAGE_LEVEL_3
          /*--- for tightly coupled use ---*/
          ExternalGNSS_RawChan = Msg_Get_ExternalGNSS_RawChan();
          pb_SocketMsg_GNSS3_Output(ExternalGNSS_RawChan);
#endif

#ifdef PBMESSAGE_LEVEL_1
          pb_SocketMsg_BlkEnd_Output();                /* block end message should be at the very end */
#endif
       
}


/*!
 *	@brief This function is Sensor_DownSampling
 *	@param inputraw sawData;  
 *	@param dataNum data Num  
 *	@return : TURE:data ready
 */
static void Sensor_Conditioning_DownSampling(struct bmi160_data_t *inputraw, u8 dataNum)
{
	int sumValue[BMI_MAX_DR_NUM_SNSR]; 
	unsigned int timetag;
	unsigned short i;
	
	tBMI160_DRSensorPtr drset = Msg_Get_ExternalDRSensor();   
	
	memset(sumValue, 0, sizeof(sumValue));
  timetag = 0;	
	if(dataNum == 0)
		return;
	
	for(i=0; i<dataNum; i++)
	{
       timetag = (unsigned int)inputraw[i].timeTamp;      /* keep using the most recent timestamp as the group's timetag */ 
       sumValue[0] += (int)inputraw[i].gyro.x;     
       sumValue[1] += (int)inputraw[i].gyro.y;     
       sumValue[2] += (int)inputraw[i].gyro.z;     
       sumValue[3] += (int)inputraw[i].accel.x;     
       sumValue[4] += (int)inputraw[i].accel.y;     
       sumValue[5] += (int)inputraw[i].accel.z;     		
	}
	
    if(drset->numDataSets < MAX_BMI160_DATA_SETS)	      /* protection only */
	{
	   drset->data_set[drset->numDataSets].timeTag = (unsigned int)(timetag);      /* the latest sample's timetag as the downsampled timetag */
	
	   /*----- conditioning sensor data -----*/
	   /* gyro x */
	   drset->data_set[drset->numDataSets].sensor[0] = (int)((double)sumValue[0]/dataNum*BOSCH160_GYR_GYR_DPS_SENSITIVITY*1000 + 0.5);
	   /* gyro y */
	   drset->data_set[drset->numDataSets].sensor[1] = (int)((double)sumValue[1]/dataNum*BOSCH160_GYR_GYR_DPS_SENSITIVITY*1000 + 0.5);
	   /* gyro z */
	   drset->data_set[drset->numDataSets].sensor[2] = (int)((double)sumValue[2]/dataNum*BOSCH160_GYR_GYR_DPS_SENSITIVITY*1000 + 0.5);

	   /* acc x */
	   drset->data_set[drset->numDataSets].sensor[3] = (int)((double)sumValue[3]/dataNum*BOSCH160_ACC_SENSITIVITY*1000 + 0.5);
	   /* acc y */
	   drset->data_set[drset->numDataSets].sensor[4] = (int)((double)sumValue[4]/dataNum*BOSCH160_ACC_SENSITIVITY*1000 + 0.5);
	   /* acc z */
	   drset->data_set[drset->numDataSets].sensor[5] = (int)((double)sumValue[5]/dataNum*BOSCH160_ACC_SENSITIVITY*1000 + 0.5);
		
     drset->data_set[drset->numDataSets].sensor[6] = 0;    /* odo, reserved */
     drset->data_set[drset->numDataSets].sensor[7] = 0;    /* reversed, reserved */

	   drset->numDataSets++;
	}
	
	/* have 10 or 10+ samples in the sensor msg buffer, sensor data is ready for being consumed */
	if(drset->numDataSets >= BMI_NUMSAMPLE_PERSEC)
	{
		pbDR_Sensor_ready = TRUE;
    
		/* once PPS calibrates CPU Timer, the integer second is the real GPS measurement epoch (receiver internal signal processing aligns its measurement to the integer GPS Time)
		 * the epoch does not include GPS its own algorithm computation delay and data transter delay via uart 
		 * record this epoch's sensor timetag so as to extrapolate GPS into the right sensor time (synchronizaiton) where DR algorithm is triggered */
		
        if(drset->numDataSets == BMI_NUMSAMPLE_PERSEC)       
           pbDR_PPS_SyncedSensorTimeTag = drset->data_set[drset->numDataSets - 1].timeTag;
	}
	else
		pbDR_Sensor_ready = FALSE;
	
  return;
}

static void pbDR_Sorting_DRData(tBMI160_DRSensorPtr pBmi160Data, tExternalDRSensorPtr pExtDRData )
{
  unsigned short i, shift = 0;
	
	if(pBmi160Data->numDataSets <= BMI_NUMSAMPLE_PERSEC)   /* no more than 10 samples, accept all */
	{
	  pExtDRData->numDataSets = pBmi160Data->numDataSets;
      shift = 0;
	}                                            /* more than 10 samples, accept the most recent 10 records */
	else
	{
	  pExtDRData->numDataSets = BMI_NUMSAMPLE_PERSEC;
      shift = pBmi160Data->numDataSets - BMI_NUMSAMPLE_PERSEC;
	}
	
	for(i=0; i<pExtDRData->numDataSets; i++)
	{
	   pExtDRData->data_set[i].timeTag   = pBmi160Data->data_set[i+shift].timeTag;
       pExtDRData->data_set[i].sensor[0] = pBmi160Data->data_set[i+shift].sensor[0];
       pExtDRData->data_set[i].sensor[1] = pBmi160Data->data_set[i+shift].sensor[1];
       pExtDRData->data_set[i].sensor[2] = pBmi160Data->data_set[i+shift].sensor[2];
       pExtDRData->data_set[i].sensor[3] = pBmi160Data->data_set[i+shift].sensor[3];		
       pExtDRData->data_set[i].sensor[4] = pBmi160Data->data_set[i+shift].sensor[4];
       pExtDRData->data_set[i].sensor[5] = pBmi160Data->data_set[i+shift].sensor[5];
       pExtDRData->data_set[i].sensor[6] = pBmi160Data->data_set[i+shift].sensor[6];
       pExtDRData->data_set[i].sensor[7] = pBmi160Data->data_set[i+shift].sensor[7];		
	}
	
	return;
	 
}

void pbDR_Push_DataBuffer( void )
{
    unsigned short i;
	unsigned int   curSensorTimeTag;
	int            deltaT;
	
	tExternalDRSensor ExternalDRData;
	
	tBMI160_DRSensorPtr pMsg_bmi160Data = Msg_Get_ExternalDRSensor();
	
	tExternalEnrichedGNSSFixPtr pExternalGnssEnrichedFix = Msg_Get_ExternalGnssEnrichedFix();
	tExternalGNSS_AuxMeasPtr    pExternalAuxChan         = Msg_Get_ExternalGnssAuxMeas();
	
#ifdef BUILD_CONFIG_TIGHTCOUPLE
    tExternalGNSS_MeasPtr       ExternalGNSS_RawChan     = Msg_Get_ExternalGNSS_RawChan();
#endif	
	memset(&ExternalDRData, 0, sizeof(ExternalDRData));
	if(pMsg_bmi160Data->numDataSets > 0)
	{
		curSensorTimeTag = pMsg_bmi160Data->data_set[pMsg_bmi160Data->numDataSets-1].timeTag;     /* most freshed time */
		
		memset(&ExternalDRData, 0, sizeof(ExternalDRData));
    pbDR_Sorting_DRData(pMsg_bmi160Data, &ExternalDRData);		
		pb_Set_External_DR_SensrorData(&ExternalDRData);               /* set data into algorithm interface */
	}
	
	if(pExternalGnssEnrichedFix != NULL)
	{
       deltaT = (int)curSensorTimeTag - (int)pbDR_PPS_SyncedSensorTimeTag;

       if(deltaT < -SNSRTIME_ROLLOVER/2)
         deltaT += SNSRTIME_ROLLOVER;

       if(deltaT > 500 || deltaT < 0)          /* 500ms gap is too long, no allow of negative dt */
          deltaT = 0;
			 
	   pb_Set_ExternalGNSS_Enriched_Fix(pExternalGnssEnrichedFix);   /* set data into algorithm interface */
     pb_Extrapolate_ExternalGNSSFix((float)(deltaT/1000));         /* extrapolate GNSS (pos only) to the current sensor time */
	}
	
    if(pExternalAuxChan != NULL)
		{
		  pb_Set_ExternalGNSS_AuxChan(pExternalAuxChan);                /* set data into algorithm interface */
		}
	
#ifdef BUILD_CONFIG_TIGHTCOUPLE	
	if(ExternalGNSS_RawChan != NULL)
		 pb_Set_ExternalGNSS_Measurement(ExternalGNSS_RawChan);       /* set data into algorithm interface */
#endif
	
	 /* fill up output buffer for output purpose only; 
	  * Reason: downsampling logic depends on numDataSets, 
	  * if not reset, the newly coming bmi160 data (at higher rate) would impact TRUE/FALSE counter 
	  * if reset, zero values in pExternalDRData which cannot be used for output 
	  * Solution: leave them in another copy for output purpose only */
   
     memset(&sensorOutputBuffer, 0, sizeof(sensorOutputBuffer));
	 sensorOutputBuffer.numDataSets = ExternalDRData.numDataSets;

	 for(i=0; i<sensorOutputBuffer.numDataSets; i++)
	 {
	    memcpy(&sensorOutputBuffer.data_set[i], &ExternalDRData.data_set[i], sizeof(tExternalDRSensorData));   
	 }
	
   
     /* gluelayer DR data is consumed (i.e. pushed into interface), reset the buffer; 
	  * all gnss related get reset in gnss decoding logic, no action needed here   */
	 	
    memset(pMsg_bmi160Data, 0, sizeof(tBMI160_DRSensor));
	
	  if(!pb_SoftwareLicense())
		{
			 char msgbuf[100];
       memset(msgbuf, 0, sizeof(msgbuf));

       snprintf(msgbuf, sizeof(msgbuf), "Please contact Pinnobot FAE for the right Software License\n");
			 data_Output("%s", msgbuf);
		}
	 
	return;
}

void DRSystemOpen( void )
{
	tNVMData bbram;
	tSysTemPositionInitStruct sysInit;
	tOutput_NavStates outputstate;
	char msgbuf[100];
  memset(msgbuf, 0, sizeof(msgbuf));	
	
	/* display software version */
  snprintf(msgbuf, sizeof(msgbuf), "%s, %s\n", pb_FirmWare_Version_Info(), pb_Version_Info());
	data_Output("%s", msgbuf);	
	
	/* gluelayer variables init */
	pb_Gluelayer_External_Init();
	
	/* uart ubx receiving and decoding init */
	init_ubxraw();
       /* uart ubx cfg  */
	ublox_cfg_in_init();
	
	/* handle non-volatitle information */
	memset(&bbram, 0, sizeof(tNVMData));
    pb_Set_NvmData(&bbram);
	/* pb_Request_NvmData(&bbram);   this should be the first task when restart */
	
	/* public nav results variable init */
	memset(&outputstate, 0, sizeof(outputstate));
    pb_Set_OutputNavigationState(&outputstate);
	
	/* algorithm related init */
	memset(&sysInit, 0, sizeof(sysInit));       
	pb_System_Init(&sysInit);
	
	
	return;
}

char* pb_FirmWare_Version_Info(void)
{
   return (FIRMWAREVERSION);
}
