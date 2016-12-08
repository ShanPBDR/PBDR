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

#ifndef __PB_IF_H__
#define __PB_IF_H__

#include "public_struct.h"

void pb_System_Init( tSysTemPositionInitStruct *sysinit );
void pb_DR_Main_external(void);
void pb_DR_Main(void);
char* pb_Version_Info(void);

unsigned char pb_SoftwareLicense(void);

/*----- set functions ------- */
void pb_Extrapolate_ExternalGNSSFix(float deltaTime);
void pb_Set_ExternalGNSS_Enriched_Fix(tExternalEnrichedGNSSFixPtr pData);
#ifdef BUILD_CONFIG_TIGHTCOUPLE
void pb_Set_ExternalGNSS_Measurement(tExternalGNSS_MeasPtr pData);
void pb_Set_ExternalGNSS_SVState(tExternalGNSS_SVStatePtr pData);
#endif
void pb_Set_ExternalGNSS_AuxChan(tExternalGNSS_AuxMeasPtr pData);

void pb_Set_External_DR_SensrorData(tExternalDRSensorPtr pData);

/*----- set unix time -----*/
void pb_Set_UnixTime(tExternalUnixTimePtr pData);

/*-------------------*/
eMOD_STATUS pb_Query_OutputNavigationState(tOutput_NavStatesPtr pdata);
eMOD_STATUS pb_Set_OutputNavigationState(tOutput_NavStatesPtr pdata);

eMOD_STATUS pb_Set_NvmData(tNVMDataPtr pdata);
eMOD_STATUS pb_Get_NvmData(tNVMDataPtr pdata);

#endif

/**@**/
