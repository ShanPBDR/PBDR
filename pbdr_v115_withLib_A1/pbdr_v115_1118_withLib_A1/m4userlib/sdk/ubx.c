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
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "ubx.h"
#include "hardware.h"
#include "public_struct.h"
#include "pb_gluelayer.h"
#include "print.h"

#define UBX_TYPE_MAX      6   // currently, we enabled 6 pieces of messages
#define UBX_ID_NAVPVT            0x0001
#define UBX_ID_NAVCOV            0x0002
#define UBX_ID_NAVSAT            0x0004
#define UBX_ID_NAVGPSTIME        0x0008
#define UBX_ID_RXMRAWX           0x0010    
#define UBX_ID_TRKD5             0x0020    /* not enabled */
#define UBX_ID_NAVCLK            0x0040

#define UBX_ID_ACKACK            0x0080
#define UBX_ID_ACKNAK            0x0100

#define UBX_MAX_NUMCHAN     (24)     /* ublox M8 32 channels */

#ifndef CHIP_VERSION
#define DECODE_UBX_MASK   (UBX_ID_NAVPVT|UBX_ID_NAVCLK|UBX_ID_NAVCOV|UBX_ID_NAVSAT|UBX_ID_NAVGPSTIME /*|UBX_ID_RXMRAWX*/)
#else
#define DECODE_UBX_MASK   (UBX_ID_NAVPVT|UBX_ID_NAVCLK|UBX_ID_NAVCOV|UBX_ID_NAVSAT|UBX_ID_NAVGPSTIME /*|UBX_ID_TRKD5*/)
#endif

#define UBX_CFG_ACK_TIME 100
static unsigned char  gpsUartRcvBuff[1024 + 32];
static unsigned int   gpsUartRcvBuff_WrPt;
static unsigned int   gpsUartRcvBuff_RdPt;
static tUbxRawData   ubxRawData;
static tUbxRawData   readyRawUbx;
static unsigned char ubxRawDataWrPt;
static unsigned char ubxRawDataRdPt;
static unsigned int ubxRawDataRcvStatus;  
static unsigned char  ubxRawDataReadyflag;  // 1:ready
static unsigned char ubxCfgAckStatus;    // UBX_ID_ACKACK:ack;UBX_ID_ACKNAK:Nak;0:Null

static unsigned char ubxCommandBuff[100];
static unsigned int   ubxCommandBuffWrPt;
static unsigned int   ubxCommandLenth;

static UbxPPS_Validation  validationPPS;

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((char *)(p)))

static unsigned short U2(unsigned char *p) {unsigned short u; memcpy(&u,p,2); return u;}
static unsigned int   U4(unsigned char *p) {unsigned int   u; memcpy(&u,p,4); return u;}
static int            I2(unsigned char *p) {short          u; memcpy(&u,p,2); return u;}
static int            I4(unsigned char *p) {int            u; memcpy(&u,p,4); return u;}
static float          R4(unsigned char *p) {float          r; memcpy(&r,p,4); return r;}
static double         R8(unsigned char *p) {double         r; memcpy(&r,p,8); return r;}

//static double         I8(unsigned char *p) {return I4(p+4)*4294967296.0+U4(p);}

/*----- protocol page 155 -----*/
void ubx_setCheckSum(unsigned char *buff, int len)
{
    unsigned short cka = 0, ckb = 0;
    int i;
    
    for (i=2; i<len-2; i++) 
    {
       cka += buff[i]; 
       ckb += cka;
    }

    buff[len-2]= (unsigned char)(cka & 0xFF);
    buff[len-1]= (unsigned char)(ckb & 0xFF);

    return;
}

int ubx_ComputeCheckSum(unsigned char *buff, int len)
{
   unsigned char cka = 0, ckb = 0;
   unsigned short ckA = 0, ckB = 0;
   int i;

   for (i = 2; i<len - 2; i++)
   {
      ckA += buff[i];
      ckB += ckA;
   }

   cka = (unsigned char)(ckA & 0xFF);
   ckb = (unsigned char)(ckB & 0xFF);

   return (cka == buff[len - 2]) && (ckb == buff[len - 1]);
}



/* ----------------- generate ublox config ----------- */
int generate_cfg(const unsigned char *msg, unsigned char *comdbuff, unsigned char msgLen)
{
   unsigned char i;
	 int n;

   comdbuff[0] = UBXSYNC1;
   comdbuff[1] = UBXSYNC2;

   for (i = 0; i < msgLen; i++)
   {
      comdbuff[i + 2] = msg[i];
   }

   n = (int)msgLen + 4;   /* sync1 + sync2 + check1 + check2*/

   ubx_setCheckSum(comdbuff, n);  /* start from class ID */

   return n;
}

/* ----------------- ublox config in init----------- */
/*!
 *	@brief This function is cfg gnss   TXE interrupt mode
 *	@param : null  
 *	@return : null
 */
void ublox_cfg_in_init( void )
{
   int m = 0;
	 
   //unsigned char msgCFG_PRT[24]      = { 6, 0, 20, 0, 1, 0, 0, 0, 192, 8, 0, 0, 0, 194, 1, 0, 7, 0, 2, 0, 0, 0, 0, 0 };  //115200
   unsigned char msgCFG_PRT[24]      = { 6, 0, 20, 0, 1, 0, 0, 0, 192, 8, 0, 0, 0, 194, 1, 0, 7, 0, 3, 0, 0, 0, 0, 0 };  //115200  ubx output
  //unsigned char msgCFG_PRT[24]      = { 6, 0, 20, 0, 1, 0, 0, 0, 192, 8, 0, 0, 128, 37, 0, 0, 7, 0, 3, 0, 0, 0, 0, 0 };  //9600
   unsigned char msgCFG_MSG_0107[12] = { 6, 1, 8, 0, 1, 7, 0, 1, 0, 0, 0, 0 };	   /* msg 0x01 0x07 on uart at 1Hz */
   unsigned char msgCFG_MSG_0122[12] = { 6, 1, 8, 0, 1, 34, 0, 1, 0, 0, 0, 0 };	   /* msg 0x01 0x22 on uart at 1Hz */
   unsigned char msgCFG_MSG_0136[12] = { 6, 1, 8, 0, 1, 54, 0, 1, 0, 0, 0, 0 };	   /* msg 0x01 0x36 on uart at 1Hz */
   unsigned char msgCFG_MSG_0135[12] = { 6, 1, 8, 0, 1, 53, 0, 1, 0, 0, 0, 0 };	   /* msg 0x01 0x35 on uart at 1Hz */
   unsigned char msgCFG_MSG_0120[12] = { 6, 1, 8, 0, 1, 32, 0, 1, 0, 0, 0, 0 };	   /* msg 0x01 0x20 on uart at 1Hz */
   unsigned char msgCFG_MSG_0215[12] = { 6, 1, 8, 0, 2, 21, 0, 1, 0, 0, 0, 0 };	   /* msg 0x02 0x15 on uart at 1Hz */
// unsigned char msgCFG_MSG_0213[12] = { 6, 1, 8, 0, 1, 19, 0, 1, 0, 0, 0, 0 };	 /* msg 0x02 0x20 on uart at 1Hz */
	 
   unsigned char msgCFG_GNSS[48] = { 6, 62, 44, 0, 0, 32, 32, 5,             /* 32 channels tracking and using ,5 constellation config */
	                                   0, 8, 16, 0, 1, 0, 1, 1,   /* enable GPS; config 8~16 channels for GPS*/
		                                 1, 1,  3, 0, 0, 0, 1, 1,   /* disable SBAS; config 1~3 channels for SBAS*/
		                                 3, 8, 16, 0, 0, 0, 1, 1,   /* disable BDS; config 8~16 channels for BDS*/
		                                 5, 0,  3, 0, 1, 0, 1, 1,   /* enable QZSS; config 0~3 channels for QZSS*/
		                                 6, 8, 16, 0, 1, 0, 1, 1 }; /* enable GLONASS; config 8~16 channels for GLONASS*/

   unsigned char msgCFG_NAV[40] = {6, 36, 36, 0, 255, 255, 4, 3,                               /* 4: automotive (7: drone 2g), 2D/3D */ 
	                                 0, 0, 0, 0, 22, 57, 0, 0, 5, 0, 250, 0, 250, 0,             /* elevation mask 5 deg, PDOP mask 25, TDOP mask 25 */
	                                 100, 0, 44, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};	 /* pos accuracy mask 100m, clock accuracy mask 300m */
	 
																	 /*-------- turn off some NMEA messages ---------*/
   unsigned char msgGGA_Off[12] = {6, 1, 8, 0, 240, 0, 0, 0, 0, 0, 0, 0};   /* turn off GGA, 0xF0 0x00 */
   unsigned char msgGLL_Off[12] = {6, 1, 8, 0, 240, 1, 0, 0, 0, 0, 0, 0};   /* turn off GLL, 0xF0 0x01 */
 //  unsigned char msgGSA_Off[12] = {6, 1, 8, 0, 240, 2, 0, 0, 0, 0, 0, 0};   /* turn off GSA, 0xF0 0x02 */
 //  unsigned char msgGSV_Off[12] = {6, 1, 8, 0, 240, 3, 0, 0, 0, 0, 0, 0};   /* turn off GSV, 0xF0 0x03 */	 
   unsigned char msgRMC_Off[12] = {6, 1, 8, 0, 240, 4, 0, 0, 0, 0, 0, 0};   /* turn off RMC, 0xF0 0x04 */
   unsigned char msgVTG_Off[12] = {6, 1, 8, 0, 240, 5, 0, 0, 0, 0, 0, 0};   /* turn off VTG, 0xF0 0x05 */
   unsigned char msgZDA_Off[12] = {6, 1, 8, 0, 240, 8, 0, 0, 0, 0, 0, 0};   /* turn off ZDA, 0xF0 0x08 */;
	 
   /*---- set port baudrate, page 235, suggest 115200 ----*/
   memset(ubxCommandBuff, 0, sizeof(ubxCommandBuff));
   m = generate_cfg(msgCFG_PRT, ubxCommandBuff, 24);
   ubxCfgAckStatus = 0;
   BSP_Uart3_SendBuff(m,ubxCommandBuff);

   delayMs(UBX_CFG_ACK_TIME);
   
   if((UBX_ID_ACKACK&ubxCfgAckStatus)==UBX_ID_ACKACK)
   {
      print("CfgGPps CFG_PRT ACK!");
   }
   else if((UBX_ID_ACKNAK&ubxCfgAckStatus)==UBX_ID_ACKNAK)
   {
      print("CfgGPps CFG_PRT NAK!");
   }
   else
   {
      print("CfgGPps CFG_PRT no resp!");
   }

   
   //usart3 set baudrate  115200 according to msgCFG_PRT
   BSP_Uart3_Init(115200); 
   delayMs(UBX_CFG_ACK_TIME);

   /*------ request  0x01 0x07 at 1Hz -------------- */
   memset(ubxCommandBuff, 0, sizeof(ubxCommandBuff));
   m = generate_cfg(msgCFG_MSG_0107, ubxCommandBuff, 12);
   if(m!=0)
   {
      ubxCfgAckStatus = 0;
      BSP_Uart3_SendBuff(m,ubxCommandBuff);
      delayMs(UBX_CFG_ACK_TIME);
   
      if((UBX_ID_ACKACK&ubxCfgAckStatus)==UBX_ID_ACKACK)
      {
         print("CfgGPps MSG_0107 ACK!");
      }
      else if((UBX_ID_ACKNAK&ubxCfgAckStatus)==UBX_ID_ACKNAK)
      {
         print("CfgGPps MSG_0107 NAK!");
      }
      else
      {
         print("CfgGPps MSG_0107 no resp!");
      }
   }
	 
   /**------  MSG 0x01 0x22-------------- */
   memset(ubxCommandBuff, 0, sizeof(ubxCommandBuff));
   m = generate_cfg(msgCFG_MSG_0122, ubxCommandBuff, 12);
   if(m!=0)
   {
      ubxCfgAckStatus = 0;
      BSP_Uart3_SendBuff(m,ubxCommandBuff);
      delayMs(UBX_CFG_ACK_TIME);
   
      if((UBX_ID_ACKACK&ubxCfgAckStatus)==UBX_ID_ACKACK)
      {
         print("CfgGPps MSG_0122 ACK!");
      }
      else if((UBX_ID_ACKNAK&ubxCfgAckStatus)==UBX_ID_ACKNAK)
      {
        print("CfgGPps MSG_0122 NAK!");
      }
      else
      {
         print("CfgGPps MSG_0122 no resp!");
      }
   }
 
	 /**------ MSG  0x01 0x36-------------- */
   memset(ubxCommandBuff, 0, sizeof(ubxCommandBuff));
   m = generate_cfg(msgCFG_MSG_0136, ubxCommandBuff, 12);
   if(m!=0)
   {
      ubxCfgAckStatus = 0;
      BSP_Uart3_SendBuff(m,ubxCommandBuff);
      delayMs(UBX_CFG_ACK_TIME);
   
      if((UBX_ID_ACKACK&ubxCfgAckStatus)==UBX_ID_ACKACK)
      {
         print("CfgGPps MSG_0136 ACK!");
      }
      else if((UBX_ID_ACKNAK&ubxCfgAckStatus)==UBX_ID_ACKNAK)
      {
         print("CfgGPps MSG_0136 NAK!");
      }
      else
   	  {
         print("CfgGPps MSG_0136 no resp!");
   	  }
   }
	 
  /**------ MSG  0x01 0x35-------------- */
   memset(ubxCommandBuff, 0, sizeof(ubxCommandBuff));
   m = generate_cfg(msgCFG_MSG_0135, ubxCommandBuff, 12);
   if(m!=0)
   {
         ubxCfgAckStatus = 0;
   	BSP_Uart3_SendBuff(m,ubxCommandBuff);
   	delayMs(UBX_CFG_ACK_TIME);
   
   	if((UBX_ID_ACKACK&ubxCfgAckStatus)==UBX_ID_ACKACK)
   	{
   		print("CfgGPps MSG_0135 ACK!");
   	}
   	else if((UBX_ID_ACKNAK&ubxCfgAckStatus)==UBX_ID_ACKNAK)
   	{
		print("CfgGPps MSG_0135 NAK!");
   	}
   	else
   	{
   		print("CfgGPps MSG_0135 no resp!");
   	}
   }
 
	 /**------ MSG  0x01 0x20-------------- */
   memset(ubxCommandBuff, 0, sizeof(ubxCommandBuff));
   m = generate_cfg(msgCFG_MSG_0120, ubxCommandBuff, 12);
   if(m!=0)
   {
 	ubxCfgAckStatus = 0;
   	BSP_Uart3_SendBuff(m,ubxCommandBuff);
   	delayMs(UBX_CFG_ACK_TIME);
   
   	if((UBX_ID_ACKACK&ubxCfgAckStatus)==UBX_ID_ACKACK)
   	{
   		print("CfgGPps MSG_0120 ACK!");
   	}
   	else if((UBX_ID_ACKNAK&ubxCfgAckStatus)==UBX_ID_ACKNAK)
   	{
		print("CfgGPps MSG_0120 NAK!");
   	}
   	else
   	{
   		print("CfgGPps MSG_0120 no resp!");
   	}
   }
 
	 /**------ MSG  0x02 0x15 (raw measurements)-------------- */
	 //(void)msgCFG_MSG_0215;
   memset(ubxCommandBuff, 0, sizeof(ubxCommandBuff));
   m = generate_cfg(msgCFG_MSG_0215, ubxCommandBuff, 12);
   if(m!=0)
   {
	 ubxCfgAckStatus = 0;
   	BSP_Uart3_SendBuff(m,ubxCommandBuff);
   	delayMs(UBX_CFG_ACK_TIME);
   
   	if((UBX_ID_ACKACK&ubxCfgAckStatus)==UBX_ID_ACKACK)
   	{
   		print("CfgGPps MSG_0215 ACK!");
   	}
   	else if((UBX_ID_ACKNAK&ubxCfgAckStatus)==UBX_ID_ACKNAK)
   	{
		print("CfgGPps MSG_0215 NAK!");
   	}
   	else
   	{
   		print("CfgGPps MSG_0215 no resp!");
   	}
   }
	 
	 	 /**------ MSG  0x02 0x13 (eph frames) not required at the current phase ------------- */
//   memset(ubxCommandBuff, 0, sizeof(ubxCommandBuff));
//   m = generate_cfg(msgCFG_MSG_0213, ubxCommandBuff, 12);
//	 if(m!=0)
//   {
//		 ubx_config_set(m);
//   }
	 
	 /*--------- config constellations ---------*/
   memset(ubxCommandBuff, 0, sizeof(ubxCommandBuff));
   m = generate_cfg(msgCFG_GNSS, ubxCommandBuff, 48);
   if(m!=0)
   {
	 ubxCfgAckStatus = 0;
   	BSP_Uart3_SendBuff(m,ubxCommandBuff);
   	delayMs(UBX_CFG_ACK_TIME);
   
   	if((UBX_ID_ACKACK&ubxCfgAckStatus)==UBX_ID_ACKACK)
   	{
   		print("CfgGPps MSG_GNSS ACK!");
   	}
   	else if((UBX_ID_ACKNAK&ubxCfgAckStatus)==UBX_ID_ACKNAK)
   	{
		print("CfgGPps MSG_GNSS NAK!");
   	}
   	else
   	{
   		print("CfgGPps MSG_GNSS no resp!");
   	}
   } 

	 
  /*--------- config GNSS Nav engine / dynamic range; ---------*/
   memset(ubxCommandBuff, 0, sizeof(ubxCommandBuff));
   m = generate_cfg(msgCFG_NAV, ubxCommandBuff, 40);
   if(m!=0)
   {
	 ubxCfgAckStatus = 0;
   	BSP_Uart3_SendBuff(m,ubxCommandBuff);
   	delayMs(UBX_CFG_ACK_TIME);
   
   	if((UBX_ID_ACKACK&ubxCfgAckStatus)==UBX_ID_ACKACK)
   	{
   		print("CfgGPps MSG_NAV ACK!");
   	}
   	else if((UBX_ID_ACKNAK&ubxCfgAckStatus)==UBX_ID_ACKNAK)
   	{
		print("CfgGPps MSG_NAV NAK!");
   	}
   	else
   	{
   		print("CfgGPps MSG_NAV no resp!");
   	}
   } 

   /*------- disable some of NMEA to save throughput ------*/
   memset(ubxCommandBuff, 0, sizeof(ubxCommandBuff));
   
   m = generate_cfg(msgGGA_Off, ubxCommandBuff, 12);
    ubxCfgAckStatus = 0;
   	BSP_Uart3_SendBuff(m,ubxCommandBuff);
   	delayMs(UBX_CFG_ACK_TIME);
   
   	if((UBX_ID_ACKACK&ubxCfgAckStatus)==UBX_ID_ACKACK)
   	{
   		print("CfgGPps MSG_GGA_Off ACK!");
   	}
   	else if((UBX_ID_ACKNAK&ubxCfgAckStatus)==UBX_ID_ACKNAK)
   	{
		print("CfgGPps MSG_GGA_Off NAK!");
   	}
   	else
   	{
   		print("CfgGPps MSG_GGA_Off no resp!");
   	}
   m = generate_cfg(msgGLL_Off, ubxCommandBuff, 12);
    ubxCfgAckStatus = 0;
   	BSP_Uart3_SendBuff(m,ubxCommandBuff);
   	delayMs(UBX_CFG_ACK_TIME);
   
   	if((UBX_ID_ACKACK&ubxCfgAckStatus)==UBX_ID_ACKACK)
   	{
   		print("CfgGPps MSG_GLL_Off ACK!");
   	}
   	else if((UBX_ID_ACKNAK&ubxCfgAckStatus)==UBX_ID_ACKNAK)
   	{
		print("CfgGPps MSG_GLL_Off NAK!");
   	}
   	else
   	{
   		print("CfgGPps MSG_GLL_Off no resp!");
   	}
   m = generate_cfg(msgRMC_Off, ubxCommandBuff, 12);
    ubxCfgAckStatus = 0;
   	BSP_Uart3_SendBuff(m,ubxCommandBuff);
   	delayMs(UBX_CFG_ACK_TIME);
   
   	if((UBX_ID_ACKACK&ubxCfgAckStatus)==UBX_ID_ACKACK)
   	{
   		print("CfgGPps MSG_RMC_Off ACK!");
   	}
   	else if((UBX_ID_ACKNAK&ubxCfgAckStatus)==UBX_ID_ACKNAK)
   	{
		print("CfgGPps MSG_RMC_Off NAK!");
   	}
   	else
   	{
   		print("CfgGPps MSG_RMC_Off no resp!");
   	}
   m = generate_cfg(msgVTG_Off, ubxCommandBuff, 12);
    ubxCfgAckStatus = 0;
   	BSP_Uart3_SendBuff(m,ubxCommandBuff);
   	delayMs(UBX_CFG_ACK_TIME);
   
   	if((UBX_ID_ACKACK&ubxCfgAckStatus)==UBX_ID_ACKACK)
   	{
   		print("CfgGPps MSG_VTG_Off ACK!");
   	}
   	else if((UBX_ID_ACKNAK&ubxCfgAckStatus)==UBX_ID_ACKNAK)
   	{
		print("CfgGPps MSG_VTG_Off NAK!");
   	}
   	else
   	{
   		print("CfgGPps MSG_VTG_Off no resp!");
   	}
   m = generate_cfg(msgZDA_Off, ubxCommandBuff, 12);
    ubxCfgAckStatus = 0;
   	BSP_Uart3_SendBuff(m,ubxCommandBuff);
   	delayMs(UBX_CFG_ACK_TIME);
   
   	if((UBX_ID_ACKACK&ubxCfgAckStatus)==UBX_ID_ACKACK)
   	{
   		print("CfgGPps MSG_ZDA_Off ACK!");
   	}
   	else if((UBX_ID_ACKNAK&ubxCfgAckStatus)==UBX_ID_ACKNAK)
   	{
		print("CfgGPps MSG_ZDA_Off NAK!");
   	}
   	else
   	{
   		print("CfgGPps MSG_ZDA_Off no resp!");
   	}
   return;
}
/*!
 *	@brief This function is start to sending  ublox cfg though usart1,TXE interrupt mode 
 *	@param : sendData: the value need to send  
 *	@return : 1:send over ;0:keep sending 
 */

int uart_send_cfgCmd(unsigned char *sendData)
{
	
	*sendData = ubxCommandBuff[ubxCommandBuffWrPt++];
	if(ubxCommandLenth == ubxCommandBuffWrPt)
	{
		return 1;
	}
	return 0;
}

/*------------------- now decode received messages ------------------*/
/*!
 *	@brief This function is init gnss ram
 *	@param : null  
 *	@return : null
 */
void init_ubxraw(void)
{
   gpsUartRcvBuff_WrPt = 0;
   gpsUartRcvBuff_RdPt = 0;
   ubxRawDataWrPt = 0;
   ubxRawDataRdPt = 0;
   ubxRawDataRcvStatus = 0;
   ubxRawDataReadyflag = FALSE;

   memset(&ubxRawData, 0, sizeof(ubxRawData));	 
   ubxRawData.frameReady = FALSE;
	
	 memset(&readyRawUbx, 0, sizeof(readyRawUbx));
	
   memset(&validationPPS, 0, sizeof(validationPPS));
	 
   return;
}
/*!
 *	@brief This function is get the Flag of ubx raw data rcv&pro ready
 *	@param : TRUE:ready   FALSE: not ready  
 *	@return : null
 */
unsigned char Flag_Get_UbxDataReady(void)
{
	return(ubxRawDataReadyflag);
}
/*!
 *	@brief This function is clear the Buff of ubx raw data 
 *	@param : null
 *	@return : null
 */
void Buff_clr_UbxData(void)
{
   ubxRawDataWrPt = 0;
   ubxRawDataRdPt = 0;
   ubxRawDataRcvStatus = 0;
   ubxRawDataReadyflag = FALSE;

   memset(&ubxRawData, 0, sizeof(ubxRawData));	

   return;
}

/*!
 *	@brief This function is receive ubxRawData from single byte to a whole frame
 *	@param : data: receive byte from gps
 *	@return : 0: succed;others:error 
 */ 
void Receive_GpsData_byte(unsigned char data)
{
    gpsUartRcvBuff[gpsUartRcvBuff_WrPt] = data;
    gpsUartRcvBuff_WrPt = (gpsUartRcvBuff_WrPt+1)%1024;
}

void Process_GpsDataInHandler(void)
{
  int loop;
  if(gpsUartRcvBuff_WrPt==gpsUartRcvBuff_RdPt)
  {
    return;
  }
  loop=0;
  do
  {
    input_ubx(&ubxRawData,gpsUartRcvBuff[gpsUartRcvBuff_RdPt]);
    gpsUartRcvBuff_RdPt = (gpsUartRcvBuff_RdPt+1)%1024;
    loop++;
    if(ubxRawData.frameReady == TRUE)
    {
      break;
    }
  }while((loop<100)&&(gpsUartRcvBuff_WrPt!=gpsUartRcvBuff_RdPt));
  Receive_UbxData_byte();
  Decode_UbxData();   // if new gnss data coming,decode
  if(ubxRawData.frameReady == TRUE)
  {
      memset(&ubxRawData, 0, sizeof(ubxRawData));
      ubxRawData.frameReady = FALSE;
  }
  
}
int Receive_UbxData_byte(void)
{
	int rtn=FALSE;
	int type;
	if(ubxRawData.frameReady == TRUE)  // one frame ready
	{
       //if ACK message, deal right now
       type = ubxRawData.buff[2];
       type = (type<<8) + ubxRawData.buff[3];
		
       if((type!=ID_ACKACK)&&(type!=ID_ACKNAK))
       {
          ubxRawDataWrPt = (ubxRawDataWrPt+1)%UBX_TYPE_MAX;
       }
       else  // process right now
       {
         if(type==ID_ACKACK)
         {
           ubxCfgAckStatus |= UBX_ID_ACKACK;
         }
         else
         {
            ubxCfgAckStatus |= UBX_ID_ACKNAK;
         }
       }
       rtn = TRUE;
    }
    return(rtn);
}

/*!
 *	@brief This function is decode ubxRawData
 *	@param : null
 *	@return : 0: succed;others:error 
 */ 
int Decode_UbxData(void)
{
	int  rtn;
	int type; 

//	if(ubxRawDataWrPt == ubxRawDataRdPt)  
//		return -1;
	rtn = 0;
	if(ubxRawData.frameReady!=TRUE)
		return -1;
	
	if(decode_ubx(&readyRawUbx)==0)
	{
	    type = readyRawUbx.buff[2];
	    type = (type<<8) + readyRawUbx.buff[3];
	    switch(type)
	    {
           case ID_NAVPVT:
               ubxRawDataRcvStatus |= UBX_ID_NAVPVT;
	        break;
           case ID_NAVCLK:
               ubxRawDataRcvStatus |= UBX_ID_NAVCLK;
	        break;
           case ID_NAVCOV:
               ubxRawDataRcvStatus |= UBX_ID_NAVCOV;
	        break;
           case ID_NAVSAT:
               ubxRawDataRcvStatus |= UBX_ID_NAVSAT;
	        break;
           case ID_NAVGPSTIME :
               ubxRawDataRcvStatus |= UBX_ID_NAVGPSTIME;
	        break;
           case ID_RXMRAWX : 
               ubxRawDataRcvStatus |= UBX_ID_RXMRAWX;
	        break;
           case ID_TRKD5   : 
               ubxRawDataRcvStatus |= UBX_ID_TRKD5;
	        break;
	     default:
		 break;
	    }
	    
        if((ubxRawDataRcvStatus&DECODE_UBX_MASK)==DECODE_UBX_MASK)
	    {
           ubxRawDataReadyflag = TRUE;
	    }
	}
	
	ubxRawDataRdPt = (ubxRawDataRdPt+1)%UBX_TYPE_MAX;
	
	return(rtn);
}


/*----- sync header --------*/
int sync_ubx(unsigned char *buff, unsigned char data)
{
    buff[0] = buff[1]; 
    buff[1] = data;
    return ((buff[0]==UBXSYNC1) && (buff[1]==UBXSYNC2));
}

/*--- input_ubx is supposed to be called from tread ---*/
int input_ubx(tUbxRawData *raw, unsigned char data)
{
    /* synchronize frame */
    if (raw->nbyte == 0) 
    {
        if (!sync_ubx(raw->buff, data)) 
           return 0;
        
        raw->nbyte=2;
        
        return 0;        /* wait for new bytes join in */
    }

    raw->buff[raw->nbyte++]=data;
    
    if (raw->nbyte == UBX_MSGSTART_SHIFT)     
    {
       raw->length = U2(raw->buff+4) + 8; 
       if (raw->length > MAXRAWLEN) 
       {
          raw->nbyte = 0;
          return -1;
       }
    }

    if((raw->nbyte < UBX_MSGSTART_SHIFT) || (raw->nbyte < raw->length))
       return 0;             /* return until length is satisfied */
    
    raw->nbyte = 0;
    raw->frameReady = TRUE;
		
		memcpy(&readyRawUbx, raw, sizeof(tUbxRawData));
    return 0;
}


/* decode ublox raw message --------------------------------------------------*/
int decode_ubx(tUbxRawData *raw)
{
    int type = (U1(raw->buff+2)<<8) + U1(raw->buff+3);
    
    /* checksum */
    if (!ubx_ComputeCheckSum(raw->buff, raw->length))
        return -1;
		
    switch (type) 
    {
       case ID_NAVPVT:
          return decode_navpvt(raw);
       case ID_NAVCLK:
          return decode_navclock(raw);
       case ID_NAVCOV:
          return decode_navcov(raw);
       case ID_NAVSAT:
          return decode_navsat(raw);
       case ID_NAVGPSTIME :
          return decode_navgpstime(raw);

//#ifndef CHIP_VERSION
//       case ID_RXMRAWX : 
//          return decode_rxmrawx(raw);
//#else
//        case ID_TRKD5   : 
//           return decode_trkd5(raw);        /* for tightly couple or DGNSS/RTK */
//#endif
        //case ID_RXMSFRBX: 
        //   return decode_rxmsfrbx(raw);    /* for tightly couple or DGNSS/RTK */
    }

    return -1;
}

/* decode ubx-nav-pvt: navigation pvt -----------------------------------*/
int decode_navpvt(tUbxRawData *raw)
{
    UbxMsg01_07_PVT pvt;
    unsigned char *p = raw->buff + UBX_MSGSTART_SHIFT;

    memset(&pvt, 0, sizeof(pvt));

    pvt.iTOW  = U4(p);
    pvt.year  = U2(p+4);
    pvt.month = U1(p+6);
    pvt.day   = U1(p+7);
    pvt.hour  = U1(p+8);
    pvt.min   = U1(p+9);
    pvt.sec   = U1(p+10);

    pvt.bitfield = U1(p+11);
    pvt.timeUnc  = U4(p+12);
    pvt.nano     = I4(p+16);

    pvt.fixType  = U1(p+20);
    pvt.flags    = U1(p+21);
    pvt.flags2   = U1(p+22);
    pvt.numSV    = U1(p+23);

    pvt.Lon      = I4(p+24);
    pvt.Lat      = I4(p+28);
    pvt.Alt      = I4(p+32);
    pvt.AltMsl   = I4(p+36);

    pvt.hErr     = U4(p+40);
    pvt.vErr     = U4(p+44);

    pvt.Vn       = I4(p+48);
    pvt.Ve       = I4(p+52);
    pvt.Vd       = I4(p+56);

    pvt.groundSpeed = I4(p+60);
    pvt.heading     = I4(p+64);
    pvt.spdErr      = U4(p+68);
    pvt.headingErr  = U4(p+72);

    pvt.PDOP        = U2(p+76);
    
    memcpy(pvt.reserved1, p+78, 6*sizeof(unsigned char));

    pvt.headVel     = I4(p+84);

    memcpy(pvt.reserved2, p+88, 4*sizeof(unsigned char));

    /* fill up interface for real time processing */
    pb_Set_ExternalUbxGNSS_PVT(&pvt);

    /*----- validate PPS -----*/
    pb_Validate_Ubx_PPS(&pvt);
	
    return 0;
}

/* decode ubx-nav-clk: navigation clock -----------------------------------*/
int decode_navclock(tUbxRawData *raw)
{
   UbxMsg01_22_CLK clk;
   unsigned char *p = raw->buff + UBX_MSGSTART_SHIFT;

   memset(&clk, 0, sizeof(clk));

   clk.iTOW         = U4(p);
   clk.clkBias      = I4(p + 4);
   clk.clkDrift     = I4(p + 8);
   clk.unc_clkBias  = U4(p + 12);
   clk.unc_clkDrift = U4(p + 16);

   /* fill up interface for real time processing */
   pb_Set_ExternalUbxGNSS_CLK(&clk);

   return 0;
}

/* decode ubx-nav-cov: navigation covariance matrix -----------------------------------*/
int decode_navcov(tUbxRawData *raw)
{
   UbxMsg01_36_COV cov;
   unsigned char *p = raw->buff + UBX_MSGSTART_SHIFT;

   memset(&cov, 0, sizeof(cov));

   cov.iTOW        = U4(p);
   cov.version     = U1(p + 4);
   cov.posCovValid = U1(p + 5); 
   cov.velCovValid = U1(p + 6);
   memcpy(cov.reserved, p + 7, 9 * sizeof(unsigned char));
   cov.covP11      = MAXUBX(0, R4(p + 16)); 
   cov.covP12      = MAXUBX(0, R4(p + 20));
   cov.covP13      = MAXUBX(0, R4(p + 24));
   cov.covP22      = MAXUBX(0, R4(p + 28));
   cov.covP23      = MAXUBX(0, R4(p + 32));
   cov.covP33      = MAXUBX(0, R4(p + 36));
   cov.covV11      = MAXUBX(0, R4(p + 40));
   cov.covV12      = MAXUBX(0, R4(p + 44));
   cov.covV13      = MAXUBX(0, R4(p + 48));
   cov.covV22      = MAXUBX(0, R4(p + 52));
   cov.covV23      = MAXUBX(0, R4(p + 56));
   cov.covV33      = MAXUBX(0, R4(p + 60));

   /* fill up interface for real time processing */
   pb_Set_ExternalUbxGNSS_COV(&cov);

   return 0;
}

/* decode ubx-nav-sat: navigation channel satellite information ---------------------*/
int decode_navsat(tUbxRawData *raw)
{
   unsigned int  tow, bitmask;
   short nsat, i;
	 short j;
   unsigned char gnssID, flag1;
	 tExternalGNSS_AuxMeas navauxmeas[UBX_MAX_NUMCHAN];    /* ubx raw channels up to 32, only pick up 20 with stronger signals */
	 tExternalGNSS_AuxMeas tempauxmeas;
	 tExternalGNSS_AuxMeasPtr    readyAuxMeas = Msg_Get_ExternalGnssAuxMeas();
	 unsigned char *p = raw->buff + UBX_MSGSTART_SHIFT;
	
   memset(navauxmeas, 0, UBX_MAX_NUMCHAN*sizeof(tExternalGNSS_AuxMeas));	
	 memset(&tempauxmeas, 0, sizeof(tExternalGNSS_AuxMeas));
	
	 memset(readyAuxMeas, 0, MAX_GNSS_NUM_CHANS * sizeof(tExternalGNSS_AuxMeas));

   tow  = U4(p);
   nsat = (short)U1(p + 5);
   nsat = MINUBX(nsat, UBX_MAX_NUMCHAN);   
	
   if(nsat == 0)
      return 0;

   for (i = 0, p += 8; i < nsat; i++, p += 12)
   {
      navauxmeas[i].measTOW = (unsigned int)(tow);           /* same software time for all channels */

      gnssID = (unsigned char)(U1(p));

      navauxmeas[i].svid = (unsigned short)(U1(p + 1));

      /* gnssID: 0 -- GPS, keep the same ID, 1 -- SBAS, keep the same ID */
      if (gnssID == GALILEO_SYS_ID)                     /* Galileo  */
         navauxmeas[i].svid += GALILEO_LOW_SVID;
      else if (gnssID == BDS_SYS_ID)                    /* BDS */
         navauxmeas[i].svid += BDS_LOW_SVID;
      else if (gnssID == GLONASS_SYS_ID)                /* GLONASS */
         navauxmeas[i].svid += GLONASS_LOW_SVID;           /* 65 ~ 96 */
      else if (gnssID == IMES_SYS_ID)
         navauxmeas[i].svid += IMES_LOW_SVID;              /* IMES */
      else if (gnssID == QZSS_SYS_ID)                   /* QZSS */
         navauxmeas[i].svid += QZSS_LOW_SVID;

      navauxmeas[i].Cn0 = (unsigned short)(U1(p + 2) * 10);

      navauxmeas[i].elevation = I1(p + 3);
      navauxmeas[i].azimuth   = (short)(I2(p + 4));

      navauxmeas[i].prResidual = (int)(I2(p + 6) * 10);   /* in cm */

      bitmask = U4(p + 8);

      flag1 = (unsigned char)(bitmask & 0x00000007);

      if (flag1 <= 2)
         navauxmeas[i].syncFlag = UBXGNSS_MEASUREMENT_STATE_UNKNOWN;

      if (flag1 >= 4)
      {
         navauxmeas[i].syncFlag = (UBXGNSS_MEASUREMENT_STATE_CODE_LOCK | UBXGNSS_MEASUREMENT_STATE_BIT_SYNC |
            UBXGNSS_MEASUREMENT_STATE_SUBFRAME_SYNC | UBXGNSS_MEASUREMENT_STATE_TOW_DECODED);

         if (flag1 >= 6)
            navauxmeas[i].syncFlag |= UBXGNSS_MEASUREMENT_STATE_CARRIER_PULLIN;
      }

      flag1 = (unsigned char)(bitmask & 0x00000008);
      if (flag1 == 0x08)
         navauxmeas[i].svUsed = 1;
      else
         navauxmeas[i].svUsed = 0;
   }
	 
	 /*------ sort channels in ascending order according to signal level -----*/
   for (j = nsat - 1; j > 0; j--)
   {
      for (i = 0; i < j; i++)
      {
         if ( navauxmeas[i+1].Cn0 > navauxmeas[i].Cn0 )   /* sorting by signal strength */
         {
            memcpy(&tempauxmeas, &navauxmeas[i], sizeof(tExternalGNSS_AuxMeas));
						memcpy(&navauxmeas[i], &navauxmeas[i+1], sizeof(tExternalGNSS_AuxMeas));
						memcpy(&navauxmeas[i+1], &tempauxmeas, sizeof(tExternalGNSS_AuxMeas));
         }
      }
   }	 
	 
	 /* get the first 20 (or less) channels, and pass to gluelayer */
	 nsat = MINUBX(nsat, MAX_GNSS_NUM_CHANS);  
	 for (i = 0; i < nsat; i++)
	 {
	    memcpy(&readyAuxMeas[i], &navauxmeas[i], sizeof(tExternalGNSS_AuxMeas));
	 }
	 
   return 0;
}

int decode_navgpstime(tUbxRawData *raw)
{
   unsigned char *p = raw->buff + UBX_MSGSTART_SHIFT;
   tExternalGNSSGPSTimePtr gpstime = Msg_Get_ExternalGnssGpsTime();

   /* do not reset this struct due to week and leapsecond */
   gpstime->iTow = U4(p);
   gpstime->fTow = I4(p+4);
   gpstime->gpsWeek = (short)(I2(p + 8));
   gpstime->leapSec = (short)(I1(p + 10));

   return 0; 
}

#ifndef CHIP_VERSION
/* decode ubx-rxm-rawx: multi-gnss raw measurement data -----------*/
int decode_rxmrawx(tUbxRawData *raw)
{
   double tow;
   short i, j, nsat;
   unsigned char gnssID, trackingStatus, halfv, halfc[UBX_MAX_NUMCHAN], tempHalfc;
   double pr, dr, cp;
   unsigned short lockt[UBX_MAX_NUMCHAN], tempLockt, week;
   unsigned char *p = raw->buff + 6;
   tExternalGNSS_Meas    rawchan[UBX_MAX_NUMCHAN];           /* 32 channels */
   tExternalGNSS_Meas    temprawchan;
   tGnssChanCycleSlipPtr lastCycleSlip = Msg_Get_CycleSlip();
	
   tExternalGNSS_MeasPtr readyRawChan  = Msg_Get_ExternalGNSS_RawChan();   /* 20 channels */

   memset(rawchan, 0, UBX_MAX_NUMCHAN*sizeof(tExternalGNSS_Meas));
   memset(&temprawchan, 0, sizeof(tExternalGNSS_Meas));
	 
   memset(readyRawChan, 0, MAX_GNSS_NUM_CHANS*sizeof(tExternalGNSS_Meas));
	
   memset(halfc, 0, sizeof(halfc));
   memset(lockt, 0, sizeof(lockt));

   tow = R8(p);
   week = U2(p + 8); 

   if (week == 0)
      return 0;

   nsat = (short)U1(p + 11);
   nsat = MINUBX(nsat, UBX_MAX_NUMCHAN);   /* up to 32 channels */
	 
   if(nsat == 0)
      return -1;

   for (i = 0, p += 16; i < nsat; i++, p += 32)
   {
      rawchan[i].measTOW = (unsigned int)(tow * 1000 + 0.5);           /* same software time for all channels */

      gnssID = (unsigned char)(U1(p + 20));

      rawchan[i].svid = (unsigned short)(U1(p + 21));

      /* gnssID: 0 -- GPS, keep the same ID, 1 -- SBAS, keep the same ID */
      if (gnssID == GALILEO_SYS_ID)                     /* Galileo  */
         rawchan[i].svid += GALILEO_LOW_SVID;
      else if (gnssID == BDS_SYS_ID)                    /* BDS */
         rawchan[i].svid += BDS_LOW_SVID;
      else if (gnssID == GLONASS_SYS_ID)                /* GLONASS */
         rawchan[i].svid += GLONASS_LOW_SVID;           /* 65 ~ 96 */
      else if (gnssID == IMES_SYS_ID)
         rawchan[i].svid += IMES_LOW_SVID;              /* IMES */
      else if (gnssID == QZSS_SYS_ID)                   /* QZSS */
         rawchan[i].svid += QZSS_LOW_SVID;

      trackingStatus = U1(p + 30); /* tracking status */

      pr = trackingStatus & 1 ? R8(p) : 0.0;
      cp = trackingStatus & 2 ? R8(p + 8) : 0.0;         /* in L1 cycles */

      dr = R4(p + 16);     /* in Hz */

      if (cp == -0.5) cp = 0.0; /* invalid phase */

      if (trackingStatus & 0x01)
      {
         rawchan[i].MeasurmentState = (unsigned int)(UBXGNSS_MEASUREMENT_STATE_CODE_LOCK | UBXGNSS_MEASUREMENT_STATE_BIT_SYNC |
            UBXGNSS_MEASUREMENT_STATE_SUBFRAME_SYNC | UBXGNSS_MEASUREMENT_STATE_TOW_DECODED);
         if (trackingStatus & 0x02)
            rawchan[i].MeasurmentState |= UBXGNSS_MEASUREMENT_STATE_CARRIER_PULLIN;
      }
      else
         rawchan[i].MeasurmentState = 0;

      rawchan[i].Pseudorange = (double)pr;                            /* in m */
      rawchan[i].CarrierPhase = (double)(cp * L1_LENGTH_UBX);          /* in m */
      rawchan[i].DeltaRange = (double)(dr * L1_LENGTH_UBX);          /* in m/s */

      rawchan[i].Cn0 = (unsigned short)(U1(p + 26) * 10);

      halfv = trackingStatus & 4 ? 1 : 0;        /* half cycle valid */
      halfc[i] = trackingStatus & 8 ? 1 : 0;        /* half cycle subtracted from phase */

      rawchan[i].HalfCycleStatus |= ((halfv ? 0 : 1) << 0);     /* bit 0, if set, half cycle invalid */

      lockt[i] = U2(p + 24);    /* lock time count (ms) */
   } // end of i
	 
	 
	 /*------ sort channels in ascending order according to signal level -----*/
   for (j = nsat - 1; j > 0; j--)
   {
      for (i = 0; i < j; i++)
      {
         if ( rawchan[i+1].Cn0 > rawchan[i].Cn0 )   /* sorting by signal strength */
         {
            memcpy(&temprawchan, &rawchan[i], sizeof(tExternalGNSS_Meas));
						memcpy(&rawchan[i],  &rawchan[i+1], sizeof(tExternalGNSS_Meas));
						memcpy(&rawchan[i+1], &temprawchan, sizeof(tExternalGNSS_Meas));
					 
            memcpy(&tempHalfc, &halfc[i], sizeof(unsigned char));
						memcpy(&halfc[i],  &halfc[i+1], sizeof(unsigned char));
						memcpy(&halfc[i+1], &tempHalfc, sizeof(unsigned char));			

					  memcpy(&tempLockt, &lockt[i], sizeof(unsigned short));
						memcpy(&lockt[i],  &lockt[i+1], sizeof(unsigned short));
						memcpy(&lockt[i+1], &tempLockt, sizeof(unsigned short));		
         }
      }
   }
	 
	 /* get the first 20 (or less) channels, and pass to gluelayer */
	 nsat = MINUBX(nsat, MAX_GNSS_NUM_CHANS);  
	 for (i = 0; i < nsat; i++)
	 {
        memcpy(&readyRawChan[i], &rawchan[i], sizeof(tExternalGNSS_Meas));
		 
        /*----------- flag if cycle slip happens ----------- */
        readyRawChan[i].CycleSlip = 1;
        for (j = 0; j < MAX_GNSS_NUM_CHANS; j++)
        {
           if (readyRawChan[i].svid == lastCycleSlip[j].svid)
           {
              if (lastCycleSlip[j].locktime > 0 && lastCycleSlip[j].locktime <= lockt[i])
                 readyRawChan[i].CycleSlip = 0;   /* no cycle slip happened */
              else
                 readyRawChan[i].CycleSlip = 1;

              if (readyRawChan[i].CarrierPhase != 0.0)
              {
                 readyRawChan[i].HalfCycleStatus |= (((halfc[i] != lastCycleSlip[j].halfcyclesubtracted) ? 1 : 0) << 1);
              }

              break;
           }
        } // end of j		 
	 }  // end of i	 

   for (i = 0; i<nsat; i++)    /* update cycleslip variable */
   {
      lastCycleSlip[i].svid = readyRawChan[i].svid;
      lastCycleSlip[i].locktime = (unsigned short)(lockt[i]);
      lastCycleSlip[i].halfcyclesubtracted = halfc[i];
   }

   return 1;
}

#else

/* decode ubx-trkd5: track multi-GNSS measurement data ---------------------------*/
int decode_trkd5(tUbxRawData *raw)
{
//   double ts, tr = -1.0, t, tau, adr, dop, snr;
//   int i, type, off, len, qi, flag;
//   unsigned char gnssID;
//   unsigned char *p = raw->buff + 6;

//   tExternalGNSS_MeasPtr rawchan = Msg_Get_ExternalGNSS_RawChan();
//   tGnssChanCycleSlipPtr lastCycleSlip = Msg_Get_CycleSlip();
//   tExternalGNSSGPSTimePtr gpstime = Msg_Get_ExternalGnssGpsTime();

//   memset(rawchan, 0, MAX_UBXGNSS_NUM_CHAN*sizeof(tExternalGNSS_Meas));

//   switch ((type = U1(p))) 
//   {
//      case 3: 
//         off = 86; 
//         len = 56; 
//         break;
//      case 6: 
//         off = 86; 
//         len = 64; 
//         break;
//      default: 
//         off = 78; 
//         len = 56; 
//         break;
//   }

//   for (i = 0, p = raw->buff + off; p - raw->buff<raw->length - 2; i++, p += len) 
//   {
//      if (U1(p + 41) < 4)
//         continue;

//      t = I8(p)*P2_32 / 1000.0;

//      gnssID = U1(p + 56);
//      
//      if (gnssID == GLONASS_SYS_ID)
//         t -= 10800.0 + gpstime->leapSec;
//      
//      if (t>tr) 
//         tr = t;
//   }

//   if (tr<0.0) 
//      return 0;

//   tr = ROUNDUBX((tr + 0.08) / 0.1)*0.1;

//   /* adjust week handover */
//   if (tr < (t - 302400.0))
//   {
//      gpstime->gpsWeek--;
//      tr += MSGSECONDS_PER_WEEK;
//   }
//   else if (tr >(t + 302400.0))
//   {
//      gpstime->gpsWeek++;
//      tr -= MSGSECONDS_PER_WEEK;
//   }


//   for (i = 0, p = raw->buff + off; p - raw->buff < raw->length - 2; i++, p += len)
//   {

//      /* quality indicator */
//      qi = U1(p + 41) & 7;
//      if (qi < 4 || 7 < qi) continue;

//      /* transmission time */
//      ts = I8(p)*P2_32 / 1000.0;
//      if (type == 6)
//         ts -= 10800.0 + gpstime->leapSec; /* glot -> gpst */

//                                                    /* signal travel time */
//      tau = tr - ts;
//      if (tau<-302400.0)
//         tau += 604800.0;
//      else if (tau> 302400.0)
//         tau -= 604800.0;

//      flag = U1(p + 54);   /* tracking status */
//      adr = qi < 6 ? 0.0 : I8(p + 8)*P2_32 + (flag & 0x01 ? 0.5 : 0.0);
//      dop = I4(p + 16)*P2_10 / 4.0;
//      snr = U2(p + 32) / 256.0;


//      rawchan[i].Pseudorange = tau * SPEED_LIGHT_UBX;
//      rawchan[i].CarrierPhase = -adr * L1_LENGTH_UBX;
//      rawchan[i].DeltaRange = dop * L1_LENGTH_UBX;

//      /*---- not completed yet ---*/

//   }

   return 1;
}
#endif

//static int decode_rxmsfrbx(tUbxRawData *raw)
//{
//   /* to be completed */
//   return 0;
//}

/*--------- push messages into public buffer --------------*/
void pb_Set_ExternalUbxGNSS_PVT(UbxMsg01_07_PVT *pvt)
{
   tExternalEnrichedGNSSFixPtr  gnssEnrichFix = Msg_Get_ExternalGnssEnrichedFix();
   double tow = 0;
   unsigned short week;

   memset(gnssEnrichFix, 0, sizeof(tExternalEnrichedGNSSFix));

   gnssEnrichFix->utc_Year   =  (unsigned short)pvt->year;
   gnssEnrichFix->utc_Month  =  (unsigned char)pvt->month;
   gnssEnrichFix->utc_Day    =  (unsigned short)pvt->day;
   gnssEnrichFix->utc_Hour   =  (unsigned char)pvt->hour;
   gnssEnrichFix->utc_Minute =  (unsigned char)pvt->min;
   gnssEnrichFix->utc_Second =  (unsigned char)pvt->sec;

   gnssEnrichFix->TimeofWeek =  (unsigned int)pvt->iTOW;
   
   Msg_UTC2GpsTime((unsigned short)pvt->year, (unsigned char)pvt->month, (unsigned short)pvt->day, (unsigned char)pvt->hour, 
                   (unsigned char)pvt->min, (unsigned char)pvt->sec, &week, &tow);

   gnssEnrichFix->weekNum =  week;

   if((pvt->flags & GNSS_FIX_OK) == GNSS_FIX_OK)
      gnssEnrichFix->numSats =  (unsigned char)pvt->numSV;

   gnssEnrichFix->Lat = (int)pvt->Lat;
   gnssEnrichFix->Lon = (int)pvt->Lon;
   gnssEnrichFix->Alt = (short)(pvt->Alt*0.1+0.5);
   gnssEnrichFix->Speed    = (short)(pvt->groundSpeed*0.1+0.5);
   gnssEnrichFix->Heading  = (int)(pvt->heading*1.0e-5*100);
   gnssEnrichFix->Accuracy = (unsigned short)(pvt->hErr*0.1+0.5);

   gnssEnrichFix->VelNED[0] = (short)(pvt->Vn * 0.1 + 0.5);
   gnssEnrichFix->VelNED[1] = (short)(pvt->Ve * 0.1 + 0.5);
   gnssEnrichFix->VelNED[2] = (short)(pvt->Vd * 0.1 + 0.5);

   gnssEnrichFix->ClkBias  = 0;
   gnssEnrichFix->ClkDrift = 0;

   return;
}

void pb_Set_ExternalUbxGNSS_CLK(UbxMsg01_22_CLK *clk)
{
   tExternalEnrichedGNSSFixPtr  gnssEnrichFix = Msg_Get_ExternalGnssEnrichedFix();

   gnssEnrichFix->ClkBias  = (int)((double)clk->clkBias * 1.0e-9 * SPEED_LIGHT_UBX * 100 + 0.5);
   gnssEnrichFix->ClkDrift = (int)((double)clk->clkDrift * 1.0e-9 * SPEED_LIGHT_UBX * 100 + 0.5);

   return;
}

void pb_Set_ExternalUbxGNSS_COV(UbxMsg01_36_COV *cov)
{
   tExternalEnrichedGNSSFixPtr  gnssEnrichFix = Msg_Get_ExternalGnssEnrichedFix();

   gnssEnrichFix->uncPosNED[0] = (unsigned short)(sqrt(cov->covP11) * 100 + 0.5);
   gnssEnrichFix->uncPosNED[1] = (unsigned short)(sqrt(cov->covP22) * 100 + 0.5);
   gnssEnrichFix->uncPosNED[2] = (unsigned short)(sqrt(cov->covP33) * 100 + 0.5);

   gnssEnrichFix->uncVelNED[0] = (unsigned short)(sqrt(cov->covV11) * 100 + 0.5);
   gnssEnrichFix->uncVelNED[1] = (unsigned short)(sqrt(cov->covV22) * 100 + 0.5);
   gnssEnrichFix->uncVelNED[2] = (unsigned short)(sqrt(cov->covV33) * 100 + 0.5);

   return;
}

UbxPPS_ValidationPtr pb_Get_PPSValidation( void )
{
   UbxPPS_ValidationPtr ppsvalid = &validationPPS;
	
	 return (ppsvalid);
}


void pb_Validate_Ubx_PPS(UbxMsg01_07_PVT *pvt)
{
	 UbxPPS_ValidationPtr ppsvalid = pb_Get_PPSValidation();
	
	 if((pvt->numSV >= 5) && (pvt->hErr < 10000))    /* horizontal accuracy within 10m, note: hErr in mm */
	 {
		 ppsvalid->PPSCnt++;
		 
		 /* rollover by 100 */
		 if(ppsvalid->PPSCnt > 100)
			 ppsvalid->PPSCnt -= 100;
	 }
	 else
		 ppsvalid->PPSCnt = 0;
	 	 
	 if(ppsvalid->PPSCnt % PPS_CAL_VALID_INTERVAL == 0)
		 ppsvalid->validforTimerCal = 1;
	 else
		 ppsvalid->validforTimerCal = 0;
		 
   return;
}
