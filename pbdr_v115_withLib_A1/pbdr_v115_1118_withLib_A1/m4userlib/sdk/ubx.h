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

#ifndef __UBX_H__
#define __UBX_H__

#define MAXRAWLEN   (1 * 1024 + 32)         /* max length could be about RXM-RAWX (0x0215) 32*32+16+8 = 1048 (ref: page384) */

#define MINUBX(a,b)  (((a) < (b)) ? (a) : (b))
#define MAXUBX(a,b)  (((a) > (b)) ? (a) : (b))
#define ROUNDUBX(x)  (int)((x)+0.5)

#define P2_32       (2.328306436538696E-10) /* 2^-32 */
#define P2_10       (0.0009765625)          /* 2^-10 */

#define SPEED_LIGHT_UBX       (299792458.0)
#define L1_LENGTH_UBX         (0.1902936727983649)

#define GPS_SYS_ID     (0)
#define SBAS_SYS_ID    (1)
#define GALILEO_SYS_ID (2)
#define BDS_SYS_ID     (3)
#define IMES_SYS_ID    (4)
#define QZSS_SYS_ID    (5)
#define GLONASS_SYS_ID (6)

#define GLONASS_LOW_SVID   (64)
#define QZSS_LOW_SVID      (192)
#define GALILEO_LOW_SVID   (300)
#define BDS_LOW_SVID       (400)
#define IMES_LOW_SVID      (500)

#define UBXSYNC1    0xB5        /* ubx message sync code 1 */
#define UBXSYNC2    0x62        /* ubx message sync code 2 */

#define UBX_MSGSTART_SHIFT  (6) /* sync1 + sync2 + class + ID + 2 bytes payloadsize */

#define ID_NAVPVT   0x0107      /* ubx message id: nav PVT info */
#define ID_NAVCLK   0x0122      /* ubx message id: nav clock    */
#define ID_NAVCOV   0x0136      /* ubx message id: nav covariance matrix */
#define ID_NAVSAT   0x0135      /* ubx message id: nav satellite information */
#define ID_NAVGPSTIME  0x0120   /* ubx message id: nav gps time */

#define ID_RXMSFRBX 0x0213      /* ubx message id: raw subframe data */
#define ID_RXMRAWX  0x0215      /* ubx message id: multi-gnss raw meas data */

#define ID_TRKD5    0x030A      /* ubx message id: for chip, trace mesurement data */
#define ID_TRKMEAS  0x0310      /* ubx message id: for chip, trace mesurement data */

#define ID_ACKACK   0x0501    /* ubx message id: ACK  ACK */
#define ID_ACKNAK   0x0500    /* ubx message id: ACK  NAK */

#define GNSS_FIX_OK   (1<<0)

#define PPS_CAL_VALID_INTERVAL    (5)    /* every 5 second (current 1Hz GNSS), set the validation flag */

//#define CHIP_VERSION

// #pragma pack(push, 1)   --- need to confirm if its members directly after each other in memory, instead of default

typedef struct 
{
   unsigned char  buff[MAXRAWLEN];
   unsigned short nbyte;          /* number of bytes in message buffer */ 
   unsigned short length;
   unsigned char  frameReady;
} tUbxRawData;

typedef struct
{
   unsigned int   iTOW;
   unsigned short year;
   unsigned char  month;
   unsigned char  day;
   unsigned char  hour;
   unsigned char  min;
   unsigned char  sec;
   unsigned char  bitfield;   /* bit 0: UTC valid data; bit 1: UTC valid time; bit 2: UTC has no second uncertainty */

   unsigned int   timeUnc;    /* in ns */
   int            nano;       /* in ns */
   
   unsigned char  fixType;
   unsigned char  flags;
   unsigned char  flags2;
   unsigned char  numSV;

   int            Lon;        /* in deg, scale by 1.0e7 */
   int            Lat;        /* in deg, scale by 1.0e7 */
   int            Alt;        /* in mm */
   int            AltMsl;     /* in mm */
   unsigned int   hErr;       /* horizontal accuracy in mm */
   unsigned int   vErr;       /* vertical accuracy in mm */
   int            Vn;         /* in mm/s */
   int            Ve;
   int            Vd;
   int            groundSpeed;   /* in mm/s */
   int            heading;       /* in deg, scaled by 1.0e5 */
   unsigned int   spdErr;        /* in mm/s */
   unsigned int   headingErr;    /* in deg, scaled by 1.0e-5 */
   unsigned short PDOP;          /* scaled by 100 */ 
   unsigned char  reserved1[6];
   int            headVel;       /* in deg, scaled by 1.0e5, heading of vehicle (2D) */
   unsigned char  reserved2[4];
} UbxMsg01_07_PVT;

typedef struct
{
   unsigned int   iTOW;
   int            clkBias;       /* in ns */
   int            clkDrift;      /* in ns/s */
   unsigned int   unc_clkBias;   /* in ns */
   unsigned int   unc_clkDrift;  /* in ns/s */
} UbxMsg01_22_CLK;

typedef struct
{
   unsigned int   iTOW;
   unsigned char  version;
   unsigned char  posCovValid;
   unsigned char  velCovValid;
   unsigned char  reserved[9];
   float          covP11;
   float          covP12;
   float          covP13;
   float          covP22;
   float          covP23;
   float          covP33;
   float          covV11;
   float          covV12;
   float          covV13;
   float          covV22;
   float          covV23;
   float          covV33;
} UbxMsg01_36_COV;

// #pragma pack(pop)   --- need to confirm

typedef struct
{
   unsigned char PPSCnt;
	 unsigned char validforTimerCal;        /* use this field for timer calibration */
} UbxPPS_Validation, *UbxPPS_ValidationPtr;

/*!
 *	@brief This function is init gnss ram
 *	@param : null  
 *	@return : null
 */
void init_ubxraw(void);

/*!
 *	@brief This function is get the Flag of ubx raw data rcv&pro ready
 *	@param : TRUE:ready   FALSE: not ready  
 *	@return : null
 */
unsigned char Flag_Get_UbxDataReady(void);

/*!
 *	@brief This function is clear the Buff of ubx raw data 
 *	@param : null
 *	@return : null
 */
void Buff_clr_UbxData(void);

/*!
 *	@brief This function is receive ubxRawData from single byte (gpsUartRcvBuff)to a whole frame
 *	@param : null
 *	@return : 0: succed;others:error 
 */ 
int Receive_UbxData_byte(void);

/*!
 *	@brief This function is decode ubxRawData
 *	@param : null
 *	@return : 0: succed;others:error 
 */ 
int Decode_UbxData(void);


/*!
 *	@brief This function is cfg gnss   TXE interrupt mode
 *	@param : null  
 *	@return : null
 */
void ublox_cfg_in_init( void );


void ubx_setCheckSum(unsigned char *buff, int len);
int  ubx_ComputeCheckSum(unsigned char *buff, int len);

int generate_cfg(const unsigned char *msg, unsigned char *comdbuff, unsigned char msgLen);
unsigned char ubx_config_set(int m);

int sync_ubx(unsigned char *buff, unsigned char data);

int input_ubx(tUbxRawData *raw, unsigned char data);
int decode_ubx(tUbxRawData *raw);

/*--- decode NAV message ---*/
int decode_navpvt(tUbxRawData *raw);
int decode_navclock(tUbxRawData *raw);
int decode_navcov(tUbxRawData *raw);
int decode_navsat(tUbxRawData *raw);
int decode_navgpstime(tUbxRawData *raw);

/*--- decode raw channel message ---*/
#ifndef CHIP_VERSION
int decode_rxmrawx(tUbxRawData *raw);     /* only for module version */
#else
int decode_trkd5(tUbxRawData *raw);     /* only for chip version   */
#endif

//static int decode_rxmsfrbx(tUbxRawData *raw);

/*--------- push messages into public buffer --------------*/
void pb_Set_ExternalUbxGNSS_PVT(UbxMsg01_07_PVT *pvt);
void pb_Set_ExternalUbxGNSS_CLK(UbxMsg01_22_CLK *clk);
void pb_Set_ExternalUbxGNSS_COV(UbxMsg01_36_COV *cov);

/*--------- validate PPS -------------------*/
UbxPPS_ValidationPtr pb_Get_PPSValidation( void );    /* API for Timer usage */

void pb_Validate_Ubx_PPS(UbxMsg01_07_PVT *pvt);

void Receive_GpsData_byte(unsigned char data);
void Process_GpsDataInHandler(void);
#endif

