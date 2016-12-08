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

#ifndef __PUBLIC_STRUCT_H__
#define __PUBLIC_STRUCT_H__

#define BUILD_CONFIG_LOOSECOUPLE  
/*#define BUILD_CONFIG_TIGHTCOUPLE */

#define MAX_GNSS_NUM_CHANS (20)

/* raw sensor data size, it can be changed if sensor is in higher rate */
#define MAX_DR_NUM_SNSR     (8)          /* 3 gyros, 3 accels, 1 odo, 1 reverse */
#define MAX_DR_DATA_SETS   (11)

/* svid (not PRN) definition can be varied in different receiver */
#define  GPS_SVID_LOW         (1)
#define  GPS_SVID_HIGH        (32)

#define UBX_SVID_NUMBERING

#ifndef UBX_SVID_NUMBERING
#define  GLONASS_SVID_LOW     (70)
#define  GLONASS_SVID_HIGH    (83)

#define  BDS_GEO_SVID_LOW     (150)
#define  BDS_GEO_SVID_HIGH    (154)
#define  BDS_IGSO_SVID_LOW    (155)
#define  BDS_IGSO_SVID_HIGH   (159)
#define  BDS_MEO_SVID_LOW     (38)
#define  BDS_MEO_SVID_HIGH    (64)

#define  GAL_SVID_LOW         (205)
#define  GAL_SVID_HIGH        (254) 

#define  QZSS_SVID_LOW        (193)
#define  QZSS_SVID_HIGH       (197)

#define  SBAS_SVID_LOW        (120)
#define  SBAS_SVID_HIGH       (138)

#else
#define  GLONASS_SVID_LOW     (65)
#define  GLONASS_SVID_HIGH    (96)

#define  BDS_GEO_SVID_LOW     (401)
#define  BDS_GEO_SVID_HIGH    (405)
#define  BDS_IGSO_SVID_LOW    (406)
#define  BDS_IGSO_SVID_HIGH   (437)
#define  BDS_MEO_SVID_LOW     (406)
#define  BDS_MEO_SVID_HIGH    (437)

#define  GAL_SVID_LOW         (211)
#define  GAL_SVID_HIGH        (246) 

#define  QZSS_SVID_LOW        (193)
#define  QZSS_SVID_HIGH       (197)

#define  SBAS_SVID_LOW        (120)
#define  SBAS_SVID_HIGH       (158)
#endif

typedef enum
{
	eMOD_FAILURE = 0,
	eMOD_SUCCESS
} eMOD_STATUS;

typedef enum     /* position source to initialize nav */
{
   eROMPOS,                          /* from ROM        */
   eSRAMPOS,                         /* from SRAM       */
   eWIFIPOS                         /* from Wi-Fi      */
} tSysTemPositionInitSource;

typedef struct     /* position source to initialize nav */
{
   int llh[3];               /* lat/lon in degree, scaled by 1.0e^7; alt in cm */
   tSysTemPositionInitSource Source;
} tSysTemPositionInitStruct;

/* ----- part 1: GNSS struct ----- */
typedef struct
{
	unsigned short utc_Year;
	unsigned char  utc_Month;
	unsigned short utc_Day;
	unsigned char  utc_Hour;
	unsigned char  utc_Minute;
	unsigned char  utc_Second;
   unsigned short weekNum;
   unsigned int   TimeofWeek;
	unsigned char  numSats;
	int            Lat;           /* in degree, scaled by 10^7 */
	int            Lon;           /* in degree, scaled by 10^7 */
	short          Alt;           /* in centi-meter, ellipsoid altitude */
	short          Speed;         /* in cm/s */
	int            Heading;       /* in degree, 0~ 360, scaled by 100 */
	unsigned short Accuracy;      /* in centi-meter, position accuracy */
   short          VelNED[3];     /* in cm/s, NED velocity */
   int            ClkBias;       /* in cm */
   int            ClkDrift;      /* in cm/s */
   unsigned short uncPosNED[3];  /* in cm, NED position uncertainty */
   unsigned short uncVelNED[3];  /* in cm/s, NED velocity uncertainty */
} tExternalEnrichedGNSSFix, *tExternalEnrichedGNSSFixPtr;

typedef struct
{
   unsigned int iTow;           /* integer part of TOW in ms */
   int          fTow;           /* fractional part of TOW in ns */
   short        gpsWeek;
   short        leapSec;
} tExternalGNSSGPSTime, *tExternalGNSSGPSTimePtr;

typedef struct
{
   unsigned short  svid;
   unsigned int    measTOW;           /* measurement time of week in msec*/
   double          Pseudorange;       /* pseudorange measurement in m */
   double          DeltaRange;        /* Doppler measurement in m/s */
   double          CarrierPhase;      /* carrier phase in m */
   char            CycleSlip;         /* cycle slip indicator, -1 not available, 0 no cycle slip, 1 cycle slip */
   unsigned char   HalfCycleStatus;   /* bit 0: if set, half cycle invalid; bit 1: if set, half cycle substrcted from phase */
   unsigned short  Cn0;               /* signal strenght in dB-Hz, scaled by 10 */
   unsigned int    MeasurmentState;   /* see above bit fields */
   unsigned short  reserved;
} tExternalGNSS_Meas, *tExternalGNSS_MeasPtr;

typedef struct
{
   unsigned short svid;
   unsigned short locktime;
   unsigned char  halfcyclesubtracted;
} tGnssChanCycleSlip, *tGnssChanCycleSlipPtr;

typedef struct
{
	unsigned short svid;
   unsigned int   svTOW;           /* sv state time of week in msec */
   double         Pos[3];          /* ECEF position in cm */
   double         Vel[3];          /* ECEF velocity in cm/s   */
   double         svClockBias;     /* in cm */
   double         svClockDrift;    /* in cm/s */
   unsigned char  EphFlag;         /* 0: not avaliable, 1: broadcast ephemeris, 2: almanac, 3 and above: extended ephemeris */
   short          Iono;            /* in cm */
   unsigned short reserved;
} tExternalGNSS_SVState, *tExternalGNSS_SVStatePtr;

typedef struct
{
	 unsigned short  svid;
   unsigned int    measTOW;           /* measurement time of week in msec*/
	 int             prResidual;        /* pseudorange measurement in cm */
   unsigned short  Cn0;               /* signal strenght in dB-Hz, scaled by 10 */ 
   unsigned char   svUsed;            /* 1 used in navigation */
   char            elevation;         /* +- 90deg  */
   short           azimuth;           /* 0 ~ 360deg */
   unsigned int    syncFlag;  
   unsigned int    reserved;
} tExternalGNSS_AuxMeas, *tExternalGNSS_AuxMeasPtr;

/*============================ end of GNSS struct ==============================*/

/*-------- DR sensor struct ---------- */
typedef struct
{
   unsigned int   timeTag;                    /* time in [ms]*/
   int            sensor[MAX_DR_NUM_SNSR];   /* 3 gyro in deg/s, 3 accel in m/s^2, 1 odo in m/s, 1 reverse; scaled by 1000 */
} tExternalDRSensorData;

typedef struct
{
   unsigned short         numDataSets;
   tExternalDRSensorData  data_set[MAX_DR_DATA_SETS];
} tExternalDRSensor, *tExternalDRSensorPtr;

/*============================ end of SENSOR struct ==============================*/

/*----- Unix Time struct -----*/
typedef struct
{
	unsigned short uvalid;        /* indicate whether unix timeTag is valid or not */
	unsigned int   unix_time;     /* unix time stamp in sec */
} tExternalUnixTime, *tExternalUnixTimePtr;

/*=============== NVM data struct ============ */
typedef struct
{
	short OKforReset;    /* true: system stays valid after a reset (only when the vehicle is static) */
	char  PreRot;        /* sensor axis orientation rotation */

	int   Lat;           /* lat in degree, scaled by 10^7 */
	int   Lon;           /* lon in degree, scaled by 10^7 */
	short Alt;           /* alt in cm */


	/*----- for DR part -----*/
	short Roll;          /* roll in degree [-90 +90], scaled by 100 */
	short Pitch;         /* pitch in degree  [-90 +90], scaled by 100 */
	short Yaw;           /* Yaw in degree  [0 360], scaled by 100 */

	short AccelBias[3];  /* accel bias in m/s^2, scaled by 10^3 */
	short GyroBias[3];   /* gyro bias in rad/s, scaled by 10^3 */

   short B2S[3];        /* body to sensor frame three mis-alignment angles, in deg, scaled by 100 */

	/* uncertainty of lat, lon, alt in cm */
	/* uncertainty of roll, pitch, yaw in degree, scaled by 100 */
	/* uncertainty of acceBias[3] in m/s^2, scaled by 10^3;  uncertainty of gyroBias[3] in rad/s, scaled by 10^3*/
	unsigned short   Unc[12];      

	short  GyroSf;           /* unitless, scaled by 100 */
   short  Gyro_sfgain;      /* unitless, scaled by 100 */
	short  OdoSf;            /* unitless, scaled by 100 */
   short  Odo_sfgain;       /* unitless, scaled by 100 */

	/* for PDR only */
	short MagBias[3];    /* mag bias in uT, scaled by 10^3 */

	int   reserved[2];      /* for potential usage */

} tNVMData, *tNVMDataPtr;


typedef struct
{
	unsigned int   SnsrTime;       /* in ms */
	unsigned int   UnixTime;       /* in sec */
	unsigned short GPSWeek;       
	unsigned int   GPSTime;        /* in ms */
   unsigned short utc_Year;
   unsigned char  utc_Month;
   unsigned short utc_Day;
   unsigned char  utc_Hour;
   unsigned char  utc_Minute;
   unsigned char  utc_Second;
   short          dT;             /* in ms */

	unsigned char  fixValid;
	
   /* 0x00: no nav                  */
	/* bit 0: reserved     (0x01)    */
   /* bit 1: gnss only    (0x02)    */
   /* bit 2: sensor only  (0x04)    */
   /* bit 3: combined solution (0x08) */
	/* bit 4: PDR                    */
	/* bit 5: loosely coupled DR     */
	/* bit 6: tightly coupled DR     */
	/* bit 7: mag in use             */
	/* bit 8: baro in use            */
	/* bit 9: odo in use             */
	/* bit 10: dual-antenna in use    */
	/* bit 11: DGNSS in use          */
	/* bit 12: RTK in use            */
	/* bit 13: map matching in use   */
	/* bit 14: sonar in use          */
	/* bit 15: wifi rssi in use      */
	/* bit 16: wifi rtt in use       */
	/* bit 17: BLE in use            */
	/* bit 18: vision aiding in use  */
	/* bit 19: lumicast in use       */
	/* bit 20: indoor harvester mode */
	/* bit 21: flight mode           */
	/* bit 22: low power mode        */
	/* others: reserved              */
	unsigned int NavModeInfo;   

	int      Lat;                         /* in degree, scaled by 10^7 */
	int      Lon;                         /* in degree, scaled by 10^7 */ 
	int      Alt;                         /* in cm */
	int      Msl;                         /* mean sea level altitude in cm */

	int      VelNED[3];                   /* in cm/s, 3D velocity in north-east-down navigation frame */

	int      Roll;                        /* in degree, scaled by 100 */
	int      Pitch;                       /* in degree, scaled by 100 */
	int      Heading;                     /* in degree, scaled by 100 */

   int      clockbias;
   int      clockdrift;

	unsigned int GroundSpeed;             /* in cm/s */
	unsigned int Distance;	              /* in cm   */

	unsigned short PosErr_Unc[3];         /* in cm, position error 1 sigma in NED */
	unsigned short VelErr_Unc[3];         /* in cm/s, velocity error 1 sigma in NED */
	unsigned short AttErr_Unc[3];         /* in degree, attitude error 1 sigma, roll/pitch/yaw, scaled by 100 */
   short          MisAngle[3];           /* in degree, misalignment angle, roll/pitch/yaw, scaled by 100 */

	unsigned short DistanceErr_Unc;       /* in cm */

   unsigned char  usedGPS;
   unsigned char  usedGLONASS;
   unsigned char  usedBeidou;
   unsigned char  usedGalileo;
   unsigned char  usedSV;

   unsigned char  inStationary;          /* set to 1 -- inStationary */

} tOutput_NavStates, *tOutput_NavStatesPtr;

#endif
/**@**/
