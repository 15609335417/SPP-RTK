#pragma once
#ifndef CONST_H

#define CONST_H
/****************************
模式：
0 ：事后 
1 ：实时
***************************/
//#define RunMode 1

//#define PI 3.1415926535898
#define EPS 1e-10

#define POLYCRC32 0xEDB88320u 

#define MaxSatNum 95
#define MaxGPSSatNum 32
#define MaxBDSSatNum 63

#define MaxNovNum 40480
#define MaxObsNum 10000

//#define ElevationMaskAngle 15

#define MaxDeltaLmw 1.5
#define MaxDeltaLgf 0.05

#define Cspeed 2.99792458e8

#define a_GPS 6378137.0
#define alpha_GPS (1/298.257223563)
#define GM_GPS 3.986005e14
#define omega_e_GPS 7.2921151467e-5

#define a_BDS 6378137
#define GM_BDS 3.986004418e14
#define omega_e_BDS 7.2921150e-5
#define alpha_BDS (1 / 298.257222101)

#define L1C 1575.42e6
#define L2P 1227.60e6

#define B1I 1561.098e6
#define B2I 1207.14e6
#define B3I 1268.520e6



#endif // !CONST_H
