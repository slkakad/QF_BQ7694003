#include "bqMaximo_Ctrl_G2553.h"

#define BQ_ADD 0x08
#define TimeOutI2C 50
#define CellCount 15
#define TempSensorCount 3
#define ExternalTemprature 3
#define TempSensorCountLimit 5
#define CellOVCount 2
#define CellUVCount 8

#define Max_Temp_Allowed 85
#define Min_Temp_Allowed 05

#define TemperatureBeta 3977 
#define RT0 10000
#define Temperature0 298
#define DieTempraturTime 5    // 1,2,3,4,5 -> External Temp   5-6, 6-7  -> Rest 7-> Die Temp  8,9 -> Rest

