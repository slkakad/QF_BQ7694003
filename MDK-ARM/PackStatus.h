#ifndef PACKSTATUS_H
#define PACKSTATUS_H

#include <stdint.h>

#define CellCount 15
#define TempSensorCount 3
#define ExternalTemprature 3

#define CellOVCount 2
#define CellUVCount 8

#define Max_Allowed_Trip_Limit 85
#define Min_Allowed_Trip_Limit 05
#define Max_Temp_Allowed_Window 75
#define Min_Temp_Allowed_Window 10
#define TempSensorWinodwCount 5


#define Cell_UV_Allowed_Window 4200
#define Cell_OV_Allowed_Window 2500
#define CellOVWinodwCount 5
#define CellUVWindowCount 5

typedef enum   //SensorIndicatorTypedef
{
	ExternalThemisterSense = 0x00,
	InternalDieSense = 0x02
}SensorIndicatorTypedef;



typedef struct		// Current status of pack and cell
{
	uint16_t CellMax;
	uint16_t CellMin;
	SensorIndicatorTypedef  TempratureSenseIndicator;
	uint16_t TempMax;
	uint16_t TempMin;
	uint8_t CellTempMaxIndex;
	uint8_t CellTempMinIndex;
	uint8_t CellVoltMaxIndex;
	uint8_t CellVoltMinIndex;
}PackStateTypedef;

typedef struct  //PackMesurmentTypedef
{
	uint16_t CellVoltage[CellCount];
	uint16_t BatteryVoltage;
	float ExternalTempraturarSense[TempSensorCount];
	float DieTempraturarSense[2];
	int32_t Current;
}PackMesurmentTypedef;
#endif 


