#ifndef PROTECTION_H
#define PROTECTION_H

#include <stdint.h>
#include "bq_setting_QF.h"
#include "PackStatus.h"


#define TemperatureBeta 3977 
#define RT0 10000
#define Temperature0 298
#define DieTempraturTime 5    // 1,2,3,4,5 -> External Temp   5-6, 6-7  -> Rest 7-> Die Temp  8,9 -> Rest

typedef enum		//OCDDelayTypedef
{
	OCD_DEALY_10ms=0x0,
	OCD_DELAY_20ms		=0x1,
	OCD_DELAY_40ms		=0x2,
	OCD_DELAY_80ms		=0x3,
	OCD_DELAY_160ms		=0x4,
	OCD_DELAY_320ms	=	0x5,
	OCD_DELAY_640ms		=0x6,
	OCD_DELAY_1280ms	=0x7
}OCDDelayTypedef;

typedef enum		//OCDThresoldTypedef
{
	OCD_THRESH_17mV_8mV=0,
	OCD_THRESH_22mV_11mV=1,
	OCD_THRESH_28mV_14mV=2,
	OCD_THRESH_33mV_17mV=3,
	OCD_THRESH_39mV_19mV	=4,
	OCD_THRESH_44mV_22mV	=5,
	OCD_THRESH_50mV_25mV	=6,
	OCD_THRESH_56mV_28MV	=7,
	OCD_THRESH_61mV_31mV	=8,
	OCD_THRESH_67mV_33mV	=9,
	OCD_THRESH_72mV_36mV	=0xA,
	OCD_THRESH_78mV_39mV	=0xB,
	OCD_THRESH_83mV_42mV	=0xC,
	OCD_THRESH_89mV_44mV	=0xD,
	OCD_THRESH_94mV_47mV	=0xE,
	OCD_THRESH_100mV_50mV =0xF
}OCDThresoldTypedef;

typedef enum 		//SCDDelayTypedef
{
	SCD_DELAY_50us=0x0,
	SCD_DELAY_100us=0x1,
	SCD_DEALY_200us=0x2,
	SCD_DELAY_400us=0x3
}SCDDelayTypedef;

typedef enum		//UVDelayTypedef
{
	UV_DELAY_1s				=1,
	UV_DELAY_4s				=4,
	UV_DELAY_8s				=8,
	UV_DELAY_16s			=16,
}UVDelayTypedef;
typedef enum		//OVDelayTypedef
{
	OV_DELAY_1s				=1,
	OV_DELAY_2s			  =2,
	OV_DELAY_4s				=4,
	OV_DELAY_8s				=8,
}OVDelayTypedef;

typedef enum 		//SCDThresoldTypedef
{
	SCD_THRESH_44mV_22mV=0,
	SCD_THRESH_67mV_33mV=1,
	SCD_THRESH_89mV_44mV=2,
	SCD_THRESH_111mV_56mV=3,
	SCD_THRESH_133mV_67mV=4,
	SCD_TRHESH_155mV_68mV=5,
	SCD_THRESH_178mV_89mV=6,
	SCD_THRESH_200mV_100mV=7
}SCDThresoldTypedef;
typedef enum  //ThermalViolationTypedef
{
	NoThermalViolation = 0x00,
	ExternalMaxViolation = 0x01,
	ExternalMinViolation = 0x02,
	InternalMaxViolation = 0x03,
	InternalMinViolation = 0x04,
	ExternalMaxViolationTrip = 0x05,
	ExternalMinViolationTrip = 0x06,
	InternalMaxViolationTrip  = 0x07,
	InternalMinViolationTrip  = 0x08,
}ThermalViolationTypedef;

typedef enum  //CellVoltViolationTypedef;
{
	NoCellVoltViolation =0x00,
	CellOVViolation=0x01,
	CellUVViolation=0x02,
	CellOVViolationTrip = 0x03,
	CellUVViolationTrip  = 0x04,	
	CellUVOVViolation = 0x05,
	CellUVOVViolationTrip = 0x06	
}CellVoltViolationTypedef;

typedef enum	//PackCurrentViolationTypdef
{
	NoCurrentViolation=0x00,
	PackOCDViolationTrip =0x01,
	PackSDCViolationTrip = 0x02,
}PackCurrentViolationTypdef;

typedef struct //PackViolationTypedef
{
	ThermalViolationTypedef ThermalViolation;
	CellVoltViolationTypedef CellVoltViolation;
	PackCurrentViolationTypdef PackCurrentViolation;
}PackViolationTypedef;

typedef enum	// Mosfet operation in switch case
{
	MOSFET_OFF =0x00,
	CHGMosfetOff = 0x01,
	CHGMosfetOn = 0x02,
	DSGMosfetOff = 0x03,
	DSGMosfetOn  = 0x04	
}MosfetOperationTypedef;

typedef struct 		//TempratureStatusTypedef
{
	uint8_t ExternalMaxThermalTimeElepsed;
	uint8_t ExternalMinThermalTimeElepsed;
	uint8_t InternalMaxDieThermalTimeElepsed;
	uint8_t InternalMinDieThermalTimeElepsed;
}TempratureStatusTypedef;

typedef struct	//CellVoltageProtectionStatusTypedef
{
	uint8_t CellOVTimeElepsed;
	uint8_t CellUVTimeElepsed;
}CellVoltageProtectionStatusTypedef;
	



typedef struct   // For window Setting and absolute settings stored in SD Card or flash 
{
	float ThresoldTempMax;	// 65 			
	float ThresoldTempMin;
	float WinodwTempMax;				// 60 
	float WinodwTempMin;
	uint8_t WindowTempTime;   //  <-  wait time in window  255 second max
	
	uint16_t ThresoldCellVoltMax;  
	uint16_t ThresoldCellVoltMin;
	uint16_t ThresoldPackVoltage;   // entire series connected cell
	
	uint16_t WindowCellVoltMax;			
	uint16_t WindowCellVoltMin;
	OVDelayTypedef OVDelay;				// Overvoltage protection Delay 
	UVDelayTypedef UVDelay;				//Undervoltage Protection Delay
	
	uint8_t CurrentShuntResistor;   // in mOhm
	
	OCDThresoldTypedef OCDThresold;
	OCDDelayTypedef OCDDelay;
	SCDThresoldTypedef SCDThresold;
	SCDDelayTypedef SCDDelay;
}UserProtectionSettingTypeDef;				//  Desktop APP
void Minimum(uint16_t *Buffer,uint8_t size,uint16_t *Minimum_val,uint8_t *Cell_index);
void Maximum(uint16_t *Buffer,uint8_t size,uint16_t *Maxvalue_val,uint8_t *Cell_index);
PackCurrentViolationTypdef CurrentProtection();
CellVoltViolationTypedef CellVoltageProtection();
ThermalViolationTypedef TempratureProtection(SensorIndicatorTypedef SensorType);
ThermalViolationTypedef CellBounderyTempratureWindowProtection(SensorIndicatorTypedef SensorType);
OperStatusTypedef MosfetSwitching(MosfetOperationTypedef FetStatus);
#endif
