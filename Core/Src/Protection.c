#include <Protection.h>
#include "bqMaximo_Ctrl_G2553.h"
#include "PackStatus.h"

extern RegisterGroup Registers;
extern PackStateTypedef CellBoundery;
extern OperStatusTypedef OperationStatus;
extern UserProtectionSettingTypeDef UserSetting;
extern TempratureStatusTypedef TempratureStatus;
extern CellVoltageProtectionStatusTypedef CellVoltageProtectionStatus;
extern PackViolationTypedef PackViolation;
extern PackMesurmentTypedef PackMesurment;

OperStatusTypedef MosfetSwitching(MosfetOperationTypedef FetStatus)
{
	uint8_t TempSysControl2;
	OperStatusTypedef MosfetOperationStatus;
	switch (FetStatus)
	{
		case MOSFET_OFF:
		{
			TempSysControl2 = Registers.SysCtrl1.SysCtrl1Byte & 0xFC;
			break;
		}
		case CHGMosfetOff:
		{
			TempSysControl2 = Registers.SysCtrl1.SysCtrl1Byte & 0xFE;
			break;
		}
		case CHGMosfetOn:
		{
			TempSysControl2 = Registers.SysCtrl1.SysCtrl1Byte | 0x01;
			break;
		}
		case DSGMosfetOff:
		{
			TempSysControl2 = Registers.SysCtrl1.SysCtrl1Byte & 0xFD;
			break;
		}
		case DSGMosfetOn:
		{
			TempSysControl2 = Registers.SysCtrl1.SysCtrl1Byte | 0x03;
			break;
		}
	}
	MosfetOperationStatus = I2CWriteRegistorWithCRC(&hi2c1,BQ_ADD,SYS_CTRL2,TempSysControl2);	
	return MosfetOperationStatus;
}

ThermalViolationTypedef CellBounderyTempratureWindowProtection(SensorIndicatorTypedef SensorType) 
{
	if(CellBoundery.TempMax <  UserSetting.ThresoldTempMax  &&  CellBoundery.TempMax > UserSetting.WinodwTempMax)  // 
	{ 
		if(SensorType == ExternalThemisterSense)
		{
			TempratureStatus.ExternalMaxThermalTimeElepsed++;
		}
		if(SensorType == InternalDieSense)
		{
			TempratureStatus.InternalMaxDieThermalTimeElepsed++;
		}
		if(TempratureStatus.ExternalMaxThermalTimeElepsed > UserSetting.WindowTempTime  ||  TempratureStatus.InternalMaxDieThermalTimeElepsed >  UserSetting.WindowTempTime)
		{
			MosfetSwitching(MOSFET_OFF);
			if (TempratureStatus.ExternalMaxThermalTimeElepsed > UserSetting.WindowTempTime)
			{
				return ExternalMaxViolationTrip;
			}
			if( TempratureStatus.InternalMaxDieThermalTimeElepsed >  UserSetting.WindowTempTime)
			{
				return InternalMaxViolationTrip;
			}
		}
		if (TempratureStatus.ExternalMaxThermalTimeElepsed != 0)
		{
			return ExternalMaxViolation;
		}
		if(TempratureStatus.InternalMaxDieThermalTimeElepsed != 0 )
		{
			return InternalMaxViolation;
		}
	}
	else if 	(CellBoundery.TempMin  >  UserSetting.ThresoldTempMin  &&  CellBoundery.TempMin  <  UserSetting.WinodwTempMin)
	{
		if(SensorType == ExternalThemisterSense)
		{
			TempratureStatus.ExternalMinThermalTimeElepsed++;
		}
		if(SensorType == InternalDieSense)
		{
			TempratureStatus.InternalMinDieThermalTimeElepsed++;
		}
		if(TempratureStatus.ExternalMinThermalTimeElepsed > UserSetting.WindowTempTime  ||  TempratureStatus.InternalMinDieThermalTimeElepsed >  UserSetting.WindowTempTime)
		{
			if (TempratureStatus.ExternalMinThermalTimeElepsed > UserSetting.WindowTempTime)
			{
				return ExternalMinViolationTrip;
			}
			if( TempratureStatus.InternalMinDieThermalTimeElepsed >  UserSetting.WindowTempTime)
			{
				return InternalMinViolationTrip;
			}
		}
		if (TempratureStatus.ExternalMinThermalTimeElepsed != 0)
		{
			return ExternalMinViolation;
		}
		if(TempratureStatus.InternalMinDieThermalTimeElepsed != 0 )
		{
			return InternalMinViolation;
		}		
	}
	return NoThermalViolation;
}

ThermalViolationTypedef TempratureProtection(SensorIndicatorTypedef SensorType)
{
	ThermalViolationTypedef ThermalViolation;
	if (SensorType == ExternalThemisterSense)
	{
		Maximum((uint16_t *)PackMesurment.ExternalTempraturarSense,3,&CellBoundery.TempMax,&CellBoundery.CellTempMaxIndex);
		Minimum((uint16_t *)PackMesurment.ExternalTempraturarSense,3,&CellBoundery.TempMin,&CellBoundery.CellTempMinIndex);
	}
	if (SensorType == InternalDieSense)
	{
		Maximum((uint16_t *)PackMesurment.DieTempraturarSense,3,&CellBoundery.TempMax,&CellBoundery.CellTempMaxIndex);
		Minimum((uint16_t *)PackMesurment.DieTempraturarSense,3,&CellBoundery.TempMin,&CellBoundery.CellTempMinIndex);
	}
	if(CellBoundery.TempMin < UserSetting.ThresoldTempMin)
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);   // LED indication GPIO for Thermal
		// Send CAN Emergency 	//log the data  // Equal Balacing on to increse the temprature based on flag 
		if (SensorType == ExternalThemisterSense)
		{
			return ExternalMaxViolationTrip;
		}
		else if (SensorType == InternalDieSense)
		{
			return InternalMaxViolationTrip;
		}
	}
	if (CellBoundery.TempMax > UserSetting.ThresoldTempMax)
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);  // LED indication
		MosfetSwitching(MOSFET_OFF);
		//Balancing OFF    // log the data   // send CAN Emergency 
		if (SensorType == ExternalThemisterSense)
		{
			return ExternalMinViolationTrip;
		}
		else if (SensorType == InternalDieSense)
		{
			return InternalMinViolationTrip;
		}
	}
	ThermalViolation = CellBounderyTempratureWindowProtection(SensorType);
	return ThermalViolation;
}

CellVoltViolationTypedef CellVoltageProtection()
{
	Maximum(PackMesurment.CellVoltage,CellCount,&CellBoundery.CellMax,&(CellBoundery.CellVoltMaxIndex));
	Minimum(PackMesurment.CellVoltage,CellCount,&CellBoundery.CellMin,&(CellBoundery.CellVoltMinIndex));
	if(CellBoundery.CellMax > UserSetting.ThresoldCellVoltMax  || CellBoundery.CellMin > UserSetting.ThresoldCellVoltMin)
	{
		MosfetSwitching(MOSFET_OFF);
	}
	if (CellBoundery.CellMax > UserSetting.ThresoldCellVoltMax  && CellBoundery.CellMin > UserSetting.ThresoldCellVoltMin )
	{
		return CellUVOVViolationTrip;
	}
	else if(CellBoundery.CellMax > UserSetting.ThresoldCellVoltMax)
	{
		return CellOVViolationTrip;
	}
	else if(CellBoundery.CellMin < UserSetting.ThresoldCellVoltMin)
	{
		return CellUVViolationTrip;
	}
	
	if (CellBoundery.CellMax >  UserSetting.WindowCellVoltMax)
	{	
		 CellVoltageProtectionStatus.CellOVTimeElepsed++;
	}
	if (CellBoundery.CellMin <  UserSetting.WindowCellVoltMin)
	{
		 CellVoltageProtectionStatus.CellUVTimeElepsed++;
	}
	
	if(CellVoltageProtectionStatus.CellOVTimeElepsed > UserSetting.OVDelay &&  CellVoltageProtectionStatus.CellUVTimeElepsed > UserSetting.UVDelay)
	{
		MosfetSwitching(MOSFET_OFF);
		return CellOVViolationTrip;
	}
	
	if(CellVoltageProtectionStatus.CellOVTimeElepsed != 0 && CellVoltageProtectionStatus.CellUVTimeElepsed != 0)
	{
		return CellUVOVViolation;
	}
	else if (CellVoltageProtectionStatus.CellOVTimeElepsed != 0)
	{
		return CellOVViolation;
	}
	else if (CellVoltageProtectionStatus.CellUVTimeElepsed != 0)
	{
		return CellUVViolation;
	}
	return NoCellVoltViolation;
}

PackCurrentViolationTypdef CurrentProtection()
{
	if(PackMesurment.Current > UserSetting.OCDThresold || PackMesurment.Current > UserSetting.SCDThresold)
	{
		MosfetSwitching(MOSFET_OFF);
		if (PackMesurment.Current > UserSetting.OCDThresold)
		{
			return PackOCDViolationTrip;
		}
		else if (PackMesurment.Current > UserSetting.SCDThresold)
		{
			return PackSDCViolationTrip;
		}
	}		
	return NoCurrentViolation;
}
void Minimum(uint16_t *Buffer,uint8_t size,uint16_t *Minimum_val,uint8_t *Cell_index)
{
	uint16_t *pPointer=NULL;
	pPointer  =  Buffer;
	*Minimum_val = 65535;
	for(uint8_t i = 0;i<=size;i++)
	{
		if (*pPointer < *Minimum_val)
		{
			*Minimum_val = *pPointer;
			*Cell_index = i;
		}
		*pPointer +=1;
	}
}
void Maximum(uint16_t *Buffer,uint8_t size,uint16_t *Maxvalue_val,uint8_t *Cell_index)
{
	uint16_t *pPointer=NULL;
	pPointer  =  Buffer;
	*Maxvalue_val = 0;
	for(uint8_t i = 0;i<=size;i++)
	{
		if (*pPointer > *Maxvalue_val)
		{
			*Maxvalue_val = *pPointer;
			*Cell_index = i;
		}
		*pPointer +=1;
	}
}

