#include "bq_setting_QF.h"
#include "bqMaximo_Ctrl_G2553.h"
I2C_HandleTypeDef hi2c1;

extern RegisterGroup Registers;
extern PackStateTypedef CellBoundery;
extern OperStatusTypedef OperationStatus;
extern UserProtectionSettingTypeDef UserSetting;
extern TempratureStatusTypedef TempratureStatus;
extern CellVoltageProtectionStatusTypedef CellVoltageProtectionStatus;
extern PackViolationTypedef PackViolation;
extern PackMesurmentTypedef PackMesurment;
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


unsigned char CRC8(unsigned char *ptr, unsigned char len,unsigned char key)
{
	unsigned char i;
	unsigned char crc=0;
	while(len--!=0)
	{
		for(i=0x80; i!=0; i/=2)
		{
			if((crc & 0x80) != 0)
			{
				crc *= 2;
				crc ^= key;
			}
			else
				crc *= 2;

			if((*ptr & i)!=0)
				crc ^= key;
		}
		ptr++;
	}
	return(crc);
}
OperStatusTypedef I2CWriteRegistorWithCRC(I2C_HandleTypeDef *hi2c,uint8_t I2C_Address,uint8_t Register,uint8_t Data)
{
	unsigned char DataBuffer[4];
	HAL_StatusTypeDef status;
  DataBuffer[0] = I2C_Address << 1;
	DataBuffer[1] = Register;
	DataBuffer[2] = Data;
	DataBuffer[3] = CRC8(DataBuffer, 3, CRC_KEY);
	status = HAL_I2C_Master_Transmit(hi2c,(I2C_Address << 1),(DataBuffer+1),3,TimeOutI2C);
	if(status != HAL_OK)
		{
			return Oper_I2CFail;
		}
	return Oper_OK;
}
OperStatusTypedef I2CWriteBlockWithCRC(I2C_HandleTypeDef *hi2c,uint8_t I2C_Address,uint8_t Register,uint8_t *Buffer,uint8_t Length)
{
	unsigned char size = 2*Length +2;
	unsigned char BufferCRC[size], *Pointer;
	int i;
	HAL_StatusTypeDef status;
	
	Pointer = BufferCRC;
	*Pointer = I2C_Address<<1;
	Pointer++;
	*Pointer = Register;
	Pointer++;
	*Pointer = *Buffer;
	Pointer++;
	*Pointer = CRC8(BufferCRC, 3, CRC_KEY);

	for(i = 1; i < Length; i++)
	{
			Pointer++;
			Buffer++;
			*Pointer = *Buffer;
			*(Pointer + 1) = CRC8(Pointer, 1, CRC_KEY);
			Pointer++;
	}
	status = HAL_I2C_Master_Transmit(&hi2c1,I2C_Address<<1,(BufferCRC+1),(2*Length + 1),TimeOutI2C);
	if (status != Oper_OK)
	{
		return Oper_I2CFail;
	}
	return Oper_OK;
}


OperStatusTypedef I2CReadBlockWithCRC(I2C_HandleTypeDef *hi2c,uint8_t I2C_Address,uint8_t Register,uint8_t *Buffer,uint8_t Length)
{
	uint8_t size =2 * Length;
	uint8_t *ReadData=NULL, StartData[size];
	unsigned char CRCInput[2];
	unsigned char CRC_Value = 0;
	HAL_StatusTypeDef status;
	int i;

	ReadData = StartData;

	status = HAL_I2C_Master_Transmit(hi2c,(I2C_Address << 1),&Register,1,TimeOutI2C);    // CRC -> N
	if( status  != HAL_OK){
		return Oper_I2CFail;
	}
	status = HAL_I2C_Master_Receive(&hi2c1,(I2C_Address<<1)+0x01,ReadData,(2*Length),TimeOutI2C);  // CRC -> Y
	if (status !=  HAL_OK){
		return Oper_I2CFail;
	}
	CRCInput[0] = (BQ_ADD<< 1) + 1;
	CRCInput[1] = *ReadData;
	
	CRC_Value = CRC8(CRCInput, 2, CRC_KEY);
	ReadData++;
	if (CRC_Value != *ReadData)
	{

		return Oper_CRCFail;
	}
	else
	{
		*Buffer = *(ReadData - 1);
	}
	for(i = 1; i < Length; i++)
	{
		ReadData++;
		CRC_Value= CRC8(ReadData, 1, CRC_KEY);
		ReadData++;
		Buffer++;

		if (CRC_Value != *ReadData)
		{


			return Oper_CRCFail;
		}
		else
			*Buffer = *(ReadData - 1);
	}



	return Oper_OK;
	
}
OperStatusTypedef I2CReadRegisterWithCRC(I2C_HandleTypeDef *hi2c,unsigned char I2CSlaveAddress, unsigned char Register, unsigned char *Data)
{
	unsigned char TargetRegister = Register;
	unsigned char ReadData[2];
	unsigned char CRCInput[2];
	unsigned char CRCValue = 0;
  int ReadStatus = 0;
  int WriteStatus = 0;
	
	WriteStatus = HAL_I2C_Master_Transmit(hi2c,(I2CSlaveAddress << 1),&TargetRegister,1,TimeOutI2C);
	ReadStatus =  HAL_I2C_Master_Receive(hi2c,(I2CSlaveAddress << 1)+1,ReadData,2,TimeOutI2C);

	if (ReadStatus != 0 || WriteStatus != 0)
	{
		return Oper_I2CFail;
	}

	CRCInput[0] = (I2CSlaveAddress << 1) + 1;
	CRCInput[1] = ReadData[0];

	CRCValue = CRC8(CRCInput, 2, CRC_KEY);

	if (CRCValue != ReadData[1])
		return Oper_CRCFail;

	*Data = ReadData[0];
	return Oper_OK;
}
OperStatusTypedef MosfetSwitching(MosfetOperationTypedef FetStatus)
{
	uint8_t TempSysControl2;
	OperStatusTypedef MosfetOperationStatus;
	switch (FetStatus)
	{
		case MOSFET_OFF:
		{
			TempSysControl2 = Registers.SysCtrl1.SysCtrl1Byte & 0xFC;
			MosfetOperationStatus = I2CWriteRegistorWithCRC(&hi2c1,BQ_ADD,SYS_CTRL2,TempSysControl2);
			break;
		}
		case CHGMosfetOff:
		{
			TempSysControl2 = Registers.SysCtrl1.SysCtrl1Byte & 0xFE;
			MosfetOperationStatus = I2CWriteRegistorWithCRC(&hi2c1,BQ_ADD,SYS_CTRL2,TempSysControl2);	
			break;
		}
		case CHGMosfetOn:
		{
			TempSysControl2 = Registers.SysCtrl1.SysCtrl1Byte | 0x01;
			MosfetOperationStatus = I2CWriteRegistorWithCRC(&hi2c1,BQ_ADD,SYS_CTRL2,TempSysControl2);	
			break;
		}
		case DSGMosfetOff:
		{
			TempSysControl2 = Registers.SysCtrl1.SysCtrl1Byte & 0xFD;
			MosfetOperationStatus = I2CWriteRegistorWithCRC(&hi2c1,BQ_ADD,SYS_CTRL2,TempSysControl2);	
			break;
		}
		case DSGMosfetOn:
		{
			TempSysControl2 = Registers.SysCtrl1.SysCtrl1Byte | 0x03;
			MosfetOperationStatus = I2CWriteRegistorWithCRC(&hi2c1,BQ_ADD,SYS_CTRL2,TempSysControl2);	
			break;
		}
	}
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
		return NoCurrentViolation;
	}		
}