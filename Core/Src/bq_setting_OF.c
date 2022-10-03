#include "bq_setting_QF.h"
#include "bqMaximo_Ctrl_G2553.h"
I2C_HandleTypeDef hi2c1;

extern RegisterGroup Registers;
extern OperStatusTypedef OperationStatus;
HAL_StatusTypeDef status;

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

	int i;

	ReadData = StartData;
	status = HAL_I2C_Master_Transmit(&hi2c1,(I2C_Address << 1),&Register,1,TimeOutI2C);    // CRC -> N
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
