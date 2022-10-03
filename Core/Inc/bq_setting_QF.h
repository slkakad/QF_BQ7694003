#ifndef BQ_SETTTING_QF_H
#define BQ_SETTTING_QF_H

#include <stdint.h>
#include "main.h"

extern I2C_HandleTypeDef hi2c1;
#define TimeOutI2C 50


#define TIM3_FREQ 1
#define TIM3_PRES 9999
#define TIM3_ARR (8400/TIM3_FREQ)-1

typedef enum		// OperStatusTypedef
{
	Oper_OK = 0x00,
	Oper_I2CFail = 0x01,
	Oper_CRCFail = 0x02,
	Oper_WRFail = 0x03
}OperStatusTypedef;



unsigned char CRC8(unsigned char *ptr, unsigned char len,unsigned char key);
OperStatusTypedef I2CWriteRegistorWithCRC(I2C_HandleTypeDef *hi2c,uint8_t I2C_Address,uint8_t Register,uint8_t Data);
OperStatusTypedef I2CWriteBlockWithCRC(I2C_HandleTypeDef *hi2c,uint8_t I2C_Address,uint8_t Register,uint8_t *Buffer,uint8_t Length);
OperStatusTypedef I2CReadBlockWithCRC(I2C_HandleTypeDef *hi2c,uint8_t I2C_Address,uint8_t Register,uint8_t *Buffer,uint8_t Length);
OperStatusTypedef I2CReadRegisterWithCRC(I2C_HandleTypeDef *hi2c,unsigned char I2CSlaveAddress, unsigned char Register, unsigned char *Data);
#endif

