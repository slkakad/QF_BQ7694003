#include "stm32f4xx_hal_i2c.h"

#define TimeOutI2C 50
#define CellCount 15
#define TempSensorCount 3
#define ExternalTemprature 3
#define TempSensorCountLimit 5
#define CellOVCount 2
#define CellUVCount 8

#define TIM3_FREQ 1
#define TIM3_PRES 9999
#define TIM3_ARR (8400/TIM3_FREQ)-1

#define Max_Temp_Allowed 85
#define Min_Temp_Allowed 05

#define TemperatureBeta 3977 
#define RT0 10000
#define Temperature0 298
#define DieTempraturTime 5    // 1,2,3,4,5 -> External Temp   5-6, 6-7  -> Rest 7-> Die Temp  8,9 -> Rest

typedef enum
{
	Oper_OK = 0x00,
	Oper_I2CFail = 0x01,
	Oper_CRCFail = 0x02,
	Oper_WRFail = 0x03
}OperStatusTypedef;



void Minimum(uint16_t *Buffer,uint8_t size,uint16_t *Minimum_val,uint8_t *Cell_index);
void Maximum(uint16_t *Buffer,uint8_t size,uint16_t *Maxvalue_val,uint8_t *Cell_index);
unsigned char CRC8(unsigned char *ptr, unsigned char len,unsigned char key);
OperStatusTypedef I2CWriteRegistorWithCRC(I2C_HandleTypeDef *hi2c,uint8_t I2C_Address,uint8_t Register,uint8_t Data);
OperStatusTypedef I2CWriteBlockWithCRC(I2C_HandleTypeDef *hi2c,uint8_t I2C_Address,uint8_t Register,uint8_t *Buffer,uint8_t Length)
OperStatusTypedef I2CReadBlockWithCRC(I2C_HandleTypeDef *hi2c,uint8_t I2C_Address,uint8_t Register,uint8_t *Buffer,uint8_t Length)
OperStatusTypedef I2CReadRegisterWithCRC(I2C_HandleTypeDef *hi2c,unsigned char I2CSlaveAddress, unsigned char Register, unsigned char *Data)