#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "battery_pack_config.h"
#define SYS_STAT 0
#define CELLBAL1 1
#define CELLBAL2 2
#define CELLBAL3 3
#define SYS_CTRL1 4
#define SYS_CTRL2 5
#define PROTECT1 0x06
#define PROTECT2 0x07
#define PROTECT3 0x08
#define OV_TRIP 0x09
#define UV_TRIP 0x0A
#define CCCFG 0x19  // modified
#define VC1_HI_BYTE 0x0C
#define TS1_HI_BYTE 0x2A
#define TS1_LO_BYTE 0x2D
#define TS2_HI_BYTE 0x2E
#define TS2_LO_BYTE 0x2F
#define TS3_HI_BYTE 0x30
#define TS3_LO_BYTE 0x31
#define BAT_HI_BYTE 0X2A
#define BAT_LO_BYTE 0X2B
#define Cell_Base 0x0C

#define ADCGAIN1 0x50
#define ADCOFFSET 0x51
#define ADCGAIN2 0x59

#define CC_HI_ADD 0x32   //address of CC_HI
#define CC_LO_ADD 0x33   // address of CC_LI

#define SCD_DELAY_50us		0x0
#define SCD_DELAY_100us		0x1
#define SCD_DEALY_200us		0x2
#define SCD_DELAY_400us		0x3

#define SCD_THRESH_44mV_22mV	0
#define SCD_THRESH_67mV_33mV	1
#define SCD_THRESH_89mV_44mV	2
#define SCD_THRESH_111mV_56mV	3
#define SCD_THRESH_133mV_67mV	4
#define SCD_TRHESH_155mV_68mV	5
#define SCD_THRESH_178mV_89mV	6
#define SCD_THRESH_200mV_100mV	7

#define OCD_DEALY_10ms		0x0
#define OCD_DELAY_20ms		0x1
#define OCD_DELAY_40ms		0x2
#define OCD_DELAY_80ms		0x3
#define OCD_DELAY_160ms		0x4
#define OCD_DELAY_320ms		0x5
#define OCD_DELAY_640ms		0x6
#define OCD_DELAY_1280ms	0x7

#define OCD_THRESH_17mV_8mV		0
#define OCD_THRESH_22mV_11mV	1
#define OCD_THRESH_28mV_14mV	2
#define OCD_THRESH_33mV_17mV	3
#define OCD_THRESH_39mV_19mV	4
#define OCD_THRESH_44mV_22mV	5
#define OCD_THRESH_50mV_25mV	6
#define OCD_THRESH_56mV_28MV	7
#define OCD_THRESH_61mV_31mV	8
#define OCD_THRESH_67mV_33mV	9
#define OCD_THRESH_72mV_36mV	0xA
#define OCD_THRESH_78mV_39mV	0xB
#define OCD_THRESH_83mV_42mV	0xC
#define OCD_THRESH_89mV_44mV	0xD
#define OCD_THRESH_94mV_47mV	0xE
#define OCD_THRESH_100mV_50mV	0xF

#define UV_DELAY_1s				0
#define UV_DELAY_4s				1
#define UV_DELAY_8s				2
#define UV_DELAY_16s			3

#define OV_DELAY_1s				0
#define OV_DELAY_2s				1
#define OV_DELAY_4s				2
#define OV_DELAY_8s				3

#define OV_THRESH_BASE			0x2008
#define UV_THRESH_BASE			0x1000
#define OV_STEP					0x10
#define UV_STEP					0x10

#define ADCGAIN_BASE			365

#define ADC_Offset 0x00

typedef struct _Register_Group
{
	union
	{
		struct
		{
			unsigned char OCD			:1;
			unsigned char SCD			:1;
			unsigned char OV			:1;
			unsigned char UV			:1;
			unsigned char OVRD_ALERT	:1;
			unsigned char DEVICE_XREADY	:1;
			unsigned char WAKE			:1;
			unsigned char CC_READY		:1;
		}StatusBit;
		unsigned char StatusByte;
	}SysStatus;

	union
	{
		struct
		{
			unsigned char RSVD			:3;
			unsigned char CB5			:1;
			unsigned char CB4			:1;
			unsigned char CB3			:1;
			unsigned char CB2			:1;
			unsigned char CB1			:1;
		}CellBal1Bit;
		unsigned char CellBal1Byte;
	}CellBal1;

	union
	{
		struct
		{
			unsigned char RSVD			:3;
			unsigned char CB10			:1;
			unsigned char CB9			:1;
			unsigned char CB8			:1;
			unsigned char CB7			:1;
			unsigned char CB6			:1;
		}CellBal2Bit;
		unsigned char CellBal2Byte;
	}CellBal2;

	union
	{
		struct
		{
			unsigned char RSVD			:3;
			unsigned char CB15			:1;
			unsigned char CB14			:1;
			unsigned char CB13			:1;
			unsigned char CB12			:1;
			unsigned char CB11			:1;
		}CellBal3Bit;
		unsigned char CellBal3Byte;
	}CellBal3;

	union
	{
		struct
		{
			unsigned char SHUT_B		:1;
			unsigned char SHUT_A		:1;
			unsigned char RSVD1			:1;
			unsigned char TEMP_SEL		:1;
			unsigned char ADC_EN		:1;
			unsigned char RSVD2			:2;
			unsigned char LOAD_PRESENT	:1;
		}SysCtrl1Bit;
		unsigned char SysCtrl1Byte;
	}SysCtrl1;

	union
	{
		struct
		{
			unsigned char CHG_ON		:1;
			unsigned char DSG_ON		:1;
			unsigned char WAKE_T		:2;
			unsigned char WAKE_EN		:1;
			unsigned char CC_ONESHOT	:1;
			unsigned char CC_EN			:1;
			unsigned char DELAY_DIS		:1;
		}SysCtrl2Bit;
		unsigned char SysCtrl2Byte;
	}SysCtrl2;

	union
	{
		struct
		{
			unsigned char SCD_THRESH	:3;
			unsigned char SCD_DELAY		:2;
			unsigned char RSVD			:2;
			unsigned char RSNS			:1;
		}Protect1Bit;
		unsigned char Protect1Byte;
	}Protect1;

	union
	{
		struct
		{
			unsigned char OCD_THRESH	:4;
			unsigned char OCD_DELAY		:3;
			unsigned char RSVD			:1;
		}Protect2Bit;
		unsigned char Protect2Byte;
	}Protect2;

	union
	{
		struct
		{
			unsigned char RSVD			:4;
			unsigned char OV_DELAY		:2;
			unsigned char UV_DELAY		:2;
		}Protect3Bit;
		unsigned char Protect3Byte;
	}Protect3;

	unsigned char OVTrip;
	unsigned char UVTrip;
	unsigned char CCCfg;			//must be 0x19

	union
	{
		struct
		{
			unsigned char VC1_HI;
			unsigned char VC1_LO;
		}VCell1Byte;
		unsigned short VCell1Word;
	}VCell1;

	union
	{
		struct
		{
			unsigned char VC2_HI;
			unsigned char VC2_LO;
		}VCell2Byte;
		unsigned short VCell2Word;
	}VCell2;

	union
	{
		struct
		{
			unsigned char VC3_HI;
			unsigned char VC3_LO;
		}VCell3Byte;
		unsigned short VCell3Word;
	}VCell3;

	union
	{
		struct
		{
			unsigned char VC4_HI;
			unsigned char VC4_LO;
		}VCell4Byte;
		unsigned short VCell4Word;
	}VCell4;

	union
	{
		struct
		{
			unsigned char VC5_HI;
			unsigned char VC5_LO;
		}VCell5Byte;
		unsigned short VCell5Word;
	}VCell5;

	union
	{
		struct
		{
			unsigned char VC6_HI;
			unsigned char VC6_LO;
		}VCell6Byte;
		unsigned short VCell6Word;
	}VCell6;

	union
	{
		struct
		{
			unsigned char VC7_HI;
			unsigned char VC7_LO;
		}VCell7Byte;
		unsigned short VCell7Word;
	}VCell7;

	union
	{
		struct
		{
			unsigned char VC8_HI;
			unsigned char VC8_LO;
		}VCell8Byte;
		unsigned short VCell8Word;
	}VCell8;

	union
	{
		struct
		{
			unsigned char VC9_HI;
			unsigned char VC9_LO;
		}VCell9Byte;
		unsigned short VCell9Word;
	}VCell9;

	union
	{
		struct
		{
			unsigned char VC10_HI;
			unsigned char VC10_LO;
		}VCell10Byte;
		unsigned short VCell10Word;
	}VCell10;

	union
	{
		struct
		{
			unsigned char VC11_HI;
			unsigned char VC11_LO;
		}VCell11Byte;
		unsigned short VCell11Word;
	}VCell11;

	union
	{
		struct
		{
			unsigned char VC12_HI;
			unsigned char VC12_LO;
		}VCell12Byte;
		unsigned short VCell12Word;
	}VCell12;

	union
	{
		struct
		{
			unsigned char VC13_HI;
			unsigned char VC13_LO;
		}VCell13Byte;
		unsigned short VCell13Word;
	}VCell13;

	union
	{
		struct
		{
			unsigned char VC14_HI;
			unsigned char VC14_LO;
		}VCell14Byte;
		unsigned short VCell14Word;
	}VCell14;

	union
	{
		struct
		{
			unsigned char VC15_HI;
			unsigned char VC15_LO;
		}VCell15Byte;
		unsigned short VCell15Word;
	}VCell15;

	union
	{
		struct
		{
			unsigned char BAT_HI;
			unsigned char BAT_LO;
		}VBatByte;
		unsigned short VBatWord;
	}VBat;

	union
	{
		struct
		{
			unsigned char TS1_HI;
			unsigned char TS1_LO;
		}TS1Byte;
		unsigned short TS1Word;
	}TS1;

	union
	{
		struct
		{
			unsigned char TS2_HI;
			unsigned char TS2_LO;
		}TS2Byte;
		unsigned short TS2Word;
	}TS2;

	union
	{
		struct
		{
			unsigned char TS3_HI;
			unsigned char TS3_LO;
		}TS3Byte;
		unsigned short TS3Word;
	}TS3;

	union
	{
		struct
		{
			unsigned char CC_HI;
			unsigned char CC_LO;
		}CCByte;
		unsigned short CCWord;
	}CC;
	signed char ADCOffset;
  float adc_gain;
}RegisterGroup;
void get_status_bits(RegisterGroup *temp);
void get_CC_Value(RegisterGroup *temp);
void set_SysCtrl_Reg(RegisterGroup *temp);
void set_SysCtr2_Reg(RegisterGroup *temp);
void set_Protect1_Reg(RegisterGroup *temp);
void set_Protect2_Reg(RegisterGroup *temp);
void set_Protect3_Reg(RegisterGroup *temp);
void set_OV_TRIP_Reg(RegisterGroup *temp,cell_config *temp1);
void set_UV_TRIP_Reg(RegisterGroup *temp,cell_config *temp1);
void set_CC_CFG_Reg(RegisterGroup *temp);
uint16_t get_Cell_Voltage(uint8_t cell_No);
//void get_Temprature(RegisterGroup *temp, uint8_t temp_No);
void get_TS1(RegisterGroup *temp);
void get_TS2(RegisterGroup *temp);
void get_TS3(RegisterGroup *temp);
void get_BAT_HI_LO(RegisterGroup *temp);
void enable_ADC(void);
void cell_balance(RegisterGroup *temp);
void cell_balance1(RegisterGroup *temp);
void cell_balance2(RegisterGroup *temp);
void cell_balance3(RegisterGroup *temp);
void cell_group(int n);
void cell_group1(int n,...);
void get_adc_gain(RegisterGroup *temp);
void get_adc_offset(RegisterGroup *temp);
//void adc_cell_value(uint16_t adc_cell [], float * volt);
