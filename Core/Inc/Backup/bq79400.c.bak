#include "bq79400x.h"
#include <string.h>
RegisterGroup R1;
//uint8_t group_numb;
uint8_t x ;
uint8_t readI2CData(uint8_t temp)    // Replace this one 
{
	return 0xff;
}
uint8_t writeI2CData(uint8_t add, uint8_t data)    // Replace this one 
{
	return 0xff;
}
bool getCCStatus()    // During testin check by removing result for higher through put
{
	uint8_t sys_stat_data;
	bool result;
	sys_stat_data=readI2CData(SYS_STAT);
	writeI2CData(SYS_STAT,sys_stat_data &0x7f);
	result=(sys_stat_data & 0x80)>>7;
	return result;	
}
void get_CCValue(RegisterGroup *temp)
{
//	bool x=0;
//	uint16_t result;
//	while(x==0)
//		{
//			x=getCCStatus();
//	  }
	temp->CC.CCByte.CC_HI=readI2CData(CC_HI_ADD);
	temp->CC.CCByte.CC_LO=readI2CData(CC_LO_ADD);
	temp->CC.CCWord=(R1.CC.CCByte.CC_HI &0XFF00)+R1.CC.CCByte.CC_LO;	
	
}
void enable_ADC(void)
{
	uint8_t sys_ctrl1;
	sys_ctrl1=readI2CData(SYS_CTRL1);
	writeI2CData(SYS_CTRL1,sys_ctrl1 | (1<<4));
	//R1.SysCtrl1.SysCtrl1Bit.ADC_EN=1;
}
void get_status_bit(RegisterGroup *temp)
{
	uint8_t sys_stat_data;
	sys_stat_data=readI2CData(SYS_STAT);
	temp->SysStatus.StatusBit.OCD = sys_stat_data & 0x01;
	temp->SysStatus.StatusBit.SCD = sys_stat_data & 0x02;
	temp->SysStatus.StatusBit.OV = sys_stat_data & 0x04;
	temp->SysStatus.StatusBit.UV = sys_stat_data & 0x08;
	temp->SysStatus.StatusBit.OVRD_ALERT = sys_stat_data & 0x10;
	temp->SysStatus.StatusBit.DEVICE_XREADY = sys_stat_data & 0x20;
	//temp->SysStatus.StatusBit.WAKE = sys_stat_data & 0x40;
	temp->SysStatus.StatusBit.CC_READY= sys_stat_data & 0x80;
}
void set_SysCtrl_Reg(RegisterGroup *temp)
{
	uint8_t data=0x00,old_data;
	data = (temp->SysCtrl1.SysCtrl1Bit.ADC_EN)<<4;
	data = (temp->SysCtrl1.SysCtrl1Bit.SHUT_A)<<1;
	data = (temp->SysCtrl1.SysCtrl1Bit.SHUT_B);
	data = (temp->SysCtrl1.SysCtrl1Bit.TEMP_SEL)<<3;
	old_data = readI2CData(SYS_CTRL1) & 0xE4 ; 
	writeI2CData(SYS_CTRL1 , old_data | data );
}
void set_SysCtr2_Reg(RegisterGroup *temp)
{
	uint8_t data=0x00,old_data;
	data= temp->SysCtrl2.SysCtrl2Bit.CHG_ON;
	data=(temp->SysCtrl2.SysCtrl2Bit.DSG_ON)<<1;
	data=(temp->SysCtrl2.SysCtrl2Bit.CC_ONESHOT)<<5;
	data=(temp->SysCtrl2.SysCtrl2Bit.CC_EN)<<6;
	data=(temp->SysCtrl2.SysCtrl2Bit.DELAY_DIS)<<7;
	old_data = readI2CData(SYS_CTRL2) & 0x1C; 
	writeI2CData(SYS_CTRL2 , old_data | data );
}
void set_Protect1_Reg(RegisterGroup *temp)
{
	uint8_t data=0x00,old_data;
	data=temp->Protect1.Protect1Bit.SCD_THRESH;
	data=(temp->Protect1.Protect1Bit.SCD_DELAY)<<3;
	data=(temp->Protect1.Protect1Bit.RSNS)<<7;
	old_data = readI2CData(PROTECT1) & 0x60; 
	writeI2CData(PROTECT1 , old_data | data );
}
void set_Protect2_Reg(RegisterGroup *temp)
{
	uint8_t data=0x00,old_data;
	data=temp->Protect2.Protect2Bit. OCD_THRESH;
	data=(temp->Protect2.Protect2Bit.OCD_DELAY)<<4;
	old_data = readI2CData(PROTECT2) & 0x80; 
	writeI2CData(PROTECT2 , old_data | data );
}	
void set_Protect3_Reg(RegisterGroup *temp)
{
	uint8_t data=0x00,old_data;
	data=(temp->Protect3.Protect3Bit.OV_DELAY	)<<4;
	data=(temp->Protect3.Protect3Bit.UV_DELAY	)<<6;
	old_data = readI2CData(PROTECT3) & 0x0F; 
	writeI2CData(PROTECT3 , old_data | data );
}	
void set_OV_TRIP_Reg(RegisterGroup *temp,cell_config *cell_set)
{
	temp->OVTrip = (unsigned char)((((unsigned short)((cell_set-> cell_OT - temp->ADCOffset)/temp->adc_gain+ 0.5f) - OV_THRESH_BASE) >> 4) & 0xFF);
	writeI2CData(OV_TRIP ,temp->OVTrip );
}
void set_UV_TRIP_Reg(RegisterGroup *temp,cell_config *cell_set)
{
  temp->UVTrip=(unsigned char)((((unsigned short)((cell_set->cell_UT - temp->ADCOffset)/temp->adc_gain+ 0.5f) - UV_THRESH_BASE) >> 4) & 0xFF);
  writeI2CData(UV_TRIP,temp->UVTrip);
}
void set_CC_CFG_Reg(RegisterGroup *temp)
{
	uint8_t data=0x00, old_data;
	data=temp->CCCfg;
	old_data = readI2CData(CCCFG) & 0xC0; 
	writeI2CData(CCCFG,old_data | data);
}
uint16_t get_Cell_Voltage( uint8_t cell_No)
{
	uint8_t cell_Voltage_Add_Lo,cell_Voltage_Add_Hi,Cell_Voltage,temp1,temp2;
	cell_Voltage_Add_Hi=   0x0C + (2*cell_No-1);
	temp1 = readI2CData(cell_Voltage_Add_Hi);	
	cell_Voltage_Add_Lo=   0x0D + (2*cell_No-1);
	temp2 = readI2CData(cell_Voltage_Add_Lo);
	Cell_Voltage = ((uint16_t)temp1 <<8) + temp2;
	return Cell_Voltage;
				
}
void get_TS1(RegisterGroup *temp)
{
	uint8_t TS1_HI_STAT,TS1_LO_STAT;
	TS1_HI_STAT=readI2CData(TS1_HI_BYTE);
	TS1_LO_STAT=readI2CData(TS1_LO_BYTE);
	temp->TS1.TS1Word=TS1_HI_STAT<<8;
	temp->TS1.TS1Word=TS1_LO_STAT;
}
void get_TS2(RegisterGroup *temp)
{
	uint8_t TS2_HI_STAT,TS2_LO_STAT;
	TS2_HI_STAT=readI2CData(TS2_HI_BYTE);
	TS2_LO_STAT=readI2CData(TS2_LO_BYTE);
	temp->TS2.TS2Word=TS2_HI_STAT<<8;
	temp->TS2.TS2Word=TS2_LO_STAT;
}
void get_TS3(RegisterGroup *temp)
{
	uint8_t TS3_HI_STAT,TS3_LO_STAT;
	TS3_HI_STAT=readI2CData(TS3_HI_BYTE);
	TS3_LO_STAT=readI2CData(TS3_LO_BYTE);
	temp->TS3.TS3Word=TS3_HI_STAT<<8;
	temp->TS3.TS3Word=TS3_LO_STAT;
}
void get_BAT_HI_LO(RegisterGroup *temp)
{
	uint8_t BAT_HI_STAT, BAT_LO_STAT;
	BAT_HI_STAT=readI2CData(BAT_HI_BYTE);
	BAT_LO_STAT=readI2CData(BAT_LO_BYTE);
	temp->VBat.VBatWord=BAT_HI_STAT<<8;
	temp->VBat.VBatWord=BAT_LO_STAT; 	
}
//void cell_group(int cell_numb)
//{
//	if(cell_numb==0||cell_numb==1||cell_numb==2||cell_numb==3||cell_numb==4||cell_numb==5)
//	{
//		group_numb=1;
//	}
//	else if(cell_numb==6||cell_numb==7||cell_numb==8||cell_numb==9||cell_numb==10)
//	{
//		group_numb=2;
//	}
//	else if(cell_numb==11||cell_numb==12||cell_numb==13||cell_numb==14||cell_numb==15)
//	{
//		group_numb=3;
//	}
//}
void cell_balance(RegisterGroup *temp)
{
	cell_balance1(temp);
	cell_balance2(temp);
	cell_balance3(temp);
	
}
void cell_balance1(RegisterGroup *temp)
{
	uint8_t cb_value=0,flag_cell=0;
	cb_value=temp->CellBal1.CellBal1Bit.CB1;
	cb_value=temp->CellBal1.CellBal1Bit.CB2<<1;
	cb_value=temp->CellBal1.CellBal1Bit.CB3<<2;
	cb_value=temp->CellBal1.CellBal1Bit.CB4<<3;
	cb_value=temp->CellBal1.CellBal1Bit.CB5<<4;
	if((cb_value &0x0A)==0)
	{
		flag_cell=0;
	}
	if((cb_value&0x15)>1 && (cb_value&0x15)>=5 && flag_cell==0)
	{
		flag_cell=1;
		cb_value=cb_value&0xF5;
		writeI2CData(0x01,cb_value);
	}
	else if((cb_value&0x0A)>2 && flag_cell==1)
	{
		flag_cell=0;
		cb_value=cb_value&0xEA;
		writeI2CData(0x01,cb_value);
	}
	else if((cb_value&0x15)==1)
	{
		cb_value=cb_value&0xE1;
		writeI2CData(0x01,cb_value);
	}
	else if((cb_value&0x0A)==2)
	{
		cb_value=cb_value&0xE2;
		writeI2CData(0x01,cb_value);
	}
}
void cell_balance2(RegisterGroup *temp)
{
	uint8_t cb_value=0,flag_cell=0;
	cb_value=temp->CellBal2.CellBal2Bit.CB6;
	cb_value=temp->CellBal2.CellBal2Bit.CB7<<1;
	cb_value=temp->CellBal2.CellBal2Bit.CB8<<2;
	cb_value=temp->CellBal2.CellBal2Bit.CB9<<3;
	cb_value=temp->CellBal2.CellBal2Bit.CB10<<4;
	if((cb_value &0x0A)==0)
	{
		flag_cell=0;
	}
	if((cb_value&0x15)>1 && (cb_value&0x15)>=5 && flag_cell==0)
	{
		flag_cell=1;
		cb_value=cb_value&0xF5;
		writeI2CData(0x02,cb_value);
	}
	else if((cb_value&0x0A)>2 && flag_cell==1)
	{
		flag_cell=0;
		cb_value=cb_value&0xEA;
		writeI2CData(0x02,cb_value);
	}
	else if((cb_value&0x15)==1)
	{
		cb_value=cb_value&0xE1;
		writeI2CData(0x02,cb_value);
	}
	else if((cb_value&0x0A)==2)
	{
		cb_value=cb_value&0xE2;
		writeI2CData(0x02,cb_value);
	}
}
void cell_balance3(RegisterGroup *temp)
{
	uint8_t cb_value=0,flag_cell=0;
	cb_value=temp->CellBal3.CellBal3Bit.CB11;
	cb_value=temp->CellBal3.CellBal3Bit.CB12<<1;
	cb_value=temp->CellBal3.CellBal3Bit.CB13<<2;
	cb_value=temp->CellBal3.CellBal3Bit.CB14<<3;
	cb_value=temp->CellBal3.CellBal3Bit.CB15<<4;
	if((cb_value &0x0A)==0)
	{
		flag_cell=0;
	}
	if((cb_value&0x15)>1 && (cb_value&0x15)>=5 && flag_cell==0)
	{
		flag_cell=1;
		cb_value=cb_value&0xF5;
		writeI2CData(0x03,cb_value);
	}
	else if((cb_value&0x0A)>2 && flag_cell==1)
	{
		flag_cell=0;
		cb_value=cb_value&0xEA;
		writeI2CData(0x03,cb_value);
	}
	else if((cb_value&0x15)==1)
	{
		cb_value=cb_value&0xE1;
		writeI2CData(0x03,cb_value);
	}
	else if((cb_value&0x0A)==2)
	{
		cb_value=cb_value&0xE2;
		writeI2CData(0x03,cb_value);
	}
}
void adc_cell_value(uint16_t adc_cell , float *volt)
{
  *volt=((float)ADCGAIN1*adc_cell)+ADCOFFSET;
}
void get_adc_gain(RegisterGroup *temp)
{ 
   uint8_t temp_var1,temp_var2,adc_gain;
	temp_var1=(readI2CData(ADCGAIN1) & 0X0C)<1;
	temp_var2=(readI2CData(ADCGAIN2) & 0XE0)>>5;
	adc_gain=temp_var1+temp_var2;
	temp->adc_gain=(365.0f+((float)adc_gain))/1000;  //value in millivolt_check it while testing
}
void get_adc_offset(RegisterGroup *temp)
{
	uint8_t temp1;
	temp1=readI2CData(ADCOFFSET);
	temp->ADCOffset=~(char)temp1+1;
}







