/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bqMaximo_Ctrl_G2553.h"
//#include "battery_pack_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BQ_Add 0x08

#define TimeOutI2C 50

#define CellCount 15
#define TempCount 3
#define ExternalTemprature 3
#define TempCountLimit 5
#define CellOVCount 2
#define CellUVCount 8

#define TIM3_FREQ 1
#define TIM3_PRES 9999
#define TIM3_ARR (8400/TIM3_FREQ)-1

#define Max_Temp_Allowed 85
#define Min_Temp_Allowed 05

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
RegisterGroup Registers;
const unsigned int OVPThreshold = 4300;
const unsigned int UVPThreshold = 2500;
const unsigned char SCDDelay = SCD_DELAY_100us;
const unsigned char SCDThresh = SCD_THRESH_89mV_44mV;
const unsigned char OCDDelay = OCD_DELAY_320ms;
const unsigned char OCDThresh = OCD_THRESH_22mV_11mV;
const unsigned char OVDelay = OV_DELAY_2s;
const unsigned char UVDelay = UV_DELAY_8s;
float Gain = 0;
int iGain = 0, Result = 0;

uint8_t MaxTempCount = 0,MinTempCount = 0;
HAL_StatusTypeDef status;
int status2= -2;
uint8_t TIM3_FLAG=0;
uint32_t CellSum=0;
uint16_t CellVoltage[15];
uint16_t  BatteryVoltage;
uint16_t TempSense[3];
uint16_t Cell_Max, Cell_Min, Temp_Max, Temp_Min;
uint8_t num=1;
	unsigned char bqMaximoProtectionConfig[5]; 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void Minimum(uint16_t *Buffer,uint8_t size,uint16_t *Minimum_val)
{
	uint16_t *pPointer=NULL;
	pPointer  =  Buffer;
	*Minimum_val = 65535;
	for(uint8_t i = 0;i<=size;i++)
	{
		if (*pPointer < *Minimum_val)
		{
			*Minimum_val = *pPointer;
		}
		*pPointer +=1;
	}
}
void Maximum(uint16_t *Buffer,uint8_t size,uint16_t *Maxvalue_val)
{
	uint16_t *pPointer=NULL;
	pPointer  =  Buffer;
	*Maxvalue_val = 0;
	for(uint8_t i = 0;i<=size;i++)
	{
		if (*pPointer > *Maxvalue_val)
		{
			*Maxvalue_val= *pPointer;
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
int I2CWriteRegistorWithCRC(I2C_HandleTypeDef *hi2c,uint8_t I2C_Address,uint8_t Register,uint8_t Data)
{
	unsigned char DataBuffer[4];
	unsigned int SentByte = 0;

  DataBuffer[0] = I2C_Address << 1;
	DataBuffer[1] = Register;
	DataBuffer[2] = Data;
	DataBuffer[3] = CRC8(DataBuffer, 3, CRC_KEY);
	HAL_I2C_Master_Transmit(hi2c,(I2C_Address << 1),(DataBuffer+1),3,TimeOutI2C);
}
int I2CWriteBlockWithCRC(I2C_HandleTypeDef *hi2c,uint8_t I2C_Address,uint8_t Register,uint8_t *Buffer,uint8_t Length)
{
	unsigned char *BufferCRC, *Pointer;
	int i;

	BufferCRC = (unsigned char*)malloc(2*Length + 2);
	if (NULL == BufferCRC)
		return -1;

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
	free(BufferCRC);
	BufferCRC = NULL;

	return status;
}


int I2CReadBlockWithCRC(I2C_HandleTypeDef *hi2c,uint8_t I2C_Address,uint8_t Register,uint8_t *Buffer,uint8_t Length)
{
	uint8_t *ReadData=NULL, *StartData = NULL;;
	unsigned char CRCInput[2];
	unsigned char CRC_Value = 0;
	StartData = (unsigned char *)malloc(2 * Length);
	 int i;
	if (NULL == StartData)
	{
    	return -1;
	}
	ReadData = StartData;

	status = HAL_I2C_Master_Transmit(hi2c,(I2C_Address << 1),&Register,1,TimeOutI2C);    // CRC -> N
	status = HAL_I2C_Master_Receive(&hi2c1,(I2C_Address<<1)+0x01,ReadData,(2*Length),TimeOutI2C);  // CRC -> Y
	
	CRCInput[0] = (BQ_Add<< 1) + 1;
	CRCInput[1] = *ReadData;
	
	CRC_Value = CRC8(CRCInput, 2, CRC_KEY);
	ReadData++;
	if (CRC_Value != *ReadData)
	{
		free(StartData);
		StartData = NULL;
		return -1;
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
			free(StartData);
			StartData = NULL;

			return -1;
		}
		else
			*Buffer = *(ReadData - 1);
	}

	free(StartData);
	StartData = NULL;

	return 0;
	
}
int I2CReadRegisterByteWithCRC(I2C_HandleTypeDef *hi2c,unsigned char I2CSlaveAddress, unsigned char Register, unsigned char *Data)
{
	unsigned char TargetRegister = Register;
	unsigned int SentByte = 0;
	unsigned char ReadData[2];
	unsigned int ReadDataCount = 0;
	unsigned char CRCInput[2];
	unsigned char CRCValue = 0;
  int ReadStatus = 0;
  int WriteStatus = 0;
	
	WriteStatus = HAL_I2C_Master_Transmit(hi2c,(I2CSlaveAddress << 1),&TargetRegister,1,TimeOutI2C);
	ReadStatus =  HAL_I2C_Master_Receive(hi2c,(I2CSlaveAddress << 1)+1,ReadData,2,TimeOutI2C);

	if (ReadStatus != 0 || WriteStatus != 0)
	{
		return -1;
	}

	CRCInput[0] = (I2CSlaveAddress << 1) + 1;
	CRCInput[1] = ReadData[0];

	CRCValue = CRC8(CRCInput, 2, CRC_KEY);

	if (CRCValue != ReadData[1])
		return -1;

	*Data = ReadData[0];
	return 0;
}

int GetADCGainOffset()
{
	int result;
	result=HAL_I2C_Mem_Read(&hi2c1,((BQ_Add)<<1)|0x01, ADCGAIN1, 1,&(Registers.ADCGain1.ADCGain1Byte),sizeof(Registers.ADCGain1.ADCGain1Byte),TimeOutI2C);
  result=HAL_I2C_Mem_Read(&hi2c1,((BQ_Add)<<1)|0x01, ADCGAIN2, 1,&(Registers.ADCGain2.ADCGain2Byte),sizeof(Registers.ADCGain1.ADCGain1Byte),TimeOutI2C);
  result=HAL_I2C_Mem_Read(&hi2c1,((BQ_Add)<<1)|0x01, ADCOFFSET, 1,&(Registers. ADCOffset),sizeof(Registers. ADCOffset),TimeOutI2C);
	return result;
}
int ConfigureBqMaximo()
{

  Result=I2CWriteBlockWithCRC(&hi2c1,BQ_Add, PROTECT1,&(Registers.Protect1.Protect1Byte),5);
  Result=I2CReadBlockWithCRC(&hi2c1,BQ_Add, PROTECT1,bqMaximoProtectionConfig,5);
	if(bqMaximoProtectionConfig[0] != Registers.Protect1.Protect1Byte
			|| bqMaximoProtectionConfig[1] != Registers.Protect2.Protect2Byte
			|| bqMaximoProtectionConfig[2] != Registers.Protect3.Protect3Byte
			|| bqMaximoProtectionConfig[3] != Registers.OVTrip
			|| bqMaximoProtectionConfig[4] != Registers.UVTrip)
	{
		Result = -1;
	}
	return Result;
}
int InitialisebqMaximo()
{
	int result = 0;
//	Registers.SysCtrl1.SysCtrl1Bit.TEMP_SEL = 1;
//	Registers.SysCtrl1.SysCtrl1Bit.ADC_EN = 1;
//	Registers.SysCtrl1.SysCtrl1Bit.SHUT_A = 0;
//	Registers.SysCtrl1.SysCtrl1Bit.SHUT_B = 0;
//	
//	Registers.SysCtrl2.SysCtrl2Bit.CC_EN = 0;
//	Registers.SysCtrl2.SysCtrl2Bit.CC_ONESHOT = 0;
//	Registers.SysCtrl2.SysCtrl2Bit.DSG_ON = 0;
//	Registers.SysCtrl2.SysCtrl2Bit.CHG_ON = 0;
	
	Registers.Protect1.Protect1Bit.SCD_DELAY = SCDDelay;
	Registers.Protect1.Protect1Bit.SCD_THRESH = SCDThresh;
	Registers.Protect2.Protect2Bit.OCD_DELAY = OCDDelay;
	Registers.Protect2.Protect2Bit.OCD_THRESH = OCDThresh;
	Registers.Protect3.Protect3Bit.OV_DELAY = OVDelay;
	Registers.Protect3.Protect3Bit.UV_DELAY = UVDelay;
	
	result = GetADCGainOffset();

	Gain = (365 + ((Registers.ADCGain1.ADCGain1Byte & 0x0C) << 1) + ((Registers.ADCGain2.ADCGain2Byte & 0xE0)>> 5)) / 1000.0;
	iGain = 365 + ((Registers.ADCGain1.ADCGain1Byte & 0x0C) << 1) + ((Registers.ADCGain2.ADCGain2Byte & 0xE0)>> 5);

    Registers.OVTrip = (unsigned char)((((unsigned short)((OVPThreshold - Registers.ADCOffset)/Gain + 0.5f) - OV_THRESH_BASE) >> 4) & 0xFF);
    Registers.UVTrip = (unsigned char)((((unsigned short)((UVPThreshold - Registers.ADCOffset)/Gain + 0.5f) - UV_THRESH_BASE) >> 4) & 0xFF);

    result = ConfigureBqMaximo();

    return result;
}
int UpdateVoltageFromBqMaximo()
{	
	uint8_t i=0;
	unsigned char *pRawADCData = NULL;
	unsigned int iTemp = 0;
	unsigned long lTemp = 0;
	
	I2CReadBlockWithCRC(&hi2c1,BQ_Add,VC1_HI_BYTE,&(Registers.VCell1.VCell1Byte.VC1_HI),(CellCount*2));
	
	pRawADCData = &Registers.VCell1.VCell1Byte.VC1_HI;
		for (i = 0; i < 15; i++)
	{
		iTemp = (unsigned int)(unsigned int)(*pRawADCData << 8) + *(pRawADCData + 1);
		lTemp = ((unsigned long)iTemp * iGain)/1000;
		lTemp += Registers.ADCOffset;
		CellVoltage[i] = lTemp;
		pRawADCData += 2;
	}
	return 0;
}
int UpdateBatteryVolatge()
{
	 Result=HAL_I2C_Mem_Read(&hi2c1,((BQ_Add)<<1)|0x01,42, 1,&(Registers.VBat.VBatByte.BAT_HI),sizeof(Registers.VBat.VBatByte.BAT_HI),TimeOutI2C);
	 Result=HAL_I2C_Mem_Read(&hi2c1,((BQ_Add)<<1)|0x01,43, 1,&(Registers.VBat.VBatByte.BAT_LO),sizeof(Registers.VBat.VBatByte.BAT_LO),TimeOutI2C);
   BatteryVoltage= 4*Gain*(((Registers.VBat.VBatByte.BAT_HI)<<8)+ Registers.VBat.VBatByte.BAT_LO)+(CellCount*Registers. ADCOffset);
	return Result;
}
int UpdateTempFromBqMaximo()
{		
	int Result;
	uint8_t i=0;
	unsigned int iTemp = 0;
	unsigned long lTemp = 0;
	unsigned char *pRawADCData = NULL ;
	uint16_t *pPointer = TempSense;
	Result  = I2CReadBlockWithCRC(&hi2c1,BQ_Add,TEMP_Hi,&(Registers.TS1.TS1Byte.TS1_HI),(ExternalTemprature*2));
	pRawADCData =  &(Registers.TS1.TS1Byte.TS1_HI);
	for (i = 0; i < ExternalTemprature; i++)
	{
		iTemp = (unsigned int)(unsigned int)(*pRawADCData << 8) + *(pRawADCData + 1);
		lTemp = iTemp*382.0f/1000;
		*pPointer =(10000*lTemp)/(3.3f-lTemp);
		pPointer += 1;
		pRawADCData += 2;
	}
	return Result;
 }
int coulomb_counter()
{
Result=HAL_I2C_Mem_Read(&hi2c1,((BQ_Add)<<1)|0x01,CC_HI_Add, 1,&(Registers.CC.CCByte.CC_HI),sizeof(Registers.CC.CCByte.CC_HI),TimeOutI2C);
Result=HAL_I2C_Mem_Read(&hi2c1,((BQ_Add)<<1)|0x01,CC_LO_Add, 1,&(Registers.CC.CCByte.CC_LO),sizeof(Registers.CC.CCByte.CC_LO),TimeOutI2C);
return Result;
}
void TempratureProtection()
{
	Maximum(TempSense,TempCount,&Temp_Max);
	Minimum(TempSense,TempCount,&Temp_Min);
	if (Temp_Min < Min_Temp_Allowed)
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);   // LED indication
		// Error Logging 
		MaxTempCount++;		
	}
	if (Temp_Max > Max_Temp_Allowed)
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);  // LED indication
		// Error Logging 
		MinTempCount++;
	}
	if (MinTempCount > TempCountLimit   ||  MaxTempCount > TempCountLimit )
	{
		//Error Loggoing 
		// OFF the Mosfet or Contector setting 
		// Give Thermal Managment system 
		// Flag ==1
	}
	if (Temp_Max < Max_Temp_Allowed && Temp_Min > Min_Temp_Allowed)
	{
		//Dealy 
		// CAN transimistion 
		// Error Loggin 
		// if Load is connected 
		// Counter arranement
	}
}
void CellProtection()
{
	static uint8_t OVPCounter=0, UVPCounter=0;
	Maximum(CellVoltage,CellCount,&Cell_Max);
	Minimum(CellVoltage,CellCount,&Cell_Min);
	if (Cell_Max >  OVPThreshold)
	{
		OVPCounter++;
	}
	if (Cell_Min <  UVPThreshold)
	{
		UVPCounter++;
	}
	if(UVPCounter >=  CellUVCount || OVPCounter >=  CellOVCount)
	{
		// CHG DSG PIN Off
	}
	
}
void MaxCurrentCal()
{
	/*if (Current  > 0)
			{
				Cal_MaxDischargeCurrent();
			  send it to CAN
			 Set into BQ Registeor
			
		}
				if (Current  > 0)
			{
				Cal_MaxDischargeCurrent();
				send it to CAN 
				Set into BQ Registeor
			} */
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	uint8_t TimerBaseFlag =0;					// Timer At 1 Second
	uint8_t TempratureFlag =0;	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);
	status2 =  InitialisebqMaximo();
	status2 = I2CReadRegisterByteWithCRC(&hi2c1,BQ_Add,SYS_CTRL1,&(Registers.SysCtrl1.SysCtrl1Byte));
	
	
	
	// external inturrupt pin on ALERT PIN and Decide sub routing 
  // Setting of Thrshold Value  and Delay's for Protection (OV, UV, OC,SC with Delay)
	// Get privios value of SD card  Intial SoC, R0,R1,C1 SoH
	// Check for Device X ready
	// CC and ADC Enable 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//Check for Device XReady flag include with below if with another flag 
		if(TIM3_FLAG== 1)				// TimerBaseFlag at every 1 second
		{
			TIM3_FLAG = 0;
		  TempratureFlag++;
			Result = UpdateVoltageFromBqMaximo();
			//coulomb_counter()
			// CellCurrentProtection();
			// MaxCurrentCal();
			CellProtection();
			CellSum = 0;
			for(uint8_t temp = 0;temp<=15;temp++)
			{
				CellSum = CellSum + CellVoltage[temp];
			}
			Result= UpdateBatteryVolatge();
			if(TempratureFlag == 1)
			{
				TempratureFlag = 0;
				status2= HAL_I2C_Mem_Write(&hi2c1,(BQ_Add)<<1,SYS_CTRL1,1,&(Registers.SysCtrl1.SysCtrl1Byte),1,TimeOutI2C);
				Result= UpdateTempFromBqMaximo();	
				TempratureProtection();
			}
//			Comparition of all cell voltage and Battery voltage and Gain 2 for varification and gain calculation 
			//RLMS adn UKF for every cell 
			// Store data in SD card 
			// SoC balancing charge discharge ideal 
			
			// SoH_Flag == 1  (after day of week ) calculate SOH  once only  store it in EPROM SD card or flash
			// CAN 
			// RS485 
//			
		  Result=coulomb_counter();
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler =TIM3_PRES;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = TIM3_ARR;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
