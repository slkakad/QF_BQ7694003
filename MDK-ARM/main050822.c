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
#include <stdlib.h>
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bqMaximo_Ctrl_G2553.h"
#include "bq_setting_QF.h"
#include "Protection.h"
#include "PackStatus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
RegisterGroup Registers;
PackStateTypedef CellBoundery;
OperStatusTypedef OperationStatus;
UserProtectionSettingTypeDef UserSetting;
TempratureStatusTypedef TempratureStatus;
CellVoltageProtectionStatusTypedef CellVoltageProtectionStatus;
PackViolationTypedef PackViolation;
PackMesurmentTypedef PackMesurment;


const unsigned int OVPThreshold = 4300;//UserSetting.WindowCellVoltMax;    
const unsigned int UVPThreshold = 2500; //UserSetting.WindowCellVoltMin;
const unsigned char SCDDelay = SCD_DELAY_50us;
const unsigned char SCDThresh = SCD_THRESH_89mV_44mV;			// User Given and proceessed 
const unsigned char OCDDelay = OCD_DELAY_320ms;				// user given
const unsigned char OCDThresh = OCD_THRESH_22mV_11mV;		// User given Drop Down Menu
const unsigned char OVDelay = OV_DELAY_2s;
const unsigned char UVDelay = UV_DELAY_8s;

float Gain = 0;
int iGain = 0, Result = 0;

uint8_t MaxTempSensorCount = 0,MinTempSensorCount = 0;
uint8_t TIM3_FLAG=0;
uint8_t SysConfig ;
uint32_t CellSum=0;



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



OperStatusTypedef GetADCGainOffset()    /* I2C byter read*/ 
{
	OperationStatus= I2CReadBlockWithCRC(&hi2c1,BQ_ADD,ADCGAIN1,&(Registers.ADCGain1.ADCGain1Byte),3);
	return OperationStatus;
}
OperStatusTypedef ConfigureBqMaximo()
{
	unsigned char bqMaximoProtectionConfig[5]; 
	OperStatusTypedef result;
  result=I2CWriteBlockWithCRC(&hi2c1,BQ_ADD, PROTECT1,&(Registers.Protect1.Protect1Byte),5);
	if (result != Oper_OK){
		return result;
	}
  result=I2CReadBlockWithCRC(&hi2c1,BQ_ADD, PROTECT1,bqMaximoProtectionConfig,5);
	if (result != Oper_OK){
		return result;
	} 
	if(bqMaximoProtectionConfig[0] != Registers.Protect1.Protect1Byte
			|| bqMaximoProtectionConfig[1] != Registers.Protect2.Protect2Byte
			|| bqMaximoProtectionConfig[2] != Registers.Protect3.Protect3Byte
			|| bqMaximoProtectionConfig[3] != Registers.OVTrip
			|| bqMaximoProtectionConfig[4] != Registers.UVTrip)
	{
		return Oper_WRFail;
	}
	return Oper_OK;
}
OperStatusTypedef InitialisebqMaximo()
{
	OperStatusTypedef result;
	Registers.Protect1.Protect1Bit.SCD_DELAY = SCDDelay;
	Registers.Protect1.Protect1Bit.SCD_THRESH = SCDThresh;
	Registers.Protect2.Protect2Bit.OCD_DELAY = OCDDelay;
	Registers.Protect2.Protect2Bit.OCD_THRESH = OCDThresh;
	Registers.Protect3.Protect3Bit.OV_DELAY = OVDelay;
	Registers.Protect3.Protect3Bit.UV_DELAY = UVDelay;
	
	result = GetADCGainOffset();
	if(result != Oper_OK)
	{
		return result;
	}

	Gain = (365 + ((Registers.ADCGain1.ADCGain1Byte & 0x0C) << 1) + ((Registers.ADCGain2.ADCGain2Byte & 0xE0)>> 5)) / 1000.0;
	iGain = 365 + ((Registers.ADCGain1.ADCGain1Byte & 0x0C) << 1) + ((Registers.ADCGain2.ADCGain2Byte & 0xE0)>> 5);

    Registers.OVTrip = (unsigned char)((((unsigned short)((OVPThreshold - Registers.ADCOffset)/Gain + 0.5f) - OV_THRESH_BASE) >> 4) & 0xFF);
    Registers.UVTrip = (unsigned char)((((unsigned short)((UVPThreshold - Registers.ADCOffset)/Gain + 0.5f) - UV_THRESH_BASE) >> 4) & 0xFF);

    result = ConfigureBqMaximo();

    return result;
}
OperStatusTypedef UpdateCellVoltage()
{	
	uint8_t i=0;
	unsigned char *pRawADCData = NULL;
	unsigned int iTemp = 0;
	unsigned long lTemp = 0;
	
	OperationStatus = I2CReadBlockWithCRC(&hi2c1,BQ_ADD,VC1_HI_BYTE,&(Registers.VCell1.VCell1Byte.VC1_HI),(CellCount*2));
	if (OperationStatus != Oper_OK)
	{
		return OperationStatus;
	}
	pRawADCData = &Registers.VCell1.VCell1Byte.VC1_HI;
	CellSum = 0;
	for (i = 0; i < 15; i++)
	{
		iTemp = (unsigned int)(unsigned int)(*pRawADCData << 8) + *(pRawADCData + 1);
		lTemp = ((unsigned long)iTemp * iGain)/1000;
		lTemp += Registers.ADCOffset;
		PackMesurment.CellVoltage[i] = lTemp;
    CellSum = CellSum + PackMesurment.CellVoltage[i];
		pRawADCData += 2;
	}
	return Oper_OK;
}
OperStatusTypedef UpdateBatteryVolatge()
{
	 OperStatusTypedef status;
	 status = I2CReadBlockWithCRC(&hi2c1,BQ_ADD,0x2A,&(Registers.VBat.VBatByte.BAT_HI),2);
	 //Result=HAL_I2C_Mem_Read(&hi2c1,((BQ_ADD)<<1)|0x01,42, 1,&(Registers.VBat.VBatByte.BAT_HI),sizeof(Registers.VBat.VBatByte.BAT_HI),TimeOutI2C);
	 //Result=HAL_I2C_Mem_Read(&hi2c1,((BQ_ADD)<<1)|0x01,43, 1,&(Registers.VBat.VBatByte.BAT_LO),sizeof(Registers.VBat.VBatByte.BAT_LO),TimeOutI2C);
   PackMesurment.BatteryVoltage= 4*Gain*(((Registers.VBat.VBatByte.BAT_HI)<<8)+ Registers.VBat.VBatByte.BAT_LO)+(CellCount*Registers. ADCOffset);
	return status;
}
OperStatusTypedef UpdateExternalTemp(float *TempStore , uint8_t length)
{		
	OperStatusTypedef status;
	uint8_t i=0,*pRawADCData = NULL ;
	uint16_t tempRes,iTemp=0;
	unsigned long lTemp = 0;
	float TempResLog = 0;
	float *pPointer = TempStore;
	
	status  = I2CReadBlockWithCRC(&hi2c1,BQ_ADD,TEMP_Hi,&(Registers.TS1.TS1Byte.TS1_HI),(length*2));
	if (status != Oper_OK)
	{
		return status;
	}
	pRawADCData =  &(Registers.TS1.TS1Byte.TS1_HI);
	for (i = 0; i < length; i++)
	{
		iTemp = (unsigned int)(unsigned int)(*pRawADCData << 8) + *(pRawADCData + 1);
		lTemp = iTemp*382/1000;
		tempRes =(10000*lTemp)/(3300-lTemp);
		TempResLog = log((double)tempRes/RT0)/TemperatureBeta;
		*pPointer= 1.0f / (TempResLog + (1.0f /Temperature0));
		*pPointer  =  *pPointer  - 273.0f;
		pPointer += 1;
		pRawADCData += 2;
	}
	return status;
 }
OperStatusTypedef UpdateDieTemp(float *TempStore , uint8_t length)
{
	OperStatusTypedef status;
	uint8_t i=0,*pRawADCData = NULL ;
	uint16_t iTemp=0;
	unsigned long lTemp = 0;
	float *pPointer = TempStore;
	
	status  = I2CReadBlockWithCRC(&hi2c1,BQ_ADD,TEMP_Hi,&(Registers.TS1.TS1Byte.TS1_HI),(length*2));
	if (status != Oper_OK)
	{
		return status;
	}
	pRawADCData =  &(Registers.TS1.TS1Byte.TS1_HI);
	for (i = 0; i < length; i++)
	{
		iTemp = (unsigned int)(unsigned int)(*pRawADCData << 8) + *(pRawADCData + 1);
		lTemp = iTemp*382/1000;
		*pPointer= 25 - ((lTemp - 1200)/4.2f);
		pPointer += 1;
		pRawADCData += 2;
	}
	return status;
}
OperStatusTypedef UpdateCurrent()
{
	OperStatusTypedef status;
	int16_t RawValue=0;
	status = I2CReadBlockWithCRC(&hi2c1,BQ_ADD,CC_HI_Add,&(Registers.CC.CCByte.CC_HI),2);
	RawValue = (Registers.CC.CCByte.CC_HI << 8) +Registers.CC.CCByte.CC_LO;
	PackMesurment.Current = (int32_t)(RawValue * (8440/ UserSetting.CurrentShuntResistor)); 
	return status;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t TempratureSelFlag=0;
	uint32_t Voltage_Check = 0;
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
	
	OperationStatus =  InitialisebqMaximo();
	// External Thermister Selection 
	Registers.SysCtrl1.SysCtrl1Bit.TEMP_SEL=1;
	OperationStatus = I2CReadRegisterWithCRC(&hi2c1,BQ_ADD,SYS_CTRL1,&(Registers.SysCtrl1.SysCtrl1Byte));
	// System Setting 	
	Registers.CCCfg = 0x19;
	OperationStatus = I2CWriteRegistorWithCRC(&hi2c1,BQ_ADD,0x0B,Registers.CCCfg);
	// Current sensing in on shot mode 
	I2CReadRegisterWithCRC(&hi2c1,BQ_ADD,SYS_CTRL2,&(Registers.SysCtrl2.SysCtrl2Byte));
	Registers.SysCtrl2.SysCtrl2Bit.CC_EN = 0;
	Registers.SysCtrl2.SysCtrl2Bit.CC_ONESHOT = 1;
	I2CWriteRegistorWithCRC(&hi2c1,BQ_ADD,SYS_CTRL2,Registers.SysCtrl2.SysCtrl2Byte);
	
	// external inturrupt pin on ALERT PIN and Decide sub routing 
  // Setting of Thrshold Value  and Delay's for Protection (OV, UV, OC,SC with Delay)
	// Get privios value of SD card  Intial SoC, R0,R1,C1 SoH
	// Check for Device X ready
	// CC and ADC Enable 
	
  /* USER CODE END 2 */
	MosfetSwitching(MOSFET_OFF);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		I2CReadRegisterWithCRC(&hi2c1,BQ_ADD,SYS_STAT,&(Registers.SysStatus.StatusByte));
		I2CReadRegisterWithCRC(&hi2c1,BQ_ADD,SYS_CTRL1,&(Registers.SysCtrl1.SysCtrl1Byte));
		I2CReadRegisterWithCRC(&hi2c1,BQ_ADD,SYS_CTRL2,&(Registers.SysCtrl2.SysCtrl2Byte));
		
		if(Registers.SysCtrl1.SysCtrl1Bit.LOAD_PRESENT  && !Registers.SysCtrl2.SysCtrl2Bit.CC_EN  &&  (Registers.SysStatus.StatusByte & 0x7F) == 0x00)
		{
			// Load is connected and DSG_PIN can be 1  and no fault is detected
			// Depends upon can bus data can be set for testing 
		}			
		I2CReadRegisterWithCRC(&hi2c1,BQ_ADD,SYS_STAT,&(Registers.SysStatus.StatusByte));
		if(TIM3_FLAG== 1 && Registers.SysStatus.StatusBit.CC_READY == 1 && SysConfig == 0)
			{
				TIM3_FLAG = 0;
				Voltage_Check++;
				TempratureSelFlag++;
				OperationStatus = UpdateCellVoltage();
				PackViolation.CellVoltViolation=CellVoltageProtection();
				OperationStatus= UpdateBatteryVolatge();
				// Check Battery Voltage and ADC Voltage
				if (TempratureSelFlag <= DieTempraturTime)
				{
					OperationStatus= UpdateExternalTemp(PackMesurment.ExternalTempraturarSense,3);
					CellBoundery.TempratureSenseIndicator = ExternalThemisterSense ;
					PackViolation.ThermalViolation=	TempratureProtection(ExternalThemisterSense);						
				}
				else if (TempratureSelFlag > DieTempraturTime)
				{
					if( TempratureSelFlag < (DieTempraturTime+2))
					{
						if(Registers.SysCtrl1.SysCtrl1Bit.TEMP_SEL !=0)
						{
							Registers.SysCtrl1.SysCtrl1Bit.TEMP_SEL = 0;
							OperationStatus = I2CWriteRegistorWithCRC(&hi2c1,BQ_ADD,SYS_CTRL1,Registers.SysCtrl1.SysCtrl1Byte);
						}						
					}
				else if (TempratureSelFlag == (DieTempraturTime+2))
				{
					OperationStatus= UpdateDieTemp(PackMesurment.DieTempraturarSense,2);
					CellBoundery.TempratureSenseIndicator = InternalDieSense;
					PackViolation.ThermalViolation=TempratureProtection(InternalDieSense);	
				}
				else if(TempratureSelFlag > (DieTempraturTime+5))
				{
					if(Registers.SysCtrl1.SysCtrl1Bit.TEMP_SEL !=1)
					{
						Registers.SysCtrl1.SysCtrl1Bit.TEMP_SEL = 1;
						OperationStatus = I2CWriteRegistorWithCRC(&hi2c1,BQ_ADD,SYS_CTRL1,Registers.SysCtrl1.SysCtrl1Byte);
					}				
				}
				else if(TempratureSelFlag == (DieTempraturTime+5))
					{
						TempratureSelFlag = 0;
					}
				}
			OperationStatus= UpdateCurrent();
			PackViolation.PackCurrentViolation=CurrentProtection();
				
			//Comparition of all cell voltage and Battery voltage and Gain 2 for varification and gain calculation 
			//RLMS adn UKF for every cell 
			// Store data in SD card 
			// SoC balancing charge discharge ideal 
			// SoH_Flag == 1  (after day of week ) calculate SOH  once only  store it in EPROM SD card or flash
			// CAN 
			// RS485 
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
