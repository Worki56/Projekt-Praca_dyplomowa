/**
  ******************************************************************************
  * File Name          : app_touchgfx.c
  ******************************************************************************
  * This file was created by TouchGFX Generator 4.21.0. This file is only
  * generated once! Delete this file from your project and re-generate code
  * using STM32CubeMX or change this file manually to update it.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_touchgfx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void touchgfx_init(void);
void touchgfx_components_init(void);
void touchgfx_taskEntry(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/**
 * PreOS Initialization function
 */
void MX_TouchGFX_PreOSInit(void)
{
}

/**
 * Initialize TouchGFX application
 */
void MX_TouchGFX_Init(void)
{
  // Calling forward to touchgfx_init in C++ domain
  touchgfx_components_init();
  touchgfx_init();
}

/**
 * TouchGFX application entry function
 */
void MX_TouchGFX_Process(void)
{
  // Calling forward to touchgfx_taskEntry in C++ domain
  touchgfx_taskEntry();
}

/**
 * TouchGFX application thread
 */
void TouchGFX_Task(void *argument)
{
  // Calling forward to touchgfx_taskEntry in C++ domain
  touchgfx_taskEntry();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
