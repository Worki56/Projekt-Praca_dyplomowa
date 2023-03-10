/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : TouchGFXGPIO.cpp
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
/* USER CODE END Header */

#include <touchgfx/hal/GPIO.hpp>

/**
 * GPIO_ID Enum
 * The signals represented by this enum are used by TouchGFX framework to signal internal events.
 *
 * VSYNC_FREQ,  /// Pin is toggled at each VSYNC
 * RENDER_TIME, /// Pin is high when frame rendering begins, low when finished
 * FRAME_RATE,  /// Pin is toggled when the frame buffers are swapped.
 * MCU_ACTIVE   /// Pin is high when framework is utilizing the MCU.
 *
 * Configure GPIO's with the same name as the GPIO_IDs above, as output, in CubeMX to export
 * the signals for performance measuring. See support.touchgfx.com for further details.
 *
 */

/* USER CODE BEGIN TouchGFXGPIO.cpp */

using namespace touchgfx;

/*
 * Perform configuration of IO pins.
 */
void GPIO::init()
{

}

/*
 * Sets a pin high.
 */
void GPIO::set(GPIO_ID id)
{

}

/*
 * Sets a pin low.
 */
void GPIO::clear(GPIO_ID id)
{

}

/*
 * Toggles a pin.
 */
void GPIO::toggle(GPIO_ID id)
{

}

/*
 * Gets the state of a pin.
 */
bool GPIO::get(GPIO_ID id)
{
    return false;
}

/* USER CODE END TouchGFXGPIO.cpp */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
