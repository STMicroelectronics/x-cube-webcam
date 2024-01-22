 /**
 ******************************************************************************
 * @file    stm32h747i_discovery_camera_ex.h
 * @author  GPM Application Team
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32H747I_DISCO_CAMERA_EX_H
#define STM32H747I_DISCO_CAMERA_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h747i_discovery_camera.h"

/* Exported constants --------------------------------------------------------*/

/*
  Set CAMERA_BOOST_PCLK=1 to operate the OV5640 camera in optimal mode.
  Note: when operating the camera in optimal mode, the pixel frequency is
  above 12MHz and FCC certification is no longer guaranteed.
*/
#ifndef CAMERA_BOOST_PCLK
#define CAMERA_BOOST_PCLK   0
#endif

/* Frame rates */
#define CAMERA_TIMING_12FPS    12
#define CAMERA_TIMING_15FPS    15
#define CAMERA_TIMING_30FPS    30

/* Exported functions --------------------------------------------------------*/
int32_t BSP_CAMERAEx_SetTiming(uint32_t Instance, uint32_t fps);

#ifdef __cplusplus
}
#endif

#endif /* STM32H747I_DISCO_CAMERA_H */

