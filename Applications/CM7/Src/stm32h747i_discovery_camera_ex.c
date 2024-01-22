 /**
 ******************************************************************************
 * @file    stm32h747i_discovery_camera_ex.c
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

/* Includes ------------------------------------------------------------------*/
#include "stm32h747i_discovery_bus.h"
#include "stm32h747i_discovery_camera_ex.h"

/* Private function prototypes -----------------------------------------------*/
static int32_t ov5640_SetTiming(uint32_t fps);
static int32_t ov9655_SetTiming(uint32_t fps);

/* Functions Definition ------------------------------------------------------*/

/**
 * @brief Set the camera frame rate
 *
 * @param Instance Camera instance
 * @param fps Frame rate. This parameter can be one of the following values:
 *              - CAMERA_TIMING_12FPS
 *              - CAMERA_TIMING_15FPS
 *              - CAMERA_TIMING_30FPS
 * @return int32_t BSP status
 */
int32_t BSP_CAMERAEx_SetTiming(uint32_t Instance, uint32_t fps)
{
  int32_t ret = BSP_ERROR_NONE;
  uint32_t camera_id;

  if(Instance >= CAMERA_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    camera_id = Camera_Ctx[0].CameraId;
    if ((camera_id == OV9655_ID) || (camera_id == OV9655_ID_2))
    {
      ret = ov9655_SetTiming(fps);
    }
    else if (camera_id == OV5640_ID)
    {
      ret = ov5640_SetTiming(fps);
    }
  }

  return ret;
}

static int32_t ov5640_SetTiming(uint32_t fps)
{
  int32_t ret = BSP_ERROR_NONE;

#if (CAMERA_BOOST_PCLK == 1)

  static const uint16_t ov5640_12fps[][2] =
  {
    {OV5640_SC_PLL_CONTRL2,  0x60}, /* Pixel clock: 24MHz */
    {OV5640_TIMING_HTS_HIGH, 0x06}, /* Total Horizontal size: 1600 */
    {OV5640_TIMING_HTS_LOW,  0x40},
    {OV5640_TIMING_VTS_HIGH, 0x04}, /* Total Vertical size:   1250 */
    {OV5640_TIMING_VTS_LOW,  0xE2}
  };

  static const uint16_t ov5640_15fps[][2] =
  {
    {OV5640_SC_PLL_CONTRL2,  0x60}, /* Pixel clock: 24MHz */
    {OV5640_TIMING_HTS_HIGH, 0x06}, /* Total Horizontal size: 1600 */
    {OV5640_TIMING_HTS_LOW,  0x40},
    {OV5640_TIMING_VTS_HIGH, 0x03}, /* Total Vertical size:   1000 */
    {OV5640_TIMING_VTS_LOW,  0xE8}
  };

  static const uint16_t ov5640_30fps[][2] =
  {
    {OV5640_SC_PLL_CONTRL2,  0xC0}, /* Pixel clock: 48MHz */
    {OV5640_TIMING_HTS_HIGH, 0x06}, /* Total Horizontal size: 1600 */
    {OV5640_TIMING_HTS_LOW,  0x40},
    {OV5640_TIMING_VTS_HIGH, 0x03}, /* Total Vertical size:   1000 */
    {OV5640_TIMING_VTS_LOW,  0xE8}
  };

  // TODO: Check if fps is supported

  switch (fps)
  {
  case CAMERA_TIMING_12FPS:
    for (uint32_t i = 0; i < (sizeof(ov5640_12fps) / 4U); i++)
    {
      if (ret == BSP_ERROR_NONE)
      {
        uint8_t tmp = (uint8_t)ov5640_12fps[i][1];
        if(BSP_I2C4_WriteReg16(CAMERA_OV5640_ADDRESS, ov5640_12fps[i][0], &tmp, 1) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          HAL_Delay(1);
        }
      }
    }
    break;

  case CAMERA_TIMING_30FPS:
    for (uint32_t i = 0; i < (sizeof(ov5640_30fps) / 4U); i++)
    {
      if (ret == BSP_ERROR_NONE)
      {
        uint8_t tmp = (uint8_t)ov5640_30fps[i][1];
        if(BSP_I2C4_WriteReg16(CAMERA_OV5640_ADDRESS, ov5640_30fps[i][0], &tmp, 1) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          HAL_Delay(1);
        }
      }
    }
    break;

  case CAMERA_TIMING_15FPS:
  default:
    for (uint32_t i = 0; i < (sizeof(ov5640_15fps) / 4U); i++)
    {
      if (ret == BSP_ERROR_NONE)
      {
        uint8_t tmp = (uint8_t)ov5640_15fps[i][1];
        if(BSP_I2C4_WriteReg16(CAMERA_OV5640_ADDRESS, ov5640_15fps[i][0], &tmp, 1) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          HAL_Delay(1);
        }
      }
    }
    break;

  }
#endif /* CAMERA_BOOST_PCLK */

  return ret;
}

static int32_t ov9655_SetTiming(uint32_t fps)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dummy_lines;

  // TODO: Check if fps is supported (with resolution check)

  /* VGA frame timing:
   * VSYNC period = (500 + dummy) * tLINE;           (tLINE = 800 * tP)
   *              = (500 + dummy) * 800 * tp;        (tp = 2x int px clock)
   *              = (500 + dummy) * 800 * 2 * 1/12e6 (12MHz pixel clock) */

  switch (fps)
  {
  case CAMERA_TIMING_12FPS: /* VGA */
    dummy_lines = 125; /* 12fps: 125 dummy frame lines (125 * 2/12MHz = 20.8us) */
    break;
  case CAMERA_TIMING_15FPS: /* VGA */
  case CAMERA_TIMING_30FPS: /* QVGA */
    dummy_lines = 0;
    break;
  }

  if (BSP_I2C4_WriteReg(CAMERA_OV9655_ADDRESS, OV9655_DMLNL, &dummy_lines, 1) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }

  return ret;
}

