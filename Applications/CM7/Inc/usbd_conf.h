 /**
 ******************************************************************************
 * @file    usbd_conf.h
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
#ifndef __USBD_CONF__H__
#define __USBD_CONF__H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

/** @addtogroup USBD_OTG_DRIVER
  * @{
  */

/** @defgroup USBD_CONF USBD_CONF
  * @brief Configuration file for Usb otg low level driver.
  * @{
  */

/** @defgroup USBD_CONF_Exported_Variables USBD_CONF_Exported_Variables
  * @brief Public variables.
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CONF_Exported_Defines USBD_CONF_Exported_Defines
  * @brief Defines for configuration of the Usb device.
  * @{
  */

#define USBD_MAX_NUM_INTERFACES        1U
#define USBD_MAX_NUM_CONFIGURATION     1U
#define USBD_MAX_STR_DESC_SIZ          0x100U
#define USBD_DEBUG_LEVEL               0U
#define USBD_LPM_ENABLED               0U
#define USBD_SELF_POWERED              1U

/* VIDEO Class Config */
#define UVC_1_1

/* bEndpointAddress in Endpoint Descriptor */
#define UVC_IN_EP                                   0x81U

#define UVC_FPS_FS_MJPEG_VGA                        15U
#define UVC_FPS_FS_MJPEG_QVGA                       30U
#define UVC_FPS_FS_YUY2_VGA                         1U
#define UVC_FPS_FS_YUY2_QVGA                        6U

#define UVC_FPS_HS_MJPEG_VGA                        15U
#define UVC_FPS_HS_MJPEG_QVGA                       30U
#define UVC_FPS_HS_YUY2_VGA                         12U
#define UVC_FPS_HS_YUY2_QVGA                        30U

#define UVC_ISO_FS_MPS                              1023U
#define UVC_ISO_HS_MPS                              1024U
#define UVC_PACKET_SIZE                             1024U
#define UVC_MAX_FRAME_SIZE_VGA                      (640U * 480U * 16U / 8U)
#define UVC_MAX_FRAME_SIZE_QVGA                     (320U * 240U * 16U / 8U)

/**
  * @}
  */

/** @defgroup USBD_CONF_Exported_Macros USBD_CONF_Exported_Macros
  * @brief Aliases.
  * @{
  */

/* Memory management macros */

/** Alias for memory allocation. */
#define USBD_malloc         malloc

/** Alias for memory release. */
#define USBD_free           free

/** Alias for memory set. */
#define USBD_memset         memset

/** Alias for memory copy. */
#define USBD_memcpy         memcpy

/** Alias for delay. */
#define USBD_Delay          HAL_Delay

/* DEBUG macros */

#if (USBD_DEBUG_LEVEL > 0)
#define USBD_UsrLog(...)    printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_UsrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 1)

#define USBD_ErrLog(...)    printf("ERROR: ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_ErrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 2)
#define USBD_DbgLog(...)    printf("DEBUG : ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_DbgLog(...)
#endif

/**
  * @}
  */

/** @defgroup USBD_CONF_Exported_Types USBD_CONF_Exported_Types
  * @brief Types.
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CONF_Exported_FunctionsPrototype USBD_CONF_Exported_FunctionsPrototype
  * @brief Declaration of public functions for Usb device.
  * @{
  */

/* Exported functions -------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd;
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF__H__ */

