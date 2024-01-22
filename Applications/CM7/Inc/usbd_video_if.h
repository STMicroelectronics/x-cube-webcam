 /**
 ******************************************************************************
 * @file    usbd_video_if.h
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
#ifndef __USBD_VIDEO_IF_H__
#define __USBD_VIDEO_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_video.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
  CAM_STATE_STOPPED = 0,
  CAM_STATE_SETUP,
  CAM_STATE_CONFIGURED
} camera_state_t;

/* Exported variables ------------------------------------------------------- */
extern USBD_VIDEO_ItfTypeDef USBD_VIDEO_fops;

extern volatile uint8_t *usb_tx_buff;
extern volatile uint32_t usb_tx_size;
extern volatile uint32_t usb_tx_complete;
extern volatile uint32_t camera_buf_size;
extern volatile uint32_t camera_format;
extern volatile uint32_t camera_res;
extern volatile uint32_t camera_res_width;
extern volatile uint32_t camera_res_height;
extern volatile camera_state_t camera_state;

#ifdef __cplusplus
}
#endif

#endif /* USBD_VIDEO_IF_H_ */

