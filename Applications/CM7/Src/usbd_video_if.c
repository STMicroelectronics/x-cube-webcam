 /**
 ******************************************************************************
 * @file    usbd_video_if.c
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
#include "usbd_video_if.h"
#include "stm32h747i_discovery_camera.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern USBD_HandleTypeDef husb;

volatile uint8_t *usb_tx_buff;
volatile uint32_t usb_tx_size;
volatile uint32_t usb_tx_complete;
volatile uint32_t camera_buf_size;
volatile uint32_t camera_format;
volatile uint32_t camera_res;
volatile uint32_t camera_res_width;
volatile uint32_t camera_res_height;
volatile camera_state_t camera_state;

/* Private function prototypes ---------------------------------------------- */
static int8_t VIDEO_Itf_Init(void);
static int8_t VIDEO_Itf_DeInit(void);
static int8_t VIDEO_Itf_Start(void);
static int8_t VIDEO_Itf_Stop(void);
static int8_t VIDEO_Itf_Control(USBD_VideoControlTypeDef *pctrl);
static int8_t VIDEO_Itf_Data(uint8_t** pbuf, uint16_t* psize, uint16_t* pcktidx);

USBD_VIDEO_ItfTypeDef USBD_VIDEO_fops =
{
  VIDEO_Itf_Init,
  VIDEO_Itf_DeInit,
  VIDEO_Itf_Start,
  VIDEO_Itf_Stop,
  VIDEO_Itf_Control,
  VIDEO_Itf_Data,
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the VIDEO media low layer
  *
  * @param  None
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t VIDEO_Itf_Init(void)
{
  return (0);
}

/**
  * @brief  DeInitializes the UVC media low layer
  *
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t VIDEO_Itf_DeInit(void)
{
  return (0);
}

/**
  * @brief  VideoStreaming interface started callback
  *
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t VIDEO_Itf_Start(void)
{
  usb_tx_complete = 1;

  return (0);
}

/**
  * @brief  VideoStreaming interface stopped callback
  *
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t VIDEO_Itf_Stop(void)
{
  BSP_CAMERA_Stop(0);
  camera_state = CAM_STATE_STOPPED;
  usb_tx_complete = 0;

  return (0);
}

/**
  * @brief  Manage the UVC class VS set commit control requests
  *
  * @param  pctrl: pointer to video controls
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t VIDEO_Itf_Control(USBD_VideoControlTypeDef *pctrl)
{
  uint8_t usb_format_index = pctrl->bFormatIndex;
  uint8_t usb_frame_index = pctrl->bFrameIndex;
  uint32_t num_pixels;

  if (usb_frame_index == 0x01)
  {
    camera_res = CAMERA_R640x480;
    camera_res_width = 640;
    camera_res_height = 480;
    num_pixels = 307200;
  }
  else
  {
    camera_res = CAMERA_R320x240;
    camera_res_width = 320;
    camera_res_height = 240;
    num_pixels = 76800;
  }

  if (usb_format_index == 0x01 /* MJPEG Format index */)
  {
    camera_format = CAMERA_PF_RGB565;
    camera_buf_size = num_pixels * 2; /* 16bpp */
  }
  else
  {
    camera_format = CAMERA_PF_YUV422;
    camera_buf_size = num_pixels * 2; /* 16bpp */
  }

  /* Trigger camera setup */
  camera_state = CAM_STATE_SETUP;

  return (0);
}

/**
  * @brief  Manage the UVC data packets
  *
  * @param  pbuf: pointer to the buffer data to be filled
  * @param  psize: pointer to the current packet size to be filled
  * @param  pcktidx: pointer to the current packet index in the current image
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t VIDEO_Itf_Data(uint8_t** pbuf, uint16_t* psize, uint16_t* pcktidx)
{
  uint16_t max_packet_size;
  if (husb.dev_speed == USBD_SPEED_HIGH)
  {
    max_packet_size = UVC_ISO_HS_MPS;
  }
  else
  {
    max_packet_size = UVC_ISO_FS_MPS;
  }

  uint32_t packet_count = usb_tx_size / ((uint16_t) (max_packet_size - 2));
  uint32_t packet_remainder = usb_tx_size % ((uint16_t) (max_packet_size - 2));
  static uint32_t packet_index = 0;

  if ((usb_tx_complete == 0) && (camera_state == CAM_STATE_CONFIGURED))
  {
    if (packet_index < packet_count)
    {
      *pcktidx = packet_index;
      *psize = (uint16_t) max_packet_size;
      *pbuf = (uint8_t *) (usb_tx_buff + packet_index * ((uint16_t) (max_packet_size - 2)));
      packet_index++;
    }
    else if (packet_index == packet_count)
    {
      *pcktidx = packet_index;
      *psize = (packet_remainder + 2);
      *pbuf = (uint8_t *) (usb_tx_buff + packet_index * ((uint16_t) (max_packet_size - 2)));
      packet_index++;
    }
    else
    {
      usb_tx_complete = 1;
      packet_index = 0U;
      *psize = 2; /* Send only the header */
    }
  }
  else
  {
    *psize = 2;
  }

  return (0);
}

