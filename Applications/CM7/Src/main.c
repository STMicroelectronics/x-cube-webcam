 /**
 ******************************************************************************
 * @file    main.c
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
#include "main.h"
#include "stm32h747i_discovery.h"
#include "stm32h747i_discovery_bus.h"
#include "stm32h747i_discovery_camera.h"
#include "stm32h747i_discovery_camera_ex.h"
#include "stm32h747i_discovery_sdram.h"
#include "jpeg_utils.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_video.h"
#include "usbd_video_if.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_WIDTH                 (640)
#define MAX_HEIGHT                (480)
#define MAX_NUM_PIXELS            (MAX_WIDTH * MAX_HEIGHT)
#define CAMERA_BITS_PER_PX        16U
#define YCBCR_BITS_PER_PX         16U /* 422 */
#define JPEG_SUBSAMPLING          JPEG_422_SUBSAMPLING
#define CAMERA_LINE_SIZE          (MAX_WIDTH * CAMERA_BITS_PER_PX / 8)
#define CAMERA_BUFF_SIZE          (MAX_NUM_PIXELS * CAMERA_BITS_PER_PX / 8)
#define YCBCR_BUFF_SIZE           (MAX_NUM_PIXELS * YCBCR_BITS_PER_PX / 8)

#define JPEG_MAX_SIZE             YCBCR_BUFF_SIZE

/* JPEG encoding image quality in % */
#define JPEG_QUALITY              90U

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*
 * Frame buffers
 * 32-byte alignment buffer for cache coherency operations
 * Bank separation for SDRAM bandwith optimization (4x8MB SDRAM banks)
 */
uint8_t pCameraLineBuffer[CAMERA_LINE_SIZE] __attribute__ ((aligned (32), section (".sram1")));
uint8_t pCameraBuff[CAMERA_BUFF_SIZE] __attribute__ ((aligned (32), section (".sdram_bank1")));
uint8_t pCopiedBuff[CAMERA_BUFF_SIZE] __attribute__ ((aligned (32), section (".sdram_bank2")));
uint8_t pYCbCrBuff[YCBCR_BUFF_SIZE] __attribute__ ((aligned (32), section (".sdram_bank2")));
uint8_t pJpegBuff[JPEG_MAX_SIZE] __attribute__ ((aligned (32), section (".sdram_bank2")));

USBD_HandleTypeDef husb;
JPEG_HandleTypeDef hjpeg;
MDMA_HandleTypeDef hmdma;
volatile uint32_t jpg_img_size;
volatile uint32_t camera_capture_complete;

/* Private function prototypes -----------------------------------------------*/
static void MPU_Config(void);
static void CPU_CACHE_Enable(void);
static void SystemClock_Config(void);
static void Error_Handler(void);
static void USB_Init(void);
static void MDMA_Init(void);

static void JPEG_Init(void);
static void JPEG_Encode(uint8_t *pSrc, uint8_t *pDst, uint32_t size);
static uint32_t JPEG_RGBToYCbCr(uint8_t *pSrc, uint8_t *pDst);
static JPEG_RGBToYCbCr_Convert_Function RGBToYCbCr;

static void CAMERA_SetMode(void);

/* Private functions ---------------------------------------------------------*/
HAL_StatusTypeDef HAL_DCMIEx_Start_DMA_MDMA(DCMI_HandleTypeDef *hdcmi, uint32_t DCMI_Mode, uint8_t *pData,
                                            uint32_t line_size, uint32_t num_lines);
static void DCMI_DMALineXferCplt(DMA_HandleTypeDef *hdma);
static void DCMI_DMAError(DMA_HandleTypeDef *hdma);
static void DCMI_MDMAFrameXferCplt(MDMA_HandleTypeDef *hmdma);
static void DCMI_MDMAError(MDMA_HandleTypeDef *hmdma);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Configure the MPU attributes as Write Through for SDRAM */
  MPU_Config();

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

 /* STM32H7xx HAL library initialization:
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);

  BSP_SDRAM_Init(0);

  /* Init camera default resolution and format */
  if (BSP_CAMERA_Init(0, CAMERA_R640x480, CAMERA_PF_RGB565) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Init MDMA for camera line buffer to frame buffer copy */
  MDMA_Init();

  USB_Init();

  while (1)
  {
    if (camera_state == CAM_STATE_SETUP)
    {
      /* Set camera resolution, format and rate from USB request */
      CAMERA_SetMode();

      /* Set frame width and height */
      JPEG_Init();

      camera_capture_complete = 0;

      const uint32_t line_size = camera_res_width * CAMERA_BITS_PER_PX / 8;
      const uint32_t num_lines = camera_res_height;
      /*
       * Start the Camera Capture
       * Using intermediate line buffer in D2-AHB domain to support high pixel clocks.
       */
      if (HAL_DCMIEx_Start_DMA_MDMA(&hcamera_dcmi, CAMERA_MODE_CONTINUOUS, pCameraBuff, line_size, num_lines) != HAL_OK)
      {
        Error_Handler();
      }

      camera_state = CAM_STATE_CONFIGURED;
    }
    else if (camera_state == CAM_STATE_CONFIGURED)
    {
      /* Wait for new camera frame */
      while ((camera_capture_complete == 0) && (camera_state == CAM_STATE_CONFIGURED));
      camera_capture_complete = 0;

      /*
       * Copy pCameraBuff as the DCMI HAL doesn't support double-buffering.
       * Inline double-word copy is faster than byte per byte copy provided with
       * newlib-nano.
       */
      for (uint32_t i = 0; i < camera_buf_size / sizeof(uint64_t); i++)
      {
        *(((uint64_t *) pCopiedBuff) + i) = *(((uint64_t *) pCameraBuff) + i);
      }

      if (camera_format == CAMERA_PF_RGB565)
      {
        uint32_t ycbcr_buf_size;
        ycbcr_buf_size = JPEG_RGBToYCbCr(pCopiedBuff, pYCbCrBuff);

        /* Wait for previous USB transmit to complete */
        while ((usb_tx_complete == 0) && (camera_state == CAM_STATE_CONFIGURED));

        JPEG_Encode(pYCbCrBuff, pJpegBuff, ycbcr_buf_size);

        /* Set USB transmit parameters */
        usb_tx_buff = pJpegBuff;
        usb_tx_size = jpg_img_size;

        /* Trigger USB transmit start on next USB data IN stage */
        usb_tx_complete = 0;
      }
      else if (camera_format == CAMERA_PF_YUV422)
      {
        /* Set USB transmit parameters */
        usb_tx_buff = pCopiedBuff;
        usb_tx_size = camera_buf_size;

        /* Trigger USB transmit start on next USB data IN stage */
        usb_tx_complete = 0;
      }
    }
  }
}

/**
 * @brief Initialize USB Video
 *
 */
void USB_Init(void)
{
  HAL_PWREx_EnableUSBVoltageDetector();

  /* Init Device Library */
  if (USBD_Init(&husb, &VIDEO_Desc, 0) != USBD_OK)
  {
    Error_Handler();
  }

  /* Add Supported Class */
  if (USBD_RegisterClass(&husb, &USBD_VIDEO) != USBD_OK)
  {
    Error_Handler();
  }

  /* Add Interface Class */
  if (USBD_VIDEO_RegisterInterface(&husb, &USBD_VIDEO_fops) != USBD_OK)
  {
    Error_Handler();
  }

  /* Start Device Process */
  USBD_Start(&husb);
}

/**
 * @brief Initialize JPEG hardware encoder and utilities
 *
 */
void JPEG_Init(void)
{
  JPEG_ConfTypeDef jpeg_conf;
  uint32_t MCU_TotalNb;

  hjpeg.Instance = JPEG;
  if (HAL_JPEG_Init(&hjpeg) != HAL_OK)
  {
    Error_Handler();
  }

  /* Set JPEG encoding parameters */
  jpeg_conf.ColorSpace        = JPEG_YCBCR_COLORSPACE;
  jpeg_conf.ChromaSubsampling = JPEG_SUBSAMPLING;
  jpeg_conf.ImageWidth        = camera_res_width;
  jpeg_conf.ImageHeight       = camera_res_height;
  jpeg_conf.ImageQuality      = JPEG_QUALITY;
  if (HAL_JPEG_ConfigEncoding(&hjpeg, &jpeg_conf) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize RGB to YCbCr color conversion tables */
  JPEG_InitColorTables();

  /* Retrieve the corresponding color conversion function and number of MCUs. */
  JPEG_GetEncodeColorConvertFunc(&jpeg_conf, &RGBToYCbCr, &MCU_TotalNb);
}


/**
 * @brief Convert image buffer into Minimum Coded Unit (MCU)
 *
 * Perform color space transformation, downsampling and block splitting.
 * @param pSrc pointer to source RGB565 image buffer
 * @param pDst pointer to destination YCbCr blocks buffer
 * @return uint32_t number of converted bytes
 */
uint32_t JPEG_RGBToYCbCr(uint8_t *pSrc, uint8_t *pDst)
{
  const uint32_t img_width = hjpeg.Conf.ImageWidth;
  const uint32_t img_height = hjpeg.Conf.ImageHeight;
  const uint32_t input_size = img_width * img_height * 2; /* 16-bpp */
  uint32_t n_ycbcr_blocks;
  uint32_t n_ycbcr_bytes;

  /* Convert RGB image to YCbCr MCU block */
  n_ycbcr_blocks = RGBToYCbCr(
      pSrc,             /* pointer to input RGB888/ARGB8888 frame buffer */
      pDst,             /* pointer to output YCbCr blocks buffer */
      0,                /* index of the input buffer first block */
      input_size,       /* number of bytes in the input buffer */
      &n_ycbcr_bytes);  /* number of converted bytes */

  (void) n_ycbcr_blocks;

  return n_ycbcr_bytes;
}

/**
 * @brief Encode YCbCr blocks into JPEG
 *
 * @param pSrc pointer to source YCbCr blocks buffer
 * @param pDst pointer to the destination jpeg buffer
 * @param size size in bytes of the input buffer
 */
void JPEG_Encode(uint8_t *pSrc, uint8_t *pDst, uint32_t size)
{
  HAL_StatusTypeDef status;

  status = HAL_JPEG_Encode(&hjpeg, pSrc, size, pDst, JPEG_MAX_SIZE, 2000);
  if (status != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief  JPEG error callback.
 * @param  hjpeg pointer to a JPEG_HandleTypeDef structure that contains
 *         the configuration information for JPEG module
 */
void HAL_JPEG_ErrorCallback(JPEG_HandleTypeDef *hjpeg)
{
  uint32_t error_code = HAL_JPEG_GetError(hjpeg);
  (void) error_code;
  Error_Handler();
}

/**
 * @brief  Decoded/Encoded Data ready  callback.
 * @param  hjpeg pointer to a JPEG_HandleTypeDef structure that contains
 *         the configuration information for JPEG module
 * @param  pDataOut pointer to the output data buffer
 * @param  OutDataLength number in bytes of data available in the specified output buffer
 */
void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *hjpeg, uint8_t *pDataOut, uint32_t OutDataLength)
{
  jpg_img_size = OutDataLength;
  if (OutDataLength > JPEG_MAX_SIZE)
  {
    Error_Handler();
  }
}

/**
 * @brief  Frame Event callback.
 * @param  Instance Camera instance.
 */
void BSP_CAMERA_FrameEventCallback(uint32_t Instance)
{
  camera_capture_complete = 1;
  BSP_LED_Toggle(LED1);
}

/**
 * @brief  Error callback.
 * @param  Instance Camera instance.
 */
void BSP_CAMERA_ErrorCallback(uint32_t Instance)
{
  Error_Handler();
}

/**
 * @brief Enables DCMI DMA request and enables DCMI capture
 *        with intermediate line buffer
 *
 * @param  hdcmi     pointer to a DCMI_HandleTypeDef structure that contains
 *                    the configuration information for DCMI.
 * @param  DCMI_Mode DCMI capture mode snapshot or continuous grab.
 * @param  pData     The destination memory Buffer address (Frame buffer).
 * @param line_size Horizontal frame size in bytes.
 * @param num_lines Vertical frame size in pixels.
 * @return HAL status
 */
HAL_StatusTypeDef HAL_DCMIEx_Start_DMA_MDMA(DCMI_HandleTypeDef *hdcmi, uint32_t DCMI_Mode, uint8_t *pData,
                                       uint32_t line_size, uint32_t num_lines)
{
  /* Check function parameters */
  assert_param(IS_DCMI_CAPTURE_MODE(DCMI_Mode));

  /* Process Locked */
  __HAL_LOCK(hdcmi);

  /* Lock the DCMI peripheral state */
  hdcmi->State = HAL_DCMI_STATE_BUSY;

  /* Enable DCMI by setting DCMIEN bit */
  __HAL_DCMI_ENABLE(hdcmi);

  /* Configure the DCMI Mode */
  hdcmi->Instance->CR &= ~(DCMI_CR_CM);
  hdcmi->Instance->CR |= (uint32_t)(DCMI_Mode);

  /* Set DMA callbacks */
  hdcmi->DMA_Handle->XferCpltCallback = DCMI_DMALineXferCplt;
  hdcmi->DMA_Handle->XferErrorCallback = DCMI_DMAError;
  hdcmi->DMA_Handle->XferAbortCallback = NULL;

  /* Set MDMA callbacks */
  hmdma.XferCpltCallback = DCMI_MDMAFrameXferCplt;
  hmdma.XferErrorCallback = DCMI_MDMAError;

  hdcmi->XferCount = 0;
  hdcmi->XferTransferNumber = num_lines;
  hdcmi->XferSize = line_size / 4U;
  hdcmi->pBuffPtr = (uint32_t) pData;

  /* Enable the DMA Stream */
  uint32_t pLineData = (uint32_t) pCameraLineBuffer;
  if (HAL_DMA_Start_IT(hdcmi->DMA_Handle, (uint32_t)&hdcmi->Instance->DR, pLineData, hdcmi->XferSize) != HAL_OK)
  {
    /* Set Error Code */
    hdcmi->ErrorCode = HAL_DCMI_ERROR_DMA;
    /* Change DCMI state */
    hdcmi->State = HAL_DCMI_STATE_READY;
    /* Release Lock */
    __HAL_UNLOCK(hdcmi);
    /* Return function status */
    return HAL_ERROR;
  }

  /* Enable Capture */
  hdcmi->Instance->CR |= DCMI_CR_CAPTURE;

  /* Release Lock */
  __HAL_UNLOCK(hdcmi);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  DMA transfer complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void DCMI_DMALineXferCplt(DMA_HandleTypeDef *hdma)
{
  DCMI_HandleTypeDef *hdcmi = (DCMI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Copy line buffer to frame buffer using MDMA */
  uint32_t line_size =  hdcmi->XferSize * 4U;
  uint8_t *pDst = (uint8_t *) hdcmi->pBuffPtr + line_size * hdcmi->XferCount;

  if (HAL_MDMA_Start_IT(&hmdma, (uint32_t) pCameraLineBuffer, (uint32_t) pDst, line_size, 1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  MDMA DCMI transfer complete callback
  * @param  hmdma  MDMA handle
  * @retval None
  */
static void DCMI_MDMAFrameXferCplt(MDMA_HandleTypeDef *hmdma)
{

  DCMI_HandleTypeDef *hdcmi = &hcamera_dcmi;

  /* Disable the MDMA channel */
  __HAL_MDMA_DISABLE(hmdma);

  hdcmi->XferCount++;

  /* Check if the frame is transferred */
  if (hdcmi->XferCount == hdcmi->XferTransferNumber)
  {
    /* Enable the Frame interrupt */
    __HAL_DCMI_ENABLE_IT(hdcmi, DCMI_IT_FRAME);

    /* When snapshot mode, set dcmi state to ready */
    if ((hdcmi->Instance->CR & DCMI_CR_CM) == DCMI_MODE_SNAPSHOT)
    {
      hdcmi->State = HAL_DCMI_STATE_READY;
    }
    else
    {
      hdcmi->XferCount = 0;
    }
  }
}

/**
  * @brief  DMA error callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void DCMI_DMAError(DMA_HandleTypeDef *hdma)
{
  DCMI_HandleTypeDef *hdcmi = (DCMI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  if (hdcmi->DMA_Handle->ErrorCode != HAL_DMA_ERROR_FE)
  {
    /* Initialize the DCMI state*/
    hdcmi->State = HAL_DCMI_STATE_READY;

    /* Set DCMI Error Code */
    hdcmi->ErrorCode |= HAL_DCMI_ERROR_DMA;
  }

  Error_Handler();
}

/**
  * @brief  MDMA DCMI error callback.
  * @param  hmdma MDMA handle
  * @retval None
  */
static void DCMI_MDMAError(MDMA_HandleTypeDef *hmdma)
{
  /* Disable the MDMA channel */
  __HAL_MDMA_DISABLE(hmdma);

  Error_Handler();
}

/**
 * @brief Initialize Master DMA
 *
 */
void MDMA_Init(void)
{
  __HAL_RCC_MDMA_CLK_ENABLE();

  hmdma.Instance = MDMA_Channel0;
  hmdma.Init.Request                  = MDMA_REQUEST_SW;
  hmdma.Init.TransferTriggerMode      = MDMA_BLOCK_TRANSFER;
  hmdma.Init.Priority                 = MDMA_PRIORITY_HIGH;
  hmdma.Init.Endianness               = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma.Init.SourceInc                = MDMA_SRC_INC_WORD;
  hmdma.Init.DestinationInc           = MDMA_DEST_INC_WORD;
  hmdma.Init.SourceDataSize           = MDMA_SRC_DATASIZE_WORD;
  hmdma.Init.DestDataSize             = MDMA_DEST_DATASIZE_WORD;
  hmdma.Init.DataAlignment            = MDMA_DATAALIGN_PACKENABLE;
  hmdma.Init.SourceBurst              = MDMA_DEST_BURST_SINGLE;
  hmdma.Init.DestBurst                = MDMA_DEST_BURST_16BEATS;
  hmdma.Init.BufferTransferLength     = 128;
  hmdma.Init.SourceBlockAddressOffset = 0;
  hmdma.Init.DestBlockAddressOffset   = 0;
  if (HAL_MDMA_Init(&hmdma) != HAL_OK)
  {
    Error_Handler();
  }

  /* NVIC configuration for MDMA transfer complete interrupt */
  HAL_NVIC_SetPriority(MDMA_IRQn, 15U, 0);
  HAL_NVIC_EnableIRQ(MDMA_IRQn);
}

/**
 * @brief Set camera resolution, format and rate
 *
 */
void CAMERA_SetMode(void)
{
  const uint32_t resolution = camera_res;
  const uint32_t format = camera_format;
  uint32_t camera_id;

  camera_id = Camera_Ctx[0].CameraId;

  if ((camera_id == OV9655_ID) || (camera_id == OV9655_ID_2))
  {
    /* Full camera DeInit-Init to for color noise issue workaround */
    BSP_CAMERA_DeInit(0);
    if (BSP_CAMERA_Init(0, resolution, format) != BSP_ERROR_NONE)
    {
      Error_Handler();
    }
  }
  else if (camera_id == OV5640_ID)
  {
    BSP_CAMERA_SetResolution(0, resolution);
    BSP_CAMERA_SetPixelFormat(0, format);
  }

  /*
   * Adjust camera frame rate to:
   * - 12fps in VGA UNCOMPRESSED mode to match USB bandwidth
   * - 15fps in VGA MJPEG mode to allow for JPEG processing time
   * - 30fps in QVGA UNCOMPRESSED and MJPEG modes (OV5640 only)
   * Note: Camera frame rate should match USB descriptor.
   */
  if ((format == CAMERA_PF_YUV422) && (resolution == CAMERA_R640x480))
  {
    BSP_CAMERAEx_SetTiming(0, CAMERA_TIMING_12FPS);
  }
  else if ((format == CAMERA_PF_RGB565) && (resolution == CAMERA_R640x480))
  {
    BSP_CAMERAEx_SetTiming(0, CAMERA_TIMING_15FPS);
  }
  else
  {
    BSP_CAMERAEx_SetTiming(0, CAMERA_TIMING_30FPS);
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 400000000 (Cortex-M7 CPU Clock)
  *            HCLK(Hz)                       = 200000000 (Cortex-M4 CPU, Bus matrix Clocks)
  *            AHB Prescaler                  = 2
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  100MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  100MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  100MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  100MHz)
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 5
  *            PLL_N                          = 160
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /*!< Supply configuration update enable */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if (ret != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure  bus clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 |
                                 RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1);

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  if (ret != HAL_OK)
  {
    Error_Handler();
  }

  /*
  Note : The activation of the I/O Compensation Cell is recommended with communication  interfaces
          (GPIO, SPI, FMC, QSPI ...)  when  operating at  high frequencies(please refer to product datasheet)
          The I/O Compensation Cell activation  procedure requires :
        - The activation of the CSI clock
        - The activation of the SYSCFG clock
        - Enabling the I/O Compensation Cell : setting bit[0] of register SYSCFG_CCCSR
  */

  __HAL_RCC_CSI_ENABLE();

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  HAL_EnableCompensationCell();
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/**
  * @brief  Configure the MPU attributes
  *         Region 0: External SDRAM Bank 2 as Write Through.
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure SDRAM as Write through, no write allocate (WT-NWA) */
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress      = 0xD0800000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_8MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
    BSP_LED_Toggle(LED2);
    HAL_Delay(100);
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

