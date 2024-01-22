# USB Webcam application

The **USB Webcam** application enables the STM32H7 to act as a USB Video Camera
device. This is an USB Video Class (UVC) device implementation example. Native 
USB video driver support is provided with Windows, Ubuntu and macOS. The firmware 
implements support for 2 formats, each offering 2 frame resolutions:

- MJPEG or YUY2 format
- VGA 640x480 or QVGA 320x240 resolution

In MJPEG format, the camera is configured to capture frames in RGB565. The
RGB565 frame is then fed into the STM32H7 hardware JPEG encoder for compression.

In YUY2 format, the camera is configured to capture frames in YUV 4:2:2. No
compression is done by the STM32.

## Purposes

- Enables photo and video capture for Machine Learning dataset creation and/or
  enrichment.
- Enables live testing of Machine Learning models on the host with the same
  camera modules as the target camera module.

## Hardware requirements

- STM32H747I-DISCO
- B-CAMS-OMV (OV5640)

## Usage

1. Load the binary onto the target STM32H747I-DISCO device.

2. Connect a USB cable to the "USB_OTG_HS" connector. The board can be powered
   either through the standard ST-LINK USB or by changing the JP6 jumper to the
   "HS" position.

3. The *STM32 Webcam* can be tested in the standard **Windows Camera app** or
   other applications with more control options like
   [ffmpeg](https://ffmpeg.org/).

*Note*: Depending on the host software, the driver may select a different format
and resolution than what is configured by default.

### Example of usage with ffmpeg on Windows

```
$ ffmpeg -f dshow -list_devices true -i dummy                ## List devices
$ ffmpeg -f dshow -list_options true -i video="STM32 Webcam" ## list options
```

```
ffplay -f dshow -i video="STM32 Webcam"
ffplay -f dshow -video_size 640x480 -vcodec mjpeg -i video="STM32 Webcam"
ffplay -f dshow -video_size 320x240 -vcodec mjpeg -i video="STM32 Webcam"
ffplay -f dshow -video_size 640x480 -pixel_format yuyv422 -i video="STM32 Webcam"
ffplay -f dshow -video_size 320x240 -pixel_format yuyv422 -i video="STM32 Webcam"
```

## Known issues

- Incorrect frame rate when using OV5640 camera unless CAMERA_BOOST_PCLK=1. By
  default, the OV5640 camera will be configured at 7.5fps in all configurations.
  Incorrect frame rate reporting may cause video recording and video playback
  speed issues. Note: reported frame rate is unchanged to maintain compatibility
  with teachablemachine.withgoogle.com
- When operating in High-Speed mode, the uncompressed VGA frame rate provided by
  default by the camera (15fps) cannot be matched by the USB transfer speed. The
  camera frame rate is slowed down in this case. To achieve an uncompressed
  15fps frame rate, additional transaction per microframe would be required but
  this feature is not currently supported by the HAL.
- When operating in Full-Speed mode, performance is degraded.
  - Slower frame rate
  - Some tearing effect can be observed.
