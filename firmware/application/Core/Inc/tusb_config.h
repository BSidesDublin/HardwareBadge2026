/* tusb_config.h — TinyUSB configuration for STM32L053 CDC application */
#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

/* MCU: STM32L0 uses the ST FSDEV full-speed device peripheral */
#define CFG_TUSB_MCU    OPT_MCU_STM32L0

/* Bare-metal, no OS */
#define CFG_TUSB_OS     OPT_OS_NONE
#define CFG_TUSB_DEBUG  0

/* RHPort 0: full-speed device */
#define CFG_TUSB_RHPORT0_MODE  (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

/* Device stack only */
#define CFG_TUD_ENABLED 1

/* EP0 packet size */
#define CFG_TUD_ENDPOINT0_SIZE 64

/* CDC class */
#define CFG_TUD_CDC             1
#define CFG_TUD_CDC_RX_BUFSIZE  256
#define CFG_TUD_CDC_TX_BUFSIZE  2048

/* DFU runtime class for reboot-to-bootloader */
#define CFG_TUD_VENDOR          0
#define CFG_TUD_DFU_RUNTIME     1

/* All other device classes disabled */
#define CFG_TUD_DFU    0
#define CFG_TUD_MSC    0
#define CFG_TUD_HID             1
#define CFG_TUD_HID_EP_BUFSIZE  16
#define CFG_TUD_MIDI   0

#endif /* TUSB_CONFIG_H_ */
