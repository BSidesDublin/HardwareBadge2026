/* tusb_config.h — TinyUSB configuration for STM32L053 DFU bootloader */
#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

/* MCU: STM32L0 uses the ST FSDEV full-speed device peripheral */
#define CFG_TUSB_MCU    OPT_MCU_STM32L0

/* Bare-metal, no OS */
#define CFG_TUSB_OS     OPT_OS_NONE
#define CFG_TUSB_DEBUG  0

/* RHPort 0: full-speed device (enables tusb_init() zero-arg form) */
#define CFG_TUSB_RHPORT0_MODE  (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

/* Device stack only */
#define CFG_TUD_ENABLED 1

/* EP0 packet size */
#define CFG_TUD_ENDPOINT0_SIZE 64

/* DFU mode device class */
#define CFG_TUD_DFU             1
#define CFG_TUD_DFU_XFER_BUFSIZE 2048

/* All other device classes disabled */
#define CFG_TUD_CDC    0
#define CFG_TUD_MSC    0
#define CFG_TUD_HID    0
#define CFG_TUD_MIDI   0
#define CFG_TUD_VENDOR 0

#endif /* TUSB_CONFIG_H_ */
