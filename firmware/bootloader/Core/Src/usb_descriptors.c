/* usb_descriptors.c — USB DFU descriptors for STM32L053 bootloader
 *
 * Enumerates as STM32 DFU (VID=0x0483 PID=0xDF11), dfu-util compatible.
 * One alternate setting exposing only the application flash region so
 * the bootloader region is never writable via DFU.
 */

#include "tusb.h"
#include "stm32l0xx_hal.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/* Constants                                                            */
/* ------------------------------------------------------------------ */

#define USBD_VID        0x0483U   /* STMicroelectronics */
#define USBD_PID        0xDF11U   /* STM32 DFU */

/* DFU attributes: download + upload, tolerant manifest (no reset needed after manifest) */
#define DFU_ATTRS  (DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_CAN_UPLOAD | DFU_ATTR_MANIFESTATION_TOLERANT)

#define ALT_COUNT   1
#define STR_MANUF   1U
#define STR_PRODUCT 2U
#define STR_SERIAL  3U
#define STR_ALT0    4U   /* first alternate-setting name (must equal STR_ALT0 + alt index) */

/* ------------------------------------------------------------------ */
/* Device descriptor                                                    */
/* ------------------------------------------------------------------ */

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0110U,  /* USB 1.1 */
    .bDeviceClass       = 0x00U,
    .bDeviceSubClass    = 0x00U,
    .bDeviceProtocol    = 0x00U,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USBD_VID,
    .idProduct          = USBD_PID,
    .bcdDevice          = 0x0200U,
    .iManufacturer      = STR_MANUF,
    .iProduct           = STR_PRODUCT,
    .iSerialNumber      = STR_SERIAL,
    .bNumConfigurations = 0x01U
};

uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*)&desc_device;
}

/* ------------------------------------------------------------------ */
/* Configuration descriptor                                             */
/* ------------------------------------------------------------------ */

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_DFU_DESC_LEN(ALT_COUNT))

static uint8_t const desc_configuration[] = {
    /* Config: num, interface count, string idx, total len, attrs, power */
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, 0x00U, 100U),

    /* DFU: interface num, alt count, first string idx, attrs, detach timeout ms, xfer size */
    TUD_DFU_DESCRIPTOR(0, ALT_COUNT, STR_ALT0, DFU_ATTRS, 1000U, CFG_TUD_DFU_XFER_BUFSIZE),
};

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

/* ------------------------------------------------------------------ */
/* String descriptors                                                   */
/* ------------------------------------------------------------------ */

/* 384 pages × 128 bytes = 48 KB at 0x08004000 */
static char const* const string_desc_arr[] = {
    (const char[]){ 0x09U, 0x04U },          /* 0: Language — English (0x0409) */
    "Traboda CyberLabs",                       /* 1: Manufacturer */
    "BSides Dublin Badge in DFU mode",        /* 2: Product */
    "",                                        /* 3: Serial — filled dynamically below */
    "@Internal Flash  /0x08004000/384*128 g", /* 4: Alt 0 target */
};

#define STR_DESC_MAX 64U
static uint16_t str_buf[STR_DESC_MAX];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t chr_count;

    if (index == 0U) {
        memcpy(&str_buf[1], string_desc_arr[0], 2U);
        chr_count = 1U;
    } else if (index == STR_SERIAL) {
        /* Build serial string from 96-bit chip UID */
        static const char hex[] = "0123456789ABCDEF";
        const uint8_t* uid = (const uint8_t*)UID_BASE;
        chr_count = 0U;
        for (uint8_t i = 0U; i < 12U && chr_count < (uint8_t)(STR_DESC_MAX - 1U); i++) {
            str_buf[1U + chr_count++] = (uint16_t)hex[uid[i] >> 4U];
            str_buf[1U + chr_count++] = (uint16_t)hex[uid[i] & 0x0FU];
        }
    } else {
        if (index >= TU_ARRAY_SIZE(string_desc_arr)) return NULL;
        const char* str = string_desc_arr[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > (uint8_t)(STR_DESC_MAX - 1U)) chr_count = (uint8_t)(STR_DESC_MAX - 1U);
        for (uint8_t i = 0U; i < chr_count; i++) {
            str_buf[1U + i] = (uint16_t)str[i];
        }
    }

    str_buf[0] = (uint16_t)((TUSB_DESC_STRING << 8U) | (2U * chr_count + 2U));
    return str_buf;
}
