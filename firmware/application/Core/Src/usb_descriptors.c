/* usb_descriptors.c — USB CDC + DFU runtime descriptors for BSides Dublin Badge
 *
 * Enumerates as BSides Dublin Badge CDC (VID=0x0483 PID=0x5740).
 * Interface 0+1: CDC providing a serial port over USB.
 * Interface 2:   DFU runtime — host sends DFU_DETACH to reboot into bootloader.
 */

#include "tusb.h"
#include "stm32l0xx_hal.h"
#include <string.h>

extern uint8_t evil_mode;  /* 0 = normal CDC+DFU, 1 = HID keyboard */

/* ------------------------------------------------------------------ */
/* Constants                                                            */
/* ------------------------------------------------------------------ */

#define USBD_VID        0x0483U   /* STMicroelectronics */
#define USBD_PID        0x5740U   /* STM32 Virtual COM Port */
#define USBD_PID_HID    0x5741U   /* HID keyboard (evil mode) */

#define STR_MANUF       1U
#define STR_PRODUCT     2U
#define STR_SERIAL      3U
#define STR_CDC_INTF    4U
#define STR_DFU_INTF    5U
#define STR_CONFIG      6U
#define STR_HID_PRODUCT 7U

/* Endpoints */
#define EPNUM_CDC_NOTIF  0x81U   /* EP1 IN  — notification */
#define EPNUM_CDC_OUT    0x02U   /* EP2 OUT — data from host */
#define EPNUM_CDC_IN     0x82U   /* EP2 IN  — data to host */
#define EPNUM_HID_IN     0x81U   /* EP1 IN  — HID keyboard (evil mode only) */

/* ------------------------------------------------------------------ */
/* Device descriptor                                                    */
/* ------------------------------------------------------------------ */

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200U,   /* USB 2.0 */
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USBD_VID,
    .idProduct          = USBD_PID,
    .bcdDevice          = 0x0100U,
    .iManufacturer      = STR_MANUF,
    .iProduct           = STR_PRODUCT,
    .iSerialNumber      = STR_SERIAL,
    .bNumConfigurations = 0x01U
};

/* HID-only device descriptor (evil mode) */
static tusb_desc_device_t const desc_device_hid = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200U,
    .bDeviceClass       = 0,
    .bDeviceSubClass    = 0,
    .bDeviceProtocol    = 0,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USBD_VID,
    .idProduct          = USBD_PID_HID,
    .bcdDevice          = 0x0100U,
    .iManufacturer      = STR_MANUF,
    .iProduct           = STR_HID_PRODUCT,
    .iSerialNumber      = STR_SERIAL,
    .bNumConfigurations = 0x01U
};

uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*)(evil_mode ? &desc_device_hid : &desc_device);
}

/* ------------------------------------------------------------------ */
/* Configuration descriptor                                             */
/* ------------------------------------------------------------------ */

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_DFU_RT_DESC_LEN)

static uint8_t const desc_configuration[] = {
    /* Config: num, interface count, string idx, total len, attrs, power_ma */
    TUD_CONFIG_DESCRIPTOR(1, 3, STR_CONFIG, CONFIG_TOTAL_LEN, 0x00U, 100U),

    /* CDC: interface num, string idx, ep_notif, ep_notif_sz, ep_out, ep_in, ep_sz */
    TUD_CDC_DESCRIPTOR(0, STR_CDC_INTF, EPNUM_CDC_NOTIF, 8U, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64U),

    /* DFU runtime (ITF 2): host sends DFU_DETACH → device reboots to bootloader DFU */
    TUD_DFU_RT_DESCRIPTOR(2, STR_DFU_INTF, 0x0DU, 1000U, 4096U),
};

/* HID-only configuration descriptor (evil mode) */
#define EVIL_CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)

static uint8_t const desc_hid_report[] = { TUD_HID_REPORT_DESC_KEYBOARD() };

static uint8_t const desc_configuration_hid[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, EVIL_CONFIG_TOTAL_LEN, 0x00U, 100U),
    TUD_HID_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_KEYBOARD,
                       sizeof(desc_hid_report), EPNUM_HID_IN, 16, 10),
};

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return evil_mode ? desc_configuration_hid : desc_configuration;
}

/* HID report descriptor callback (required by TinyUSB HID class) */
uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
    (void)instance;
    return desc_hid_report;
}

/* ------------------------------------------------------------------ */
/* String descriptors                                                   */
/* ------------------------------------------------------------------ */

static char const* const string_desc_arr[] = {
    (const char[]){ 0x09U, 0x04U },        /* 0: Language — English (0x0409) */
    "Traboda CyberLabs",                    /* 1: Manufacturer */
    "BSides Dublin Badge CDC",              /* 2: Product */
    "",                                     /* 3: Serial — filled dynamically */
    "BSides Dublin Badge CDC",              /* 4: CDC interface name */
    "BSides Dublin Badge DFU Runtime",      /* 5: DFU runtime interface name */
    NULL,                                   /* 6: Config — decoded at runtime */
};

#define STR_DESC_MAX 64U
static uint16_t str_buf[STR_DESC_MAX];

/* CTF flag 2 — XOR-obfuscated so `strings` won't find it; decoded for USB response */
#define CTF2_KEY 0x42U
static const uint8_t ctf_flag2_enc[] = {
    0x24,0x2E,0x23,0x25,0x39,0x2E,0x37,0x77,
    0x20,0x1D,0x34,0x1D,0x73,0x71,0x71,0x75,0x3F
};

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
    } else if (index == STR_CONFIG) {
        chr_count = (uint8_t)sizeof(ctf_flag2_enc);
        for (uint8_t i = 0; i < chr_count; i++) {
            str_buf[1U + i] = (uint16_t)(ctf_flag2_enc[i] ^ CTF2_KEY);
        }
    } else if (index == STR_HID_PRODUCT) {
        const char *str = "BSides Dublin Badge HID";
        chr_count = (uint8_t)strlen(str);
        for (uint8_t i = 0U; i < chr_count; i++) {
            str_buf[1U + i] = (uint16_t)str[i];
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
