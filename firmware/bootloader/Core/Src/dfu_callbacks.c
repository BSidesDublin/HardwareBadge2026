/* dfu_callbacks.c — TinyUSB DFU class callbacks for STM32L053 bootloader
 *
 * Alt 0 maps to the application flash region (0x08004000 – 0x08010000, 48 KB).
 * The bootloader region (0x08000000 – 0x08003FFF) is never accessible.
 */

#include "tusb.h"
#include "stm32l0xx_hal.h"
#include <string.h>
#include <stdbool.h>

extern volatile bool dfu_active;

#define APP_BASE_ADDR   0x08004000UL
#define APP_END_ADDR    0x08010000UL   /* exclusive: 0x08004000 + 48 KB */
#define FLASH_PAGE_SZ   128UL

/* -------------------------------------------------------------------------- */
/* Poll timeout                                                                */
/* -------------------------------------------------------------------------- */

uint32_t tud_dfu_get_timeout_cb(uint8_t alt, uint8_t state) {
    (void)alt;
    /* Give 60 ms per block during programming to avoid host timeout */
    return (state == DFU_DNBUSY) ? 60UL : 0UL;
}

/* -------------------------------------------------------------------------- */
/* Download (host → device)                                                   */
/* -------------------------------------------------------------------------- */

void tud_dfu_download_cb(uint8_t alt, uint16_t block_num,
                          uint8_t const* data, uint16_t length) {
    dfu_active = true;
    (void)alt;
    uint32_t addr = APP_BASE_ADDR + (uint32_t)block_num * CFG_TUD_DFU_XFER_BUFSIZE;

    /* Safety: reject any write that touches outside the application region */
    if (addr < APP_BASE_ADDR || (addr + (uint32_t)length) > APP_END_ADDR) {
        tud_dfu_finish_flashing(DFU_STATUS_ERR_ADDRESS);
        return;
    }

    HAL_FLASH_Unlock();

    /* Erase all pages covered by this block */
    uint32_t n_pages = ((uint32_t)length + FLASH_PAGE_SZ - 1UL) / FLASH_PAGE_SZ;
    FLASH_EraseInitTypeDef erase = {
        .TypeErase   = FLASH_TYPEERASE_PAGES,
        .PageAddress = addr,
        .NbPages     = n_pages,
    };
    uint32_t page_err = 0UL;
    if (HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK) {
        HAL_FLASH_Lock();
        tud_dfu_finish_flashing(DFU_STATUS_ERR_ERASE);
        return;
    }

    /* Program word by word; pad the last word with 0xFF if unaligned */
    for (uint16_t i = 0U; i < length; i += 4U) {
        uint32_t word = 0xFFFFFFFFUL;
        uint16_t chunk = (length - i >= 4U) ? 4U : (length - i);
        memcpy(&word, data + i, chunk);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + (uint32_t)i, word) != HAL_OK) {
            HAL_FLASH_Lock();
            tud_dfu_finish_flashing(DFU_STATUS_ERR_WRITE);
            return;
        }
    }

    HAL_FLASH_Lock();
    tud_dfu_finish_flashing(DFU_STATUS_OK);
}

/* -------------------------------------------------------------------------- */
/* Manifest (download sequence complete)                                      */
/* -------------------------------------------------------------------------- */

void tud_dfu_manifest_cb(uint8_t alt) {
    (void)alt;
    /* MANIFESTATION_TOLERANT is set, so no bus reset is needed.
       Signal success; the host will send DFU_GETSTATUS to confirm. */
    tud_dfu_finish_flashing(DFU_STATUS_OK);
}

/* -------------------------------------------------------------------------- */
/* Upload (device → host)                                                     */
/* -------------------------------------------------------------------------- */

uint16_t tud_dfu_upload_cb(uint8_t alt, uint16_t block_num,
                             uint8_t* data, uint16_t length) {
    (void)alt;
    dfu_active = true;
    uint32_t addr = APP_BASE_ADDR + (uint32_t)block_num * CFG_TUD_DFU_XFER_BUFSIZE;
    if (addr >= APP_END_ADDR) return 0U; /* EOF */

    uint32_t avail = APP_END_ADDR - addr;
    uint16_t xfer  = (avail < (uint32_t)length) ? (uint16_t)avail : length;
    memcpy(data, (const void*)addr, xfer);
    return xfer;
}

/* -------------------------------------------------------------------------- */
/* Abort / Detach                                                              */
/* -------------------------------------------------------------------------- */

void tud_dfu_abort_cb(uint8_t alt) {
    (void)alt;
}

void tud_dfu_detach_cb(void) {
    /* Host issued DFU_DETACH — reset the MCU so it boots the new firmware */
    NVIC_SystemReset();
}
