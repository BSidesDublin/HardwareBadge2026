/* dfu_callbacks.c — DFU runtime reboot-to-bootloader callback
 *
 * When the host sends DFU_DETACH on the DFU runtime interface (ITF 2),
 * TinyUSB calls tud_dfu_runtime_reboot_to_dfu_cb().  We set the boot flag
 * and reset; the bootloader picks up the flag and stays in DFU mode.
 *
 * The same flag+reset is triggered by sending "dfu\n" over the CDC port
 * (handled in main.c).
 */

#include "tusb.h"
#include "stm32l0xx_hal.h"

#define DFU_BOOT_FLAG  0xB00710ADUL

extern volatile uint32_t dfu_boot_flag;

void tud_dfu_runtime_reboot_to_dfu_cb(void) {
    dfu_boot_flag = DFU_BOOT_FLAG;
    NVIC_SystemReset();
}
