BOOT_DIR   := bootloader
APP_DIR    := application
OUT_DIR    := build

BOOT_BIN   := $(BOOT_DIR)/build/stm32l053.bin
APP_BIN    := $(APP_DIR)/build/stm32l053-app.bin
UNIFIED_BIN := $(OUT_DIR)/stm32l053-unified.bin
UNIFIED_HEX := $(OUT_DIR)/stm32l053-unified.hex

BOOT_ADDR  := 0x08000000
APP_ADDR   := 0x08004000
BOOT_SIZE  := 16384

OBJCOPY    := arm-none-eabi-objcopy

.PHONY: all unified bootloader application clean flash

all: bootloader application unified

bootloader:
	$(MAKE) -C $(BOOT_DIR) -j16

application:
	$(MAKE) -C $(APP_DIR) -j16

unified: $(UNIFIED_BIN) $(UNIFIED_HEX)
	@boot_sz=$$(stat -c '%s' $(BOOT_BIN)); \
	 app_sz=$$(stat -c '%s' $(APP_BIN)); \
	 uni_sz=$$(stat -c '%s' $(UNIFIED_BIN)); \
	 echo ""; \
	 echo "  bootloader:  $$boot_sz bytes ($(BOOT_ADDR))"; \
	 echo "  application: $$app_sz bytes ($(APP_ADDR))"; \
	 echo "  unified:     $$uni_sz bytes"; \
	 echo ""; \
	 echo "  $(UNIFIED_BIN)"; \
	 echo "  $(UNIFIED_HEX)"

$(OUT_DIR):
	mkdir -p $(OUT_DIR)

$(UNIFIED_BIN): $(BOOT_BIN) $(APP_BIN) | $(OUT_DIR)
	dd if=/dev/zero bs=1 count=$(BOOT_SIZE) 2>/dev/null | tr '\0' '\377' > $@
	dd if=$(BOOT_BIN) of=$@ conv=notrunc 2>/dev/null
	cat $(APP_BIN) >> $@

$(UNIFIED_HEX): $(UNIFIED_BIN)
	$(OBJCOPY) -I binary -O ihex --change-addresses $(BOOT_ADDR) $< $@

flash: $(UNIFIED_BIN)
	st-flash write $(UNIFIED_BIN) $(BOOT_ADDR)

clean:
	$(MAKE) -C $(BOOT_DIR) clean
	$(MAKE) -C $(APP_DIR) clean
	rm -rf $(OUT_DIR)
