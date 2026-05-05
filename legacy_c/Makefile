# Top-level orchestrator for the candrive firmware.
# Builds three independently-linked images at fixed addresses:
#   bootloader.bin @ 0x08000000  (4 KiB)
#   common.bin     @ 0x08001000  (4 KiB)  — exposes API table at +0x000
#   app.bin        @ 0x08002400  (55 KiB)
#
# Use `make` for release, `make DEBUG=1` for -Og + symbols.

MAKEFLAGS += --no-print-directory
TARGETS    := bootloader common app
BUILD_DIR  := build

.PHONY: all clean $(TARGETS) flash flash-bootloader flash-common flash-app size vendor

all: $(TARGETS)

$(TARGETS): vendor-check
	@mkdir -p $(BUILD_DIR)/$@
	@$(MAKE) -f make/target.mk TARGET=$@ BUILD_DIR=$(BUILD_DIR)/$@ DEBUG=$(DEBUG)

vendor-check:
	@test -f vendor/cmsis_core/CMSIS/Core/Include/core_cm3.h || { \
	  echo "Vendor sources missing. Run: ./vendor/setup.sh"; exit 1; }

vendor:
	./vendor/setup.sh

clean:
	rm -rf $(BUILD_DIR)

size: all
	@for t in $(TARGETS); do \
	  echo "== $$t =="; arm-none-eabi-size -Ax $(BUILD_DIR)/$$t/$$t.elf | head -20; done

# Convenience flash targets (require openocd + stlink config).
OPENOCD ?= openocd
OOCD_CFG := -f interface/stlink.cfg -f target/stm32f1x.cfg

flash-bootloader: bootloader
	$(OPENOCD) $(OOCD_CFG) -c "program $(BUILD_DIR)/bootloader/bootloader.elf verify reset exit"

flash-common: common
	$(OPENOCD) $(OOCD_CFG) -c "program $(BUILD_DIR)/common/common.elf verify reset exit"

flash-app: app
	$(OPENOCD) $(OOCD_CFG) -c "program $(BUILD_DIR)/app/app.elf verify reset exit"

flash: bootloader common app
	$(OPENOCD) $(OOCD_CFG) \
	  -c "init" -c "reset halt" \
	  -c "program $(BUILD_DIR)/bootloader/bootloader.elf verify" \
	  -c "program $(BUILD_DIR)/common/common.elf verify" \
	  -c "program $(BUILD_DIR)/app/app.elf verify" \
	  -c "reset run" -c "exit"
