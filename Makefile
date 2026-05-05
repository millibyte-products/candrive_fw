# Orchestrator for the candrive Rust firmware.
#
# Builds three independently-linked images at fixed addresses:
#   bootloader.bin @ 0x08000000  (4 KiB)
#   common.bin     @ 0x08002000  (4 KiB)  — exposes API table at +0
#   app.bin        @ 0x08003400  (51 KiB)
#
# Each is a member of the fw/ Cargo workspace. We invoke `cargo build`
# per package and objcopy the resulting ELF into a flat .bin / .hex.

CARGO     ?= cargo
OBJCOPY   ?= arm-none-eabi-objcopy
SIZE      ?= arm-none-eabi-size

PROFILE   ?= release
CARGO_FLAGS :=
ifeq ($(PROFILE),release)
CARGO_FLAGS += --release
TARGET_DIR  := fw/target/thumbv7m-none-eabi/release
else
TARGET_DIR  := fw/target/thumbv7m-none-eabi/debug
endif

BUILD_DIR := build
TARGETS   := bootloader common app
ELFS      := $(addprefix $(BUILD_DIR)/, $(addsuffix .elf, $(TARGETS)))
BINS      := $(ELFS:.elf=.bin)
HEXES     := $(ELFS:.elf=.hex)

.PHONY: all clean test size $(TARGETS) flash flash-bootloader flash-common flash-app rust-tools

all: $(BINS) $(HEXES)
	@$(MAKE) -s size

$(BUILD_DIR):
	@mkdir -p $@

# Build each crate via cargo (run from fw/ so .cargo/config.toml applies
# and the thumbv7m-none-eabi target is selected).
define BUILD_RULE
$(BUILD_DIR)/$(1).elf: | $(BUILD_DIR)
	@echo "  CARGO $(1)"
	@cd fw && $(CARGO) build $(CARGO_FLAGS) -p candrive-$(1)
	@cp $(TARGET_DIR)/$(1) $$@

$(1): $(BUILD_DIR)/$(1).elf
endef
$(foreach t,$(TARGETS),$(eval $(call BUILD_RULE,$(t))))

%.bin: %.elf
	@echo "  BIN   $@"
	@$(OBJCOPY) -O binary $< $@

%.hex: %.elf
	@echo "  HEX   $@"
	@$(OBJCOPY) -O ihex $< $@

clean:
	cd fw && $(CARGO) clean
	rm -rf $(BUILD_DIR)

# `cargo test` for the shared crate — runs on the host. Tests of no_std
# code typically use a tiny no_std test harness; for the shared crate we
# build a host stdlib version using the default-target override.
test:
	cd fw && $(CARGO) test -p candrive-shared --target $$(rustc -vV | sed -n 's/host: //p')

size: $(ELFS)
	@for elf in $(ELFS); do \
	  echo "== $$elf =="; \
	  $(SIZE) $$elf; \
	done

# Convenience flash targets via OpenOCD + STLink. dev_setup.py is the
# preferred path for development; these are quick one-offs.
OPENOCD ?= openocd
OOCD_CFG := -f interface/stlink.cfg -f target/stm32f1x.cfg

flash-bootloader: $(BUILD_DIR)/bootloader.elf
	$(OPENOCD) $(OOCD_CFG) -c "program $< verify reset exit"

flash-common: $(BUILD_DIR)/common.elf
	$(OPENOCD) $(OOCD_CFG) -c "program $< verify reset exit"

flash-app: $(BUILD_DIR)/app.elf
	$(OPENOCD) $(OOCD_CFG) -c "program $< verify reset exit"

flash: $(ELFS)
	$(OPENOCD) $(OOCD_CFG) \
	  -c "init" -c "reset halt" \
	  $(foreach e,$(ELFS),-c "program $(e) verify") \
	  -c "reset run" -c "exit"
