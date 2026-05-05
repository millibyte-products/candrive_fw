# Per-image build rules. Invoked from the top-level Makefile with:
#   TARGET = bootloader | common | app
#   BUILD_DIR = build/<target>
#
# Output: $(BUILD_DIR)/$(TARGET).elf, .bin, .hex, .map

ifndef TARGET
$(error TARGET not set)
endif
ifndef BUILD_DIR
$(error BUILD_DIR not set)
endif

# ---------------- toolchain ----------------
CROSS    ?= arm-none-eabi-
CC       := $(CROSS)gcc
AS       := $(CROSS)gcc -x assembler-with-cpp
LD       := $(CROSS)gcc
OBJCOPY  := $(CROSS)objcopy
SIZE     := $(CROSS)size

# ---------------- flags ----------------
CPU      := -mcpu=cortex-m3 -mthumb -mfloat-abi=soft

WARN     := -Wall -Wextra -Wpedantic -Werror -Wshadow -Wundef \
            -Wmissing-prototypes -Wstrict-prototypes \
            -Wno-unused-parameter -Wno-unused-function

ifeq ($(DEBUG),1)
OPT      := -Og -g3
else
OPT      := -Os -g
endif

DEFINES  := -DSTM32F103xB \
            -DUSE_FULL_LL_DRIVER \
            -DHSE_VALUE=8000000U \
            -DHSI_VALUE=8000000U \
            -DTARGET_$(shell echo $(TARGET) | tr a-z A-Z)

INCLUDES := \
  -Ivendor/cmsis_core/CMSIS/Core/Include \
  -Ivendor/cmsis_device_f1/Include \
  -Ivendor/stm32f1xx_hal_driver/Inc \
  -Isrc/common \
  -Isrc/$(TARGET)

CFLAGS   := $(CPU) $(OPT) $(WARN) $(DEFINES) $(INCLUDES) \
            -ffunction-sections -fdata-sections \
            -fno-common -fno-builtin \
            -std=c11 -MMD -MP

ASFLAGS  := $(CPU) $(OPT) $(DEFINES) -MMD -MP

LDSCRIPT := linker/$(TARGET).ld
LDFLAGS  := $(CPU) -T$(LDSCRIPT) \
            -Wl,--gc-sections \
            -Wl,-Map=$(BUILD_DIR)/$(TARGET).map \
            -Wl,--print-memory-usage \
            -nostartfiles -specs=nano.specs -specs=nosys.specs \
            -Wl,--no-warn-rwx-segments

# ---------------- sources ----------------
# Common LL driver subset we use (kept tight; add as needed)
LL_SRC_DIR := vendor/stm32f1xx_hal_driver/Src
LL_SOURCES := \
  $(LL_SRC_DIR)/stm32f1xx_ll_rcc.c \
  $(LL_SRC_DIR)/stm32f1xx_ll_utils.c \
  $(LL_SRC_DIR)/stm32f1xx_ll_gpio.c \
  $(LL_SRC_DIR)/stm32f1xx_ll_usart.c

CMSIS_SOURCES := \
  vendor/cmsis_device_f1/Source/Templates/system_stm32f1xx.c

# Per-target source lists.
ifeq ($(TARGET),bootloader)
SRC_C := \
  src/bootloader/main.c \
  src/common/clocks.c \
  src/common/delay.c \
  src/common/usart_dbg.c \
  src/common/bxcan.c \
  src/common/crc32.c \
  src/common/flash.c \
  $(LL_SOURCES) \
  $(CMSIS_SOURCES)
SRC_S := src/common/startup.S
endif

ifeq ($(TARGET),common)
# common.elf provides only the API table + functions reachable from it.
# Functions are linked but the only globally exported symbol is the table at +0.
# We deliberately do NOT include startup.S; common is never entered at reset.
# system_stm32f1xx.c is also excluded — it has the writable SystemCoreClock
# global which would violate common's no-.data/.bss rule. Just the const
# prescaler tables it provides are vendored locally as cmsis_tables.c.
SRC_C := \
  src/common/common_api.c \
  src/common/delay.c \
  src/common/cmsis_tables.c \
  src/common/bxcan.c \
  src/common/crc32.c \
  src/common/flash.c \
  src/common/usart_dbg.c \
  $(LL_SOURCES)
SRC_S :=
endif

ifeq ($(TARGET),app)
SRC_C := \
  src/app/main.c \
  src/common/clocks.c \
  src/common/delay.c \
  $(LL_SOURCES) \
  $(CMSIS_SOURCES)
SRC_S := src/common/startup.S
endif

# ---------------- objects ----------------
OBJS := $(addprefix $(BUILD_DIR)/, $(notdir $(SRC_C:.c=.o) $(SRC_S:.S=.o)))
DEPS := $(OBJS:.o=.d)

VPATH := $(sort $(dir $(SRC_C) $(SRC_S)))

# ---------------- rules ----------------
.PHONY: all
all: $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex
	@echo
	@$(SIZE) $(BUILD_DIR)/$(TARGET).elf

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(@D)
	@echo "  CC    $<"
	@$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.S
	@mkdir -p $(@D)
	@echo "  AS    $<"
	@$(AS) $(ASFLAGS) -c $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	@echo "  LD    $@"
	@$(LD) $(LDFLAGS) $(OBJS) -o $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	@echo "  BIN   $@"
	@$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	@echo "  HEX   $@"
	@$(OBJCOPY) -O ihex $< $@

-include $(DEPS)
