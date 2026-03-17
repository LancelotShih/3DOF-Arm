# ABOUTME: Builds the 3DOF arm firmware from the STM32Cube project with a terminal workflow.
# ABOUTME: Outputs artifacts in build and can flash the board with st-flash.

TARGET = firmware
BUILD_DIR = build

APP_SOURCES = \
$(wildcard Core/Src/*.c) \
$(wildcard Drivers/IO/Src/*.c)

HAL_SOURCES = $(wildcard Drivers/STM32F4xx_HAL_Driver/Src/*.c)

C_SOURCES = \
$(APP_SOURCES) \
$(HAL_SOURCES)

ASM_SOURCES = \
Core/Startup/startup_stm32f446zetx.s

CPU = -mcpu=cortex-m4
USE_FPU ?= 1
FPU =
FLOAT_ABI =
ifeq ($(USE_FPU),1)
FPU = -mfpu=fpv4-sp-d16
FLOAT_ABI = -mfloat-abi=hard
else
FLOAT_ABI = -mfloat-abi=soft
endif
MCU = $(CPU) -mthumb $(FPU) $(FLOAT_ABI)

C_DEFS = \
-DUSE_HAL_DRIVER \
-DSTM32F446xx

C_INCLUDES = \
-ICore/Inc \
-IDrivers/IO/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) -O0 -Wall -fdata-sections -ffunction-sections -g -gdwarf-2

LDSCRIPT = STM32F446ZETX_FLASH.ld

LIBS = -lc -lm -lnosys
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(CP) -O binary $< $@

$(BUILD_DIR):
	mkdir -p $@

clean:
	-rm -fR $(BUILD_DIR)

flash: $(BUILD_DIR)/$(TARGET).bin
	st-flash write $(BUILD_DIR)/$(TARGET).bin 0x8000000

.PHONY: all clean flash
