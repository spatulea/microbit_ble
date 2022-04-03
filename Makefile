PROJECT_NAME := microbit_ble_s110_pca20006

export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

TEMPLATE_PATH = ./components/toolchain/gcc

GNU_INSTALL_ROOT := /usr/local/Cellar/arm-none-eabi-gcc/9-2019-q4-major
GNU_VERSION := 9.2.1
GNU_PREFIX := arm-none-eabi

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
$(abspath ./components/toolchain/system_nrf51.c) \
$(abspath ./components/drivers_nrf/delay/nrf_delay.c) \
$(abspath ./components/libraries/util/app_error.c) \
$(abspath ./components/libraries/fifo/app_fifo.c) \
$(abspath ./components/libraries/util/app_util_platform.c) \
$(abspath ./components/libraries/util/nrf_assert.c) \
$(abspath ./components/libraries/uart/retarget.c) \
$(abspath ./components/libraries/uart/app_uart_fifo.c) \
$(abspath ./components/drivers_nrf/common/nrf_drv_common.c) \
$(abspath ./components/drivers_nrf/uart/nrf_drv_uart.c) \
$(abspath ./components/drivers_nrf/twi_master/nrf_drv_twi.c) \
$(abspath ./$(wildcard *.c))

# Bluetooth related source
C_SOURCE_FILES += \
$(abspath ./components/libraries/timer/app_timer.c) \
$(abspath ./components/ble/common/ble_advdata.c) \
$(abspath ./components/ble/common/ble_conn_params.c) \
$(abspath ./components/ble/ble_services/ble_lbs/ble_lbs.c) \
$(abspath ./components/softdevice/common/softdevice_handler/softdevice_handler.c) \
$(abspath ./components/drivers_nrf/gpiote/nrf_drv_gpiote.c) \
$(abspath ./components/libraries/button/app_button.c)

# C_SOURCE_FILES += $(abspath ./components/)
# C_SOURCE_FILES += $(wildcard **/*.c)


#assembly files common to all targets
ASM_SOURCE_FILES  = $(abspath ./components/toolchain/gcc/gcc_startup_nrf51.s)

#includes common to all targets
# INC_PATHS  = -I$(abspath ./../config/microbit_ble_s110_pca20006)
# INC_PATHS += -I$(abspath ./../config)
INC_PATHS += -I$(abspath ./components/toolchain/gcc)
INC_PATHS += -I$(abspath ./components/toolchain)
INC_PATHS += -I$(abspath ./components/softdevice/s110/headers)
INC_PATHS += -I$(abspath ./)
INC_PATHS += -I$(abspath ./bsp)
INC_PATHS += -I$(abspath ./components/device)
INC_PATHS += -I$(abspath ./components/drivers_nrf/delay)
INC_PATHS += -I$(abspath ./components/drivers_nrf/hal)

INC_PATHS += -I$(abspath ./components/drivers_nrf/nrf_soc_nosd)
INC_PATHS += -I$(abspath ./components/libraries/uart)
INC_PATHS += -I$(abspath ./components/libraries/util)
INC_PATHS += -I$(abspath ./components/drivers_nrf/uart)
INC_PATHS += -I$(abspath ./components/drivers_nrf/twi_master)
INC_PATHS += -I$(abspath ./components/drivers_nrf/common)
INC_PATHS += -I$(abspath ./components/drivers_nrf/config)
INC_PATHS += -I$(abspath ./components/libraries/fifo)
INC_PATHS += -I$(abspath ./components/libraries/timer)
INC_PATHS += -I$(abspath ./components/libraries/button)

INC_PATHS += -I$(abspath ./components/ble/common)
INC_PATHS += -I$(abspath ./components/ble/ble_services/ble_lbs)
INC_PATHS += -I$(abspath ./components/softdevice/common/softdevice_handler)
INC_PATHS += -I$(abspath ./components/drivers_nrf/gpiote)


OBJECT_DIRECTORY = build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -DS110
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DBOARD_MICROBIT
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
# CFLAGS += -Wall -Werror -O0
CFLAGS += -Wall -O0
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums
CFLAGS += -ggdb
# CFLAGS += -Og
CFLAGS += -DDEBUG

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys
# make it print floats for debug
# TODO: remove for "production"
LDFLAGS += -u _printf_float

# Linker libraries (after linker objects)
LIBS += -lm

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DS110
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DBOARD_MICROBIT
ASMFLAGS += -DBSP_DEFINES_ONLY
#default target - first one defined
default: clean nrf51822_xxaa_s110

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51822_xxaa_s110

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf51822_xxaa_s110


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrf51822_xxaa_s110: OUTPUT_FILENAME := nrf51822_xxaa_s110
nrf51822_xxaa_s110: LINKER_SCRIPT=microbit_ble_gcc_nrf51.ld
nrf51822_xxaa_s110: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<


# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash: $(MAKECMDGOALS)
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/nrf51822_xxaa_s110.hex
	# nrfjprog --program $(OUTPUT_BINARY_DIRECTORY)/$<.hex -f nrf51  --chiperase
	# nrfjprog --reset
	openocd -f interface/cmsis-dap.cfg -f target/nrf51.cfg -c "program $(OUTPUT_BINARY_DIRECTORY)/nrf51822_xxaa_s110.hex verify reset exit"


## Flash softdevice