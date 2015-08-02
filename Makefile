#CROSS_COMPILE	?= arm-none-eabi-
CROSS_COMPILE	?= /Users/cooltouya/Programs/linaro-arm-none-eabi/bin/arm-none-eabi-

DEBUG	?= 1
USE_FPU	?= 1

# Now #define for project
ifeq ($(DEBUG), 1)
CFLAGS += -DDEBUG_PRINT_ON_UART
endif


######## Location Configuration ########
LIB = lib
CMSIS = $(LIB)/CMSIS
CMSIS_INCLUDE = $(CMSIS)/Include
CMSIS_DEVICE = $(CMSIS)/Device
CMSIS_DEVICE_SOURCE = $(CMSIS_DEVICE)/Source
CMSIS_DEVICE_INCLUDE = $(CMSIS_DEVICE)/Include
CMSIS_DEVICE_STARTUP = $(CMSIS_DEVICE_SOURCE)/gcc
ST_HAL_DRIVER = $(LIB)/STM32F4xx_HAL_Driver
ST_BSP = $(LIB)/BSP
MIDDLEWARES = $(LIB)/Middlewares
THIRD_PARTY = $(MIDDLEWARES)/Third_Party
RTOS = $(THIRD_PARTY)/FreeRTOS/Source
LINKER_DIR = $(LIB)/linker
LDSCRIPT = $(LINKER_DIR)/flash.ld
CONFIGS = configs

INIT = init
DRIVERS = drivers
MODULES = modules
HAL = hal
PLATFORM = platform

######## Build Configuration ########
# CMSIS
VPATH += $(CMSIS_DEVICE_SOURCE)
VPATH += $(CMSIS_DEVICE_STARTUP)

INCLUDES += -I$(CMSIS_DEVICE_INCLUDE)
INCLUDES += -I$(CMSIS_INCLUDE)

CRT0 = startup_stm32f411xe.o system_stm32f4xx.o

OBJ = $(CRT0)
OBJ += syscalls.o
# ST Lib
VPATH += $(ST_HAL_DRIVER)/Src
VPATH += $(ST_BSP)/STM32F4xx-Nucleo
VPATH += $(ST_BSP)/Components/lsm6ds0
VPATH += $(ST_BSP)/Components/lsm6ds3
VPATH += $(ST_BSP)/Components/hts221
VPATH += $(ST_BSP)/Components/lis3mdl
VPATH += $(ST_BSP)/Components/lps25h
VPATH += $(ST_BSP)/Components/lps25hb
VPATH += $(ST_BSP)/X_NUCLEO_IKS01A1

INCLUDES += -I$(ST_HAL_DRIVER)/Inc
INCLUDES += -I$(ST_BSP)/STM32F4xx-Nucleo
INCLUDES += -I$(ST_BSP)/Components/Common
INCLUDES += -I$(ST_BSP)/Components/lsm6ds0
INCLUDES += -I$(ST_BSP)/Components/lsm6ds3
INCLUDES += -I$(ST_BSP)/Components/hts221
INCLUDES += -I$(ST_BSP)/Components/lis3mdl
INCLUDES += -I$(ST_BSP)/Components/lps25h
INCLUDES += -I$(ST_BSP)/Components/lps25hb
INCLUDES += -I$(ST_BSP)/X_NUCLEO_IKS01A1

OBJ +=  stm32f4xx_nucleo.o
OBJ += $(patsubst %.c,%.o,$(notdir $(wildcard $(ST_HAL_DRIVER)/Src/*.c)))
OBJ += $(patsubst %.c,%.o,$(notdir $(wildcard $(ST_BSP)/Components/lsm6ds0/*.c)))
OBJ += $(patsubst %.c,%.o,$(notdir $(wildcard $(ST_BSP)/Components/lsm6ds3/*.c)))
OBJ += $(patsubst %.c,%.o,$(notdir $(wildcard $(ST_BSP)/Components/lis3mdl/*.c)))
OBJ += $(patsubst %.c,%.o,$(notdir $(wildcard $(ST_BSP)/X_NUCLEO_IKS01A1/*.c)))

# RTOS
VPATH += $(RTOS)
VPATH += $(RTOS)/portable/GCC/ARM_CM4F
VPATH += $(RTOS)/portable/MemMang

INCLUDES += -I$(RTOS)/include
INCLUDES += -I$(RTOS)/portable/GCC/ARM_CM4F
MEMMANG_OBJ = heap_4.o
OBJ += port.o
OBJ += list.o tasks.o queue.o timers.o $(MEMMANG_OBJ)

# Project
VPATH += $(DRIVERS)/src $(MODULES)/src $(PLATFORM)/src $(INIT) $(HAL)/src
OBJ += main.o stm32f4xx_hal_msp.o stm32f4xx_it.o uart.o platform.o error_handler.o
OBJ += stabilizer.o system.o imu.o MARG_Filter.o

INCLUDES += -I$(DRIVERS)/inc -I$(MODULES)/inc -I$(PLATFORM)/inc -I$(HAL)/inc -I$(CONFIGS)

######## Compilation Configuration ########

AS = $(CROSS_COMPILE)as
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)gcc
SIZE = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy

ifeq ($(USE_FPU), 1)
PROCESSOR = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
else
PROCESSOR = -mcpu=cortex-m4 -mthumb
endif

LIBS = --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lm -lrdimon -lnosys -Wl,--end-group
STFLAGS = -DUSE_STM32F4XX_NUCLEO -DSTM32F411xE -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 
ifeq ($(USE_FPU), 1)
STFLAGS += -D__FPU_USED
endif

ifeq ($(DEBUG), 1)
#CFLAGS += -O0 -g
CFLAGS += -Os -g
else
CFLAGS += -Os -g3
endif

CFLAGS += $(PROCESSOR) $(INCLUDES) $(STFLAGS) -Wall -fno-strict-aliasing

#-Wextra -Wimplicit-function-declaration \
	  -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes 
# Compiler flags to generate dependency files:
CFLAGS += -MD -MP -MF $(BIN)/dep/$(@).d -MQ $(@)

# Permits to remove un-used functions and global variables from output file
CFLAGS += -ffunction-sections -fdata-sections

ASFLAGS = $(PROCESSOR) $(INCLUDES)
LDFLAGS = $(PROCESSOR) --static $(LIBS) -T$(LDSCRIPT) -nostartfiles -Wl,-Map=$(PROG).map,--cref,--gc-sections 


# Program name
PROG = Copter
# Where to compile the .o
BIN = bin
VPATH += $(BIN)

# Dependency files to include
DEPS := $(foreach o,$(OBJ),$(BIN)/dep/$(o).d)

######## Targets ########

all: build
build: compile
compile: $(PROG).elf $(PROG).bin $(PROG).hex

CC_COMMAND = $(CC) $(CFLAGS) -c $< -o $(BIN)/$@
.c.o:
	@$(CC_COMMAND)
.S.o:
	@$(CC_COMMAND)

LD_COMMAND = $(LD) $(LDFLAGS) $(foreach o,$(OBJ),$(BIN)/$(o)) -o $@
$(PROG).elf: $(OBJ) $(LDSCRIPT)
	@$(LD_COMMAND)

HEX_COMMAND = $(OBJCOPY) $< -O ihex $@
$(PROG).hex: $(PROG).elf
	@$(HEX_COMMAND)

BIN_COMMAND = $(OBJCOPY) $< -O binary --pad-to 0 $@
$(PROG).bin: $(PROG).elf
	@$(BIN_COMMAND)

#CLEAN_O_COMMAND = rm -f $(foreach o,$(OBJ),$(BIN)/$(o))
CLEAN_O_COMMAND = rm -f $(BIN)/*.o
clean_o:
	@$(CLEAN_O_COMMAND)

CLEAN_COMMAND = rm -f $(PROG).elf $(PROG).bin $(PROG).hex $(PROG).map $(BIN)/dep/*.d
clean: clean_o
	@$(CLEAN_COMMAND)

#WRITE_COMMAND = ./write_bin.sh openocd.cfg $(PROG).elf
#write:
#	@$(WRITE_COMMAND)
