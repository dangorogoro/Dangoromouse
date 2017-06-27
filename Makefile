#######################################
# Options
#######################################
PROJECT_NAME := stm32f4_template_project
OUTPUT_DIR := build

CC := arm-none-eabi-gcc
CXX := arm-none-eabi-g++
LD := arm-none-eabi-g++
AS := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size

INCLUDE := \
	-Iinc \
	-Ilib/CMSIS/Device/Include \
	-Ilib/CMSIS/Include \
	-Ilib/STM32F4xx_StdPeriph_Driver/inc \
	-IMazeSolver2015 \
	-I/usr/include/eigen3
	##-Ilib/FreeRTOS/include \
	##-Ilib/FreeRTOS/portable \##
DEFINE_MACRO := \
	-DSTM32F40_41xxx \
	-DSTM32F405xx \
	-DARM_MATH_CM4 \
	-D__FPU_PRESENT \
	-DHSE_VALUE=8000000 \
	-DUSE_STDPERIPH_DRIVER \
	-DDEBUG 

TARGET_ARCH := \
	-mcpu=cortex-m4 \
	-mthumb \
	-mfloat-abi=hard \
	-mfpu=fpv4-sp-d16

CFLAGS :=  $(TARGET_ARCH) $(DEFINE_MACRO) \
	-O0 -g3 -Wall \
	-fmessage-length=0 \
	-fsigned-char \
	-ffunction-sections \
	-fdata-sections \
	-mslow-flash-data

CXXFLAGS := $(CFLAGS) \
	-fno-use-cxa-atexit \
	-fno-exceptions \
	-fno-rtti \
	-fno-threadsafe-statics \
	-std=c++11 \
	-include stdint.h

ASFLAGS := -x assembler-with-cpp -c $(CFLAGS)

LDFLAGS := $(TARGET_ARCH) -T LinkerScript.ld -Wl,--gc-sections
LDLIBS := -lm


#######################################
# Dependencies
#######################################
C_SRCS := $(wildcard src/*.c) \
	$(wildcard lib/CMSIS/Device/Source/*.c) \
	$(wildcard lib/STM32F4xx_StdPeriph_Driver/src/*.c)
#	$(wildcard lib/FreeRTOS/*.c) \
#	$(wildcard lib/FreeRTOS/portable/*.c) \
#	lib/FreeRTOS/portable/MemMang/heap_3.c \

C_SRCS := $(filter-out %/stm32f4xx_fmc.c, $(C_SRCS))

CPP_SRCS := $(wildcard src/*.cpp) \
	$(wildcard MazeSolver2015/*.cpp)

ASM_SRCS := $(wildcard src/*.s) \
	lib/CMSIS/Device/Source/startup_stm32f405xx.s

OBJS := $(patsubst %,$(OUTPUT_DIR)/%, $(C_SRCS:.c=.o) $(CPP_SRCS:.cpp=.o) $(ASM_SRCS:.s=.o))
DEPS := $(OBJS:.o=.d)


#######################################
# Targets
#######################################
.PHONY: all clean

all: $(OUTPUT_DIR) $(OUTPUT_DIR)/$(PROJECT_NAME).elf

$(OUTPUT_DIR):
	@mkdir $(OUTPUT_DIR)

$(OUTPUT_DIR)/$(PROJECT_NAME).elf: $(OBJS)
	@echo "LD $@"
	@$(LD) $(LDFLAGS) $(LDLIBS) -Wl,-Map,$(@:.elf=.map) -o $@ $^
	@$(OBJCOPY) -O ihex $@ $(@:.elf=.hex)
	@$(OBJCOPY) -O binary $@ $(@:.elf=.bin)
	@$(SIZE) $@

$(OUTPUT_DIR)/%.o: %.c
	@echo "CC $<"
	@if [ ! -d $(dir $@) ]; then mkdir -p $(dir $@); fi
	@$(CC) $(CFLAGS) $(INCLUDE) -MMD -MP -MF$(@:.o=.d) -MT$@ -o $@ -c $<

$(OUTPUT_DIR)/%.o: %.cpp
	@echo "CXX $<"
	@if [ ! -d $(dir $@) ]; then mkdir -p $(dir $@); fi
	@$(CXX) $(CXXFLAGS) $(INCLUDE) -MMD -MP -MF$(@:.o=.d) -MT$@ -o $@ -c $<

$(OUTPUT_DIR)/%.o: %.s
	@echo "AS $<"
	@if [ ! -d $(dir $@) ]; then mkdir -p $(dir $@); fi
	@$(AS) $(ASFLAGS) -MMD -MP -MF$(@:.o=.d) -MT$@ -o $@ -c $<

.clang_complete: Makefile
	@echo $(INCLUDE) $(DEFINE_MACRO) | tr " " "\n" > $@


clean:
	@rm -rf $(OUTPUT_DIR)/*


-include $(DEPS)

