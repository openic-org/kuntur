################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Utilities/subdir.mk
-include System/Interfaces/subdir.mk
-include System/Config/Debug_GPIO/subdir.mk
-include STM32_BLE/Target/subdir.mk
-include STM32_BLE/App/subdir.mk
-include Middlewares/STM32_BLE/subdir.mk
-include Drivers/STM32WB0x_HAL_Driver/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Common/BLE/Startup/subdir.mk
-include Common/BLE/Modules/RTDebug/subdir.mk
-include Common/BLE/Modules/RADIO_utils/subdir.mk
-include Common/BLE/Modules/Profiles/subdir.mk
-include Common/BLE/Modules/PKAMGR/subdir.mk
-include Common/BLE/Modules/NVMDB/subdir.mk
-include Common/BLE/Modules/Flash/subdir.mk
-include Common/BLE/Modules/subdir.mk
-include Common/BLE/Interface/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := kuntur-mcu
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
kuntur-mcu.elf \

MAP_FILES += \
kuntur-mcu.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
kuntur-mcu.list \

OBJCOPY_HEX += \
kuntur-mcu.hex \


# All Target
all: main-build

# Main-build Target
main-build: kuntur-mcu.elf secondary-outputs

# Tool invocations
kuntur-mcu.elf kuntur-mcu.map: $(OBJS) $(USER_OBJS) ../STM32WB09TEFX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "kuntur-mcu.elf" @"objects.list" $(USER_OBJS) -Wl,--start-group $(LIBS) -Wl,--end-group -mcpu=cortex-m0plus -T"../STM32WB09TEFX_FLASH.ld" --specs=nosys.specs -Wl,-Map="kuntur-mcu.map" -Wl,--gc-sections -static -L/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/cryptolib -L/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/stack/lib -z noexecstack --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

kuntur-mcu.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "kuntur-mcu.list"
	@echo 'Finished building: $@'
	@echo ' '

kuntur-mcu.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "kuntur-mcu.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout kuntur-mcu.elf kuntur-mcu.hex kuntur-mcu.list kuntur-mcu.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
