################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app_entry.c \
../Core/Src/cpu_bench.c \
../Core/Src/fpga_spi.c \
../Core/Src/gpio_bench.c \
../Core/Src/main.c \
../Core/Src/stm32wb0x_hal_msp.c \
../Core/Src/stm32wb0x_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32wb0x.c 

OBJS += \
./Core/Src/app_entry.o \
./Core/Src/cpu_bench.o \
./Core/Src/fpga_spi.o \
./Core/Src/gpio_bench.o \
./Core/Src/main.o \
./Core/Src/stm32wb0x_hal_msp.o \
./Core/Src/stm32wb0x_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32wb0x.o 

C_DEPS += \
./Core/Src/app_entry.d \
./Core/Src/cpu_bench.d \
./Core/Src/fpga_spi.d \
./Core/Src/gpio_bench.d \
./Core/Src/main.d \
./Core/Src/stm32wb0x_hal_msp.d \
./Core/Src/stm32wb0x_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32wb0x.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32WB09 -c -I../Core/Inc -I../STM32_BLE/App -I../STM32_BLE/Target -I../System/Config/Debug_GPIO -I../System/Interfaces -I$(STM32CUBE_FW_WB0)/Utilities/trace/adv_trace -I$(STM32CUBE_FW_WB0)/Drivers/STM32WB0x_HAL_Driver/Inc -I$(STM32CUBE_FW_WB0)/Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Interfaces -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/RTDebug -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/RADIO_utils/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/Profiles/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/PKAMGR/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/NVMDB/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/Flash -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Startup -I$(STM32CUBE_FW_WB0)/Utilities/misc -I$(STM32CUBE_FW_WB0)/Utilities/sequencer -I$(STM32CUBE_FW_WB0)/Utilities/lpm/tiny_lpm -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/Common -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CBC -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CMAC -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/Common -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/ECB -I$(STM32CUBE_FW_WB0)/Drivers/CMSIS/Device/ST/STM32WB0X/Include -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/evt_handler/inc -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/queued_writes/inc -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/stack/include -I$(STM32CUBE_FW_WB0)/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/app_entry.cyclo ./Core/Src/app_entry.d ./Core/Src/app_entry.o ./Core/Src/app_entry.su ./Core/Src/cpu_bench.cyclo ./Core/Src/cpu_bench.d ./Core/Src/cpu_bench.o ./Core/Src/cpu_bench.su ./Core/Src/fpga_spi.cyclo ./Core/Src/fpga_spi.d ./Core/Src/fpga_spi.o ./Core/Src/fpga_spi.su ./Core/Src/gpio_bench.cyclo ./Core/Src/gpio_bench.d ./Core/Src/gpio_bench.o ./Core/Src/gpio_bench.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32wb0x_hal_msp.cyclo ./Core/Src/stm32wb0x_hal_msp.d ./Core/Src/stm32wb0x_hal_msp.o ./Core/Src/stm32wb0x_hal_msp.su ./Core/Src/stm32wb0x_it.cyclo ./Core/Src/stm32wb0x_it.d ./Core/Src/stm32wb0x_it.o ./Core/Src/stm32wb0x_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32wb0x.cyclo ./Core/Src/system_stm32wb0x.d ./Core/Src/system_stm32wb0x.o ./Core/Src/system_stm32wb0x.su

.PHONY: clean-Core-2f-Src

