################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../System/Interfaces/usart_if.c 

OBJS += \
./System/Interfaces/usart_if.o 

C_DEPS += \
./System/Interfaces/usart_if.d 


# Each subdirectory must supply rules for building sources it contributes
System/Interfaces/%.o System/Interfaces/%.su System/Interfaces/%.cyclo: ../System/Interfaces/%.c System/Interfaces/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32WB09 -c -I../Core/Inc -I../STM32_BLE/App -I../STM32_BLE/Target -I../System/Config/Debug_GPIO -I../System/Interfaces -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Utilities/trace/adv_trace -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Drivers/STM32WB0x_HAL_Driver/Inc -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Projects/Common/BLE/Interfaces -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Projects/Common/BLE/Modules -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Projects/Common/BLE/Modules/RTDebug -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Projects/Common/BLE/Modules/RADIO_utils/Inc -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Projects/Common/BLE/Modules/Profiles/Inc -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Projects/Common/BLE/Modules/PKAMGR/Inc -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Projects/Common/BLE/Modules/NVMDB/Inc -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Projects/Common/BLE/Modules/Flash -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Projects/Common/BLE/Startup -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Utilities/misc -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Utilities/sequencer -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Utilities/lpm/tiny_lpm -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/cryptolib/Inc -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/Common -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CBC -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CMAC -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/Common -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/ECB -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Drivers/CMSIS/Device/ST/STM32WB0X/Include -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/evt_handler/inc -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/queued_writes/inc -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/stack/include -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-System-2f-Interfaces

clean-System-2f-Interfaces:
	-$(RM) ./System/Interfaces/usart_if.cyclo ./System/Interfaces/usart_if.d ./System/Interfaces/usart_if.o ./System/Interfaces/usart_if.su

.PHONY: clean-System-2f-Interfaces

