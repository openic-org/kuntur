################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../STM32_BLE/App/app_ble.c \
../STM32_BLE/App/dis.c \
../STM32_BLE/App/dis_app.c \
../STM32_BLE/App/stream.c \
../STM32_BLE/App/stream_app.c 

OBJS += \
./STM32_BLE/App/app_ble.o \
./STM32_BLE/App/dis.o \
./STM32_BLE/App/dis_app.o \
./STM32_BLE/App/stream.o \
./STM32_BLE/App/stream_app.o 

C_DEPS += \
./STM32_BLE/App/app_ble.d \
./STM32_BLE/App/dis.d \
./STM32_BLE/App/dis_app.d \
./STM32_BLE/App/stream.d \
./STM32_BLE/App/stream_app.d 


# Each subdirectory must supply rules for building sources it contributes
STM32_BLE/App/%.o STM32_BLE/App/%.su STM32_BLE/App/%.cyclo: ../STM32_BLE/App/%.c STM32_BLE/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32WB09 -c -I../Core/Inc -I../STM32_BLE/App -I../STM32_BLE/Target -I../System/Config/Debug_GPIO -I../System/Interfaces -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Utilities/trace/adv_trace -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Drivers/STM32WB0x_HAL_Driver/Inc -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Projects/Common/BLE/Interfaces -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Projects/Common/BLE/Modules -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Projects/Common/BLE/Modules/RTDebug -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Projects/Common/BLE/Modules/RADIO_utils/Inc -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Projects/Common/BLE/Modules/Profiles/Inc -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Projects/Common/BLE/Modules/PKAMGR/Inc -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Projects/Common/BLE/Modules/NVMDB/Inc -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Projects/Common/BLE/Modules/Flash -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Projects/Common/BLE/Startup -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Utilities/misc -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Utilities/sequencer -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Utilities/lpm/tiny_lpm -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/cryptolib/Inc -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/Common -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CBC -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CMAC -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/Common -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/ECB -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Drivers/CMSIS/Device/ST/STM32WB0X/Include -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/evt_handler/inc -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/queued_writes/inc -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Middlewares/ST/STM32_BLE/stack/include -I/home/manuel/STM32Cube/Repository/STM32Cube_FW_WB0_V1.4.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-STM32_BLE-2f-App

clean-STM32_BLE-2f-App:
	-$(RM) ./STM32_BLE/App/app_ble.cyclo ./STM32_BLE/App/app_ble.d ./STM32_BLE/App/app_ble.o ./STM32_BLE/App/app_ble.su ./STM32_BLE/App/dis.cyclo ./STM32_BLE/App/dis.d ./STM32_BLE/App/dis.o ./STM32_BLE/App/dis.su ./STM32_BLE/App/dis_app.cyclo ./STM32_BLE/App/dis_app.d ./STM32_BLE/App/dis_app.o ./STM32_BLE/App/dis_app.su ./STM32_BLE/App/stream.cyclo ./STM32_BLE/App/stream.d ./STM32_BLE/App/stream.o ./STM32_BLE/App/stream.su ./STM32_BLE/App/stream_app.cyclo ./STM32_BLE/App/stream_app.d ./STM32_BLE/App/stream_app.o ./STM32_BLE/App/stream_app.su

.PHONY: clean-STM32_BLE-2f-App

