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
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32WB09 -c -I../Core/Inc -I../STM32_BLE/App -I../STM32_BLE/Target -I../System/Config/Debug_GPIO -I../System/Interfaces -I$(STM32CUBE_FW_WB0)/Utilities/trace/adv_trace -I$(STM32CUBE_FW_WB0)/Drivers/STM32WB0x_HAL_Driver/Inc -I$(STM32CUBE_FW_WB0)/Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Interfaces -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/RTDebug -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/RADIO_utils/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/Profiles/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/PKAMGR/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/NVMDB/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/Flash -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Startup -I$(STM32CUBE_FW_WB0)/Utilities/misc -I$(STM32CUBE_FW_WB0)/Utilities/sequencer -I$(STM32CUBE_FW_WB0)/Utilities/lpm/tiny_lpm -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/Common -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CBC -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CMAC -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/Common -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/ECB -I$(STM32CUBE_FW_WB0)/Drivers/CMSIS/Device/ST/STM32WB0X/Include -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/evt_handler/inc -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/queued_writes/inc -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/stack/include -I$(STM32CUBE_FW_WB0)/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-STM32_BLE-2f-App

clean-STM32_BLE-2f-App:
	-$(RM) ./STM32_BLE/App/app_ble.cyclo ./STM32_BLE/App/app_ble.d ./STM32_BLE/App/app_ble.o ./STM32_BLE/App/app_ble.su ./STM32_BLE/App/dis.cyclo ./STM32_BLE/App/dis.d ./STM32_BLE/App/dis.o ./STM32_BLE/App/dis.su ./STM32_BLE/App/dis_app.cyclo ./STM32_BLE/App/dis_app.d ./STM32_BLE/App/dis_app.o ./STM32_BLE/App/dis_app.su ./STM32_BLE/App/stream.cyclo ./STM32_BLE/App/stream.d ./STM32_BLE/App/stream.o ./STM32_BLE/App/stream.su ./STM32_BLE/App/stream_app.cyclo ./STM32_BLE/App/stream_app.d ./STM32_BLE/App/stream_app.o ./STM32_BLE/App/stream_app.su

.PHONY: clean-STM32_BLE-2f-App

