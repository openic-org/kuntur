################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../STM32_BLE/Target/blenvm.c \
../STM32_BLE/Target/bleplat.c \
../STM32_BLE/Target/bleplat_cntr.c 

OBJS += \
./STM32_BLE/Target/blenvm.o \
./STM32_BLE/Target/bleplat.o \
./STM32_BLE/Target/bleplat_cntr.o 

C_DEPS += \
./STM32_BLE/Target/blenvm.d \
./STM32_BLE/Target/bleplat.d \
./STM32_BLE/Target/bleplat_cntr.d 


# Each subdirectory must supply rules for building sources it contributes
STM32_BLE/Target/%.o STM32_BLE/Target/%.su STM32_BLE/Target/%.cyclo: ../STM32_BLE/Target/%.c STM32_BLE/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32WB09 -c -I../Core/Inc -I../STM32_BLE/App -I../STM32_BLE/Target -I../System/Config/Debug_GPIO -I../System/Interfaces -I$(STM32CUBE_FW_WB0)/Utilities/trace/adv_trace -I$(STM32CUBE_FW_WB0)/Drivers/STM32WB0x_HAL_Driver/Inc -I$(STM32CUBE_FW_WB0)/Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Interfaces -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/RTDebug -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/RADIO_utils/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/Profiles/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/PKAMGR/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/NVMDB/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/Flash -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Startup -I$(STM32CUBE_FW_WB0)/Utilities/misc -I$(STM32CUBE_FW_WB0)/Utilities/sequencer -I$(STM32CUBE_FW_WB0)/Utilities/lpm/tiny_lpm -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/Common -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CBC -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CMAC -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/Common -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/ECB -I$(STM32CUBE_FW_WB0)/Drivers/CMSIS/Device/ST/STM32WB0X/Include -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/evt_handler/inc -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/queued_writes/inc -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/stack/include -I$(STM32CUBE_FW_WB0)/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-STM32_BLE-2f-Target

clean-STM32_BLE-2f-Target:
	-$(RM) ./STM32_BLE/Target/blenvm.cyclo ./STM32_BLE/Target/blenvm.d ./STM32_BLE/Target/blenvm.o ./STM32_BLE/Target/blenvm.su ./STM32_BLE/Target/bleplat.cyclo ./STM32_BLE/Target/bleplat.d ./STM32_BLE/Target/bleplat.o ./STM32_BLE/Target/bleplat.su ./STM32_BLE/Target/bleplat_cntr.cyclo ./STM32_BLE/Target/bleplat_cntr.d ./STM32_BLE/Target/bleplat_cntr.o ./STM32_BLE/Target/bleplat_cntr.su

.PHONY: clean-STM32_BLE-2f-Target

