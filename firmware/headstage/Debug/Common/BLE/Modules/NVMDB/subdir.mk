################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/NVMDB/Src/nvm_db.c \
$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/NVMDB/Src/nvm_db_conf.c 

OBJS += \
./Common/BLE/Modules/NVMDB/nvm_db.o \
./Common/BLE/Modules/NVMDB/nvm_db_conf.o 

C_DEPS += \
./Common/BLE/Modules/NVMDB/nvm_db.d \
./Common/BLE/Modules/NVMDB/nvm_db_conf.d 


# Each subdirectory must supply rules for building sources it contributes
Common/BLE/Modules/NVMDB/nvm_db.o: $(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/NVMDB/Src/nvm_db.c Common/BLE/Modules/NVMDB/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32WB09 -c -I../Core/Inc -I../STM32_BLE/App -I../STM32_BLE/Target -I../System/Config/Debug_GPIO -I../System/Interfaces -I$(STM32CUBE_FW_WB0)/Utilities/trace/adv_trace -I$(STM32CUBE_FW_WB0)/Drivers/STM32WB0x_HAL_Driver/Inc -I$(STM32CUBE_FW_WB0)/Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Interfaces -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/RTDebug -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/RADIO_utils/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/Profiles/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/PKAMGR/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/NVMDB/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/Flash -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Startup -I$(STM32CUBE_FW_WB0)/Utilities/misc -I$(STM32CUBE_FW_WB0)/Utilities/sequencer -I$(STM32CUBE_FW_WB0)/Utilities/lpm/tiny_lpm -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/Common -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CBC -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CMAC -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/Common -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/ECB -I$(STM32CUBE_FW_WB0)/Drivers/CMSIS/Device/ST/STM32WB0X/Include -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/evt_handler/inc -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/queued_writes/inc -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/stack/include -I$(STM32CUBE_FW_WB0)/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Common/BLE/Modules/NVMDB/nvm_db_conf.o: $(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/NVMDB/Src/nvm_db_conf.c Common/BLE/Modules/NVMDB/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32WB09 -c -I../Core/Inc -I../STM32_BLE/App -I../STM32_BLE/Target -I../System/Config/Debug_GPIO -I../System/Interfaces -I$(STM32CUBE_FW_WB0)/Utilities/trace/adv_trace -I$(STM32CUBE_FW_WB0)/Drivers/STM32WB0x_HAL_Driver/Inc -I$(STM32CUBE_FW_WB0)/Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Interfaces -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/RTDebug -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/RADIO_utils/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/Profiles/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/PKAMGR/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/NVMDB/Inc -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Modules/Flash -I$(STM32CUBE_FW_WB0)/Projects/Common/BLE/Startup -I$(STM32CUBE_FW_WB0)/Utilities/misc -I$(STM32CUBE_FW_WB0)/Utilities/sequencer -I$(STM32CUBE_FW_WB0)/Utilities/lpm/tiny_lpm -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/Common -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CBC -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CMAC -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/Common -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/ECB -I$(STM32CUBE_FW_WB0)/Drivers/CMSIS/Device/ST/STM32WB0X/Include -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/evt_handler/inc -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/queued_writes/inc -I$(STM32CUBE_FW_WB0)/Middlewares/ST/STM32_BLE/stack/include -I$(STM32CUBE_FW_WB0)/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Common-2f-BLE-2f-Modules-2f-NVMDB

clean-Common-2f-BLE-2f-Modules-2f-NVMDB:
	-$(RM) ./Common/BLE/Modules/NVMDB/nvm_db.cyclo ./Common/BLE/Modules/NVMDB/nvm_db.d ./Common/BLE/Modules/NVMDB/nvm_db.o ./Common/BLE/Modules/NVMDB/nvm_db.su ./Common/BLE/Modules/NVMDB/nvm_db_conf.cyclo ./Common/BLE/Modules/NVMDB/nvm_db_conf.d ./Common/BLE/Modules/NVMDB/nvm_db_conf.o ./Common/BLE/Modules/NVMDB/nvm_db_conf.su

.PHONY: clean-Common-2f-BLE-2f-Modules-2f-NVMDB

