################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/NimaLTD_Driver/EE/ee.c 

OBJS += \
./Middlewares/Third_Party/NimaLTD_Driver/EE/ee.o 

C_DEPS += \
./Middlewares/Third_Party/NimaLTD_Driver/EE/ee.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/NimaLTD_Driver/EE/%.o Middlewares/Third_Party/NimaLTD_Driver/EE/%.su Middlewares/Third_Party/NimaLTD_Driver/EE/%.cyclo: ../Middlewares/Third_Party/NimaLTD_Driver/EE/%.c Middlewares/Third_Party/NimaLTD_Driver/EE/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32C071xx -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/NimaLTD_Driver/EE -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-NimaLTD_Driver-2f-EE

clean-Middlewares-2f-Third_Party-2f-NimaLTD_Driver-2f-EE:
	-$(RM) ./Middlewares/Third_Party/NimaLTD_Driver/EE/ee.cyclo ./Middlewares/Third_Party/NimaLTD_Driver/EE/ee.d ./Middlewares/Third_Party/NimaLTD_Driver/EE/ee.o ./Middlewares/Third_Party/NimaLTD_Driver/EE/ee.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-NimaLTD_Driver-2f-EE

