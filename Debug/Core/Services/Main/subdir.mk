################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Services/Main/main.c 

OBJS += \
./Core/Services/Main/main.o 

C_DEPS += \
./Core/Services/Main/main.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Services/Main/%.o Core/Services/Main/%.su Core/Services/Main/%.cyclo: ../Core/Services/Main/%.c Core/Services/Main/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../Core/Applications -I../Core/ComplexDrivers -I../Core/ComplexDrivers/Startup -I../Core/EcuAbstraction -I../Core/MCAL -I../Core/RTE -I../Core/Services/Main -I../Core/Services/OS -I../Core/Services -I../Core/Services/Display -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Services-2f-Main

clean-Core-2f-Services-2f-Main:
	-$(RM) ./Core/Services/Main/main.cyclo ./Core/Services/Main/main.d ./Core/Services/Main/main.o ./Core/Services/Main/main.su

.PHONY: clean-Core-2f-Services-2f-Main

