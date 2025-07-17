################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Applications/MAFilterButton.c \
../Core/Applications/MAFilterVolt.c \
../Core/Applications/VoltInterpolation.c 

OBJS += \
./Core/Applications/MAFilterButton.o \
./Core/Applications/MAFilterVolt.o \
./Core/Applications/VoltInterpolation.o 

C_DEPS += \
./Core/Applications/MAFilterButton.d \
./Core/Applications/MAFilterVolt.d \
./Core/Applications/VoltInterpolation.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Applications/%.o Core/Applications/%.su Core/Applications/%.cyclo: ../Core/Applications/%.c Core/Applications/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../Core/Applications -I../Core/ComplexDrivers -I../Core/ComplexDrivers/Startup -I../Core/EcuAbstraction -I../Core/MCAL -I../Core/RTE -I../Core/Services/Main -I../Core/Services/OS -I../Core/Services -I../Core/Services/Display -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Applications

clean-Core-2f-Applications:
	-$(RM) ./Core/Applications/MAFilterButton.cyclo ./Core/Applications/MAFilterButton.d ./Core/Applications/MAFilterButton.o ./Core/Applications/MAFilterButton.su ./Core/Applications/MAFilterVolt.cyclo ./Core/Applications/MAFilterVolt.d ./Core/Applications/MAFilterVolt.o ./Core/Applications/MAFilterVolt.su ./Core/Applications/VoltInterpolation.cyclo ./Core/Applications/VoltInterpolation.d ./Core/Applications/VoltInterpolation.o ./Core/Applications/VoltInterpolation.su

.PHONY: clean-Core-2f-Applications

