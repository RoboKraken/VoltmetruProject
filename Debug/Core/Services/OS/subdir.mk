################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Services/OS/freertos.c 

OBJS += \
./Core/Services/OS/freertos.o 

C_DEPS += \
./Core/Services/OS/freertos.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Services/OS/%.o Core/Services/OS/%.su Core/Services/OS/%.cyclo: ../Core/Services/OS/%.c Core/Services/OS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../Core/Applications -I../Core/ComplexDrivers -I../Core/ComplexDrivers/Startup -I../Core/EcuAbstraction -I../Core/MCAL -I../Core/RTE -I../Core/Services/Main -I../Core/Services/OS -I../Core/Services -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Services-2f-OS

clean-Core-2f-Services-2f-OS:
	-$(RM) ./Core/Services/OS/freertos.cyclo ./Core/Services/OS/freertos.d ./Core/Services/OS/freertos.o ./Core/Services/OS/freertos.su

.PHONY: clean-Core-2f-Services-2f-OS

