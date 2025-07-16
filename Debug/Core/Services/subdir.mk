################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Services/syscalls.c \
../Core/Services/sysmem.c 

OBJS += \
./Core/Services/syscalls.o \
./Core/Services/sysmem.o 

C_DEPS += \
./Core/Services/syscalls.d \
./Core/Services/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Services/%.o Core/Services/%.su Core/Services/%.cyclo: ../Core/Services/%.c Core/Services/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../Core/Applications -I../Core/ComplexDrivers -I../Core/ComplexDrivers/Startup -I../Core/EcuAbstraction -I../Core/MCAL -I../Core/RTE -I../Core/Services/Main -I../Core/Services/OS -I../Core/Services -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Services

clean-Core-2f-Services:
	-$(RM) ./Core/Services/syscalls.cyclo ./Core/Services/syscalls.d ./Core/Services/syscalls.o ./Core/Services/syscalls.su ./Core/Services/sysmem.cyclo ./Core/Services/sysmem.d ./Core/Services/sysmem.o ./Core/Services/sysmem.su

.PHONY: clean-Core-2f-Services

