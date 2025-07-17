################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/ComplexDrivers/Startup/startup_stm32f091rctx.s 

C_SRCS += \
../Core/ComplexDrivers/Startup/startup.c 

OBJS += \
./Core/ComplexDrivers/Startup/startup.o \
./Core/ComplexDrivers/Startup/startup_stm32f091rctx.o 

S_DEPS += \
./Core/ComplexDrivers/Startup/startup_stm32f091rctx.d 

C_DEPS += \
./Core/ComplexDrivers/Startup/startup.d 


# Each subdirectory must supply rules for building sources it contributes
Core/ComplexDrivers/Startup/%.o Core/ComplexDrivers/Startup/%.su Core/ComplexDrivers/Startup/%.cyclo: ../Core/ComplexDrivers/Startup/%.c Core/ComplexDrivers/Startup/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../Core/Applications -I../Core/ComplexDrivers -I../Core/ComplexDrivers/Startup -I../Core/EcuAbstraction -I../Core/MCAL -I../Core/RTE -I../Core/Services/Main -I../Core/Services/OS -I../Core/Services -I../Core/Services/Display -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/ComplexDrivers/Startup/%.o: ../Core/ComplexDrivers/Startup/%.s Core/ComplexDrivers/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0 -g3 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Core-2f-ComplexDrivers-2f-Startup

clean-Core-2f-ComplexDrivers-2f-Startup:
	-$(RM) ./Core/ComplexDrivers/Startup/startup.cyclo ./Core/ComplexDrivers/Startup/startup.d ./Core/ComplexDrivers/Startup/startup.o ./Core/ComplexDrivers/Startup/startup.su ./Core/ComplexDrivers/Startup/startup_stm32f091rctx.d ./Core/ComplexDrivers/Startup/startup_stm32f091rctx.o

.PHONY: clean-Core-2f-ComplexDrivers-2f-Startup

