################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/MCAL/stm32f0xx_hal_msp.c \
../Core/MCAL/stm32f0xx_hal_timebase_tim.c \
../Core/MCAL/stm32f0xx_it.c \
../Core/MCAL/system_stm32f0xx.c 

OBJS += \
./Core/MCAL/stm32f0xx_hal_msp.o \
./Core/MCAL/stm32f0xx_hal_timebase_tim.o \
./Core/MCAL/stm32f0xx_it.o \
./Core/MCAL/system_stm32f0xx.o 

C_DEPS += \
./Core/MCAL/stm32f0xx_hal_msp.d \
./Core/MCAL/stm32f0xx_hal_timebase_tim.d \
./Core/MCAL/stm32f0xx_it.d \
./Core/MCAL/system_stm32f0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/MCAL/%.o Core/MCAL/%.su Core/MCAL/%.cyclo: ../Core/MCAL/%.c Core/MCAL/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../Core/Applications -I../Core/ComplexDrivers -I../Core/ComplexDrivers/Startup -I../Core/EcuAbstraction -I../Core/MCAL -I../Core/RTE -I../Core/Services/Main -I../Core/Services/OS -I../Core/Services -I../Core/Services/Display -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-MCAL

clean-Core-2f-MCAL:
	-$(RM) ./Core/MCAL/stm32f0xx_hal_msp.cyclo ./Core/MCAL/stm32f0xx_hal_msp.d ./Core/MCAL/stm32f0xx_hal_msp.o ./Core/MCAL/stm32f0xx_hal_msp.su ./Core/MCAL/stm32f0xx_hal_timebase_tim.cyclo ./Core/MCAL/stm32f0xx_hal_timebase_tim.d ./Core/MCAL/stm32f0xx_hal_timebase_tim.o ./Core/MCAL/stm32f0xx_hal_timebase_tim.su ./Core/MCAL/stm32f0xx_it.cyclo ./Core/MCAL/stm32f0xx_it.d ./Core/MCAL/stm32f0xx_it.o ./Core/MCAL/stm32f0xx_it.su ./Core/MCAL/system_stm32f0xx.cyclo ./Core/MCAL/system_stm32f0xx.d ./Core/MCAL/system_stm32f0xx.o ./Core/MCAL/system_stm32f0xx.su

.PHONY: clean-Core-2f-MCAL

