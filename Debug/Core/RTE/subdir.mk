################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/RTE/Rte.c \
../Core/RTE/Rte_Tasks.c 

OBJS += \
./Core/RTE/Rte.o \
./Core/RTE/Rte_Tasks.o 

C_DEPS += \
./Core/RTE/Rte.d \
./Core/RTE/Rte_Tasks.d 


# Each subdirectory must supply rules for building sources it contributes
Core/RTE/%.o Core/RTE/%.su Core/RTE/%.cyclo: ../Core/RTE/%.c Core/RTE/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../Core/Applications -I../Core/ComplexDrivers -I../Core/ComplexDrivers/Startup -I../Core/EcuAbstraction -I../Core/MCAL -I../Core/RTE -I../Core/Services/Main -I../Core/Services/OS -I../Core/Services -I../Core/Services/Display -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-RTE

clean-Core-2f-RTE:
	-$(RM) ./Core/RTE/Rte.cyclo ./Core/RTE/Rte.d ./Core/RTE/Rte.o ./Core/RTE/Rte.su ./Core/RTE/Rte_Tasks.cyclo ./Core/RTE/Rte_Tasks.d ./Core/RTE/Rte_Tasks.o ./Core/RTE/Rte_Tasks.su

.PHONY: clean-Core-2f-RTE

