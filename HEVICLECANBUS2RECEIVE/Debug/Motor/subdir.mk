################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Motor/DC_MOTOR.c \
../Motor/DC_MOTOR_cfg.c 

OBJS += \
./Motor/DC_MOTOR.o \
./Motor/DC_MOTOR_cfg.o 

C_DEPS += \
./Motor/DC_MOTOR.d \
./Motor/DC_MOTOR_cfg.d 


# Each subdirectory must supply rules for building sources it contributes
Motor/DC_MOTOR.o: ../Motor/DC_MOTOR.c Motor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../lcd16x2 -I../Motor -O0 -ffunction-sections -Wall -u _printf_float -fstack-usage -MMD -MP -MF"Motor/DC_MOTOR.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Motor/DC_MOTOR_cfg.o: ../Motor/DC_MOTOR_cfg.c Motor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../lcd16x2 -I../Motor -O0 -ffunction-sections -Wall -u _printf_float -fstack-usage -MMD -MP -MF"Motor/DC_MOTOR_cfg.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

