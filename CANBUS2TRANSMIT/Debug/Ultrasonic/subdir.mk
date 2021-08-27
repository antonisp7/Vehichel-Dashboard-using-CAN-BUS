################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Ultrasonic/Ultrasonic.c 

OBJS += \
./Ultrasonic/Ultrasonic.o 

C_DEPS += \
./Ultrasonic/Ultrasonic.d 


# Each subdirectory must supply rules for building sources it contributes
Ultrasonic/Ultrasonic.o: ../Ultrasonic/Ultrasonic.c Ultrasonic/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../lcd16x2 -I../Temperature -I../Pressure -I../PWM -I../Ultrasonic -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Ultrasonic/Ultrasonic.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

