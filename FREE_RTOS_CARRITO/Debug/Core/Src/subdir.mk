################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MPU9250-DMP.c \
../Core/Src/encoder_task.c \
../Core/Src/freertos.c \
../Core/Src/globals.c \
../Core/Src/inv_mpu.c \
../Core/Src/inv_mpu_dmp_motion_driver.c \
../Core/Src/main.c \
../Core/Src/motor_task.c \
../Core/Src/mpu_task.c \
../Core/Src/servo_task.c \
../Core/Src/stm32_mpu9250_clk.c \
../Core/Src/stm32_mpu9250_i2c.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/uart_task.c \
../Core/Src/ultrasonic_task.c \
../Core/Src/ultrasonico.c 

OBJS += \
./Core/Src/MPU9250-DMP.o \
./Core/Src/encoder_task.o \
./Core/Src/freertos.o \
./Core/Src/globals.o \
./Core/Src/inv_mpu.o \
./Core/Src/inv_mpu_dmp_motion_driver.o \
./Core/Src/main.o \
./Core/Src/motor_task.o \
./Core/Src/mpu_task.o \
./Core/Src/servo_task.o \
./Core/Src/stm32_mpu9250_clk.o \
./Core/Src/stm32_mpu9250_i2c.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/uart_task.o \
./Core/Src/ultrasonic_task.o \
./Core/Src/ultrasonico.o 

C_DEPS += \
./Core/Src/MPU9250-DMP.d \
./Core/Src/encoder_task.d \
./Core/Src/freertos.d \
./Core/Src/globals.d \
./Core/Src/inv_mpu.d \
./Core/Src/inv_mpu_dmp_motion_driver.d \
./Core/Src/main.d \
./Core/Src/motor_task.d \
./Core/Src/mpu_task.d \
./Core/Src/servo_task.d \
./Core/Src/stm32_mpu9250_clk.d \
./Core/Src/stm32_mpu9250_i2c.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/uart_task.d \
./Core/Src/ultrasonic_task.d \
./Core/Src/ultrasonico.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

