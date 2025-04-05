################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../APP/MPU6050/inv_mpu.c \
../APP/MPU6050/inv_mpu_dmp_motion_driver.c \
../APP/MPU6050/mpu6050.c 

OBJS += \
./APP/MPU6050/inv_mpu.o \
./APP/MPU6050/inv_mpu_dmp_motion_driver.o \
./APP/MPU6050/mpu6050.o 

C_DEPS += \
./APP/MPU6050/inv_mpu.d \
./APP/MPU6050/inv_mpu_dmp_motion_driver.d \
./APP/MPU6050/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
APP/MPU6050/%.o APP/MPU6050/%.su APP/MPU6050/%.cyclo: ../APP/MPU6050/%.c APP/MPU6050/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"E:/Files/STM32_Project/Hello/APP/MPU6050" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-APP-2f-MPU6050

clean-APP-2f-MPU6050:
	-$(RM) ./APP/MPU6050/inv_mpu.cyclo ./APP/MPU6050/inv_mpu.d ./APP/MPU6050/inv_mpu.o ./APP/MPU6050/inv_mpu.su ./APP/MPU6050/inv_mpu_dmp_motion_driver.cyclo ./APP/MPU6050/inv_mpu_dmp_motion_driver.d ./APP/MPU6050/inv_mpu_dmp_motion_driver.o ./APP/MPU6050/inv_mpu_dmp_motion_driver.su ./APP/MPU6050/mpu6050.cyclo ./APP/MPU6050/mpu6050.d ./APP/MPU6050/mpu6050.o ./APP/MPU6050/mpu6050.su

.PHONY: clean-APP-2f-MPU6050

