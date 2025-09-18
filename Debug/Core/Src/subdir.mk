################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BN055_IT.c \
../Core/Src/BNO055.c \
../Core/Src/Debug.c \
../Core/Src/MQ135.c \
../Core/Src/PN532.c \
../Core/Src/RC522.c \
../Core/Src/can_receive.c \
../Core/Src/can_send_noblocking.c \
../Core/Src/can_tranceive.c \
../Core/Src/checkstatus.c \
../Core/Src/display.c \
../Core/Src/led_control.c \
../Core/Src/liquidcrystal_i2c.c \
../Core/Src/main.c \
../Core/Src/nvic_config.c \
../Core/Src/rfid.c \
../Core/Src/sensor_test.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/ultrasonic_sensor.c 

OBJS += \
./Core/Src/BN055_IT.o \
./Core/Src/BNO055.o \
./Core/Src/Debug.o \
./Core/Src/MQ135.o \
./Core/Src/PN532.o \
./Core/Src/RC522.o \
./Core/Src/can_receive.o \
./Core/Src/can_send_noblocking.o \
./Core/Src/can_tranceive.o \
./Core/Src/checkstatus.o \
./Core/Src/display.o \
./Core/Src/led_control.o \
./Core/Src/liquidcrystal_i2c.o \
./Core/Src/main.o \
./Core/Src/nvic_config.o \
./Core/Src/rfid.o \
./Core/Src/sensor_test.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/ultrasonic_sensor.o 

C_DEPS += \
./Core/Src/BN055_IT.d \
./Core/Src/BNO055.d \
./Core/Src/Debug.d \
./Core/Src/MQ135.d \
./Core/Src/PN532.d \
./Core/Src/RC522.d \
./Core/Src/can_receive.d \
./Core/Src/can_send_noblocking.d \
./Core/Src/can_tranceive.d \
./Core/Src/checkstatus.d \
./Core/Src/display.d \
./Core/Src/led_control.d \
./Core/Src/liquidcrystal_i2c.d \
./Core/Src/main.d \
./Core/Src/nvic_config.d \
./Core/Src/rfid.d \
./Core/Src/sensor_test.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/ultrasonic_sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BN055_IT.cyclo ./Core/Src/BN055_IT.d ./Core/Src/BN055_IT.o ./Core/Src/BN055_IT.su ./Core/Src/BNO055.cyclo ./Core/Src/BNO055.d ./Core/Src/BNO055.o ./Core/Src/BNO055.su ./Core/Src/Debug.cyclo ./Core/Src/Debug.d ./Core/Src/Debug.o ./Core/Src/Debug.su ./Core/Src/MQ135.cyclo ./Core/Src/MQ135.d ./Core/Src/MQ135.o ./Core/Src/MQ135.su ./Core/Src/PN532.cyclo ./Core/Src/PN532.d ./Core/Src/PN532.o ./Core/Src/PN532.su ./Core/Src/RC522.cyclo ./Core/Src/RC522.d ./Core/Src/RC522.o ./Core/Src/RC522.su ./Core/Src/can_receive.cyclo ./Core/Src/can_receive.d ./Core/Src/can_receive.o ./Core/Src/can_receive.su ./Core/Src/can_send_noblocking.cyclo ./Core/Src/can_send_noblocking.d ./Core/Src/can_send_noblocking.o ./Core/Src/can_send_noblocking.su ./Core/Src/can_tranceive.cyclo ./Core/Src/can_tranceive.d ./Core/Src/can_tranceive.o ./Core/Src/can_tranceive.su ./Core/Src/checkstatus.cyclo ./Core/Src/checkstatus.d ./Core/Src/checkstatus.o ./Core/Src/checkstatus.su ./Core/Src/display.cyclo ./Core/Src/display.d ./Core/Src/display.o ./Core/Src/display.su ./Core/Src/led_control.cyclo ./Core/Src/led_control.d ./Core/Src/led_control.o ./Core/Src/led_control.su ./Core/Src/liquidcrystal_i2c.cyclo ./Core/Src/liquidcrystal_i2c.d ./Core/Src/liquidcrystal_i2c.o ./Core/Src/liquidcrystal_i2c.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/nvic_config.cyclo ./Core/Src/nvic_config.d ./Core/Src/nvic_config.o ./Core/Src/nvic_config.su ./Core/Src/rfid.cyclo ./Core/Src/rfid.d ./Core/Src/rfid.o ./Core/Src/rfid.su ./Core/Src/sensor_test.cyclo ./Core/Src/sensor_test.d ./Core/Src/sensor_test.o ./Core/Src/sensor_test.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/ultrasonic_sensor.cyclo ./Core/Src/ultrasonic_sensor.d ./Core/Src/ultrasonic_sensor.o ./Core/Src/ultrasonic_sensor.su

.PHONY: clean-Core-2f-Src

