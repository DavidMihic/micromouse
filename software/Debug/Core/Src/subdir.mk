################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app.c \
../Core/Src/button.c \
../Core/Src/encoder.c \
../Core/Src/imu.c \
../Core/Src/ir_array.c \
../Core/Src/ir_sensors.c \
../Core/Src/main.c \
../Core/Src/motion.c \
../Core/Src/motor.c \
../Core/Src/navigator.c \
../Core/Src/neopixel.c \
../Core/Src/robot_controller.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/velocity_pi.c 

OBJS += \
./Core/Src/app.o \
./Core/Src/button.o \
./Core/Src/encoder.o \
./Core/Src/imu.o \
./Core/Src/ir_array.o \
./Core/Src/ir_sensors.o \
./Core/Src/main.o \
./Core/Src/motion.o \
./Core/Src/motor.o \
./Core/Src/navigator.o \
./Core/Src/neopixel.o \
./Core/Src/robot_controller.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/velocity_pi.o 

C_DEPS += \
./Core/Src/app.d \
./Core/Src/button.d \
./Core/Src/encoder.d \
./Core/Src/imu.d \
./Core/Src/ir_array.d \
./Core/Src/ir_sensors.d \
./Core/Src/main.d \
./Core/Src/motion.d \
./Core/Src/motor.d \
./Core/Src/navigator.d \
./Core/Src/neopixel.d \
./Core/Src/robot_controller.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/velocity_pi.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/app.cyclo ./Core/Src/app.d ./Core/Src/app.o ./Core/Src/app.su ./Core/Src/button.cyclo ./Core/Src/button.d ./Core/Src/button.o ./Core/Src/button.su ./Core/Src/encoder.cyclo ./Core/Src/encoder.d ./Core/Src/encoder.o ./Core/Src/encoder.su ./Core/Src/imu.cyclo ./Core/Src/imu.d ./Core/Src/imu.o ./Core/Src/imu.su ./Core/Src/ir_array.cyclo ./Core/Src/ir_array.d ./Core/Src/ir_array.o ./Core/Src/ir_array.su ./Core/Src/ir_sensors.cyclo ./Core/Src/ir_sensors.d ./Core/Src/ir_sensors.o ./Core/Src/ir_sensors.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motion.cyclo ./Core/Src/motion.d ./Core/Src/motion.o ./Core/Src/motion.su ./Core/Src/motor.cyclo ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/motor.su ./Core/Src/navigator.cyclo ./Core/Src/navigator.d ./Core/Src/navigator.o ./Core/Src/navigator.su ./Core/Src/neopixel.cyclo ./Core/Src/neopixel.d ./Core/Src/neopixel.o ./Core/Src/neopixel.su ./Core/Src/robot_controller.cyclo ./Core/Src/robot_controller.d ./Core/Src/robot_controller.o ./Core/Src/robot_controller.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/velocity_pi.cyclo ./Core/Src/velocity_pi.d ./Core/Src/velocity_pi.o ./Core/Src/velocity_pi.su

.PHONY: clean-Core-2f-Src

