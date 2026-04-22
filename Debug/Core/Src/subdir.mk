################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CCSDS.c \
../Core/Src/GENERIC_IMU.c \
../Core/Src/NOVATEL_OEM615.c \
../Core/Src/anti_spoofing.c \
../Core/Src/main.c \
../Core/Src/parser.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/utils.c 

OBJS += \
./Core/Src/CCSDS.o \
./Core/Src/GENERIC_IMU.o \
./Core/Src/NOVATEL_OEM615.o \
./Core/Src/anti_spoofing.o \
./Core/Src/main.o \
./Core/Src/parser.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/utils.o 

C_DEPS += \
./Core/Src/CCSDS.d \
./Core/Src/GENERIC_IMU.d \
./Core/Src/NOVATEL_OEM615.d \
./Core/Src/anti_spoofing.d \
./Core/Src/main.d \
./Core/Src/parser.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/utils.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/CCSDS.cyclo ./Core/Src/CCSDS.d ./Core/Src/CCSDS.o ./Core/Src/CCSDS.su ./Core/Src/GENERIC_IMU.cyclo ./Core/Src/GENERIC_IMU.d ./Core/Src/GENERIC_IMU.o ./Core/Src/GENERIC_IMU.su ./Core/Src/NOVATEL_OEM615.cyclo ./Core/Src/NOVATEL_OEM615.d ./Core/Src/NOVATEL_OEM615.o ./Core/Src/NOVATEL_OEM615.su ./Core/Src/anti_spoofing.cyclo ./Core/Src/anti_spoofing.d ./Core/Src/anti_spoofing.o ./Core/Src/anti_spoofing.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/parser.cyclo ./Core/Src/parser.d ./Core/Src/parser.o ./Core/Src/parser.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/utils.cyclo ./Core/Src/utils.d ./Core/Src/utils.o ./Core/Src/utils.su

.PHONY: clean-Core-2f-Src

