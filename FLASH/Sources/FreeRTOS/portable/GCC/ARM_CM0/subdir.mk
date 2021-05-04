################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../../../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/FreeRTOS/portable/GCC/ARM_CM0/port.c" \

C_SRCS += \
../Sources/FreeRTOS/portable/GCC/ARM_CM0/port.c \

OBJS += \
./Sources/FreeRTOS/portable/GCC/ARM_CM0/port.o \

C_DEPS += \
./Sources/FreeRTOS/portable/GCC/ARM_CM0/port.d \

OBJS_QUOTED += \
"./Sources/FreeRTOS/portable/GCC/ARM_CM0/port.o" \

C_DEPS_QUOTED += \
"./Sources/FreeRTOS/portable/GCC/ARM_CM0/port.d" \

OBJS_OS_FORMAT += \
./Sources/FreeRTOS/portable/GCC/ARM_CM0/port.o \


# Each subdirectory must supply rules for building sources it contributes
Sources/FreeRTOS/portable/GCC/ARM_CM0/port.o: ../Sources/FreeRTOS/portable/GCC/ARM_CM0/port.c
	@echo 'Building file: $<'
	@echo 'Executing target #20 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/FreeRTOS/portable/GCC/ARM_CM0/port.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/FreeRTOS/portable/GCC/ARM_CM0/port.o"
	@echo 'Finished building: $<'
	@echo ' '


