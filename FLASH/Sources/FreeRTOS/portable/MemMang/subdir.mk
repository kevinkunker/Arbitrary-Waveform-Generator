################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/FreeRTOS/portable/MemMang/heap_1.c" \

C_SRCS += \
../Sources/FreeRTOS/portable/MemMang/heap_1.c \

OBJS += \
./Sources/FreeRTOS/portable/MemMang/heap_1.o \

C_DEPS += \
./Sources/FreeRTOS/portable/MemMang/heap_1.d \

OBJS_QUOTED += \
"./Sources/FreeRTOS/portable/MemMang/heap_1.o" \

C_DEPS_QUOTED += \
"./Sources/FreeRTOS/portable/MemMang/heap_1.d" \

OBJS_OS_FORMAT += \
./Sources/FreeRTOS/portable/MemMang/heap_1.o \


# Each subdirectory must supply rules for building sources it contributes
Sources/FreeRTOS/portable/MemMang/heap_1.o: ../Sources/FreeRTOS/portable/MemMang/heap_1.c
	@echo 'Building file: $<'
	@echo 'Executing target #19 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/FreeRTOS/portable/MemMang/heap_1.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/FreeRTOS/portable/MemMang/heap_1.o"
	@echo 'Finished building: $<'
	@echo ' '


