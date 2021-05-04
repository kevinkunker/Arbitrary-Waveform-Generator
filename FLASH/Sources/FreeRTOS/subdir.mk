################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/FreeRTOS/croutine.c" \
"../Sources/FreeRTOS/event_groups.c" \
"../Sources/FreeRTOS/list.c" \
"../Sources/FreeRTOS/queue.c" \
"../Sources/FreeRTOS/stream_buffer.c" \
"../Sources/FreeRTOS/tasks.c" \
"../Sources/FreeRTOS/timers.c" \

C_SRCS += \
../Sources/FreeRTOS/croutine.c \
../Sources/FreeRTOS/event_groups.c \
../Sources/FreeRTOS/list.c \
../Sources/FreeRTOS/queue.c \
../Sources/FreeRTOS/stream_buffer.c \
../Sources/FreeRTOS/tasks.c \
../Sources/FreeRTOS/timers.c \

OBJS += \
./Sources/FreeRTOS/croutine.o \
./Sources/FreeRTOS/event_groups.o \
./Sources/FreeRTOS/list.o \
./Sources/FreeRTOS/queue.o \
./Sources/FreeRTOS/stream_buffer.o \
./Sources/FreeRTOS/tasks.o \
./Sources/FreeRTOS/timers.o \

C_DEPS += \
./Sources/FreeRTOS/croutine.d \
./Sources/FreeRTOS/event_groups.d \
./Sources/FreeRTOS/list.d \
./Sources/FreeRTOS/queue.d \
./Sources/FreeRTOS/stream_buffer.d \
./Sources/FreeRTOS/tasks.d \
./Sources/FreeRTOS/timers.d \

OBJS_QUOTED += \
"./Sources/FreeRTOS/croutine.o" \
"./Sources/FreeRTOS/event_groups.o" \
"./Sources/FreeRTOS/list.o" \
"./Sources/FreeRTOS/queue.o" \
"./Sources/FreeRTOS/stream_buffer.o" \
"./Sources/FreeRTOS/tasks.o" \
"./Sources/FreeRTOS/timers.o" \

C_DEPS_QUOTED += \
"./Sources/FreeRTOS/croutine.d" \
"./Sources/FreeRTOS/event_groups.d" \
"./Sources/FreeRTOS/list.d" \
"./Sources/FreeRTOS/queue.d" \
"./Sources/FreeRTOS/stream_buffer.d" \
"./Sources/FreeRTOS/tasks.d" \
"./Sources/FreeRTOS/timers.d" \

OBJS_OS_FORMAT += \
./Sources/FreeRTOS/croutine.o \
./Sources/FreeRTOS/event_groups.o \
./Sources/FreeRTOS/list.o \
./Sources/FreeRTOS/queue.o \
./Sources/FreeRTOS/stream_buffer.o \
./Sources/FreeRTOS/tasks.o \
./Sources/FreeRTOS/timers.o \


# Each subdirectory must supply rules for building sources it contributes
Sources/FreeRTOS/croutine.o: ../Sources/FreeRTOS/croutine.c
	@echo 'Building file: $<'
	@echo 'Executing target #13 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/FreeRTOS/croutine.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/FreeRTOS/croutine.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/FreeRTOS/event_groups.o: ../Sources/FreeRTOS/event_groups.c
	@echo 'Building file: $<'
	@echo 'Executing target #14 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/FreeRTOS/event_groups.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/FreeRTOS/event_groups.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/FreeRTOS/list.o: ../Sources/FreeRTOS/list.c
	@echo 'Building file: $<'
	@echo 'Executing target #15 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/FreeRTOS/list.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/FreeRTOS/list.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/FreeRTOS/queue.o: ../Sources/FreeRTOS/queue.c
	@echo 'Building file: $<'
	@echo 'Executing target #16 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/FreeRTOS/queue.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/FreeRTOS/queue.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/FreeRTOS/stream_buffer.o: ../Sources/FreeRTOS/stream_buffer.c
	@echo 'Building file: $<'
	@echo 'Executing target #17 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/FreeRTOS/stream_buffer.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/FreeRTOS/stream_buffer.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/FreeRTOS/tasks.o: ../Sources/FreeRTOS/tasks.c
	@echo 'Building file: $<'
	@echo 'Executing target #18 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/FreeRTOS/tasks.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/FreeRTOS/tasks.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/FreeRTOS/timers.o: ../Sources/FreeRTOS/timers.c
	@echo 'Building file: $<'
	@echo 'Executing target #19 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/FreeRTOS/timers.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/FreeRTOS/timers.o"
	@echo 'Finished building: $<'
	@echo ' '


