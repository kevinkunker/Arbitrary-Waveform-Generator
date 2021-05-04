################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/FreeRTOS_ADC_Experiment_main_Kevin_Kunker.c" \
"../Sources/KL25Z_NVIC.c" \
"../Sources/KL25Z_gpio.c" \
"../Sources/KL25Z_pit.c" \
"../Sources/KL25Z_port.c" \
"../Sources/OpenSDA_UART.c" \
"../Sources/Shield_LEDs.c" \
"../Sources/Shield_pushbuttons.c" \
"../Sources/analog.c" \
"../Sources/awg_main.c" \
"../Sources/mcg.c" \
"../Sources/sa_mtb.c" \

C_SRCS += \
../Sources/FreeRTOS_ADC_Experiment_main_Kevin_Kunker.c \
../Sources/KL25Z_NVIC.c \
../Sources/KL25Z_gpio.c \
../Sources/KL25Z_pit.c \
../Sources/KL25Z_port.c \
../Sources/OpenSDA_UART.c \
../Sources/Shield_LEDs.c \
../Sources/Shield_pushbuttons.c \
../Sources/analog.c \
../Sources/awg_main.c \
../Sources/mcg.c \
../Sources/sa_mtb.c \

OBJS += \
./Sources/FreeRTOS_ADC_Experiment_main_Kevin_Kunker.o \
./Sources/KL25Z_NVIC.o \
./Sources/KL25Z_gpio.o \
./Sources/KL25Z_pit.o \
./Sources/KL25Z_port.o \
./Sources/OpenSDA_UART.o \
./Sources/Shield_LEDs.o \
./Sources/Shield_pushbuttons.o \
./Sources/analog.o \
./Sources/awg_main.o \
./Sources/mcg.o \
./Sources/sa_mtb.o \

C_DEPS += \
./Sources/FreeRTOS_ADC_Experiment_main_Kevin_Kunker.d \
./Sources/KL25Z_NVIC.d \
./Sources/KL25Z_gpio.d \
./Sources/KL25Z_pit.d \
./Sources/KL25Z_port.d \
./Sources/OpenSDA_UART.d \
./Sources/Shield_LEDs.d \
./Sources/Shield_pushbuttons.d \
./Sources/analog.d \
./Sources/awg_main.d \
./Sources/mcg.d \
./Sources/sa_mtb.d \

OBJS_QUOTED += \
"./Sources/FreeRTOS_ADC_Experiment_main_Kevin_Kunker.o" \
"./Sources/KL25Z_NVIC.o" \
"./Sources/KL25Z_gpio.o" \
"./Sources/KL25Z_pit.o" \
"./Sources/KL25Z_port.o" \
"./Sources/OpenSDA_UART.o" \
"./Sources/Shield_LEDs.o" \
"./Sources/Shield_pushbuttons.o" \
"./Sources/analog.o" \
"./Sources/awg_main.o" \
"./Sources/mcg.o" \
"./Sources/sa_mtb.o" \

C_DEPS_QUOTED += \
"./Sources/FreeRTOS_ADC_Experiment_main_Kevin_Kunker.d" \
"./Sources/KL25Z_NVIC.d" \
"./Sources/KL25Z_gpio.d" \
"./Sources/KL25Z_pit.d" \
"./Sources/KL25Z_port.d" \
"./Sources/OpenSDA_UART.d" \
"./Sources/Shield_LEDs.d" \
"./Sources/Shield_pushbuttons.d" \
"./Sources/analog.d" \
"./Sources/awg_main.d" \
"./Sources/mcg.d" \
"./Sources/sa_mtb.d" \

OBJS_OS_FORMAT += \
./Sources/FreeRTOS_ADC_Experiment_main_Kevin_Kunker.o \
./Sources/KL25Z_NVIC.o \
./Sources/KL25Z_gpio.o \
./Sources/KL25Z_pit.o \
./Sources/KL25Z_port.o \
./Sources/OpenSDA_UART.o \
./Sources/Shield_LEDs.o \
./Sources/Shield_pushbuttons.o \
./Sources/analog.o \
./Sources/awg_main.o \
./Sources/mcg.o \
./Sources/sa_mtb.o \


# Each subdirectory must supply rules for building sources it contributes
Sources/FreeRTOS_ADC_Experiment_main_Kevin_Kunker.o: ../Sources/FreeRTOS_ADC_Experiment_main_Kevin_Kunker.c
	@echo 'Building file: $<'
	@echo 'Executing target #1 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/FreeRTOS_ADC_Experiment_main_Kevin_Kunker.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/FreeRTOS_ADC_Experiment_main_Kevin_Kunker.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/KL25Z_NVIC.o: ../Sources/KL25Z_NVIC.c
	@echo 'Building file: $<'
	@echo 'Executing target #2 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/KL25Z_NVIC.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/KL25Z_NVIC.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/KL25Z_gpio.o: ../Sources/KL25Z_gpio.c
	@echo 'Building file: $<'
	@echo 'Executing target #3 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/KL25Z_gpio.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/KL25Z_gpio.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/KL25Z_pit.o: ../Sources/KL25Z_pit.c
	@echo 'Building file: $<'
	@echo 'Executing target #4 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/KL25Z_pit.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/KL25Z_pit.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/KL25Z_port.o: ../Sources/KL25Z_port.c
	@echo 'Building file: $<'
	@echo 'Executing target #5 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/KL25Z_port.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/KL25Z_port.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/OpenSDA_UART.o: ../Sources/OpenSDA_UART.c
	@echo 'Building file: $<'
	@echo 'Executing target #6 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/OpenSDA_UART.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/OpenSDA_UART.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/Shield_LEDs.o: ../Sources/Shield_LEDs.c
	@echo 'Building file: $<'
	@echo 'Executing target #7 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/Shield_LEDs.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/Shield_LEDs.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/Shield_pushbuttons.o: ../Sources/Shield_pushbuttons.c
	@echo 'Building file: $<'
	@echo 'Executing target #8 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/Shield_pushbuttons.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/Shield_pushbuttons.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/analog.o: ../Sources/analog.c
	@echo 'Building file: $<'
	@echo 'Executing target #9 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/analog.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/analog.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/awg_main.o: ../Sources/awg_main.c
	@echo 'Building file: $<'
	@echo 'Executing target #10 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/awg_main.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/awg_main.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/mcg.o: ../Sources/mcg.c
	@echo 'Building file: $<'
	@echo 'Executing target #11 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/mcg.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/mcg.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/sa_mtb.o: ../Sources/sa_mtb.c
	@echo 'Building file: $<'
	@echo 'Executing target #12 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/sa_mtb.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/sa_mtb.o"
	@echo 'Finished building: $<'
	@echo ' '


