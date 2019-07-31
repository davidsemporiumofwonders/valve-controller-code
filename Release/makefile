################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
solarboiler_controller.lss \

FLASH_IMAGE += \
solarboiler_controller.hex \

EEPROM_IMAGE += \
solarboiler_controller.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: solarboiler_controller.elf secondary-outputs

# Tool invocations
solarboiler_controller.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,solarboiler_controller.map -mmcu=atmega88 -o "solarboiler_controller.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

solarboiler_controller.lss: solarboiler_controller.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S solarboiler_controller.elf  >"solarboiler_controller.lss"
	@echo 'Finished building: $@'
	@echo ' '

solarboiler_controller.hex: solarboiler_controller.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex solarboiler_controller.elf  "solarboiler_controller.hex"
	@echo 'Finished building: $@'
	@echo ' '

solarboiler_controller.eep: solarboiler_controller.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex solarboiler_controller.elf  "solarboiler_controller.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: solarboiler_controller.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega88 solarboiler_controller.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) solarboiler_controller.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents

-include ../makefile.targets
