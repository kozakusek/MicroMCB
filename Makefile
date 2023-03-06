CC 			= /opt/arm/bin/arm-eabi-gcc
OBJCOPY 	= /opt/arm/bin/arm-eabi-objcopy
FLAGS 		= -mthumb -mcpu=cortex-m4
CPPFLAGS 	= -DSTM32F411xE
CFLAGS 		= $(FLAGS) -Wall -g \
			-O2 -ffunction-sections -fdata-sections \
			-I/opt/arm/stm32/inc \
			-I/opt/arm/stm32/CMSIS/Include \
			-I/opt/arm/stm32/CMSIS/Device/ST/STM32F4xx/Include
LDFLAGS 	= $(FLAGS) -Wl,--gc-sections -nostartfiles \
			-L/opt/arm/stm32/lds -Tstm32f411re.lds \
			-specs=nosys.specs

vpath %.c /opt/arm/stm32/src

OBJECTS 	= main.o startup_stm32.o delay.o gpio.o i2c_conf.o accelerometer.o usart.o
TARGET 		= prog

.SECONDARY: $(TARGET).elf $(OBJECTS)

all: $(TARGET).bin

accelerometer.o : accelerometer.h accelerometer.c
i2c_conf.o : i2c_conf.h i2c_conf.c
usart.o : usart.h usart.c

%.elf : $(OBJECTS)
	$(CC) $(LDFLAGS) $^ -o $@

%.bin : %.elf
	$(OBJCOPY) $< $@ -O binary

clean :
	rm -f *.bin *.elf *.hex *.d *.o *.bak *~