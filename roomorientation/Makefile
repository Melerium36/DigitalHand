# ===== MCU Settings =====
MCU     = atmega328p
F_CPU   = 16000000UL
BAUD    = 115200
PORT    = /dev/ttyACM0
PROGRAMMER = arduino

TARGET  = blink
SRC     = src/main.c src/TWI_interface/twi_interface.c src/Uart_api/Uart_api.c

# ===== Flags =====
CFLAGS  = -std=c11 -Os -Wall -Wextra -mmcu=$(MCU) -DF_CPU=$(F_CPU)
LDFLAGS = -mmcu=$(MCU)

# ===== Build Artifacts =====
ELF = $(TARGET).elf
HEX = $(TARGET).hex

all: $(HEX)

$(ELF): $(SRC)
	avr-gcc $(CFLAGS) $(LDFLAGS) -o $@ $^

$(HEX): $(ELF)
	avr-objcopy -O ihex -R .eeprom $< $@
	# avr-size --mcu=$(MCU) --format=avr $<

flash: $(HEX)
	avrdude -v -p $(MCU) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -D -U flash:w:$(HEX):i

clean:
	rm -f $(ELF) $(HEX)

.PHONY: all flash clean
