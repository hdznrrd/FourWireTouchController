AVR_CHIP = atmega32u4
AVR_CHIP_FREQUENCY = 16000000
CODE_MAIN = main

AVR_PROGRAMMER = dfu-programmer
AVR_COMPILER = avr-gcc
AVR_HEX = avr-objcopy

all: $(CODE_MAIN).hex
	
%.o: %.c
	$(AVR_COMPILER) -DF_CPU=$(AVR_CHIP_FREQUENCY) -Os -mmcu=$(AVR_CHIP) -o $@ $<
	
%.hex: %.o
	$(AVR_HEX) -O ihex $< $@
	
flash: $(CODE_MAIN).hex
	sudo $(AVR_PROGRAMMER) $(AVR_CHIP) erase
	sudo $(AVR_PROGRAMMER) $(AVR_CHIP) flash $<
	sudo $(AVR_PROGRAMMER) $(AVR_CHIP) start ; true

clean:
	rm -f $(CODE_MAIN).hex $(CODE_MAIN).o

.PHONY: flash clean
