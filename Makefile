####################################################################################################
#
# mightyohm.com Geiger Counter firmware Makefile
#
# Copyright (c) 2011 Jeff Keyzer, MightyOhm Engineering, https://mightyohm.com/, jeff at mightyohm dot com
# Copyright (c) 2019 Philippe Kehl, flipflip industries, https://oinkzwurgl.org/projaeggd/geiger, flipflip at oinkzwurgl dot org
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <http://www.gnu.org/licenses/>.
#
####################################################################################################

PROGRAM		= geiger
DEVICE		= attiny2313
CLOCK		= 8000000
PROGRAMMER	= usbtiny
PORT		= usb

# Fuse configuration:
# For a really nice guide to AVR fuses, see http://www.engbedded.com/fusecalc/
# LFUSE: SUT0, CKSEL0 (Ext Xtal 8+Mhz, 0ms startup time)
LFUSE		= 0xEE
# HFUSE: SPIEN, BODLEVEL0 (Serial programming enabled, Brownout = 1.8V
HFUSE		= 0xDD
# EFUSE: no fuses programmed
EFUSE		= 0xFF

# source files
SOURCES    := $(PROGRAM).c

####################################################################################################

AVRDUDE = avrdude -c $(PROGRAMMER) -P $(PORT) -p $(DEVICE)
COMPILE = avr-gcc -g -Wall -Wextra -std=gnu99 -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)
LDFLAGS	= -Wl,-Map=$(PROGRAM).map -Wl,--cref
SIZE	= avr-size -C --mcu=$(DEVICE)

####################################################################################################

.PHONY:	all
all: $(PROGRAM).hex $(PROGRAM).elf $(PROGRAM).lst $(PROGRAM).size

####################################################################################################

$(PROGRAM).hex: $(PROGRAM).elf

$(PROGRAM).elf: $(SOURCES) Makefile
	$(COMPILE) -o $@ $(SOURCES) $(LDFLAGS)

$(PROGRAM).lst: $(PROGRAM).elf
	avr-objdump -h -S $< > $@

$(PROGRAM).hex: $(PROGRAM).elf
	avr-objcopy -j .text -j .data -O ihex $< $@

$(PROGRAM).size: $(PROGRAM).elf
	$(SIZE) $(PROGRAM).elf | tee $@

####################################################################################################

.PHONY:	flash
flash: $(PROGRAM).hex
	$(AVRDUDE) -U flash:w:$<:i

.PHONY:	fuse
fuse:
	$(AVRDUDE) -U hfuse:w:$(HFUSE):m -U lfuse:w:$(LFUSE):m -U efuse:w:$(EFUSE):m

.PHONY:	install
install: flash fuse

.PHONY:	clean
clean:
	rm -f $(PROGRAM).hex $(PROGRAM).elf $(PROGRAM).lst $(PROGRAM).map

####################################################################################################

