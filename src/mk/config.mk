include $(ARLINKED_INS)/mk/define.mk


COMPILER_CXX = $(ARDUINO_HOME)/hardware/tools/avr/bin/avr-g++
COMPILER_C = $(ARDUINO_HOME)/hardware/tools/avr/bin/avr-gcc
ARCHIVER = $(ARDUINO_HOME)/hardware/tools/avr/bin/avr-ar
LINKER = $(ARDUINO_HOME)/hardware/tools/avr/bin/avr-gcc
OBJCOPY = $(ARDUINO_HOME)/hardware/tools/avr/bin/avr-objcopy

DEFAULT_INCLUDE_PATHS = \
	$(ARDUINO_HOME)/hardware/arduino/avr/cores/arduino \
	$(ARDUINO_HOME)/hardware/arduino/avr/variants/standard \

DEFAULT_COMPILER_FLAGS = -g -Os -w -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=156 -DARDUINO_AVR_PRO -DARDUINO_ARCH_AVR -fno-exceptions
DEFAULT_LINKER_FLAGS = -Os -Wl,--gc-sections -mmcu=atmega328p -lm

DEFAULT_CORE_SOURCES = \
	malloc.c \
	realloc.c \
	hooks.c \
	WInterrupts.c \
	wiring.c \
	wiring_analog.c \
	wiring_digital.c \
	wiring_pulse.c \
	wiring_shift.c \
	CDC.cpp \
	HardwareSerial.cpp \
	HardwareSerial0.cpp \
	HardwareSerial1.cpp \
	HardwareSerial2.cpp \
	HardwareSerial3.cpp \
	HID.cpp \
	IPAddress.cpp \
	main.cpp \
	new.cpp \
	Print.cpp \
	Stream.cpp \
	Tone.cpp \
	USBCore.cpp \
	WMath.cpp \
	WString.cpp \

vpath %.c $(ARDUINO_HOME)/hardware/arduino/avr/cores/arduino/avr-libc
vpath %.c $(ARDUINO_HOME)/hardware/arduino/avr/cores/arduino
vpath %.cpp $(ARDUINO_HOME)/hardware/arduino/avr/cores/arduino

DEPENDENCY_DIR = dps
