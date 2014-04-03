include $(ARLINKED_INS)/mk/config.mk

PROGRAM = client
SOURCES = \
	../../common/can/mcp2515/mcp2515_can.cpp \
	client.ino \

LIBRARIES = \
	$(ARDUINO_LIB_PATH)/SPI/SPI.cpp \

INCLUDE_PATHS = . $(ARDUINO_LIB_PATH)/SPI

include $(ARLINKED_INS)/mk/make.mk
