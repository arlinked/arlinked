include $(ARLINKED_INS)/mk/config.mk

PROGRAM = client
SOURCES = \
	$(ARDUINO_LIB_PATH)/SPI/SPI.cpp \
	../../common/can/mcp2515/mcp2515_can.cpp \
	client.ino \

INCLUDE_PATHS = . $(ARDUINO_LIB_PATH)/SPI

include $(ARLINKED_INS)/mk/make.mk
