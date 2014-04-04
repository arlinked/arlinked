include $(ARLINKED_INS)/mk/config.mk

PROGRAM = client
SOURCES = \
	$(ARDUINO_LIB_PATH)/SPI/SPI.cpp \
	../../common/can/mcp2515_can.cpp \
	arlinked_main.ino \
	arlinked_client.cpp \

INCLUDE_PATHS = . $(ARDUINO_LIB_PATH)/SPI

include $(ARLINKED_INS)/mk/make.mk
