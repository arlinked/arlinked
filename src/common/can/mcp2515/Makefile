include $(ARLINKED_INS)/mk/config.mk

LIBRARY = arlinked-can

SOURCES = \
	mcp2515_can.cpp \
	$(ARDUINO_LIB_PATH)/SPI/SPI.cpp

INCLUDE_PATHS = \
	$(ARDUINO_LIB_PATH)/SPI/

HEADERS_INSTALL_CMD = \
	mkdir -p $(ARLINKED_INS)/include/common/can; \
	cp -rpf mcp2515_can.h mcp2515_define.h $(ARLINKED_INS)/include/common/can; \

include $(ARLINKED_INS)/mk/make.mk
