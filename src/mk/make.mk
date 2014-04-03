
.PHONY: all clean upload target_with_dir_holder ins

TOTAL_CORE_SOURCES += $(DEFAULT_CORE_SOURCES)
TOTAL_USER_SOURCES += $(SOURCES)

objects_tmp_1_1 = $(TOTAL_CORE_SOURCES:.c=.core.o)
objects_tmp_1_2 = $(objects_tmp_1_1:.cpp=.core.o)
objects_tmp_1_3 = $(objects_tmp_1_2:.ino=.core.o)
TOTAL_CORE_OBJECTS = $(addprefix $(DPS)/, $(notdir $(objects_tmp_1_3)))

objects_tmp_3_1 = $(TOTAL_USER_SOURCES:.c=.user.o)
objects_tmp_3_2 = $(objects_tmp_3_1:.cpp=.user.o)
objects_tmp_3_3 = $(objects_tmp_3_2:.ino=.user.o)
TOTAL_USER_OBJECTS = $(addprefix $(DPS)/, $(notdir $(objects_tmp_3_3)))

vpath %.c . $(dir $(TOTAL_USER_SOURCES)) $(dir $(TOTAL_CORE_SOURCES))
vpath %.cpp . $(dir $(TOTAL_USER_SOURCES)) $(dir $(TOTAL_CORE_SOURCES))
vpath %.ino . $(dir $(TOTAL_USER_SOURCES)) $(dir $(TOTAL_CORE_SOURCES))

TOTAL_INCLUDE_PATHS = $(DEFAULT_INCLUDE_PATHS) $(INCLUDE_PATHS)
CORE_INC_I = $(addprefix -I, $(DEFAULT_INCLUDE_PATHS))
USER_INC_I = $(addprefix -I, $(INCLUDE_PATHS))
TOTAL_INC_I = $(CORE_INC_I) $(USER_INC_I)

TOTAL_COMPILER_FLAGS = $(DEFAULT_COMPILER_FLAGS) $(COMPILER_FLAGS) 
TOTAL_LINKER_FLAGS_ELF = $(DEFAULT_LINKER_FLAGS) $(LINKER_FLAGS)
TOTAL_LINKER_FLAGS = $(DEFAULT_LINKER_FLAGS) $(LINKER_FLAGS)

CORE_AR = $(DPS)/core.a
PROGRAM_ELF = $(DPS)/$(PROGRAM).elf
PROGRAM_HEX = $(DPS)/$(PROGRAM).hex

LIBRARY_AR = $(DPS)/lib$(LIBRARY).a

ifneq "$(PROGRAM)" ""
DEST = $(PROGRAM_HEX)
DEST_INS = $(ARLINKED_INS)/hex/$(notdir $(PROGRAM_HEX))
else
ifneq "$(LIBRARY)" ""
DEST = $(LIBRARY_AR)
DEST_INS = $(ARLINKED_INS)/lib/$(notdir $(LIBRARY_AR))
else
all:
	@echo "missing target"
endif
endif

all: target_with_dir_holder

target_with_dir_holder: $(DPS_DIR_HOLDER) $(DEST)

ifneq "$(PROGRAM)" ""
$(PROGRAM_HEX): $(PROGRAM_ELF)
	$(OBJCOPY) -O ihex -R .eeprom $(PROGRAM_ELF) $(PROGRAM_HEX)

$(PROGRAM_ELF): $(CORE_AR) $(TOTAL_USER_OBJECTS)
	$(LINKER) $(TOTAL_LINKER_FLAGS_ELF) $(TOTAL_USER_OBJECTS) $(CORE_AR) -L$(DPS) -o $(PROGRAM_ELF)
endif

ifneq "$(LIBRARY)" ""
$(LIBRARY_AR): $(CORE_AR) $(TOTAL_USER_OBJECTS)
	$(ARCHIVER) rcs $(LIBRARY_AR) $(TOTAL_USER_OBJECTS) $(CORE_AR)
endif

$(CORE_AR): $(TOTAL_CORE_OBJECTS)
	$(ARCHIVER) rcs $(CORE_AR) $(TOTAL_CORE_OBJECTS)

$(DPS)/%.core.o: %.c
	$(COMPILER_C) $(TOTAL_COMPILER_FLAGS) $(CORE_INC_I) -c $< -o $@
$(DPS)/%.core.o: %.cpp
	$(COMPILER_CXX) $(TOTAL_COMPILER_FLAGS) $(CORE_INC_I) -c $< -o $@
$(DPS)/%.core.o: $(DPS)/%.core.cpp
	$(COMPILER_CXX) $(TOTAL_COMPILER_FLAGS) $(CORE_INC_I) -c $< -o $@

$(DPS)/%.user.o: %.c
	$(COMPILER_C) $(TOTAL_COMPILER_FLAGS) $(TOTAL_INC_I) -c $< -o $@
$(DPS)/%.user.o: %.cpp
	$(COMPILER_CXX) $(TOTAL_COMPILER_FLAGS) $(TOTAL_INC_I) -c $< -o $@
$(DPS)/%.user.o: $(DPS)/%.user.cpp
	$(COMPILER_CXX) $(TOTAL_COMPILER_FLAGS) $(TOTAL_INC_I) -c $< -o $@

$(DPS)/%.user.cpp $(DPS_CORE)/%.core.cpp: %.ino
	cp -pf $< $@

ifneq "$(DEST_INS)" ""
install: all
	cp -rpf $(DEST) $(DEST_INS)
else
install: all
	$(error no place to install, DEST_INS is empty)
endif

ifneq "$(HEADERS_INSTALL_CMD)" ""
install_headers:
	$(HEADERS_INSTALL_CMD)
endif

ifneq "$(PROGRAM)" ""
upload_%: $(PROGRAM_HEX)
	$(UPLOADER) -v -p$(UPLOAD_DEVICE) -P$(subst upload_,,$@) -c arduino -b57600 -D -C$(UPLOADER_CFG) -U flash:w:$(PROGRAM_HEX):i
upload: $(PROGRAM_HEX)
	$(UPLOADER) -v -p$(UPLOAD_DEVICE) -P$(UPLOAD_PORT) -c arduino -b57600 -D -C$(UPLOADER_CFG) -U flash:w:$(PROGRAM_HEX):i
else
upload%:
	$(error PROGRAM is empty, no target to upload)
endif
	
%_dir_holder_:
	@mkdir -p $(dir $@)
	@touch $@

clean:
	rm -rf $(DPS)

