
.PHONY: all clean upload

TOTAL_CORE_SOURCES += $(DEFAULT_CORE_SOURCES)
TOTAL_LIBRARY_SOURCES += $(LIBRARIES)
TOTAL_USER_SOURCES += $(SOURCES)

objects_tmp_1_1 = $(TOTAL_CORE_SOURCES:.c=.core.o)
objects_tmp_1_2 = $(objects_tmp_1_1:.cpp=.core.o)
objects_tmp_1_3 = $(objects_tmp_1_2:.ino=.core.o)
TOTAL_CORE_OBJECTS = $(addprefix $(DPS_CORE)/, $(notdir $(objects_tmp_1_3)))

objects_tmp_2_1 = $(TOTAL_LIBRARY_SOURCES:.c=.lib.o)
objects_tmp_2_2 = $(objects_tmp_2_1:.cpp=.lib.o)
objects_tmp_2_3 = $(objects_tmp_2_2:.ino=.lib.o)
TOTAL_LIB_OBJECTS = $(addprefix $(DPS_LIB)/, $(notdir $(objects_tmp_2_3)))

objects_tmp_3_1 = $(TOTAL_USER_SOURCES:.c=.user.o)
objects_tmp_3_2 = $(objects_tmp_3_1:.cpp=.user.o)
objects_tmp_3_3 = $(objects_tmp_3_2:.ino=.user.o)
TOTAL_USER_OBJECTS = $(addprefix $(DPS)/, $(notdir $(objects_tmp_3_3)))

vpath %.c . $(dir $(TOTAL_USER_SOURCES)) $(dir $(TOTAL_CORE_SOURCES)) $(dir $(TOTAL_LIBRARY_SOURCES))
vpath %.cpp . $(dir $(TOTAL_USER_SOURCES)) $(dir $(TOTAL_CORE_SOURCES)) $(dir $(TOTAL_LIBRARY_SOURCES))
vpath %.ino . $(dir $(TOTAL_USER_SOURCES)) $(dir $(TOTAL_CORE_SOURCES)) $(dir $(TOTAL_LIBRARY_SOURCES))

TOTAL_INCLUDE_PATHS = $(DEFAULT_INCLUDE_PATHS) $(INCLUDE_PATHS)
CORE_INC_I = $(addprefix -I, $(DEFAULT_INCLUDE_PATHS))
USER_INC_I = $(addprefix -I, $(INCLUDE_PATHS))
TOTAL_INC_I = $(CORE_INC_I) $(USER_INC_I)

TOTAL_COMPILER_FLAGS = $(DEFAULT_COMPILER_FLAGS) $(COMPILER_FLAGS) 
TOTAL_LINKER_FLAGS_ELF = $(DEFAULT_LINKER_FLAGS) $(LINKER_FLAGS)
TOTAL_LINKER_FLAGS = $(DEFAULT_LINKER_FLAGS) $(LINKER_FLAGS)

PROGRAM_CORE_AR = $(DPS)/$(PROGRAM).core.a
PROGRAM_ELF = $(DPS)/$(PROGRAM).elf
PROGRAM_HEX = $(DPS)/$(PROGRAM).hex

all: $(PROGRAM_HEX)

$(PROGRAM_HEX): $(DPS_DIR_HOLDER) $(PROGRAM_ELF)
	$(OBJCOPY) -O ihex -R .eeprom $(PROGRAM_ELF) $(PROGRAM_HEX)

$(PROGRAM_ELF): $(PROGRAM_CORE_AR) $(TOTAL_USER_OBJECTS) $(TOTAL_LIB_OBJECTS)
	$(LINKER) $(TOTAL_LINKER_FLAGS_ELF) $(TOTAL_LIB_OBJECTS) $(TOTAL_USER_OBJECTS) $(PROGRAM_CORE_AR) -L$(DPS) -o $(PROGRAM_ELF)

$(PROGRAM_CORE_AR): $(TOTAL_CORE_OBJECTS)
	$(ARCHIVER) rcs $(PROGRAM_CORE_AR) $(TOTAL_CORE_OBJECTS)

$(DPS_CORE)/%.core.o: %.c
	$(COMPILER_C) $(TOTAL_COMPILER_FLAGS) $(CORE_INC_I) -c $< -o $(DPS_CORE)/$(notdir $@)
$(DPS_CORE)/%.core.o: %.cpp
	$(COMPILER_CXX) $(TOTAL_COMPILER_FLAGS) $(CORE_INC_I) -c $< -o $(DPS_CORE)/$(notdir $@)
$(DPS_CORE)/%.core.o: $(DPS_CORE)/%.core.cpp
	$(COMPILER_CXX) $(TOTAL_COMPILER_FLAGS) $(CORE_INC_I) -c $< -o $(DPS_CORE)/$(notdir $@)

$(DPS_LIB)/%.lib.o: %.c
	$(COMPILER_C) $(TOTAL_COMPILER_FLAGS) $(CORE_INC_I) -c $< -o $(DPS_LIB)/$(notdir $@)
$(DPS_LIB)/%.lib.o: %.cpp
	$(COMPILER_CXX) $(TOTAL_COMPILER_FLAGS) $(CORE_INC_I) -c $< -o $(DPS_LIB)/$(notdir $@)
$(DPS_LIB)/%.lib.o: $(DPS_LIB)/%.lib.cpp
	$(COMPILER_CXX) $(TOTAL_COMPILER_FLAGS) $(CORE_INC_I) -c $< -o $(DPS_LIB)/$(notdir $@)

$(DPS)/%.user.o: %.c
	$(COMPILER_C) $(TOTAL_COMPILER_FLAGS) $(TOTAL_INC_I) -c $< -o $(DPS)/$(notdir $@)
$(DPS)/%.user.o: %.cpp
	$(COMPILER_CXX) $(TOTAL_COMPILER_FLAGS) $(TOTAL_INC_I) -c $< -o $(DPS)/$(notdir $@)
$(DPS)/%.user.o: $(DPS)/%.user.cpp
	$(COMPILER_CXX) $(TOTAL_COMPILER_FLAGS) $(TOTAL_INC_I) -c $< -o $(DPS)/$(notdir $@)

$(DPS)/%.user.cpp $(DPS_LIB)/%.lib.cpp $(DPS_CORE)/%.core.cpp: %.ino
	cp -pf $< $@

upload_%: $(PROGRAM_HEX)
	$(UPLOADER) -v -p$(UPLOAD_DEVICE) -P$(subst upload_,,$@) -c arduino -b57600 -D -C$(UPLOADER_CFG) -U flash:w:$(PROGRAM_HEX):i
upload: $(PROGRAM_HEX)
	$(UPLOADER) -v -p$(UPLOAD_DEVICE) -P$(UPLOAD_PORT) -c arduino -b57600 -D -C$(UPLOADER_CFG) -U flash:w:$(PROGRAM_HEX):i
	
%_dir_holder_:
	@mkdir -p $(dir $@)
	@touch $@

clean:
	rm -rf $(DPS)

