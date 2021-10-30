# Name of application to build
TARGET := gltf64

DEBUG ?= 0

PLATFORM := native

### Text variables ###

# These use the fact that += always adds a space to create a variable that is just a space
# Space has a single space, indent has 2
space :=
space +=

indent =
indent += 
indent += 

### Tools ###

# System tools
CD := cd
CP := cp
RM := rm

MKDIR := mkdir
MKDIR_OPTS := -p

RMDIR := rm
RMDIR_OPTS := -rf

PRINT := printf '
ENDCOLOR := \033[0m
WHITE     := \033[0m
ENDWHITE  := $(ENDCOLOR)
GREEN     := \033[0;32m
ENDGREEN  := $(ENDCOLOR)
BLUE      := \033[0;34m
ENDBLUE   := $(ENDCOLOR)
YELLOW    := \033[0;33m
ENDYELLOW := $(ENDCOLOR)
ENDLINE := \n'

RUN := 

SUFFIX := -9

# Build tools
CC      := gcc$(SUFFIX)
AS      := as
CPP     := cpp$(SUFFIX)
CXX     := g++$(SUFFIX)
LD      := g++$(SUFFIX)
OBJCOPY := objcopy

### Files and Directories ###

# Source files
SRC_DIRS     := src lib/gltf lib/meshoptimizer/src lib/glm
C_SRCS       := $(foreach src_dir,$(SRC_DIRS),$(wildcard $(src_dir)/*.c))
CXX_SRCS     := $(foreach src_dir,$(SRC_DIRS),$(wildcard $(src_dir)/*.cpp)) $(foreach src_dir,$(SRC_DIRS),$(wildcard $(src_dir)/*.cc))
ASM_SRCS     := $(foreach src_dir,$(SRC_DIRS),$(wildcard $(src_dir)/*.s))
BIN_FILES    := $(foreach src_dir,$(SRC_DIRS),$(wildcard $(src_dir)/*.bin))

SRC_DIRS     += lib/fmt/src
CXX_SRCS     += lib/fmt/src/format.cc
# Build folders
ifeq ($(DEBUG),0)
BUILD_ROOT     := build/$(PLATFORM)/release
else
BUILD_ROOT     := build/$(PLATFORM)/debug
endif
BUILD_DIRS     := $(addprefix $(BUILD_ROOT)/,$(SRC_DIRS))

# Linked libraries
LIBS           :=
LIBS_INC_DIRS  := lib/fmt/include lib/tinygltf lib/meshoptimizer/src lib/glm lib/span/include
LIBS_INC_FLAGS := $(addprefix -I,$(LIBS_INC_DIRS))
LIBS_LD_DIRS   := 
LIBS_LD_FLAGS  := $(addprefix -L,$(LIBS_LD_DIRS)) $(addprefix -l,$(LIBS))

# Build files
C_OBJS   := $(addprefix $(BUILD_ROOT)/,$(C_SRCS:.c=.o))
CXX_OBJS := $(addprefix $(BUILD_ROOT)/,$(CXX_SRCS:.cpp=.o))
CXX_OBJS := $(CXX_OBJS:.cc=.o)
ASM_OBJS := $(addprefix $(BUILD_ROOT)/,$(ASM_SRCS:.s=.o))
BIN_OBJS := $(addprefix $(BUILD_ROOT)/,$(BIN_FILES:.bin=.o))
OBJS     := $(C_OBJS) $(CXX_OBJS) $(ASM_OBJS) $(BIN_OBJS)
SEG_OBJS := $(addprefix $(BUILD_ROOT)/,$(SEG_C_SRCS:.c=.o)) $(addprefix $(BUILD_ROOT)/,$(SEG_CPP_SRCS:.cpp=.o))
D_FILES  := $(C_OBJS:.o=.d) $(CXX_OBJS:.o=.d) $(LD_CPP).d $(SEG_OBJS:.o=.d)

APP      := $(TARGET)

### Flags ###

# Build tool flags

CFLAGS     := -fdata-sections -ffunction-sections
CXXFLAGS   := -std=c++2a -fno-rtti -fdata-sections -ffunction-sections
CPPFLAGS   := -I include $(LIBS_INC_FLAGS) -DAPP_NAME=\"$(TARGET)\" -DTINYGLTF_USE_CPP14
WARNFLAGS  := -Wall -Wextra -Wpedantic -Wdouble-promotion -Wfloat-conversion
ASFLAGS    := 
LDFLAGS    := -Wl,-gc-sections $(LIBS_LD_FLAGS)

ifneq ($(DEBUG),0)
CPPFLAGS   += -DDEBUG_MODE
OPT_FLAGS  := -O0 -g -ggdb
else
CPPFLAGS   += -DNDEBUG
OPT_FLAGS  := -O3 -flto
LDFLAGS    += -flto
# LDFLAGS    += -s
endif

### Rules ###

# Default target, all
all: $(APP)

# Make directories
$(BUILD_ROOT) $(BUILD_DIRS) :
	@$(PRINT)$(GREEN)Creating directory: $(ENDGREEN)$(BLUE)$@$(ENDBLUE)$(ENDLINE)
	@$(MKDIR) $@ $(MKDIR_OPTS)

# .cpp -> .o
$(BUILD_ROOT)/%.o : %.cpp | $(BUILD_DIRS)
	@$(PRINT)$(GREEN)Compiling C++ source file: $(ENDGREEN)$(BLUE)$<$(ENDBLUE)$(ENDLINE)
	@$(CXX) $< -o $@ -c -MMD -MF $(@:.o=.d) $(CXXFLAGS) $(CPPFLAGS) $(OPT_FLAGS) $(WARNFLAGS)
	
# .cc -> .o
$(BUILD_ROOT)/%.o : %.cc | $(BUILD_DIRS)
	@$(PRINT)$(GREEN)Compiling C++ source file: $(ENDGREEN)$(BLUE)$<$(ENDBLUE)$(ENDLINE)
	@$(CXX) $< -o $@ -c -MMD -MF $(@:.o=.d) $(CXXFLAGS) $(CPPFLAGS) $(OPT_FLAGS) $(WARNFLAGS)

# .c -> .o
$(BUILD_ROOT)/%.o : %.c | $(BUILD_DIRS)
	@$(PRINT)$(GREEN)Compiling C source file: $(ENDGREEN)$(BLUE)$<$(ENDBLUE)$(ENDLINE)
	@$(CC) $< -o $@ -c -MMD -MF $(@:.o=.d) $(CFLAGS) $(CPPFLAGS) $(OPT_FLAGS) $(WARNFLAGS)

# .bin -> .o
$(BUILD_ROOT)/%.o : %.bin | $(BUILD_DIRS)
	@$(PRINT)$(GREEN)Objcopying binary file: $(ENDGREEN)$(BLUE)$<$(ENDBLUE)$(ENDLINE)
	@$(OBJCOPY) -I binary -O elf32-big $< $@

# .s -> .o
$(BUILD_ROOT)/%.o : %.s | $(BUILD_DIRS)
	@$(PRINT)$(GREEN)Compiling ASM source file: $(ENDGREEN)$(BLUE)$<$(ENDBLUE)$(ENDLINE)
	@$(AS) $< -o $@ $(ASFLAGS)

# .o -> application
$(APP) : $(OBJS) $(SEG_OBJS)
	@$(PRINT)$(GREEN)Linking application: $(ENDGREEN)$(BLUE)$@$(ENDBLUE)$(ENDLINE)
	@$(LD) -o $@ $^ $(LDFLAGS)
	@$(PRINT)$(WHITE)Application Built!$(ENDWHITE)$(ENDLINE)

clean:
	@$(PRINT)$(YELLOW)Cleaning build$(ENDYELLOW)$(ENDLINE)
	@$(RMDIR) $(BUILD_ROOT) $(RMDIR_OPTS)
	@$(RM) -f $(APP)

run: $(APP)
	@$(PRINT)$(GREEN)Running $(APP)$(ENDGREEN)$(ENDLINE)
	@$(RUN) ./$(APP) -d

.PHONY: all clean load

-include $(D_FILES)

print-% : ; $(info $* is a $(flavor $*) variable set to [$($*)]) @true
