program_NAME := mfc-cli-cm-dr2
program_C_SRCS := cm-dr2-cli.c ../../lib/extras.c
program_CXX_SRCS := $(wildcard *.cpp)
program_C_OBJS := ${program_C_SRCS:.c=.o}
program_CXX_OBJS := ${program_CXX_SRCS:.cpp=.o}
program_OBJS := $(program_C_OBJS) $(program_CXX_OBJS)
program_INCLUDE_DIRS := ../../lib ../
program_LIBRARY_DIRS := ../../lib
program_LIBRARIES :=
program_INSTALL_DIR := /opt/mfc/clients/cm-dr2

CPPFLAGS += -Wall $(foreach includedir,$(program_INCLUDE_DIRS),-I$(includedir))
LDFLAGS += $(foreach librarydir,$(program_LIBRARY_DIRS),-L$(librarydir))
LDFLAGS += $(foreach library,$(program_LIBRARIES),-l$(library))

.PHONY: all clean distclean

all: $(program_NAME)

$(program_NAME): $(program_OBJS)
	$(LINK.cc) $(program_OBJS) -o $(program_NAME)
	mkdir -p $(program_INSTALL_DIR)
	cp ./race.sh $(program_INSTALL_DIR)
	cp $(program_NAME) $(program_INSTALL_DIR)

clean:
	@- $(RM) $(program_NAME)
	@- $(RM) $(program_OBJS)

distclean: clean
