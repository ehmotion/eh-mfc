program_NAME := mfc-xtract
program_C_SRCS := mfc-xtract.c ../lib/extras.c ../lib/ff_decode.c
program_CXX_SRCS := $(wildcard *.cpp)
program_C_OBJS := ${program_C_SRCS:.c=.o}
program_CXX_OBJS := ${program_CXX_SRCS:.cpp=.o}
program_OBJS := $(program_C_OBJS) $(program_CXX_OBJS)
program_INCLUDE_DIRS := ../lib ../usbxtract/include/
program_LIBRARY_DIRS := ../lib
program_LIBRARIES :=usb-1.0 rt

CPPFLAGS += $(foreach includedir,$(program_INCLUDE_DIRS),-I$(includedir)) -Wall
LDFLAGS += $(foreach librarydir,$(program_LIBRARY_DIRS),-L$(librarydir))
LDLIBS += $(foreach library,$(program_LIBRARIES),-l$(library))

.PHONY: all clean distclean

install: $(program_NAME)
	mkdir -p /opt/mfc/mfcxtract
	cp $(program_NAME) /opt/mfc/mfcxtract
	cp ./race.sh /opt/mfc/mfcxtract

all: $(program_NAME)

$(program_NAME): $(program_OBJS)
	$(LINK.cc) $(program_OBJS) $(LDLIBS) -o $(program_NAME)

clean:
	@- $(RM) $(program_NAME)
	@- $(RM) $(program_OBJS)

distclean: clean
