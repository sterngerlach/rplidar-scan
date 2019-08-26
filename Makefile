
# Makefile

RPLIDAR_SDK_DIR = ~/rplidar_sdk-release-v1.11.0/sdk
RPLIDAR_SDK_INC_DIR = $(RPLIDAR_SDK_DIR)/sdk/include
RPLIDAR_SDK_SRC_DIR = $(RPLIDAR_SDK_DIR)/sdk/src
RPLIDAR_SDK_LIB_DIR = $(RPLIDAR_SDK_DIR)/output/Linux/Release

CXX = g++
CFLAGS = -Os -Wall -Wextra -std=c++1z \
	-I $(RPLIDAR_SDK_INC_DIR) \
	-I $(RPLIDAR_SDK_SRC_DIR)
LDFLAGS = -lm -lpthread -L $(RPLIDAR_SDK_LIB_DIR) -lrplidar_sdk

OBJDIR = ./obj
OUTDIR = ./bin

TARGET = $(OUTDIR)/rplidar_scan
SOURCES = $(notdir $(wildcard ./*.cpp))
OBJECTS = $(addprefix $(OBJDIR)/, $(notdir $(SOURCES:.cpp=.o)))
DEPENDS = $(OBJECTS:.o=.d)

default: $(TARGET)

debug: CFLAGS += -ggdb
debug: $(TARGET)

$(TARGET) : $(OBJECTS)
	mkdir -p $(OUTDIR)
	$(CXX) $(CFLAGS) $^ $(LDFLAGS) -o $@
	# $(CXX) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(OBJDIR)/%.o : %.cpp
	mkdir -p $(OBJDIR)
	$(CXX) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJECTS) $(DEPENDS)

.PHONY: clean
.PHONY: debug

-include $(DEPENDS)

