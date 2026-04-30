TARGET = mags-logger
CC=g++
CFLAGS = -g -Og -ggdb -fomit-frame-pointer -I./ -I./src/ -I./lib/ -I./lib/loguru/
LIBS= -lpthread

#CFLAGS += -DPICOJSON_USE_INT64

SRC_DIR = src
BUILD_DIR=./obj
LIB_DIR = lib

SRCS = main.cpp\
    config.cpp\
	task_base.cpp

MAIN_DEPS = \
    $(SRC_DIR)/config.h\
    $(SRC_DIR)/task_base.h

MAIN_OBJS := $(SRCS:%.cpp=$(BUILD_DIR)/src-%.o)

MAGS_DIR = $(SRC_DIR)/mags-logger
MAGS_SRCS =\
    nmea_sentence_reader.cpp\
    mags_logger.cpp\
    mags_calculator.cpp
MAGS_DEPS = \
    $(MAGS_DIR)/nmea_sentence_reader.h\
    $(MAGS_DIR)/mags_logger.h\
    $(MAGS_DIR)/mags_calculator.h
MAGS_OBJS := $(MAGS_SRCS:%.cpp=$(BUILD_DIR)/mags-logger-%.o)


UTILS_DIR = $(SRC_DIR)/utils
UTILS_SRCS = \
    time_utils.cpp\
    parse_utils.cpp\
    CsvParser.cpp
UTILS_DEPS = \
    $(UTILS_DIR)/time_utils.h\
    $(UTILS_DIR)/math_utils.h\
    $(UTILS_DIR)/parse_utils.h\
    $(UTILS_DIR)/CsvParser.h
UTILS_OBJS := $(UTILS_SRCS:%.cpp=$(BUILD_DIR)/utils-%.o)

CON_DIR = $(SRC_DIR)/connection
CON_SRCS =\
    connection.cpp\
    tcp_connection.cpp\
    com_connection.cpp
CON_DEPS = \
    $(CON_DIR)/connection.h\
    $(CON_DIR)/tcp_connection.h\
    $(CON_DIR)/com_connection.h
CON_OBJS := $(CON_SRCS:%.cpp=$(BUILD_DIR)/connection-%.o)

MAV_DIR = $(SRC_DIR)/mav
MAV_SRCS = \
    mavlink_provider.cpp\
    mavlink_provider_messages.cpp
MAV_DEPS = \
    $(MAV_DIR)/flight_mode.h\
    $(MAV_DIR)/mavlink_provider.h
MAV_OBJS := $(MAV_SRCS:%.cpp=$(BUILD_DIR)/mav-%.o)

LIB_LOGURU_SRCS := $(shell find $(LIB_DIR)/loguru -name '*.cpp' -or -name '*.c' -or -name '*.s')
LIB_LOGURU_OBJS := $(LIB_LOGURU_SRCS:$(LIB_DIR)/loguru/%.cpp=$(BUILD_DIR)/lib-loguru-%.o)

OBJS = $(MAIN_OBJS) $(MAGS_OBJS) $(MAV_OBJS) $(UTILS_OBJS) $(CON_OBJS) $(LIB_LOGURU_OBJS)

$(BUILD_DIR)/lib-loguru-%.o: $(LIB_DIR)/loguru/%.cpp
	$(CC) -c -o $@ $< $(CFLAGS)

$(BUILD_DIR)/connection-%.o: $(CON_DIR)/%.cpp $(CON_DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

$(BUILD_DIR)/utils-%.o: $(UTILS_DIR)/%.cpp $(UTILS_DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

$(BUILD_DIR)/mav-%.o: $(MAV_DIR)/%.cpp $(MAV_DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

$(BUILD_DIR)/mags-logger-%.o: $(MAGS_DIR)/%.cpp $(MAGS_DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

$(BUILD_DIR)/src-%.o: $(SRC_DIR)/%.cpp $(MAIN_DEPS) | $(BUILD_DIR)
	$(CC) -c -o $@ $< $(CFLAGS)

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

print: $(wildcard *.c)
	ls -la  $?

$(BUILD_DIR):
	mkdir -p $@

#The .PHONY rule keeps make from doing something with a file named clean.
.PHONY: clean

clean:
	-rm -f $(BUILD_DIR)/*/*.o
	-rm -f $(BUILD_DIR)/*.o
	-rm -f $(TARGET)
