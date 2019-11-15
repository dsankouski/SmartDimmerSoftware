BOARD_TAG = micro
ARDUINO_PORT ?= /dev/ttyACM0
ARDUINO_LIBS =
ARDUINO_DIR ?= /opt/arduino-1.8.10
AVR_TOOLS_DIR ?= /usr

include build.config
export DEBUG_ENABLE = y
CFLAGS_STD := -DDEBUG_ENABLE
CXXFLAGS := -DDEBUG_ENABLE

include /usr/share/arduino/Arduino.mk