CC = g++
CFLAGS = -g -Wall -Iinclude

TARGET = main
BUILD_DIR = build

INC = \
-I"libs/eigen-3.4.0/Eigen" \
-I"libs/igrf" \
-I"libs/attitude"

SRC = \
main.cpp \
libs/igrf/igrf.c \
libs/attitude/dcm.c \
libs/attitude/maps.c \
libs/attitude/quaternion.c \
libs/attitude/rigid.cpp \


all:$(TARGET)

$(TARGET): $(SRC)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INC) -o $(BUILD_DIR)/$(TARGET) $(SRC)

run: all 
	./$(BUILD_DIR)/$(TARGET)

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean