CC = g++
CFLAGS = -g -Wall -Wextra -Werror -pedantic -fsanitize=undefined,address -lm

TARGET = tests
BUILD_DIR = build

INC = -I"./../eigen-3.4.0/Eigen"

SRC = \
dcm.c \
maps.c \
quaternion.c \
main.cpp \
rigid.cpp


all: $(TARGET)

$(TARGET): $(SRC)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INC) -o $(BUILD_DIR)/$(TARGET) $(SRC)

run: all
	./$(BUILD_DIR)/$(TARGET)

plot:
	octave plot_output.m

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean
