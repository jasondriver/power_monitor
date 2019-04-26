TARGET 	:= bin/output
CC      := g++
CFLAGS  := -Wall -Werror -std=c++17
INC := -I include

SRC_DIR := src
OBJ_DIR := build
INC_DIR   := include

_OBJ = serial.o serial_bytestream.o main.o
OBJ = $(patsubst %,$(OBJ_DIR)/%,$(_OBJ))

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -l sqlite3 -pthread $^ -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cc $(INC_DIR)/%.h
	$(CC) $(CFLAGS) $(INC) -c $< -o $@

clean:
	rm -f $(OBJ_DIR)/*.o $(TARGET)

.PHONY: clean