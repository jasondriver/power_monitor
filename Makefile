.Phoney: all debug clean

# target
lib := serial_bytesteam

CC = gcc
CFLAGS := -Wall -Werror -Wextra

all:
	$(CC) $(CFLAGS) serial_bytestream.c -o $(lib)

debug:
	$(CC) $(CFLAGS) -ggdb serial_bytestream.c -o $(lib)

clean:
	rm -f $(lib)
