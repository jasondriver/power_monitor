# target
lib := serial_bytesteam

all:
	gcc -g serial_bytestream.c -o $(lib)

clean:
	rm -f $(lib)
