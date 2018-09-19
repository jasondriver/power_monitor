.Phoney: all debug clean

# target
lib := serial_bytesteam

CC = gcc
CFLAGS := -Wall -Werror -Wextra

all:
	$(CC) $(CFLAGS) serial_bytestream.c -o $(lib)

debug:
	$(CC) $(CFLAGS) -ggdb serial_bytestream.c -o $(lib)

run:
	export FLASK_APP=web_server.py
	flask run --host=0.0.0.0

debug_web:
	python web_server.py  #export FLASK_APP=web_server.py
	#export FLASK_ENV=development
	#flask run

clean:
	rm -f $(lib) web_server.pyc
