# target
lib := serial_bytesteam
deps := serial_bytestream.c serial_bytestream.h
obj := serial_bytestream.o

CC = gcc
CFLAGS := -Wall -Werror -Wextra

## Debug flag
ifneq ($(D),1)
CFLAGS	+= -O2
else
CFLAGS	+= -O0
CFLAGS	+= -g
endif

all: $(obj)
	$(CC) $(CFLAGS) $(obj) -o $(lib)

$(obj): $(deps)
	$(CC) -c $(CFLAGS) serial_bytestream.c

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
	rm -f $(lib) $(obj) web_server.pyc

.Phoney: all debug clean
