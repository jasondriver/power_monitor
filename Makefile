# target
lib := serial_bytestream
src := serial_bytestream.cc
deps := $(src)
deps += serial_bytestream.h
obj := serial_bytestream.o

CC = g++
#CFLAGS := -Wall -Werror -Wextra
CFLAGS := -Wall -Wextra
CFLAGS += -l sqlite3

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
	$(CC) -c $(CFLAGS) $(src) 

debug:
	$(CC) $(CFLAGS) -ggdb $(src) -o $(lib)

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
