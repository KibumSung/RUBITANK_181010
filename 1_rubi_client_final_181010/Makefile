# Set compiler to use
CC=g++
CFLAGS=-g -I. -fpermissive
LDFLAGS=
DEPS=keyboard.h
OBJS=rubi_client.o
DEBUG=0

all:: rubi_client.exe

rubi_client.exe: $(OBJS)
	$(CC) -o $@ $^ -std=c++11 $(CFLAGS) $(LDFLAGS)

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

clean: 
	rm -f *.exe *.o *~ *.stackdump
