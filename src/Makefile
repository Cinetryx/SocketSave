CC=g++
#CFLAGS+=-g
CFLAGS+=`pkg-config --cflags opencv`
LDFLAGS+=`pkg-config --libs opencv`

PROG=SocketSave
OBJS=$(PROG).o

.PHONY: all clean
$(PROG): $(OBJS)
	$(CC) -o $(PROG) -O2 $(OBJS) $(LDFLAGS)
%.o: %.cpp
	$(CC) -c $(CFLAGS) $<
all: 
	$(PROG)
	rm -f $(OBJS)
clean:
	rm -f $(OBJS) $(PROG)
