CFLAGS=-Wall -O2  -ggdb3
LDFLAGS=-ggdb3 -lTML_lib -lz
#CFLAGS=-Wall -O2 -g0 -fomit-frame-pointer
#LDFLAGS=-O2 -g0 -s

OBJS = $(patsubst %.c,%.o,$(wildcard *.c))
HEADERS = $(wildcard *.h)
OUTFILE = $(notdir $(PWD))

$(OUTFILE) : $(OBJS)
	${CC} $^ $(LDFLAGS) -o $@

%.o : %.c $(HEADERS)
	${CC} $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OUTFILE) *.o
