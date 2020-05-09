###########################################################
# Makefile
#   build producer and consumer.
#
# $ make clean && make
#
# A Product of ZhangLiang's
###########################################################

PREFIX=.

CC=gcc

# build for release
CFLAGS=-D_GNU_SOURCE -DNDEBUG -O2

INCDIRS=-I$(PREFIX)

# objects
OBJS=

# app bins
BINPROD=producer
BINCONS=consumer


all: $(BINPROD) $(BINCONS)


# producer
$(BINPROD): $(OBJS) $(PREFIX)/producer.c
	$(CC) $(CFLAGS) $(PREFIX)/producer.c $(INCDIRS) -o $@ \
	$(OBJS) \
	-lpthread -lrt -lm

# consumer
$(BINCONS): $(OBJS) $(PREFIX)/consumer.c
	$(CC) $(CFLAGS) $(PREFIX)/consumer.c $(INCDIRS) -o $@ \
	$(OBJS) \
	-lpthread -lrt -lm

clean:
	-rm -f $(PREFIX)/*.o
	-rm -f $(PREFIX)/$(BINPROD)
	-rm -f $(PREFIX)/$(BINCONS)