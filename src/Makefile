# build create, post, wait
#

PREFIX=.

CC=gcc

# build for release
CFLAGS=-D_GNU_SOURCE -DNDEBUG -O2

INCDIRS=-I$(PREFIX)

# objects
OBJS=sem.o shmmap.o

# app bins
BINCREAT=creat
BINPOST=post
BINWAIT=wait

BINPROD=producer
BINCONS=consumer


all: $(BINCREAT) $(BINPOST) $(BINWAIT) $(BINPROD) $(BINCONS)

sem.o: $(PREFIX)/sem.c
	$(CC) $(CFLAGS) -c $(PREFIX)/sem.c -o $@

shmmap.o: $(PREFIX)/shmmap.c
	$(CC) $(CFLAGS) -c $(PREFIX)/shmmap.c -o $@

# creat
$(BINCREAT): $(OBJS) $(PREFIX)/creat.c
	$(CC) $(CFLAGS) $(PREFIX)/creat.c $(INCDIRS) -o $@ \
	$(OBJS) \
	-lpthread -lrt -lm

# post
$(BINPOST): $(OBJS) $(PREFIX)/post.c
	$(CC) $(CFLAGS) $(PREFIX)/post.c $(INCDIRS) -o $@ \
	$(OBJS) \
	-lpthread -lrt -lm

# wait
$(BINWAIT): $(OBJS) $(PREFIX)/wait.c
	$(CC) $(CFLAGS) $(PREFIX)/wait.c $(INCDIRS) -o $@ \
	$(OBJS) \
	-lpthread -lrt -lm

# producer
$(BINPROD): $(OBJS) $(PREFIX)/shmproducer.c
	$(CC) $(CFLAGS) $(PREFIX)/shmproducer.c $(INCDIRS) -o $@ \
	$(OBJS) \
	-lpthread -lrt -lm

# consumer
$(BINCONS): $(OBJS) $(PREFIX)/shmconsumer.c
	$(CC) $(CFLAGS) $(PREFIX)/shmconsumer.c $(INCDIRS) -o $@ \
	$(OBJS) \
	-lpthread -lrt -lm

clean:
	-rm -f $(PREFIX)/*.o
	-rm -f $(PREFIX)/$(BINCREAT)
	-rm -f $(PREFIX)/$(BINPOST)
	-rm -f $(PREFIX)/$(BINWAIT)
	-rm -f $(PREFIX)/$(BINPROD)
	-rm -f $(PREFIX)/$(BINCONS)