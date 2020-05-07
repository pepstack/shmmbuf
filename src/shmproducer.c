/* shmproducer.c */
#include "shmmap.h"


int main(int argc, const char *argv[])
{
    int ok, len;
    char msg[256];

    shmmap_ringbuf_t *shmbuf;

    shmbuf = shmmap_ringbuf_create(SHMMAP_FILENAME_DEFAULT, SHMMAP_FILEMODE_DEFAULT, 1024);
    if (! shmbuf) {
        printf("shmmap_ringbuf_create failed: %s\n", strerror(errno));
        exit(1);
    }

    printf("shmproducer start...\n");

    srand(time(0));

    len = snprintf(msg, sizeof(msg), "{%d|%d|%d|%d|%d|%d|%d|%d}\n",
            rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand());

    ok = shmmap_ringbuf_write(shmbuf, (const void *) msg, (size_t) len);

    printf("shmmap_ringbuf_write(%s): %.*s\n", (ok? "success" : "failed"), len, msg);

    shmmap_ringbuf_close(shmbuf);

    printf("shmproducer exit ok.\n");
    return (0);
}