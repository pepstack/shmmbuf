/* shmconsumer.c */
#include "shmmap.h"

//#define SHM_READMSG_NOCOPY

int on_shm_readmsg(const shmmap_msg_t *msg, void *arg)
{
    printf("shmmap_ringbuf_read(msgsz=%llu): %.*s\n", msg->size, (int)msg->size, msg->chunk);
    return 1;
}


int main(int argc, const char *argv[])
{
    ssize_t msgsz;

    char msgbuf[256];

    shmmap_ringbuf_t *shmbuf;

    shmbuf = shmmap_ringbuf_create(SHMMAP_FILENAME_DEFAULT, SHMMAP_FILEMODE_DEFAULT, 1024);
    if (! shmbuf) {
        printf("shmmap_ringbuf_create failed: %s\n", strerror(errno));
        exit(1);
    }

#ifdef SHM_READMSG_NOCOPY
    // no copy
    shmmap_ringbuf_readmsg_cb(shmbuf, on_shm_readmsg, 0);
#else
    // copy to msgbuf
    msgsz = shmmap_ringbuf_read_copy(shmbuf, msgbuf, sizeof(msgbuf));
    if (msgsz > 0 && msgsz < (ssize_t) sizeof(msgbuf)) {
        printf("shmmap_ringbuf_read(msgsz=%lld): %.*s\n", msgsz, (int)msgsz, msgbuf);
    }
#endif

    shmmap_ringbuf_close(shmbuf);
    return (0);
}