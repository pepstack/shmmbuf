/***********************************************************************
 * Copyright (c) 2008-2080, 350137278@qq.com
 *
 * ALL RIGHTS RESERVED.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************/

/**
 * shmconsumer.c
 *   a sample app to comsume data from shared memory.
 *
 * 2020-05-08
 */

#define SHMMAP_TRACE_PRINT_OFF

#include "shmmap.h"


char rdbuf[1024];

int NUMPAGES = 8192;
int MESSAGES = 10000;


#define SHM_READMSG_NOCOPY


int next_shmmap_entry (const shmmap_entry_t *entry, void *arg)
{
    int id = ((int) (uintptr_t) (void*) arg);

    printf("(shmconsumer.c:%d) shmmap_ringbuf_read_nextcb(%d) ok: %.*s\n",
        __LINE__, id, (int)entry->size, entry->chunk);
    return SHMMAP_READ_NEXT;
}


int main(int argc, const char *argv[])
{
    int i;
    size_t rdlen;

    shmmap_ringbuf_t *shmbuf;

    if (argc != 3) {
        printf("Usage:\n"
               "  $ .%s NUMPAGES MESSAGES\n"
               "Sample:\n"
               "  $ .%s 8192 10000\n",
               strrchr(argv[0], '/'),
               strrchr(argv[0], '/'));
        exit(0);
    }

    NUMPAGES = atoi(argv[1]);
    MESSAGES = atoi(argv[2]);

    shmbuf = shmmap_ringbuf_create(SHMMAP_FILENAME_DEFAULT, SHMMAP_FILEMODE_DEFAULT,
                SHMMAP_PAGE_SIZE * NUMPAGES);

    if (! shmbuf) {
        printf("(shmconsumer.c:%d) shmmap_ringbuf_create failed: %s\n", __LINE__, strerror(errno));
        exit(1);
    }

    for (i = 1; i <= MESSAGES; i++) {
    #ifdef SHM_READMSG_NOCOPY
        // no copy
        rdlen = shmmap_ringbuf_read_nextcb(shmbuf, next_shmmap_entry, ((void*) (uintptr_t) (int) (i)));
        if (rdlen == SHMMAP_READ_FATAL) {
            exit(EXIT_FAILURE);
        }
        if (rdlen == SHMMAP_READ_AGAIN) {
            printf("(shmconsumer.c:%d) shmmap_ringbuf_read_nextcb(%d) endup!\n", __LINE__, i);
            break;
        }
    #else
        // copy to rdbuf
        rdlen = shmmap_ringbuf_read_copy(shmbuf, rdbuf, sizeof(rdbuf));
        if (rdlen == SHMMAP_READ_FATAL) {
            exit(EXIT_FAILURE);
        }

        if (rdlen <= sizeof(rdbuf)) {
            printf("(shmconsumer.c:%d) shmmap_ringbuf_read_copy(%d) ok: %.*s\n",
                 __LINE__, i, (int)rdlen, rdbuf);
        } else if (rdlen > sizeof(rdbuf)) {
            printf("(shmconsumer.c:%d) shmmap_ringbuf_read_copy(%d) fail: insufficient rdbuf(%" PRIu64").\n",
                 __LINE__, i, sizeof(rdbuf));
            break;
        } else {
            printf("(shmconsumer.c:%d) shmmap_ringbuf_read_copy(%d) endup!\n", __LINE__, i);
            break;
        }
    #endif
    }

    shmmap_ringbuf_close(shmbuf);
    return (0);
}