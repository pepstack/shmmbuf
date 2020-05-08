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

#define SHMMAP_TRACE_PRINT_ON
#include "shmmap.h"

char rdbuf[1024];


#define SHM_READMSG_NOCOPY


int next_shmmap_entry (const shmmap_entry_t *entry, void *arg)
{
    printf("(shmconsumer.c:%d) shmmap_ringbuf_read_nextcb(%" PRIu64"): %.*s\n",
        __LINE__, entry->size, (int)entry->size, entry->chunk);
    return SHMMAP_READ_NEXT;
}


int main(int argc, const char *argv[])
{
    size_t rdlen;

    shmmap_ringbuf_t *shmbuf;

    shmbuf = shmmap_ringbuf_create(SHMMAP_FILENAME_DEFAULT, SHMMAP_FILEMODE_DEFAULT, SHMMAP_FILESIZE_DEFAULT);
    if (! shmbuf) {
        printf("(shmconsumer.c:%d) shmmap_ringbuf_create failed: %s\n", __LINE__, strerror(errno));
        exit(1);
    }

#ifdef SHM_READMSG_NOCOPY
    // no copy
    rdlen = shmmap_ringbuf_read_nextcb(shmbuf, next_shmmap_entry, 0);
    if (rdlen == SHMMAP_READ_FATAL) {
        exit(EXIT_FAILURE);
    }
    if (rdlen == SHMMAP_READ_AGAIN) {
        printf("(shmconsumer.c:%d) shmmap_ringbuf_read_nextcb(%" PRIu64"): read no entry.\n", __LINE__, rdlen);
    }
#else
    // copy to rdbuf
    rdlen = shmmap_ringbuf_read_copy(shmbuf, rdbuf, sizeof(rdbuf));
    if (rdlen == SHMMAP_READ_FATAL) {
        exit(EXIT_FAILURE);
    }

    if (rdlen <= sizeof(rdbuf)) {
        printf("(shmconsumer.c:%d) shmmap_ringbuf_read_copy(%" PRIu64"): %.*s\n",
             __LINE__, rdlen, (int)rdlen, rdbuf);
    } else if (rdlen > sizeof(rdbuf)) {
        printf("(shmconsumer.c:%d) shmmap_ringbuf_read_copy(%" PRIu64"): insufficient read buf(%" PRIu64").\n",
             __LINE__, rdlen, sizeof(rdbuf));
    } else {
        printf("(shmconsumer.c:%d) shmmap_ringbuf_read_copy(%" PRIu64"): read no entry.\n", __LINE__, rdlen);
    }
#endif

    shmmap_ringbuf_close(shmbuf);
    return (0);
}