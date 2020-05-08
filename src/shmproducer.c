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
 * shmproducer.c
 *   a sample app to produce data into shared memory.
 *
 * 2020-05-08
 */

#define SHMMAP_TRACE_PRINT_ON
#include "shmmap.h"

char msg[1024];


int main(int argc, const char *argv[])
{
    int wok, len;

    shmmap_ringbuf_t *shmbuf;

    shmbuf = shmmap_ringbuf_create(SHMMAP_FILENAME_DEFAULT, SHMMAP_FILEMODE_DEFAULT, SHMMAP_FILESIZE_DEFAULT);
    if (! shmbuf) {
        printf("shmmap_ringbuf_create failed: %s\n", strerror(errno));
        exit(1);
    }

    srand(time(0));

    len = snprintf(msg, sizeof(msg), "{%d|%d|%d|%d|%d|%d|%d|%d|%d|%d}\n",
            rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand());

    wok = shmmap_ringbuf_write(shmbuf, (const void *) msg, (size_t) len);

    if (wok == SHMMAP_WRITE_SUCCESS) {
        printf("(shmproducer.c:%d) shmmap_ringbuf_write(%d): %.*s\n", __LINE__, wok, len, msg);
    } else if (wok == SHMMAP_WRITE_AGAIN) {
        printf("(shmproducer.c:%d) shmmap_ringbuf_write(%d): no space to write!\n", __LINE__, wok);
    }

    shmmap_ringbuf_close(shmbuf);
    return (0);
}