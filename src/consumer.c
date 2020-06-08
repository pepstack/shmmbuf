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
 * consumer.c
 *   A sample app to comsume data from shared memory.
 *
 * 2020-06-08
 */

#define SHMMBUF_TRACE_PRINT_OFF

#include "shmmbuf.h"


ub8token_t token = 12345678;

char rdbuf[1024];

int NUMPAGES = 8192;
int MESSAGES = 100;


#define SHM_READMSG_NOCOPY


int next_shmmbuf_entry (const shmmbuf_entry_t *entry, void *arg)
{
    ub8 msgid = *((ub8*)arg);

    msgid++;

    printf("(consumer.c:%d) shmmap_buffer_read_next(%" PRIu64") success: %.*s\n",
        __LINE__, msgid, (int)entry->size, entry->chunk);

    *((ub8*)arg) = msgid;

    return SHMMBUF_READ_NEXT;
}


int main(int argc, const char *argv[])
{
    int ret, num;
    size_t rdlen;

    ub8 msgid;

    sb8 dtms;
    struct timespec t1, t2;

    pid_t pid = getpid();

    shmmap_buffer_t *shmbuf;

    if (argc == 3) {
        NUMPAGES = atoi(argv[1]);
        MESSAGES = atoi(argv[2]);
    }

    ret = shmmap_buffer_create(&shmbuf,
                SHMMBUF_FILENAME_DEFAULT,
                SHMMBUF_FILEMODE_DEFAULT,
                SHMMBUF_PAGE_SIZE * NUMPAGES, /* 0: do not care */
                &token, NULL, NULL);

    if (ret) {
        printf("(consumer.c:%d) shmmap_buffer_create error(%d)\n", __LINE__, ret);
        exit(EXIT_FAILURE);
    }

    // shmmap_buffer_force_unlock(shmbuf, SHMMBUF_READSTATE_LOCK|SHMMBUF_WRITESTATE_LOCK);

    msgid = 0;
    while (1) {
        shmmap_gettimeofday(&t1);
        
        /* max wait = 3000 ms */
        shmmap_buffer_wait(shmbuf, 3000*1000);

    #ifdef SHM_READMSG_NOCOPY

        // batch read without copy
        while ((num = shmmap_buffer_read_next_batch(shmbuf, next_shmmbuf_entry, (void *)&msgid, 20)) > 0) {
            // read num success
        }

        if (num == SHMMBUF_READ_FATAL) {
            printf("(consumer.c:%d) shmmap_buffer_read_next_batch fatal.\n", __LINE__);
            break;
        }

        shmmap_gettimeofday(&t2);

        dtms = shmmap_difftime_msec(&t1, &t2);

        if (dtms > 1000) {
            printf("(consumer.c:%d) process(%d) total read %" PRIu64" messages.\n", __LINE__, (int)pid, msgid);
        }

    #else
        // copy to rdbuf
        rdlen = shmmap_buffer_read_copy(shmbuf, rdbuf, sizeof(rdbuf));
        if (rdlen == SHMMBUF_READ_FATAL) {
            exit(EXIT_FAILURE);
        }

        if (rdlen <= sizeof(rdbuf)) {
            printf("(consumer.c:%d) shmmap_buffer_read_copy(%d) success: %.*s\n",
                 __LINE__, i, (int)rdlen, rdbuf);
        } else if (rdlen > sizeof(rdbuf)) {
            printf("(consumer.c:%d) shmmap_buffer_read_copy(%d) failure: insufficient rdbuf(%" PRIu64").\n",
                 __LINE__, i, sizeof(rdbuf));
            break;
        } else {
            printf("(consumer.c:%d) shmmap_buffer_read_copy(%d) endup!\n", __LINE__, i);
        }
    #endif
    }

    shmmap_buffer_close(shmbuf);
    return (0);
}