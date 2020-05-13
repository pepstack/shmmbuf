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
 * producer.c
 *   A sample app to produce data into shared memory.
 *
 * 2020-05-10
 */

#define SHMMAP_TRACE_PRINT_OFF

#include "shmmap.h"

ub8token_t token = 12345678;

int NUMPAGES = 8192;
int MESSAGES = 100;

char msg[1024];


int main(int argc, const char *argv[])
{
    int wok, len, ret, i;

    shmmap_buffer_t *shmbuf;

    if (argc == 3) {
        NUMPAGES = atoi(argv[1]);
        MESSAGES = atoi(argv[2]);
    }

    ret = shmmap_buffer_create(&shmbuf,
                SHMMAP_FILENAME_DEFAULT,
                SHMMAP_FILEMODE_DEFAULT,
                SHMMAP_PAGE_SIZE * NUMPAGES,
                &token, NULL, NULL);
    if (ret) {
        printf("(producer.c:%d) shmmap_buffer_create error(%d)\n", __LINE__, ret);
        exit(EXIT_FAILURE);
    }

    srand(time(0));

    for (i = 1; i <= MESSAGES; i++) {
        len = snprintf(msg, sizeof(msg), "{%d|%d|%d|%d|%d|%d|%d|%d|%d|%d}\n",
                rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand());

        wok = shmmap_buffer_write(shmbuf, (const void *) msg, (size_t) len);

        if (wok == SHMMAP_WRITE_SUCCESS) {
            printf("(producer.c:%d) shmmap_buffer_write(%d/%d) success: %.*s\n", __LINE__, i, MESSAGES, len, msg);

            shmmap_buffer_post(shmbuf, SHMMAP_TIMEOUT_NOWAIT);
        } else if (wok == SHMMAP_WRITE_AGAIN) {
            printf("(producer.c:%d) shmmap_buffer_write(%d/%d) failure: No space left!\n", __LINE__, i, MESSAGES);
            usleep(1000);
        }
    }

    shmmap_buffer_close(shmbuf);
    return (0);
}