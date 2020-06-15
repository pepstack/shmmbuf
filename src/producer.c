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
 * 2020-06-08
 */

#define SHMMBUF_TRACE_PRINT_OFF

#include "shmmbuf.h"

ub8token_t token = 12345678;

int NUMPAGES = 8192;
int MESSAGES = 10000;

char msg[1024];


int main(int argc, const char *argv[])
{
    int wok, len, ret, i = 0;

    struct timespec t1, t2;

    sb8 elapsed_ms;

    shmmap_buffer_t *shmbuf;

    if (argc == 2) {
        MESSAGES = atoi(argv[1]);
    } else if (argc == 3) {
        MESSAGES = atoi(argv[1]);
        NUMPAGES = atoi(argv[2]);
    } else {
        printf("producer MESSAGES <NUMPAGES>\n");
        exit(0);
    }

    ret = shmmap_buffer_create(&shmbuf,
                SHMMBUF_FILENAME_DEFAULT,
                SHMMBUF_FILEMODE_DEFAULT,
                SHMMBUF_PAGE_SIZE * NUMPAGES,
                &token, NULL, NULL);
    if (ret) {
        printf("(producer.c:%d) shmmap_buffer_create error(%d)\n", __LINE__, ret);
        exit(EXIT_FAILURE);
    }

    srand(time(0));

    shmmap_gettimeofday(&t1);

    len = snprintf(msg, sizeof(msg), "{%d|%d|%d|%d|%d|%d|%d|%d|%d|%d}\n",
                rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand());

    for (i = 1; i <= MESSAGES; i++) {
    #ifdef NO_TEST_SPEED
        len = snprintf(msg, sizeof(msg), "{%d|%d|%d|%d|%d|%d|%d|%d|%d|%d}\n",
                    rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand());
    #endif

        wok = shmmap_buffer_write(shmbuf, (const void *) msg, (size_t) len);

        if (wok == SHMMBUF_WRITE_SUCCESS) {
            if (i % 100000 == 0) {
                printf("(producer.c:%d) shmmap_buffer_write(%d/%d) success: %.*s\n", __LINE__, i, MESSAGES, len, msg);
            }

            shmmap_buffer_post(shmbuf, SHMMBUF_TIMEOUT_NOWAIT);
        } else if (wok == SHMMBUF_WRITE_AGAIN) {
            printf("(producer.c:%d) shmmap_buffer_write(%d/%d) failure: No space left!\n", __LINE__, i, MESSAGES);
            usleep(1000);
        }
    }

    shmmap_buffer_close(shmbuf);
    
    shmmap_gettimeofday(&t2);

    elapsed_ms = shmmap_difftime_msec(&t1, &t2);

    printf("(producer.c:%d) total % "PRIu64 " messages produced. elapsed %" PRId64 "ms. speed = %d/S.\n", __LINE__,
        MESSAGES, elapsed_ms,
        (int)(MESSAGES / (elapsed_ms / 1000)));

    return (0);
}