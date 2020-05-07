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
 * @filename   shmmap.h
 *   shared memory for messages IO only for Linux.
 *
 * @author     Liang Zhang <350137278@qq.com>
 * @version    1.0.0
 * @create     2020-05-01 12:46:50
 * @update     2020-05-07 21:20:10
 */
#ifndef SHMMAP_H__
#define SHMMAP_H__

#if defined(__cplusplus)
extern "C"
{
#endif

#include "unitypes.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>

/**
 * constants cannot be changed!
 */
#define SHMMAP_STATE_INVALID    ((ub8)(-1))

#define SHMMAP_ALIGN_BSIZE(sz)  \
    ((size_t)((((size_t)(sz) + sizeof(shmmap_msg_t) - 1) / sizeof(shmmap_msg_t)) * sizeof(shmmap_msg_t)))

#define SHMMAP_ALIGN_MSGSIZE(msgsz)  SHMMAP_ALIGN_BSIZE(msgsz + sizeof(shmmap_msg_t))

#define SHMMAP_FILENAME_DEFAULT   "shmmap-ringbuf"

#define SHMMAP_FILEMODE_DEFAULT   0666


typedef struct _shmmap_msg_t
{
    union {
        ub8 size;
        char _alignb[sizeof(ub8)/sizeof(char)];
        struct _shmmap_msg_t *_alignp;
    };

    char chunk[0];
} shmmap_msg_t;


/**
 * POSIX.1-2008, glibc 2.12+
 *
 * http://www.linuxhowtos.org/manpages/3/pthread_mutexattr_setrobust.htm
 */
static void shmmap_mutex_consistent (pthread_mutex_t *mutexp)
{
    if (pthread_mutex_consistent(mutexp)) {
        perror("pthread_mutex_consistent");
        exit(EXIT_FAILURE);
    }

    if (pthread_mutex_unlock(mutexp)) {
        perror("pthread_mutex_unlock");
        exit(EXIT_FAILURE);
    }
}


typedef struct
{
    /* atomic state value */
    ub8 state;

    /* process-wide state lock */
    pthread_mutex_t mutex;
} shmmap_state_t;


static int shmmap_state_init (shmmap_state_t *st, ub8 state)
{
    int err;
    pthread_mutexattr_t attr;

    err = pthread_mutexattr_init(&attr);
    if (err) {
        return err;
    }

    err = pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    if (err) {
        pthread_mutexattr_destroy(&attr);
        return err;
    }

    err = pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
    if (err) {
        pthread_mutexattr_destroy(&attr);
        return err;
    }

    err = pthread_mutex_init(&st->mutex, &attr);
    if (err) {
        pthread_mutexattr_destroy(&attr);
        return err;
    }

    st->state = state;

    pthread_mutexattr_destroy(&attr);
    return 0;
}


static int shmmap_state_uninit (shmmap_state_t *st)
{
    int err = pthread_mutex_destroy(&st->mutex);
    return err;
}


static ub8 shmmap_state_get (shmmap_state_t *st)
{
    ub8 val = SHMMAP_STATE_INVALID;
    int err = pthread_mutex_lock(&st->mutex);

    if (! err) {
        val = st->state;
        pthread_mutex_unlock(&st->mutex);
    } else if (err == EOWNERDEAD) {
        shmmap_mutex_consistent(&st->mutex);
        return shmmap_state_get(st);
    }

    return val;
}


static ub8 shmmap_state_set (shmmap_state_t *st, ub8 newval)
{
    ub8 oldval = SHMMAP_STATE_INVALID;
    int err = pthread_mutex_lock(&st->mutex);

    if (! err) {
        oldval = st->state;
        st->state = newval;
        pthread_mutex_unlock(&st->mutex);
    } else if (err == EOWNERDEAD) {
        shmmap_mutex_consistent(&st->mutex);
        return shmmap_state_set(st, newval);
    }

    return oldval;
}


static ub8 shmmap_state_comp_exch (shmmap_state_t *st, ub8 comp, ub8 exch)
{
    ub8 oldval = SHMMAP_STATE_INVALID;
    int err = pthread_mutex_lock(&st->mutex);

    if (! err) {
        oldval = st->state;
        if (st->state == comp) {
            st->state = exch;
        }
        pthread_mutex_unlock(&st->mutex);
    } else if (err == EOWNERDEAD) {
        shmmap_mutex_consistent(&st->mutex);
        return shmmap_state_comp_exch(st, comp, exch);
    }

    return oldval;
}


typedef struct
{
    pthread_mutex_t lock;
    pthread_cond_t nonzero;
    sb8 count;
} shmmap_semaphore_t, *shmmap_semaphore;


static int shmmap_semaphore_init (shmmap_semaphore_t *semap)
{
    int err;

    pthread_mutexattr_t mattr;
    pthread_condattr_t cattr;

    err = pthread_mutexattr_init(&mattr);
    if (err) {
        perror("pthread_mutexattr_init");
        return err;
    }

    err = pthread_condattr_init(&cattr);
    if (err) {
        perror("pthread_condattr_init");
        pthread_mutexattr_destroy(&mattr);
        return err;
    }

    err = pthread_mutexattr_setpshared(&mattr, PTHREAD_PROCESS_SHARED);
    if (err) {
        perror("pthread_mutexattr_setpshared");
        goto error_exit;
    }

    err = pthread_mutexattr_setrobust(&mattr, PTHREAD_MUTEX_ROBUST);
    if (err) {
        perror("pthread_mutexattr_setrobust");
        goto error_exit;
    }

    err = pthread_condattr_setpshared(&cattr, PTHREAD_PROCESS_SHARED);
    if (err) {
        perror("pthread_condattr_setpshared");
        goto error_exit;
    }

    err = pthread_mutex_init(&semap->lock, &mattr);
    if (err) {
        perror("pthread_mutex_init");
        goto error_exit;
    }

    err = pthread_cond_init(&semap->nonzero, &cattr);
    if (err) {
        perror("pthread_cond_init");
        pthread_mutex_destroy(&semap->lock);
    }

    /* success */
    semap->count = 0;

error_exit:

    pthread_mutexattr_destroy(&mattr);
    pthread_condattr_destroy(&cattr);
    return err;
}


static void shmmap_semaphore_post (shmmap_semaphore_t * semap)
{
    int err;

    err = pthread_mutex_lock(&semap->lock);
    if (! err) {
        if (semap->count == 0) {
            pthread_cond_signal(&semap->nonzero);
        }
        semap->count++;
        pthread_mutex_unlock(&semap->lock);
    } else if (err == EOWNERDEAD) {
        shmmap_mutex_consistent(&semap->lock);
        shmmap_semaphore_post(semap);
    } else {
        perror("pthread_mutex_lock");
        exit(EXIT_FAILURE);
    }
}


static void shmmap_semaphore_wait (shmmap_semaphore_t * semap)
{
    int err;

    err = pthread_mutex_lock(&semap->lock);
    if (! err) {
        while (semap->count == 0) {
            pthread_cond_wait(&semap->nonzero, &semap->lock);
        }
        semap->count--;
        pthread_mutex_unlock(&semap->lock);
    } else if (err == EOWNERDEAD) {
        shmmap_mutex_consistent(&semap->lock);
        shmmap_semaphore_wait(semap);
    } else {
        perror("pthread_mutex_lock");
        exit(EXIT_FAILURE);
    }
}


typedef struct
{
    /**
     * https://linux.die.net/man/3/pthread_mutexattr_init
     */
    shmmap_semaphore_t  semaphore;

    /**
     * RingBuffer
     *
     * Length(L) = 10, Read(R), Write(W), wrap factor=0,1
     * Space(S)
     *
     * +           R        W        +                             +
     * |--+--+--+--+--+--+--+--+--+--|--+--+--+--+--+--+--+--+--+--|--+--+--+--
     * 0  1  2  3  4  5  6  7  8  9  0  1  2  3  4  5  6  7  8  9  0  1  2  3
     *
     *  Sw = L - (fL + W - R), W - R < L
     *  Sr = fL + W - R
     *
     *    Sw > 0: writable
     *    Sr > 0: readable
     */

    /* Read Lock. 0 - readable, 1 unreadable */
    shmmap_state_t RLock;

    /* Write Lock. 0 - writeable, 1 unwriteable */
    shmmap_state_t WLock;

    /* Wrap Factor: 0 or l */
    shmmap_state_t wrapfactor;

    /* Write Offset to the Buffer */
    shmmap_state_t WOffset;

    /* Read Offset to the Buffer */
    shmmap_state_t ROffset;

    /* Length of ring Buffer: total size in bytes */
    ub8 Length;

    /* shared memory buffer with Length */
    char Buffer[0];
} shmmap_ringbuf_t;



static void shmmap_ringbuf_close (shmmap_ringbuf_t *shmbuf)
{
    size_t mbufsize = (size_t) shmbuf->Length;
    munmap(shmbuf, sizeof(shmmap_ringbuf_t) + mbufsize);
}


static int shmmap_ringbuf_delete (const char *shmfilename)
{
    return shm_unlink(shmfilename);
}


static shmmap_ringbuf_t * shmmap_ringbuf_create (const char *shmfilename, mode_t filemode, ub8 maxbufsize)
{
    int err, mapfd;
    
    shmmap_ringbuf_t *shmbuf;

    /* aligned shared memory bufsize */
    size_t BSZ;

    int exist = 0;

    mapfd = shm_open(shmfilename, O_RDWR|O_CREAT|O_EXCL, 0666);
    if (mapfd == -1 && errno == EEXIST) {
        mapfd = shm_open(shmfilename, O_RDWR|O_CREAT, filemode);
        exist = 1;
	}
    if (mapfd == -1) {
        perror("shm_open");
        return (NULL);
    }

    BSZ = SHMMAP_ALIGN_BSIZE(maxbufsize);

    err = ftruncate(mapfd, sizeof(shmmap_ringbuf_t) + BSZ);
    if (err) {
        perror("ftruncate");
        close(mapfd);
        return (NULL);
    }

    shmbuf = (shmmap_ringbuf_t *) mmap(NULL, sizeof(shmmap_ringbuf_t) + BSZ, PROT_READ | PROT_WRITE, MAP_SHARED, mapfd, 0);

    close(mapfd);

    if (! shmbuf) {
        perror("mmap");
        return (NULL);
    }

    if (! exist) {
        memset(shmbuf, 0, sizeof(shmmap_ringbuf_t) + BSZ);

        err = shmmap_semaphore_init(&shmbuf->semaphore);
        if (err) {
            goto error_exit;
        }

        err = shmmap_state_init(&shmbuf->RLock, 0);
        if (err) {
            goto error_exit;
        }

        err = shmmap_state_init(&shmbuf->WLock, 0);
        if (err) {
            goto error_exit;
        }

        err = shmmap_state_init(&shmbuf->wrapfactor, 0);
        if (err) {
            goto error_exit;
        }

        err = shmmap_state_init(&shmbuf->WOffset, 0);
        if (err) {
            goto error_exit;
        }

        err = shmmap_state_init(&shmbuf->ROffset, 0);
        if (err) {
            goto error_exit;
        }

        shmbuf->Length = (ub8) BSZ;
    }

    /* success */
    return shmbuf;

    /* error */
error_exit:

    shmmap_ringbuf_close(shmbuf);
    shmmap_ringbuf_delete(shmfilename);
    return NULL;
}


static int shmmap_ringbuf_write (shmmap_ringbuf_t *shmbuf, const void *chunk, size_t chunksz)
{
    ub8 wrap, R, W, L = shmbuf->Length;

    size_t MSGSZ = SHMMAP_ALIGN_MSGSIZE(chunksz);
    if (! MSGSZ || MSGSZ > L / sizeof(shmmap_msg_t)) {
        /* no space for chunk */
        return 0;
    }

    if (! shmmap_state_comp_exch(&shmbuf->WLock, 0, 1)) {
        wrap = shmmap_state_get(&shmbuf->wrapfactor);
        if (wrap == SHMMAP_STATE_INVALID) {
            shmmap_state_set(&shmbuf->WLock, 0);
            printf("(shmmap.h:%d) fatal error.\n", __LINE__);
            return -1;
        }

        R = shmmap_state_get(&shmbuf->ROffset);
        if (R == SHMMAP_STATE_INVALID) {
            shmmap_state_set(&shmbuf->WLock, 0);
            printf("(shmmap.h:%d) fatal error.\n", __LINE__);
            return -1;
        }

        W = shmbuf->WOffset.state;

        printf("shmmap_ringbuf_write(MSGSZ=%llu): L=%llu wrap=%llu R=%llu W=%llu\n", MSGSZ, L, wrap, R, W);

        /* Sw = L - (wrap*L + W - R) */
        if (L + R - wrap*L - W >= MSGSZ) {
            shmmap_msg_t *msg;

            if (wrap) {
                /* 0 .. W < R < L */
                msg = (shmmap_msg_t *)&shmbuf->Buffer[W];
                msg->size = (ub8)chunksz;
                memcpy(msg->chunk, chunk, chunksz);

                /* WOffset = W + MSGSZ */
                shmmap_state_set(&shmbuf->WOffset, W + MSGSZ);
            } else {
                /* 0 .. R < W < L */
                if (L - W >= MSGSZ) {
                    msg = (shmmap_msg_t *)&shmbuf->Buffer[W];
                    msg->size = (ub8)chunksz;
                    memcpy(msg->chunk, chunk, chunksz);

                    /* WOffset = W + MSGSZ */
                    shmmap_state_set(&shmbuf->WOffset, W + MSGSZ);
                } else if (R - 0 >= MSGSZ) {
                    /* wrap W to 0 */
                    msg = (shmmap_msg_t *)&shmbuf->Buffer[0];
                    msg->size = (ub8)chunksz;
                    memcpy(msg->chunk, chunk, chunksz);

                    /* w3: WOffset = MSGSZ */
                    shmmap_state_set(&shmbuf->WOffset, MSGSZ);

                    /* w4: wrap = 1 */
                    shmmap_state_set(&shmbuf->wrapfactor, 1);
                } else {
                    /* no space to write */
                    shmmap_state_set(&shmbuf->WLock, 0);
                    return 0;
                }
            }

            /* write success */
            shmmap_state_set(&shmbuf->WLock, 0);
            return 1;
        }

        shmmap_state_set(&shmbuf->WLock, 0);
    }

    /* write locked */
    return 0;
}


static size_t shmmap_ringbuf_read_copy (shmmap_ringbuf_t *shmbuf, void *chunkbuf, size_t chunkbufsz)
{
    ub8 wrap, R, W, L = shmbuf->Length;

    size_t msgcb, MSGSZ, MINSZ = SHMMAP_ALIGN_MSGSIZE(0);

    if (! shmmap_state_comp_exch(&shmbuf->RLock, 0, 1)) {
        wrap = shmmap_state_get(&shmbuf->wrapfactor);
        if (wrap == SHMMAP_STATE_INVALID) {
            shmmap_state_set(&shmbuf->RLock, 0);
            printf("(shmmap.h:%d) fatal error.\n", __LINE__);
            return -1;
        }

        W = shmmap_state_get(&shmbuf->WOffset);
        if (W == SHMMAP_STATE_INVALID) {
            shmmap_state_set(&shmbuf->RLock, 0);
            printf("(shmmap.h:%d) fatal error.\n", __LINE__);
            return -1;
        }

        R = shmbuf->ROffset.state;

        printf("shmmap_ringbuf_read_copy(): L=%llu wrap=%llu R=%llu W=%llu\n", L, wrap, R, W);

        /* Sr = f*L + W - R */
        if (wrap*L + W - R > MINSZ) {
            shmmap_msg_t *msg;

            if (wrap) {
                /* 0 .. W < R < L */
                if (L - R > MINSZ) {
                    msg = (shmmap_msg_t *)&shmbuf->Buffer[R];
                    if (msg->size) {
                        msgcb = msg->size;

                        MSGSZ = SHMMAP_ALIGN_MSGSIZE(msgcb);

                        if (L - R >= MSGSZ) {
                            memcpy(chunkbuf, msg->chunk, msgcb);

                            /* success read chunk */
                            msg->size = 0;

                            shmmap_state_set(&shmbuf->ROffset, R + MSGSZ);

                            shmmap_state_set(&shmbuf->RLock, 0);
                            return msgcb;
                        } else {
                            /* fatal bug or invalid shared memory */
                            shmmap_state_set(&shmbuf->RLock, 0);
                            printf("(shmmap.h:%d) fatal code bug.\n", __LINE__);
                            return -1;
                        }
                    } else {
                        /* reset read offset to 0 (set nowrap) */
                        shmmap_state_set(&shmbuf->ROffset, 0);
                        shmmap_state_set(&shmbuf->wrapfactor, 0);
                    }
                } else if (W - 0 > MINSZ) {
                    msg = (shmmap_msg_t *)&shmbuf->Buffer[0];
                    if (msg->size) {
                        msgcb = msg->size;

                        MSGSZ = SHMMAP_ALIGN_MSGSIZE(msgcb);

                        if (W - 0 >= MSGSZ) {
                            memcpy(chunkbuf, msg->chunk, msgcb);

                            shmmap_state_set(&shmbuf->ROffset, MSGSZ);

                            shmmap_state_set(&shmbuf->wrapfactor, 0);

                            /* success read chunk */
                            msg->size = 0;
                            shmmap_state_set(&shmbuf->RLock, 0);
                            return msgcb;
                        } else {
                            /* fatal bug or invalid shared memory */
                            shmmap_state_set(&shmbuf->RLock, 0);
                            printf("(shmmap.h:%d) fatal code bug.\n", __LINE__);
                            return -1;
                        }
                    } else {
                        /* reset read offset to 0 (set nowrap) */
                        shmmap_state_set(&shmbuf->ROffset, 0);
                        shmmap_state_set(&shmbuf->wrapfactor, 0);
                    }
                }
            } else {
                /* 0 .. R < W < L */
                msg = (shmmap_msg_t *)&shmbuf->Buffer[R];
                if (msg->size) {
                    msgcb = msg->size;

                    MSGSZ = SHMMAP_ALIGN_MSGSIZE(msgcb);

                    if (W - R >= MSGSZ) {
                        memcpy(chunkbuf, msg->chunk, msgcb);

                        /* success read chunk */
                        msg->size = 0;
                        
                        shmmap_state_set(&shmbuf->ROffset, R + MSGSZ);

                        shmmap_state_set(&shmbuf->RLock, 0);
                        return msgcb;
                    } else {
                        /* fatal bug or invalid shared memory */
                        shmmap_state_set(&shmbuf->RLock, 0);
                        printf("(shmmap.h:%d) fatal code bug.\n", __LINE__);
                        return -1;
                    }
                } else {
                    /* fatal bug or invalid shared memory */
                    shmmap_state_set(&shmbuf->RLock, 0);
                    printf("(shmmap.h:%d) fatal code bug.\n", __LINE__);
                    return -1;
                }
            }
        }

        shmmap_state_set(&shmbuf->RLock, 0);
    }

    /* read locked fail, retry required */
    return 0;
}


static int shmmap_ringbuf_readmsg_cb (shmmap_ringbuf_t *shmbuf, int (*readmsg_cb)(const shmmap_msg_t *msg, void *arg), void *arg)
{
    ub8 wrap, R, W, L = shmbuf->Length;

    size_t MSGSZ, MINSZ = SHMMAP_ALIGN_MSGSIZE(0);

    if (! shmmap_state_comp_exch(&shmbuf->RLock, 0, 1)) {
        wrap = shmmap_state_get(&shmbuf->wrapfactor);
        if (wrap == SHMMAP_STATE_INVALID) {
            shmmap_state_set(&shmbuf->RLock, 0);
            printf("(shmmap.h:%d) fatal error.\n", __LINE__);
            return -1;
        }

        W = shmmap_state_get(&shmbuf->WOffset);
        if (W == SHMMAP_STATE_INVALID) {
            shmmap_state_set(&shmbuf->RLock, 0);
            printf("(shmmap.h:%d) fatal error.\n", __LINE__);
            return -1;
        }

        R = shmbuf->ROffset.state;

        printf("shmmap_ringbuf_read_copy(): L=%llu wrap=%llu R=%llu W=%llu\n", L, wrap, R, W);

        /* Sr = f*L + W - R */
        if (wrap*L + W - R > MINSZ) {
            shmmap_msg_t *msg;

            if (wrap) {
                /* 0 .. W < R < L */
                if (L - R > MINSZ) {
                    msg = (shmmap_msg_t *)&shmbuf->Buffer[R];
                    if (msg->size) {
                        MSGSZ = SHMMAP_ALIGN_MSGSIZE(msg->size);

                        if (L - R >= MSGSZ) {
                            /* success read chunk */
                            if (readmsg_cb(msg, arg)) {
                                /* 1: move to next msg */
                                msg->size = 0;

                                shmmap_state_set(&shmbuf->ROffset, R + MSGSZ);
                                shmmap_state_set(&shmbuf->RLock, 0);
                                return 1;
                            }
                        } else {
                            /* fatal bug or invalid shared memory */
                            shmmap_state_set(&shmbuf->RLock, 0);
                            printf("(shmmap.h:%d) fatal code bug.\n", __LINE__);
                            return -1;
                        }
                    } else {
                        /* reset read offset to 0 (set nowrap) */
                        shmmap_state_set(&shmbuf->ROffset, 0);
                        shmmap_state_set(&shmbuf->wrapfactor, 0);
                    }
                } else if (W - 0 > MINSZ) {
                    msg = (shmmap_msg_t *)&shmbuf->Buffer[0];
                    if (msg->size) {
                        MSGSZ = SHMMAP_ALIGN_MSGSIZE(msg->size);

                        if (W - 0 >= MSGSZ) {
                            /* success read chunk */
                            if (readmsg_cb(msg, arg)) {
                                /* 1: move to next msg */
                                msg->size = 0;

                                shmmap_state_set(&shmbuf->ROffset, MSGSZ);
                                shmmap_state_set(&shmbuf->wrapfactor, 0);

                                shmmap_state_set(&shmbuf->RLock, 0);
                                return 1;
                            } 
                        } else {
                            /* fatal bug or invalid shared memory */
                            shmmap_state_set(&shmbuf->RLock, 0);
                            printf("(shmmap.h:%d) fatal code bug.\n", __LINE__);
                            return -1;
                        }
                    } else {
                        /* reset read offset to 0 (set nowrap) */
                        shmmap_state_set(&shmbuf->ROffset, 0);
                        shmmap_state_set(&shmbuf->wrapfactor, 0);
                    }
                }
            } else {
                /* 0 .. R < W < L */
                msg = (shmmap_msg_t *)&shmbuf->Buffer[R];
                if (msg->size) {
                    MSGSZ = SHMMAP_ALIGN_MSGSIZE(msg->size);

                    if (W - R >= MSGSZ) {                        
                        /* success read chunk */
                        if (readmsg_cb(msg, arg)) {
                            /* 1: move to next msg */
                            msg->size = 0;

                            shmmap_state_set(&shmbuf->ROffset, R + MSGSZ);
                            shmmap_state_set(&shmbuf->RLock, 0);
                            return 1;
                        }
                    } else {
                        /* fatal bug or invalid shared memory */
                        shmmap_state_set(&shmbuf->RLock, 0);
                        printf("(shmmap.h:%d) fatal code bug.\n", __LINE__);
                        return -1;
                    }
                } else {
                    /* fatal bug or invalid shared memory */
                    shmmap_state_set(&shmbuf->RLock, 0);
                    printf("(shmmap.h:%d) fatal code bug.\n", __LINE__);
                    return -1;
                }
            }
        }

        shmmap_state_set(&shmbuf->RLock, 0);
    }

    /* read locked fail, retry required */
    return 0;
}

#ifdef __cplusplus
}
#endif
#endif /* SHMMAP_H__ */