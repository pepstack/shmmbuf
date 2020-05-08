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
 *   MP and MT-safe shared memory IO for Linux (POSIX.1-2008, glibc 2.12+).
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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>


#define SHMMAP_TRACE_ON


#ifndef NOWARNING_UNUSED
    # if defined(__GNUC__) || defined(__CYGWIN__)
        # define NOWARNING_UNUSED(x) __attribute__((unused)) x
    # else
        # define NOWARNING_UNUSED(x) x
    # endif
#endif


/* see: "/dev/shm/shmmap-ringbuf" */
#define SHMMAP_FILENAME_DEFAULT        "shmmap-ringbuf"

#define SHMMAP_FILEMODE_DEFAULT        0666


/**
 * constants cannot be changed!
 */
#define SHMMAP_STATE_INVALID           ((size_t)(-1))
#define SHMMAP_SIZEOF_MSG              (sizeof(shmmap_msg_t))

#define SHMMAP_ALIGN_BSIZE(sz)  \
    ((size_t)((((size_t)(sz)+SHMMAP_SIZEOF_MSG-1)/SHMMAP_SIZEOF_MSG)*SHMMAP_SIZEOF_MSG))

#define SHMMAP_ALIGN_MSGSIZE(msgsz)    SHMMAP_ALIGN_BSIZE(msgsz+SHMMAP_SIZEOF_MSG)

#define SHMMAP_MSG_CAST(p)             ((shmmap_msg_t *)(p))


typedef struct _shmmap_msg_t
{
    size_t size;
    char chunk[0];
} shmmap_msg_t;


/**
 * http://www.linuxhowtos.org/manpages/3/pthread_mutexattr_setrobust.htm
 */
NOWARNING_UNUSED(static)
void shmmap_mutex_consistent (pthread_mutex_t *mutexp)
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


typedef struct _shmmap_state_t
{
    /* atomic state value */
    size_t state;

    /* process-wide state lock */
    pthread_mutex_t mutex;
} shmmap_state_t;


NOWARNING_UNUSED(static)
int shmmap_state_init (shmmap_state_t *st, size_t state)
{
    pthread_mutexattr_t attr;
    int err = pthread_mutexattr_init(&attr);
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


NOWARNING_UNUSED(static)
int shmmap_state_uninit (shmmap_state_t *st)
{
    int err = pthread_mutex_destroy(&st->mutex);
    return err;
}


NOWARNING_UNUSED(static)
size_t shmmap_state_get (shmmap_state_t *st)
{
    size_t val = SHMMAP_STATE_INVALID;
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


NOWARNING_UNUSED(static)
size_t shmmap_state_set (shmmap_state_t *st, size_t newval)
{
    size_t oldval = SHMMAP_STATE_INVALID;
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


NOWARNING_UNUSED(static)
size_t shmmap_state_comp_exch (shmmap_state_t *st, size_t comp, size_t exch)
{
    size_t oldval = SHMMAP_STATE_INVALID;
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


typedef struct _shmmap_semaphore_t
{
    pthread_mutex_t lock;
    pthread_cond_t  nonzero;
    ssize_t         count;
} shmmap_semaphore_t, *shmmap_semaphore;


NOWARNING_UNUSED(static)
int shmmap_semaphore_init (shmmap_semaphore_t *semap)
{
    pthread_mutexattr_t mattr;
    pthread_condattr_t cattr;

    int err = pthread_mutexattr_init(&mattr);
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


NOWARNING_UNUSED(static)
void shmmap_semaphore_post (shmmap_semaphore_t * semap)
{
    int err = pthread_mutex_lock(&semap->lock);
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


NOWARNING_UNUSED(static)
void shmmap_semaphore_wait (shmmap_semaphore_t * semap)
{
    int err = pthread_mutex_lock(&semap->lock);
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


typedef struct _shmmap_ringbuf_t
{
    /**
     * https://linux.die.net/man/3/pthread_mutexattr_init
     */
    shmmap_semaphore_t  semaphore;

    /**
     * RingBuffer
     *
     * Length(L) = 10, Read(R), Write(W), wf factor=0,1
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
    size_t Length;

    /* ring buffer in shared memory with Length */
    char Buffer[0];
} shmmap_ringbuf_t;


NOWARNING_UNUSED(static)
void shmmap_ringbuf_close (shmmap_ringbuf_t *shmbuf)
{
    size_t bsize = shmbuf->Length;
    munmap(shmbuf, sizeof(shmmap_ringbuf_t) + bsize);
}


NOWARNING_UNUSED(static)
int shmmap_ringbuf_delete (const char *shmfilename)
{
    return shm_unlink(shmfilename);
}

#define SHMMAP_CHECK_ERR(err)  if (err) goto error_exit


NOWARNING_UNUSED(static)
shmmap_ringbuf_t * shmmap_ringbuf_create (const char *shmfilename, mode_t filemode, size_t maxbufsize)
{
    int err, mfd;
    
    shmmap_ringbuf_t *shmbuf;

    /* aligned shared memory bufsize as length */
    size_t rbLength;

    /* total size of shmmap file */
    size_t mfdSize;

    int exist = 0;

    mfd = shm_open(shmfilename, O_RDWR|O_CREAT|O_EXCL, 0666);
    if (mfd == -1 && errno == EEXIST) {
        mfd = shm_open(shmfilename, O_RDWR|O_CREAT, filemode);
        exist = 1;
	}
    if (mfd == -1) {
        perror("shm_open");
        return (NULL);
    }

    rbLength = SHMMAP_ALIGN_BSIZE(maxbufsize);
    mfdSize = sizeof(shmmap_ringbuf_t) + rbLength;

    err = ftruncate(mfd, mfdSize);
    if (err) {
        perror("ftruncate");
        close(mfd);
        return (NULL);
    }

    shmbuf = (shmmap_ringbuf_t *) mmap(NULL, mfdSize, PROT_READ|PROT_WRITE, MAP_SHARED, mfd, 0);
    if (! shmbuf) {
        perror("mmap");
        close(mfd);
        return (NULL);
    }
    close(mfd);

    if (! exist) {
        bzero(shmbuf, mfdSize);

        err = shmmap_semaphore_init(&shmbuf->semaphore);
        SHMMAP_CHECK_ERR(err);

        err = shmmap_state_init(&shmbuf->RLock, 0);
        SHMMAP_CHECK_ERR(err);

        err = shmmap_state_init(&shmbuf->WLock, 0);
        SHMMAP_CHECK_ERR(err);

        err = shmmap_state_init(&shmbuf->wrapfactor, 0);
        SHMMAP_CHECK_ERR(err);

        err = shmmap_state_init(&shmbuf->WOffset, 0);
        SHMMAP_CHECK_ERR(err);

        err = shmmap_state_init(&shmbuf->ROffset, 0);
        SHMMAP_CHECK_ERR(err);

        shmbuf->Length = rbLength;
    }

    /* success */
    return shmbuf;

    /* error */
error_exit:

    shmmap_ringbuf_close(shmbuf);
    shmmap_ringbuf_delete(shmfilename);
    return NULL;
}


/**
 * shmmap_ringbuf_write()
 *   write data chunk into shmmap ringbuffer. 
 *
 * returns:
 *    1 - write success
 *    0 - fail to write
 *   -1 - fatal error
 */
NOWARNING_UNUSED(static)
int shmmap_ringbuf_write (shmmap_ringbuf_t *shmbuf, const void *chunk, size_t chunksz)
{
    shmmap_msg_t *msg;

    ssize_t R, W, wf,
        L = shmbuf->Length,
        MSGSZ = (ssize_t)SHMMAP_ALIGN_MSGSIZE(chunksz);

    if (! MSGSZ || MSGSZ == SHMMAP_STATE_INVALID || MSGSZ > (L / SHMMAP_SIZEOF_MSG)) {
        /* fail to write for wrong chunksz */
        return 0;
    }

    if (! shmmap_state_comp_exch(&shmbuf->WLock, 0, 1)) {
        wf = shmmap_state_get(&shmbuf->wrapfactor);
        if (wf == SHMMAP_STATE_INVALID) {
            shmmap_state_set(&shmbuf->WLock, 0);
        # ifdef SHMMAP_TRACE_ON
            printf("(shmmap.h:%d) fatal error.\n", __LINE__);
        # endif
            return -1;
        }

        R = shmmap_state_get(&shmbuf->ROffset);
        if (R == SHMMAP_STATE_INVALID) {
            shmmap_state_set(&shmbuf->WLock, 0);
        # ifdef SHMMAP_TRACE_ON
            printf("(shmmap.h:%d) fatal error.\n", __LINE__);
        # endif
            return -1;
        }

        W = shmbuf->WOffset.state;

    # ifdef SHMMAP_TRACE_ON
        printf("shmmap_ringbuf_write(MSGSZ=%llu): L=%llu wf=%llu R=%llu W=%llu\n", MSGSZ, L, wf, R, W+MSGSZ);
    # endif

        /* Sw = L - (wf*L + W - R) */
        if (L - (wf*L + W - R) >= MSGSZ) {
            if (wf) {
                /* 0 .. W < R < L */
                msg = SHMMAP_MSG_CAST(&shmbuf->Buffer[W]);
                msg->size = chunksz;
                memcpy(msg->chunk, chunk, chunksz);

                /* WOffset = W + MSGSZ */
                shmmap_state_set(&shmbuf->WOffset, W + MSGSZ);
            } else {
                /* 0 .. R < W < L */
                if (L - W >= MSGSZ) {
                    msg = SHMMAP_MSG_CAST(&shmbuf->Buffer[W]);
                    msg->size = chunksz;
                    memcpy(msg->chunk, chunk, chunksz);

                    /* WOffset = W + MSGSZ */
                    shmmap_state_set(&shmbuf->WOffset, W + MSGSZ);
                } else if (R - 0 >= MSGSZ) {
                    /* clear W slot first */
                    bzero(&shmbuf->Buffer[W], L - W); 

                    /* W wrap to 0 */
                    msg = SHMMAP_MSG_CAST(&shmbuf->Buffer[0]);
                    msg->size = chunksz;
                    memcpy(msg->chunk, chunk, chunksz);

                    /* WOffset = MSGSZ */
                    shmmap_state_set(&shmbuf->WOffset, MSGSZ);

                    /* set wrap: wf = 1 */
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


/**
 * shmmap_ringbuf_read_copy()
 *   copy read data from shmmap ringbuffer into copybuf.
 *
 * returns:
 *    1 - write success
 *    0 - fail to write
 *   -1 - fatal error
 */
NOWARNING_UNUSED(static)
ssize_t shmmap_ringbuf_read_copy (shmmap_ringbuf_t *shmbuf, void *copybuf, size_t copybufsz)
{
    shmmap_msg_t *msg;

    ssize_t R, W, wf, msgsize, MSGSZ, 
        L = shmbuf->Length,
        HEADSZ = (ssize_t) SHMMAP_ALIGN_MSGSIZE(0);

    if (! shmmap_state_comp_exch(&shmbuf->RLock, 0, 1)) {
        wf = shmmap_state_get(&shmbuf->wrapfactor);
        if (wf == SHMMAP_STATE_INVALID) {
            shmmap_state_set(&shmbuf->RLock, 0);
        #ifdef SHMMAP_TRACE_ON
            printf("(shmmap.h:%d) fatal error.\n", __LINE__);
        #endif
            return -1;
        }

        W = shmmap_state_get(&shmbuf->WOffset);
        if (W == SHMMAP_STATE_INVALID) {
            shmmap_state_set(&shmbuf->RLock, 0);
        #ifdef SHMMAP_TRACE_ON
            printf("(shmmap.h:%d) fatal error.\n", __LINE__);
        #endif
            return -1;
        }

        R = shmbuf->ROffset.state;

    #ifdef SHMMAP_TRACE_ON
        printf("shmmap_ringbuf_read_copy(): L=%llu wf=%llu R=%llu W=%llu\n", L, wf, R, W);
    #endif

        /* Sr = f*L + W - R */
        if (wf*L + W - R > HEADSZ) {
            if (wf) {
                /* 0 .. W < R < L */
                if (L - R > HEADSZ) {
                    msg = SHMMAP_MSG_CAST(&shmbuf->Buffer[R]);
                    if (msg->size) {
                        MSGSZ = SHMMAP_ALIGN_MSGSIZE(msg->size);

                        if (L - R >= MSGSZ) {
                            msgsize = msg->size;
                            if (msgsize > copybufsz) {
                                /* fail on insufficent copybuf */
                                shmmap_state_set(&shmbuf->RLock, 0);
                                return (ssize_t) msgsize;
                            }

                            /* success read chunk */
                            memcpy(copybuf, msg->chunk, msgsize);

                            shmmap_state_set(&shmbuf->ROffset, R + MSGSZ);
                            shmmap_state_set(&shmbuf->RLock, 0);
                            return (ssize_t) msgsize;
                        }

                        /* fatal bug or invalid shared memory */
                        shmmap_state_set(&shmbuf->RLock, 0);
                    #ifdef SHMMAP_TRACE_ON
                        printf("(shmmap.h:%d) fatal code bug. msgsize=%llu\n", __LINE__, msg->size);
                    #endif
                        return -1;
                    } else {
                        /* reset ROffset to 0 (set nowrap) */
                        shmmap_state_set(&shmbuf->ROffset, 0);
                        shmmap_state_set(&shmbuf->wrapfactor, 0);

                        shmmap_state_set(&shmbuf->RLock, 0);
                        
                        /* tell user to call this method again */
                        return -2;
                    }
                } else if (W - 0 > HEADSZ) {
                    msg = SHMMAP_MSG_CAST(&shmbuf->Buffer[0]);
                    if (msg->size) {
                        MSGSZ = SHMMAP_ALIGN_MSGSIZE(msg->size);

                        if (W - 0 >= MSGSZ) {
                            msgsize = msg->size;
                            if (msgsize > copybufsz) {
                                /* fail on insufficent copybuf */
                                shmmap_state_set(&shmbuf->RLock, 0);
                                return (ssize_t) msgsize;
                            }

                            /* success read chunk */
                            memcpy(copybuf, msg->chunk, msgsize);

                            shmmap_state_set(&shmbuf->ROffset, MSGSZ);
                            shmmap_state_set(&shmbuf->wrapfactor, 0);

                            shmmap_state_set(&shmbuf->RLock, 0);
                            return (ssize_t) msgsize;
                        }
                    }
                    
                    /* fatal bug or invalid shared memory */
                    shmmap_state_set(&shmbuf->RLock, 0);
                #ifdef SHMMAP_TRACE_ON
                    printf("(shmmap.h:%d) fatal code bug.\n", __LINE__);
                #endif
                    return -1;
                }
            } else {
                /* 0 .. R < W < L */
                msg = SHMMAP_MSG_CAST(&shmbuf->Buffer[R]);
                if (msg->size) {
                    MSGSZ = SHMMAP_ALIGN_MSGSIZE(msg->size);

                    if (W - R >= MSGSZ) {
                        msgsize = msg->size;

                        if (msgsize > copybufsz) {
                            /* fail on insufficent copybuf */
                            shmmap_state_set(&shmbuf->RLock, 0);
                            return (ssize_t) msgsize;
                        }

                        /* success read chunk */
                        memcpy(copybuf, msg->chunk, msgsize);

                        shmmap_state_set(&shmbuf->ROffset, R + MSGSZ);
                        shmmap_state_set(&shmbuf->RLock, 0);
                        return msgsize;
                    }
                }

                /* fatal bug or invalid shared memory */
                shmmap_state_set(&shmbuf->RLock, 0);
            #ifdef SHMMAP_TRACE_ON
                printf("(shmmap.h:%d) fatal code bug.\n", __LINE__);
            #endif
                return -1;
            }
        }

        shmmap_state_set(&shmbuf->RLock, 0);
    }

    /* read locked fail, retry required */
    return 0;
}

// TODO:
NOWARNING_UNUSED(static)
int shmmap_ringbuf_readmsg_cb (shmmap_ringbuf_t *shmbuf, int (*readmsg_cb)(const shmmap_msg_t *msg, void *arg), void *arg)
{
    shmmap_msg_t *msg;

    size_t wf, R, W, L = shmbuf->Length;

    size_t MSGSZ, HEADSZ = SHMMAP_ALIGN_MSGSIZE(0);

    if (! shmmap_state_comp_exch(&shmbuf->RLock, 0, 1)) {
        wf = shmmap_state_get(&shmbuf->wrapfactor);
        if (wf == SHMMAP_STATE_INVALID) {
            shmmap_state_set(&shmbuf->RLock, 0);
        #ifdef SHMMAP_TRACE_ON
            printf("(shmmap.h:%d) fatal error.\n", __LINE__);
        #endif
            return -1;
        }

        W = shmmap_state_get(&shmbuf->WOffset);
        if (W == SHMMAP_STATE_INVALID) {
            shmmap_state_set(&shmbuf->RLock, 0);
        #ifdef SHMMAP_TRACE_ON
            printf("(shmmap.h:%d) fatal error.\n", __LINE__);
        #endif
            return -1;
        }

        R = shmbuf->ROffset.state;

    #ifdef SHMMAP_TRACE_ON
        printf("shmmap_ringbuf_read_copy(): L=%llu wf=%llu R=%llu W=%llu\n", L, wf, R, W);
    #endif

        /* Sr = f*L + W - R */
        if (wf*L + W - R > HEADSZ) {
            if (wf) {
                /* 0 .. W < R < L */
                if (L - R > HEADSZ) {
                    msg = SHMMAP_MSG_CAST(&shmbuf->Buffer[R]);
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
                        #ifdef SHMMAP_TRACE_ON
                            printf("(shmmap.h:%d) fatal code bug.\n", __LINE__);
                        #endif
                            return -1;
                        }
                    } else {
                        /* reset read offset to 0 (set nowrap) */
                        shmmap_state_set(&shmbuf->ROffset, 0);
                        shmmap_state_set(&shmbuf->wrapfactor, 0);
                    }
                } else if (W - 0 > HEADSZ) {
                    msg = SHMMAP_MSG_CAST(&shmbuf->Buffer[0]);
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
                        #ifdef SHMMAP_TRACE_ON
                            printf("(shmmap.h:%d) fatal code bug.\n", __LINE__);
                        #endif
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
                msg = SHMMAP_MSG_CAST(&shmbuf->Buffer[R]);
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
                    #ifdef SHMMAP_TRACE_ON
                        printf("(shmmap.h:%d) fatal code bug.\n", __LINE__);
                    #endif
                        return -1;
                    }
                } else {
                    /* fatal bug or invalid shared memory */
                    shmmap_state_set(&shmbuf->RLock, 0);
                #ifdef SHMMAP_TRACE_ON
                    printf("(shmmap.h:%d) fatal code bug.\n", __LINE__);
                #endif
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