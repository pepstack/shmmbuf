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
 * @update     2020-05-08 16:10:10
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

#ifndef __STDC_FORMAT_MACROS
# define __STDC_FORMAT_MACROS
#endif
#include <inttypes.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>


/**
 * Prior to include this file, define as following to enable
 *  trace print in spite of speed penalty.
 *
 * #define SHMMAP_TRACE_PRINT_ON
 * #include "shmmap.h"
 */


#ifndef NOWARNING_UNUSED
    # if defined(__GNUC__) || defined(__CYGWIN__)
        # define NOWARNING_UNUSED(x) __attribute__((unused)) x
    # else
        # define NOWARNING_UNUSED(x) x
    # endif
#endif


/**
 * Default constants only show you the usage for shmmap api.
 *
 * Shared memory file lies in: "/dev/shm/shmmap-buffer"
 */
#define SHMMAP_FILENAME_DEFAULT        "shmmap-buffer"

#define SHMMAP_FILEMODE_DEFAULT        0666

#define SHMMAP_PAGE_SIZE               4096


/**
 * Below definitions SHOULD NOT be changed!
 */
#define SHMMAP_INVALID_STATE           ((size_t)(-1))
#define SHMMAP_SIZEOF_ENTRY            (sizeof(shmmap_entry_t))

#define SHMMAP_ALIGN_BSIZE(bsz)  \
    ((size_t)((((size_t)(bsz)+SHMMAP_SIZEOF_ENTRY-1)/SHMMAP_SIZEOF_ENTRY)*SHMMAP_SIZEOF_ENTRY))

#define SHMMAP_ALIGN_ENTRYSZ(bsz)      SHMMAP_ALIGN_BSIZE((bsz) + SHMMAP_SIZEOF_ENTRY)

#define SHMMAP_ENTRY_CAST(p)           ((shmmap_entry_t *)(p))

#define SHMMAP_VERIFY_STATE(val)    do { \
        if ((val) == SHMMAP_INVALID_STATE) { \
            exit(EXIT_FAILURE); \
        } \
    } while(0)


/**
 * Returns of shmmap_buffer_write()
 */
#define SHMMAP_WRITE_SUCCESS     ((int)(1))
#define SHMMAP_WRITE_AGAIN       ((int)(0))
#define SHMMAP_WRITE_FATAL       ((int)(-1))


/**
 * Returns of shmmap_buffer_read_? ()
 */
#define SHMMAP_READ_NEXT         ((int)(1))
#define SHMMAP_READ_AGAIN        ((int)(0))
#define SHMMAP_READ_FATAL        ((int)(-1))


/**
 * The layout of memory for any one entry in shmmap.
 */
typedef struct _shmmap_entry_t
{
    size_t size;
    char chunk[0];
} shmmap_entry_t;


/**
 * The atomic struct for state of shmmap.
 */
typedef struct _shmmap_state_t
{
    /* atomic state value */
    size_t state;

    /* process-wide state lock */
    pthread_mutex_t mutex;
} shmmap_state_t;


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
    size_t val = SHMMAP_INVALID_STATE;
    int err = pthread_mutex_lock(&st->mutex);

    if (! err) {
        val = st->state;
        pthread_mutex_unlock(&st->mutex);
        return val;
    } else if (err == EOWNERDEAD) {
        shmmap_mutex_consistent(&st->mutex);
        return shmmap_state_get(st);
    }

    printf("pthread_mutex_lock fatal(%d): %s.\n", err, strerror(err));
    SHMMAP_VERIFY_STATE(val);
    return val;
}


NOWARNING_UNUSED(static)
size_t shmmap_state_set (shmmap_state_t *st, size_t newval)
{
    size_t oldval = SHMMAP_INVALID_STATE;
    int err = pthread_mutex_lock(&st->mutex);

    if (! err) {
        oldval = st->state;
        st->state = newval;
        pthread_mutex_unlock(&st->mutex);
        return oldval;
    } else if (err == EOWNERDEAD) {
        shmmap_mutex_consistent(&st->mutex);
        return shmmap_state_set(st, newval);
    }

    printf("pthread_mutex_lock fatal(%d): %s.\n", err, strerror(err));
    SHMMAP_VERIFY_STATE(oldval);
    return oldval;
}


NOWARNING_UNUSED(static)
size_t shmmap_state_comp_exch (shmmap_state_t *st, size_t comp, size_t exch)
{
    size_t oldval = SHMMAP_INVALID_STATE;
    int err = pthread_mutex_lock(&st->mutex);

    if (! err) {
        oldval = st->state;
        if (st->state == comp) {
            st->state = exch;
        }
        pthread_mutex_unlock(&st->mutex);
        return oldval;
    } else if (err == EOWNERDEAD) {
        shmmap_mutex_consistent(&st->mutex);
        return shmmap_state_comp_exch(st, comp, exch);
    }

    printf("pthread_mutex_lock fatal(%d): %s.\n", err, strerror(err));
    SHMMAP_VERIFY_STATE(oldval);
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


typedef struct _shmmap_buffer_t
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
    size_t Length;

    /* ring buffer in shared memory with Length */
    char Buffer[0];
} shmmap_buffer_t;


NOWARNING_UNUSED(static)
void shmmap_buffer_close (shmmap_buffer_t *shmbuf)
{
    size_t bsize = shmbuf->Length;
    munmap(shmbuf, sizeof(shmmap_buffer_t) + bsize);
}


NOWARNING_UNUSED(static)
int shmmap_buffer_delete (const char *shmfilename)
{
    return shm_unlink(shmfilename);
}

#define SHMMAP_CHECK_ERR(err)  if (err) goto error_exit


NOWARNING_UNUSED(static)
shmmap_buffer_t * shmmap_buffer_create (const char *shmfilename, mode_t filemode, size_t maxbufsize)
{
    int err, mfd;
    
    shmmap_buffer_t *shmbuf;

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
    mfdSize = sizeof(shmmap_buffer_t) + rbLength;

    err = ftruncate(mfd, mfdSize);
    if (err) {
        perror("ftruncate");
        close(mfd);
        return (NULL);
    }

    shmbuf = (shmmap_buffer_t *) mmap(NULL, mfdSize, PROT_READ|PROT_WRITE, MAP_SHARED, mfd, 0);
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

    shmmap_buffer_close(shmbuf);
    shmmap_buffer_delete(shmfilename);
    return NULL;
}


/**
 * shmmap_buffer_write()
 *   Write chunk data of entry into shmmap ring buffer. 
 *
 * Returns:
 *   SHMMAP_WRITE_SUCCESS(1) - write success
 *   SHMMAP_WRITE_AGAIN(0)   - write again
 *   SHMMAP_WRITE_FATAL(-1)  - fatal write error
 */
NOWARNING_UNUSED(static)
int shmmap_buffer_write (shmmap_buffer_t *shmbuf, const void *chunk, size_t chunksz)
{
    shmmap_entry_t *entry;
    ssize_t W, R, wrap,
            L = (ssize_t)shmbuf->Length,
            AENTSZ = (ssize_t)SHMMAP_ALIGN_ENTRYSZ(chunksz);

    if (! AENTSZ || AENTSZ == SHMMAP_INVALID_STATE || AENTSZ > (L / SHMMAP_SIZEOF_ENTRY)) {
        /* fatal error should not occurred! */
    # ifdef SHMMAP_TRACE_PRINT_ON
            printf("(shmmap.h:%d) fatal error: invalid chunksz(=%" PRIu64").\n", __LINE__, chunksz);
    # endif
        return SHMMAP_WRITE_FATAL;
    }

    if (! shmmap_state_comp_exch(&shmbuf->WLock, 0, 1)) {
        /* 1st get wrap */
        wrap = shmmap_state_get(&shmbuf->wrapfactor);

        /* 2nd get ROffset */
        R = shmmap_state_get(&shmbuf->ROffset);

        /* 3rd copy WOffset without lock */
        W = shmbuf->WOffset.state;

    # ifdef SHMMAP_TRACE_PRINT_ON
        if (wrap) {
            printf("(shmmap.h:%d) shmmap_buffer_write(%" PRIu64":%" PRIu64"): W=%" PRId64" R=%" PRId64" L=%" PRId64"\n",
                __LINE__, chunksz, AENTSZ, W, R, L);
        } else {
            printf("(shmmap.h:%d) shmmap_buffer_write(%" PRIu64":%" PRIu64"): R=%" PRId64" W=%" PRId64" L=%" PRId64"\n",
                __LINE__, chunksz, AENTSZ, R, W, L);
        }
    # endif

        /* Sw = L - (wrap*L + W - R) */
        if (L - (wrap*L + W - R) >= AENTSZ) {
            if (wrap) { /* 0 .. W < R < L */
                entry = SHMMAP_ENTRY_CAST(&shmbuf->Buffer[W]);
                entry->size = chunksz;
                memcpy(entry->chunk, chunk, chunksz);

                /* WOffset = W + AENTSZ */
                shmmap_state_set(&shmbuf->WOffset, W + AENTSZ);
            } else { /* 0 .. R < W < L */
                if (L - W >= AENTSZ) {
                    entry = SHMMAP_ENTRY_CAST(&shmbuf->Buffer[W]);
                    entry->size = chunksz;
                    memcpy(entry->chunk, chunk, chunksz);

                    /* WOffset = W + AENTSZ */
                    shmmap_state_set(&shmbuf->WOffset, W + AENTSZ);
                } else if (R - 0 >= AENTSZ) {
                    /* clear W slot before wrap W */
                    bzero(&shmbuf->Buffer[W], L - W); 

                    /* wrap W to 0 */
                    entry = SHMMAP_ENTRY_CAST(&shmbuf->Buffer[0]);
                    entry->size = chunksz;
                    memcpy(entry->chunk, chunk, chunksz);

                    /* WOffset = AENTSZ */
                    shmmap_state_set(&shmbuf->WOffset, AENTSZ);

                    /* set wrap: wrap = 1 */
                    shmmap_state_set(&shmbuf->wrapfactor, 1);
                } else {
                    /* no space left to write, expect to call again */
                    shmmap_state_set(&shmbuf->WLock, 0);
                    return SHMMAP_WRITE_AGAIN;
                }
            }

            shmmap_state_set(&shmbuf->WLock, 0);
            return SHMMAP_WRITE_SUCCESS;
        }

        /* no space left to write */
        shmmap_state_set(&shmbuf->WLock, 0);
        return SHMMAP_WRITE_AGAIN;
    }

    /* lock fail to write, expect to call again */
    return SHMMAP_WRITE_AGAIN;
}


/**
 * shmmap_buffer_read_copy()
 *   Copy entry from shmmap ringbuffer into rdbuf.
 *
 * returns:
 *   SHMMAP_READ_AGAIN(0)       - read again
 *   SHMMAP_READ_FATAL(-1)      - fatal read error
 *
 *   ret > 0 and ret <= rdbufsz - read success
 *   ret > rdbufsz              - no read for insufficient buffer
 */
NOWARNING_UNUSED(static)
size_t shmmap_buffer_read_copy (shmmap_buffer_t *shmbuf, char *rdbuf, size_t rdbufsz)
{
    shmmap_entry_t *entry;

    ssize_t R, W, wrap, entsize, AENTSZ, 
            L = (ssize_t)shmbuf->Length,
            HENTSZ = (ssize_t)SHMMAP_ALIGN_ENTRYSZ(0);

    if (! shmmap_state_comp_exch(&shmbuf->RLock, 0, 1)) {
        /* 1st get wrap */
        wrap = shmmap_state_get(&shmbuf->wrapfactor);

        /* 2nd get ROffset */
        W = shmmap_state_get(&shmbuf->WOffset);

        /* 3rd copy WOffset without lock */
        R = shmbuf->ROffset.state;

    # ifdef SHMMAP_TRACE_PRINT_ON
        if (wrap) {
            printf("(shmmap.h:%d) shmmap_buffer_read_copy(): W=%" PRId64" R=%" PRId64" L=%" PRId64"\n", __LINE__, W, R, L);
        } else {
            printf("(shmmap.h:%d) shmmap_buffer_read_copy(): R=%" PRId64" W=%" PRId64" L=%" PRId64"\n", __LINE__, R, W, L);
        }
    # endif

        /* Sr = f*L + W - R */
        if (wrap*L + W - R > HENTSZ) {
            if (wrap) {  /* 0 .. W < R < L */
                if (L - R > HENTSZ) {
                    entry = SHMMAP_ENTRY_CAST(&shmbuf->Buffer[R]);
                    if (entry->size) {
                        AENTSZ = SHMMAP_ALIGN_ENTRYSZ(entry->size);

                        if (L - R >= AENTSZ) {
                            entsize = entry->size;
                            if (entsize > rdbufsz) {
                                /* read buf is insufficient for entry.
                                 * read nothing if returned entsize > rdbufsz */
                                shmmap_state_set(&shmbuf->RLock, 0);
                                return (size_t)entsize;
                            }

                            /* read entry chunk into rdbuf ok */
                            memcpy(rdbuf, entry->chunk, entsize);

                            shmmap_state_set(&shmbuf->ROffset, R + AENTSZ);

                            /* read success if returned entsize <= rdbufsz */
                            shmmap_state_set(&shmbuf->RLock, 0);
                            return (size_t)entsize;
                        }

                        printf("(shmmap.h:%d) SHOULD NEVER RUN TO THIS! fatal bug.\n", __LINE__);
                        shmmap_state_set(&shmbuf->RLock, 0);
                        return SHMMAP_READ_FATAL;
                    } else {
                        /* reset ROffset to 0 (set wrap = 0) */
                        shmmap_state_set(&shmbuf->ROffset, 0);
                        shmmap_state_set(&shmbuf->wrapfactor, 0);

                        /* expect to read again */
                        shmmap_state_set(&shmbuf->RLock, 0);

                        return shmmap_buffer_read_copy(shmbuf, rdbuf, rdbufsz);
                    }
                } else if (W - 0 > HENTSZ) { 
                    /* reset ROffset to 0 */
                    entry = SHMMAP_ENTRY_CAST(&shmbuf->Buffer[0]);
                    if (entry->size) {
                        AENTSZ = SHMMAP_ALIGN_ENTRYSZ(entry->size);

                        if (W - 0 >= AENTSZ) {
                            entsize = entry->size;
                            if (entsize > rdbufsz) {
                                /* read buf is insufficient for entry.
                                 * read nothing if returned entsize > rdbufsz */
                                shmmap_state_set(&shmbuf->RLock, 0);
                                return (size_t)entsize;
                            }

                            /* read entry chunk into rdbuf ok */
                            memcpy(rdbuf, entry->chunk, entsize);

                            shmmap_state_set(&shmbuf->ROffset, AENTSZ);
                            shmmap_state_set(&shmbuf->wrapfactor, 0);

                            /* read success if returned entsize <= rdbufsz */
                            shmmap_state_set(&shmbuf->RLock, 0);
                            return (size_t)entsize;
                        }
                    }
                    
                    printf("(shmmap.h:%d) SHOULD NEVER RUN TO THIS! fatal bug.\n", __LINE__);
                    shmmap_state_set(&shmbuf->RLock, 0);
                    return SHMMAP_READ_FATAL;
                }
            } else {  /* 0 .. R < W < L */
                entry = SHMMAP_ENTRY_CAST(&shmbuf->Buffer[R]);
                if (entry->size) {
                    AENTSZ = SHMMAP_ALIGN_ENTRYSZ(entry->size);

                    if (W - R >= AENTSZ) {
                        entsize = entry->size;
                        if (entsize > rdbufsz) {
                            /* read buf is insufficient for entry.
                             * read nothing if returned entsize > rdbufsz */
                            shmmap_state_set(&shmbuf->RLock, 0);
                            return (size_t)entsize;
                        }

                        /* read entry chunk into rdbuf ok */
                        memcpy(rdbuf, entry->chunk, entsize);

                        shmmap_state_set(&shmbuf->ROffset, R + AENTSZ);

                        /* read success if returned entsize <= rdbufsz */
                        shmmap_state_set(&shmbuf->RLock, 0);
                        return (size_t)entsize;
                    }
                }

                printf("(shmmap.h:%d) SHOULD NEVER RUN TO THIS! fatal bug.\n", __LINE__);
                shmmap_state_set(&shmbuf->RLock, 0);
                return SHMMAP_READ_FATAL;
            }
        }

        /* no entry to read, retry again */
        shmmap_state_set(&shmbuf->RLock, 0);
        return SHMMAP_READ_AGAIN;
    }

    /* read locked fail, retry again */
    return SHMMAP_READ_AGAIN;
}


/**
 * shmmap_buffer_read_nextcb()
 *   Read next entry from shmmap ring buffer into callback (no copy data).
 *
 * params:
 *   nextentry_cb() - Callback implemented by caller should ONLY
 *                     return SHMMAP_READ_NEXT(1) or SHMMAP_READ_AGAIN(0)
 *                    DO NOT change and members of entry in nextentry_cb().
 * returns:
 *   SHMMAP_READ_NEXT(1)    - read for next one
 *   SHMMAP_READ_AGAIN(0)   - read current one again
 *   SHMMAP_READ_FATAL(-1)  - fatal read error
 */
NOWARNING_UNUSED(static)
int shmmap_buffer_read_nextcb (shmmap_buffer_t *shmbuf, int (*nextentry_cb)(const shmmap_entry_t *entry, void *arg), void *arg)
{
    shmmap_entry_t *entry;

    ssize_t R, W, wrap, AENTSZ, 
            L = (ssize_t)shmbuf->Length,
            HENTSZ = (ssize_t)SHMMAP_ALIGN_ENTRYSZ(0);

    if (! shmmap_state_comp_exch(&shmbuf->RLock, 0, 1)) {
        /* 1st get wrap */
        wrap = shmmap_state_get(&shmbuf->wrapfactor);

        /* 2nd get ROffset */
        W = shmmap_state_get(&shmbuf->WOffset);

        /* 3rd copy WOffset without lock */
        R = shmbuf->ROffset.state;

    # ifdef SHMMAP_TRACE_PRINT_ON
        if (wrap) {
            printf("(shmmap.h:%d) shmmap_buffer_read_copy(): W=%" PRId64" R=%" PRId64" L=%" PRId64"\n", __LINE__, W, R, L);
        } else {
            printf("(shmmap.h:%d) shmmap_buffer_read_copy(): R=%" PRId64" W=%" PRId64" L=%" PRId64"\n", __LINE__, R, W, L);
        }
    # endif

        /* Sr = f*L + W - R */
        if (wrap*L + W - R > HENTSZ) {
            if (wrap) {  /* 0 .. W < R < L */
                if (L - R > HENTSZ) {
                    entry = SHMMAP_ENTRY_CAST(&shmbuf->Buffer[R]);
                    if (entry->size) {
                        AENTSZ = SHMMAP_ALIGN_ENTRYSZ(entry->size);

                        if (L - R >= AENTSZ) {
                            if (nextentry_cb(entry, arg)) {
                                /* read success and set ROffset to next entry */
                                shmmap_state_set(&shmbuf->ROffset, R + AENTSZ);
                                shmmap_state_set(&shmbuf->RLock, 0);
                                return SHMMAP_READ_NEXT;
                            } else {
                                /* read paused by caller */
                                shmmap_state_set(&shmbuf->RLock, 0);
                                return SHMMAP_READ_AGAIN;
                            }
                        }

                        printf("(shmmap.h:%d) SHOULD NEVER RUN TO THIS! fatal bug.\n", __LINE__);
                        shmmap_state_set(&shmbuf->RLock, 0);
                        return SHMMAP_READ_FATAL;
                    } else {
                        /* reset ROffset to 0 (set wrap = 0) */
                        shmmap_state_set(&shmbuf->ROffset, 0);
                        shmmap_state_set(&shmbuf->wrapfactor, 0);

                        /* expect to read again */
                        shmmap_state_set(&shmbuf->RLock, 0);

                        return shmmap_buffer_read_nextcb(shmbuf, nextentry_cb, arg);
                    }
                } else if (W - 0 > HENTSZ) { 
                    /* reset ROffset to 0 */
                    entry = SHMMAP_ENTRY_CAST(&shmbuf->Buffer[0]);
                    if (entry->size) {
                        AENTSZ = SHMMAP_ALIGN_ENTRYSZ(entry->size);

                        if (W - 0 >= AENTSZ) {
                            if (nextentry_cb(entry, arg)) {
                                /* read success and set ROffset to next entry */
                                shmmap_state_set(&shmbuf->ROffset, AENTSZ);
                                shmmap_state_set(&shmbuf->wrapfactor, 0);
                                shmmap_state_set(&shmbuf->RLock, 0);
                                return SHMMAP_READ_NEXT;
                            } else {
                                /* read paused by caller */
                                shmmap_state_set(&shmbuf->RLock, 0);
                                return SHMMAP_READ_AGAIN;
                            }
                        }
                    }
                    
                    printf("(shmmap.h:%d) SHOULD NEVER RUN TO THIS! fatal bug.\n", __LINE__);
                    shmmap_state_set(&shmbuf->RLock, 0);
                    return SHMMAP_READ_FATAL;
                }
            } else {  /* 0 .. R < W < L */
                entry = SHMMAP_ENTRY_CAST(&shmbuf->Buffer[R]);
                if (entry->size) {
                    AENTSZ = SHMMAP_ALIGN_ENTRYSZ(entry->size);

                    if (W - R >= AENTSZ) {
                        if (nextentry_cb(entry, arg)) {
                            /* read success and set ROffset to next entry */
                            shmmap_state_set(&shmbuf->ROffset, R + AENTSZ);
                            shmmap_state_set(&shmbuf->RLock, 0);
                            return SHMMAP_READ_NEXT;
                        } else {
                            /* read paused by caller */
                            shmmap_state_set(&shmbuf->RLock, 0);
                            return SHMMAP_READ_AGAIN;
                        }
                    }
                }

                printf("(shmmap.h:%d) SHOULD NEVER RUN TO THIS! fatal bug.\n", __LINE__);
                shmmap_state_set(&shmbuf->RLock, 0);
                return SHMMAP_READ_FATAL;
            }
        }

        /* no entry to read, retry again */
        shmmap_state_set(&shmbuf->RLock, 0);
        return SHMMAP_READ_AGAIN;
    }

    /* read locked fail, retry again */
    return SHMMAP_READ_AGAIN;
}

#ifdef __cplusplus
}
#endif
#endif /* SHMMAP_H__ */