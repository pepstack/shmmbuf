/**
 * sem.h
 *
 * The following code is for three separate processes that
 *  create, post, and wait on a semaphore in the file:
 *    /tmp/shm-semaphore
 *
 * Once the file is created, the post and wait programs
 *  increment and decrement the counting semaphore (waiting
 *  and waking as required) even though they did not
 *  initialize the semaphore. 
 *
 * https://linux.die.net/man/3/pthread_mutexattr_init
 */
#ifndef SEM_H__
#define SEM_H__

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>


#define DEFAULT_PROCESSED_SHARED_SEM   "/tmp/shm-semaphore"


typedef struct
{
    pthread_mutex_t lock;
    pthread_cond_t nonzero;
    size_t count;
} semaphore_t;

semaphore_t *semaphore_create(char *semaphore_name);
semaphore_t *semaphore_open(char *semaphore_name);

void semaphore_post(semaphore_t *semap);
void semaphore_wait(semaphore_t *semap);
void semaphore_close(semaphore_t *semap);


#endif /* SEM_H__ */