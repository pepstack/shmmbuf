/* wait.c */
#include "sem.h"
#include <stdio.h>

int main(int argc, const char *argv[])
{
    semaphore_t *semap;

    semap = semaphore_open(DEFAULT_PROCESSED_SHARED_SEM);
    if (semap == NULL) {
        perror("semaphore_open");
        exit(1);
    }

    printf("block waiting semaphore: %s\n", DEFAULT_PROCESSED_SHARED_SEM);

    semaphore_wait(semap);

    printf("do something...\n");

    semaphore_close(semap);

    return (0);
}