/* create.c */
#include "sem.h"


int main(int argc, const char *argv[])
{
    semaphore_t *semap;

    semap = semaphore_create(DEFAULT_PROCESSED_SHARED_SEM);
    if (semap == NULL) {
        perror("semaphore_create failed");
        exit(1);
    }

    printf("semaphore create success: %s\n", DEFAULT_PROCESSED_SHARED_SEM);

    semaphore_close(semap);
    return (0);
}