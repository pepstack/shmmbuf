/* post.c */
#include "sem.h"

int main(int argc, const char *argv[])
{
    semaphore_t *semap;

    semap = semaphore_open(DEFAULT_PROCESSED_SHARED_SEM);
    if (semap == NULL) {
        perror("semaphore_create");
        exit(1);
    }

    semaphore_post(semap);

    printf("semaphore post: %s\n", DEFAULT_PROCESSED_SHARED_SEM);

    semaphore_close(semap);
    return (0);
}