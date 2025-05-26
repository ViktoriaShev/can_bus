#ifndef QUEUE_H
#define QUEUE_H

#include <pthread.h>
#include <stdint.h>
#include "common.h"

typedef struct {
    FrameWithFD buffer[BUFFER_SIZE];
    int head;
    int tail;
    int count;
    pthread_mutex_t mutex;
    pthread_cond_t not_empty;
    pthread_cond_t not_full;
} FrameQueue;

typedef struct {
    int fd;
    uint16_t can_id;
    FrameQueue* queue;
} TTYListenerArgs;

void queue_init(FrameQueue* q);
void queue_push(FrameQueue* q, const FrameWithFD* item);
void queue_pop(FrameQueue* q, FrameWithFD* item);

#endif // QUEUE_H
