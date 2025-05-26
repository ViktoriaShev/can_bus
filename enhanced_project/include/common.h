#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <linux/can.h>

#define MAX_PKT 8
#define TIMEOUT_SECONDS 2
#define FRAME_DATA_LEN 8
#define BUFFER_SIZE 64
#define MAX_IDS 128
#define MAX_TTY_LISTENERS 8
#define TTY_QUEUE_SIZE 256

typedef struct {
    uint16_t ids[MAX_IDS];
    int count;
} IDSet;

typedef struct {
    uint16_t can_id[MAX_IDS];
    uint32_t product_code[MAX_IDS];
    int count;
} PrCodeCanIDSet;

typedef struct {
    uint32_t can_id[MAX_IDS];
    int fd[MAX_IDS];
    int count;
} PrCodeModulesSet;

typedef struct {
    struct can_frame frame;
    int fd;
} FrameWithFD;

typedef struct {
    uint32_t product_code[MAX_PKT];
    int count;
} PrCodesSet;

extern const PrCodesSet MODELS;

#endif // COMMON_H
