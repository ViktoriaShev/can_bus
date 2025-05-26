#define _GNU_SOURCE
#include <termios.h>
#include "tty.h"

FrameQueue tty_queue;
extern const PrCodesSet MODELS;

int open_port(int n) {
    char path[32];
    snprintf(path, sizeof(path), "/dev/virt/ttyS%d", n);
    int fd = open(path, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror(path);
        return -1;
    }
    struct termios tty;
    if (tcgetattr(fd, &tty) < 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }
    // raw mode, 9600 8N1, без управления потоком
    cfmakeraw(&tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200)  ;
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tty) < 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }
    return fd;
}

PrCodeModulesSet open_tty(PrCodeCanIDSet* CodeSet) {
    PrCodeModulesSet result = {
        .count = 0
    };
    
    
    for (int i = 0; i < CodeSet->count; i++) {
        uint32_t current_product_code = CodeSet->product_code[i];
        uint16_t current_can_id = CodeSet->can_id[i];
        printf("pr code and can_id: %x, %X \n",current_product_code,current_can_id );
        int is_supported = 0;
        for (int j = 0; j < MODELS.count; j++) {
            if (current_product_code == MODELS.product_code[j]) {
                is_supported = 1;
                break;
            }
        }

        if (!is_supported) {
            printf("⚠ Пропущен неподдерживаемый Product Code: 0x%X (CAN ID: 0x%X)\n", current_product_code, current_can_id);
            continue;
        }

        // Открываем отдельный порт для каждого can_id

        int tty_index = 11 + result.count;
        printf("Открываю порт: /dev/ttyS%d для CAN ID 0x%X\n", tty_index, current_can_id);
        int fd = open_port(tty_index);
        printf("CAN ID 0x%X назначен к ttyS%d (fd=%d)\n", 
            result.can_id[result.count], 
            tty_index, fd);

        if (fd < 0) {
            fprintf(stderr, "Не удалось открыть порт для CAN ID 0x%X\n", current_can_id);
            continue;
        }

        result.can_id[result.count] = current_can_id;
        result.fd[result.count] = fd;
        result.count++;

        printf("✔ Назначен fd %d для CAN ID 0x%X (Product Code 0x%X)\n", fd, current_can_id, current_product_code);
    }

    return result;
}

void* tty_listener_thread(void* arg) {
    TTYListenerArgs* args = (TTYListenerArgs*)arg;
    printf("TTY(fd=%d, CAN ID=0x%X) -> Queue\n", args->fd, args->can_id);

    while (1) {
        uint8_t buf[8];
        ssize_t len = read(args->fd, buf, sizeof(buf));
        printf("READ %zd bytes from fd %d\n", len, args->fd);
        if (len <= 0) continue;

        FrameWithFD frame = {
            .fd = args->fd,
            .frame = {}
        };
        frame.frame.can_id = args->can_id;
        frame.frame.can_dlc = len;
        memcpy(frame.frame.data, buf, len);

        queue_push(args->queue, &frame);
    }

    free(arg);  // если поток завершится (что редко, но возможно)
    return NULL;
}

void* tty_to_can_thread(void* arg) {
    FrameQueue* queue = (FrameQueue*)arg;
    int sock = create_can_socket("can0");
    if (sock < 0) {
        perror("CAN socket");
        return NULL;
    }

    while (1) {
        FrameWithFD item;
        queue_pop(queue, &item);

        struct can_frame frame = {};
        frame.can_id = 0x200 + item.frame.can_id;
        frame.can_dlc = item.frame.can_dlc;

        printf("tty_to_can fd=%d\n",item.fd);

        memcpy(frame.data, item.frame.data, frame.can_dlc);
        printf("Sending to CAN bus: ID=0x%X, DLC=%d, Data=", frame.can_id, frame.can_dlc);
        for (int i = 0; i < frame.can_dlc; ++i) {
            printf("%02X ", frame.data[i]);
        }
        printf("\n");

        if (write(sock, &frame, sizeof(frame)) < 0) {
            perror("CAN write");
        } else {
            printf("Queue -> CAN(0x%X): %d bytes\n", frame.can_id, frame.can_dlc);
        }
    }

    close(sock);
    return NULL;
}
