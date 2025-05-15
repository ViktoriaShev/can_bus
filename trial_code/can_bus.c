#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <pty.h>
#include <utmp.h>
#include <dirent.h>
#include <ctype.h>
#include <sys/stat.h>
#include <termios.h>

#define FIRST_PORT 11
#define LAST_PORT  18
#define MAX_PKT    8

typedef struct {
    int key;                        // CAN ID
    int fd;                         // Файловый дескриптор для записи
} port_address;

int createCanSocket(const char* interface) {
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        perror("socket");
        return -1;
    }

    struct ifreq ifr;
    struct sockaddr_can addr;

    strncpy(ifr.ifr_name, interface, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        close(sock);
        return -1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sock);
        return -1;
    }

    return sock;
}

// Открыть и настроить порт /dev/ttyS<n>
static int open_port(int n) {
    char path[32];
    snprintf(path, sizeof(path), "/dev/ttyS%d", n);
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
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
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

void frame_process(struct can_frame* frame, port_address* setTty, int* count) {
    if (!frame || !setTty || !count) return;

    int can_id = frame->can_id;
    int short_id = can_id - 0x700;

    // Проверка допустимого диапазона ID
    if (short_id < 1 || short_id > (LAST_PORT - FIRST_PORT + 1)) {
        fprintf(stderr, "CAN ID 0x%X (short_id=%d) вне диапазона [%d..%d]\n",
                can_id, short_id, 0x701, 0x708);
        return;
    }

    // Поиск уже открытого порта
    for (int i = 0; i < *count; ++i) {
        if (setTty[i].key == short_id) {
        
            // Записываем строку в TTY
            ssize_t written = write(setTty[i].fd, frame->data, frame->can_dlc);

            if (written < 0) {
                perror("Ошибка записи в tty");
            } else {
                printf("CAN ID 0x%X → ttyS%d (fd=%d), %ld байт: ",
                       can_id, FIRST_PORT + short_id - 1, setTty[i].fd, written);
                for (int j = 0; j < frame->can_dlc; ++j) {
                    printf("%02X ", frame->data[j]);
                }
                printf("\n");
            }
            return;
        }
    }
    // Новый ID — открываем соответствующий порт
    int port_num = FIRST_PORT + short_id - 1;
    printf("Открываем ttyS%d для нового CAN ID 0x%X\n", port_num, can_id);

    int fd = open_port(port_num);
    if (fd < 0) {
        fprintf(stderr, "Не удалось открыть ttyS%d для CAN ID 0x%X\n", port_num, can_id);
        return;
    }

    // Очистка порта перед использованием
    tcflush(fd, TCIOFLUSH);

    setTty[*count].key = short_id;
    setTty[*count].fd = fd;
    (*count)++;

    // Записываем строку в новый TTY
    ssize_t written = write(fd, frame->data, frame->can_dlc);
    if (written < 0) {
        perror("Ошибка записи в новый tty");
    } else {
        printf("CAN ID 0x%X → новый ttyS%d (fd=%d), %ld байт: ",
               can_id, port_num, fd, written);
        for (int j = 0; j < frame->can_dlc; ++j) {
            printf("%02X ", frame->data[j]);
        }
        printf("\n");
    }
}

int main() {
    const char* can_interface = "can0";

    int sock = createCanSocket(can_interface);
    if (sock < 0) {
        return 1;
    }

    port_address setTty[MAX_PKT];  // Таблица tty-портов, заполняется по мере поступления ID
    int tty_count = 0;

    while (1) {
        struct can_frame frame;
        int nbytes = read(sock, &frame, sizeof(frame));

        if (nbytes < 0) {
            perror("read");
            break;
        } else if (nbytes < (int)sizeof(struct can_frame)) {
            fprintf(stderr, "Incomplete CAN frame\n");
            continue;
        } else if (nbytes == 0) {
            fprintf(stderr, "CAN buffer overflow detected!\n");
            continue;
        }

        if (frame.can_id >= 0x700 && frame.can_id <= 0x77F) {
            if (tty_count < MAX_PKT) {
                frame_process(&frame, setTty, &tty_count);
            } else {
                fprintf(stderr, "Достигнуто максимальное число tty (%d), ID 0x%X не будет обработан\n", MAX_PKT, frame.can_id);
            }
        }
        for (int i = 0; i < frame.can_dlc; ++i) {
            printf("%02X ", frame.data[i]);
        }
        printf("\n");
    }

    close(sock);
    for (int i = 0; i < tty_count; i++) {
        close(setTty[i].fd);
    }

    return 0;
}



