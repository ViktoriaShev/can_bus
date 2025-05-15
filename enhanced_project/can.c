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
#include <time.h>  // Добавляем для clock_gettime
#include <stdint.h>
#include <poll.h>
#include <pthread.h>

#define MAX_PKT 8
#define TIMEOUT_SECONDS 2

#define MAX_IDS 128  // Максимальное количество уникальных ID

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
    uint32_t product_code[MAX_PKT];
    int count;
} PrCodesSet;

typedef struct {
    uint32_t can_id[MAX_IDS];
    int fd[MAX_IDS];
    int count;
} PrCodeModulesSet;

static const PrCodesSet MODELS = {
    .product_code = {0x3,0x2},  // Остальные = 0 автоматически
    .count = 2
};

// Структура для передачи аргументов в поток
typedef struct {
    uint16_t can_id;
    int fd;
    const char* can_interface;
    pthread_mutex_t* write_mutex;
} ListenerArgs;

// Проверка, содержится ли ID уже в структуре
int idset_contains(const IDSet* set, uint16_t id) {
    for (int i = 0; i < set->count; ++i) {
        if (set->ids[i] == id) {
            return 1;
        }
    }
    return 0;
}

// Добавление ID, если он ещё не был добавлен
int idset_add(IDSet* set, uint16_t id) {
    if (idset_contains(set, id)) {
        return 0;  // Уже есть
    }
    if (set->count < MAX_IDS) {
        set->ids[set->count++] = id;
        return 1;
    }
    return 0;  // Переполнение
}

int create_can_socket(const char* interface) {
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

IDSet collect_can_ID(const char* can_interface) {

    int sock = create_can_socket(can_interface);
    if (sock < 0) {
        perror("socket");
        exit(1);
    }

    IDSet idSet = { .count = 0 };
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_nsec - start.tv_nsec) / 1e9;
        if (elapsed >= TIMEOUT_SECONDS) {
            printf("Таймер 2 секунды истёк. Завершаем.\n");
            break;
        }

        struct can_frame frame;
        int nbytes = read(sock, &frame, sizeof(frame));
        if (nbytes < 0) {
            perror("read");
            break;
        } else if (nbytes < (int)sizeof(struct can_frame)) {
            fprintf(stderr, " Incomplete CAN frame\n");
            continue;
        }

        if (frame.can_id >= 0x700 && frame.can_id <= 0x77F) {
            uint16_t current_id = frame.can_id - 0x700;
            if (idset_add(&idSet, current_id)) {
                printf("➕ Добавлен новый ID: 0x%X\n", current_id);
            } else {
                printf("ℹ  ID 0x%X уже присутствует или превышен лимит\n", current_id);
            }
        }

        printf(" Данные: ");
        for (int i = 0; i < frame.can_dlc; ++i) {
            printf("%02X ", frame.data[i]);
        }
        printf("\n");
    }
    close(sock);
    return idSet;
}

PrCodeCanIDSet collect_product_code(const char* can_interface, IDSet* idSet) {
    PrCodeCanIDSet result = {0};  // Инициализируем нулями

    int sock = create_can_socket(can_interface);
    if (sock < 0) {
        perror("createCanSocket");
        return result;
    }

    // Устанавливаем фильтр: принимать только ответы (0x580 - 0x5FF)
    struct can_filter filter;
    filter.can_id = 0x580;
    filter.can_mask = 0x7F0;
    if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
        perror("setsockopt (filter)");
        close(sock);
        return result;
    }

    for (int i = 0; i < idSet->count; ++i) {
        uint16_t node_id = idSet->ids[i];
        uint16_t request_id = 0x600 + node_id;

        struct can_frame req = {0};
        req.can_id = request_id;
        req.can_dlc = 8;
        req.data[0] = 0x40;      // SDO upload (read)
        req.data[1] = 0x18;      // Index 0x1018
        req.data[2] = 0x10;
        req.data[3] = 0x02;      // Sub-index 2 (Product code)
        // Остальные байты = 0

        if (write(sock, &req, sizeof(req)) != sizeof(req)) {
            perror("write");
            continue;
        }

        struct pollfd pfd = { .fd = sock, .events = POLLIN };
        int poll_ret = poll(&pfd, 1, 500);  // Ждём до 500 мс

        if (poll_ret <= 0) {
            printf("Нет ответа от 0x%X\n", node_id);
            continue;
        }

        struct can_frame resp;
        int nbytes = read(sock, &resp, sizeof(resp));
        if (nbytes < 0) {
            perror("read");
            continue;
        }

        if ((resp.can_id & 0x7F0) != 0x580) {
            printf("⚠ Неверный ID ответа: 0x%X\n", resp.can_id);
            continue;
        }

        uint32_t prod_code = resp.data[4]
                           | (resp.data[5] << 8)
                           | (resp.data[6] << 16)
                           | (resp.data[7] << 24);

        result.can_id[result.count] = node_id;
        result.product_code[result.count] = prod_code;
        result.count++;

        printf("✔ Product Code от 0x%X: 0x%08X\n", node_id, prod_code);
    }

    close(sock);
    return result;
}

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

PrCodeModulesSet open_tty(PrCodeCanIDSet* PrCodeSet) {
    int fd[8];
    PrCodeModulesSet result= {0};
    for (int j = 11; j-11 < (MODELS.count - 1);j++) {
        fd[j-11] = open_port(j);
    }

    for (int i=0; i< PrCodeSet->count;i++) {
        for (int j=0; j< MODELS.count;j++) {
            if (PrCodeSet->product_code[i] == MODELS.product_code[j]) {
                result.can_id[i] = PrCodeSet->can_id[i];
                result.fd[i] = fd[j];
                result.count++;
                printf("Собранные уникальные:\n %X, %X",PrCodeSet->product_code[i],MODELS.product_code[j]);

            }
        }
    }
    return result;
}

/*
void* listen_thread(void* arg) {
    ListenerArgs* args = (ListenerArgs*)arg;

    // Создаем socket CAN и биндимся к интерфейсу
    int sock = createCanSocket(args->can_interface);
    if (sock < 0) {
        perror("createCanSocket в потоке");
        return NULL;
    }

    // Устанавливаем фильтр на конкретный can_id
    struct can_filter filter;
    filter.can_id = args->can_id;
    filter.can_mask = CAN_SFF_MASK; // фильтр по точному ID

    if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
        perror("setsockopt filter");
        close(sock);
        return NULL;
    }

    while (1) {
        struct can_frame frame;
        int nbytes = read(sock, &frame, sizeof(frame));
        if (nbytes < 0) {
            perror("read в потоке");
            break;
        } else if (nbytes < (int)sizeof(struct can_frame)) {
            fprintf(stderr, "Incomplete CAN frame\n");
            continue;
        }

        pthread_mutex_lock(args->write_mutex);
        ssize_t written = write(args->fd, frame.data, frame.can_dlc);
        pthread_mutex_unlock(args->write_mutex);

        if (written < 0) {
            perror("write в tty");
            break;
        }

        printf("CAN ID 0x%X -> tty fd %d: ", frame.can_id, args->fd);
        for (int i = 0; i < frame.can_dlc; i++) {
            printf("%02X ", frame.data[i]);
        }
        printf("\n");
    }

    close(sock);
    return NULL;
}

int listenCan(PrCodeModulesSet* PrCodeSet) {
    pthread_t threads[MAX_IDS];
    ListenerArgs args[MAX_IDS];
    pthread_mutex_t fd_mutexes[MAX_IDS];
    int used_fds[MAX_IDS] = {0};
    int mutex_count = 0;
    int thread_count = 0;

    for (int i = 0; i < PrCodeSet->count; ++i) {
        pthread_mutex_t* mutex = NULL;
        for (int j = 0; j < mutex_count; ++j) {
            if (used_fds[j] == PrCodeSet->fd[i]) {
                mutex = &fd_mutexes[j];
                break;
            }
        }
        if (!mutex) {
            used_fds[mutex_count] = PrCodeSet->fd[i];
            mutex = &fd_mutexes[mutex_count];
            pthread_mutex_init(mutex, NULL);
            mutex_count++;
        }

        args[i].can_id = PrCodeSet->can_id[i] + 0x700;
        args[i].fd = PrCodeSet->fd[i];
        args[i].can_interface = CAN_INTERFACE;
        args[i].write_mutex = mutex;

        if (pthread_create(&threads[i], NULL, listen_thread, &args[i]) != 0) {
            perror("pthread_create");
        } else {
            thread_count++;
        }
    }

    for (int i = 0; i < thread_count; ++i) {
        pthread_join(threads[i], NULL);
    }

    return 0;
}
*/
int main() {
    const char* can_interface = "can0";

    IDSet idSet = collect_can_ID(can_interface);

    printf("Собранные уникальные CAN ID:\n");
    for (int i = 0; i < idSet.count; ++i) {
        printf(" - 0x%X\n", idSet.ids[i]);
    }

    PrCodeCanIDSet prCodeSet = collect_product_code(can_interface, &idSet);
    printf("Собранные уникальные  product code + can_id:\n");
    for (int i = 0; i < prCodeSet.count; ++i) {
        printf(" - 0x%X", prCodeSet.product_code[i]);
        printf(" - 0x%X\n", prCodeSet.can_id[i]);
    }

    PrCodeModulesSet prCodeModulesSet = open_tty(&prCodeSet);
    printf("Собранные уникальные fd + can_id: %d\n", prCodeModulesSet.count);
    for (int i = 0; i < prCodeModulesSet.count; ++i) {
        printf(" fd- 0x%d", prCodeModulesSet.fd[i]);
        printf(" - 0x%X\n", prCodeModulesSet.can_id[i]);
    }

    return 0;
}