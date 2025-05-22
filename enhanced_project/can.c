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
#define FRAME_DATA_LEN 8
#define BUFFER_SIZE 64

#define MAX_IDS 128  // Максимальное количество уникальных ID
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
    uint32_t product_code[MAX_PKT];
    int count;
} PrCodesSet;

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

FrameQueue queue;
FrameQueue tty_queue;

static const PrCodesSet MODELS = {
    .product_code = {0x3,0x2},  // Остальные = 0 автоматически
    .count = 2
};

// Функция сравнения (можно вынести выше)
int compare_uint16(const void* a, const void* b) {
    uint16_t ua = *(const uint16_t*)a;
    uint16_t ub = *(const uint16_t*)b;
    return (ua > ub) - (ua < ub);
}

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

    }
    qsort(idSet.ids, idSet.count, sizeof(uint16_t), 
          (int (*)(const void*, const void*))compare_uint16);

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
    for (int i = 0; i < result.count - 1; i++) {
        for (int j = 0; j < result.count - i - 1; j++) {
            if (result.can_id[j] > result.can_id[j + 1]) {
                uint16_t temp_id = result.can_id[j];
                result.can_id[j] = result.can_id[j + 1];
                result.can_id[j + 1] = temp_id;
                uint32_t temp_code = result.product_code[j];
                result.product_code[j] = result.product_code[j + 1];
                result.product_code[j + 1] = temp_code;
            }
        }
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
    PrCodeModulesSet result = {0};

    for (int i = 0; i < PrCodeSet->count; i++) {
        uint32_t current_product_code = PrCodeSet->product_code[i];
        uint16_t current_can_id = PrCodeSet->can_id[i];

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


void queue_init(FrameQueue* q) {
    q->head = q->tail = q->count = 0;
    pthread_mutex_init(&q->mutex, NULL);
    pthread_cond_init(&q->not_empty, NULL);
    pthread_cond_init(&q->not_full, NULL);
}

void queue_push(FrameQueue* q, const FrameWithFD* item) {
    pthread_mutex_lock(&q->mutex);
    while (q->count == BUFFER_SIZE) {
        pthread_cond_wait(&q->not_full, &q->mutex);
    }

    q->buffer[q->tail] = *item;
    q->tail = (q->tail + 1) % BUFFER_SIZE;
    q->count++;

    pthread_cond_signal(&q->not_empty);
    pthread_mutex_unlock(&q->mutex);
}

void queue_pop(FrameQueue* q, FrameWithFD* item) {
    pthread_mutex_lock(&q->mutex);
    while (q->count == 0) {
        pthread_cond_wait(&q->not_empty, &q->mutex);
    }

    *item = q->buffer[q->head];
    q->head = (q->head + 1) % BUFFER_SIZE;
    q->count--;

    pthread_cond_signal(&q->not_full);
    pthread_mutex_unlock(&q->mutex);
}

void* listen_can(void* arg) {
    printf("start listening...\n");
    PrCodeModulesSet* args = (PrCodeModulesSet*)arg; 
    int sock = create_can_socket("can0");
    if (sock < 0) {
        perror("createCanSocket в потоке");
        return NULL;
    }

    struct can_filter filters[MAX_IDS];
    for (int i = 0; i < args->count; ++i) {
        filters[i].can_id  = args->can_id[i];
        filters[i].can_mask = 0x7F;               // сравнивать только младшие 7 бит
    }

    if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER,
                   filters, sizeof(struct can_filter) * args->count) < 0) {
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

        uint16_t func_code = frame.can_id & ~0x7F;
        if (func_code != 0x180) {
            continue;  // Пропустить нежеланный кадр
        }

        FrameWithFD item;
        item.frame = frame;

        item.fd = -1;

        for (int i = 0; i < args->count; ++i) {
            if ((frame.can_id & 0x7F) == args->can_id[i] ) {
                item.fd = args->fd[i];
                break;
            }
        }
        if (item.fd != -1 && nbytes == sizeof(struct can_frame)) {
            queue_push(&queue, &item);
        } else {
            fprintf(stderr, "Не найден fd для CAN ID 0x%X\n", frame.can_id);
        }
    

        printf("CAN ID 0x%X -> tty fd %d: ", frame.can_id, item.fd);

        for (int i = 0; i < frame.can_dlc; i++) {
            printf("%02X ", frame.data[i]);
        }
        printf("\n");
    }

    close(sock);
    return NULL;
}

void* process_data(void* arg) {
    printf("start writing...\n");
    FrameQueue* queue = (FrameQueue*)arg;

    while (1) {

        FrameWithFD item;
        queue_pop(queue, &item);
        printf("Обработка CAN ID 0x%X -> tty fd %d: ", item.frame.can_id, item.fd);
        for (int i = 0; i < item.frame.can_dlc; ++i) {
            printf("%02X ", item.frame.data[i]);
        }
        printf("\n");
        unsigned char data[item.frame.can_dlc];

        for (int i=0; i < item.frame.can_dlc;i++) {

            data[i]=(char)item.frame.data[i];
            printf("данные: %02X %02X %d\n", data[i],item.frame.data[i], item.fd );

        }
        if (item.fd != -1) {
            // TODO: обработка frame.data[]
            ssize_t written = write(item.fd, data, item.frame.can_dlc);
            if (written < 0) {
                perror("Ошибка записи в tty");
            }
        }
    }

    return NULL;
}

void* tty_listener_thread(void* arg) {
    TTYListenerArgs* args = (TTYListenerArgs*)arg;
    printf("TTY(fd=%d, CAN ID=0x%X) -> Queue: %d bytes\n", args->fd, args->can_id);
    while (1) {
        uint8_t buf[8];
        ssize_t len = read(args->fd, buf, sizeof(buf));
        printf("READ %zd bytes from fd %d\n", len, args->fd);
        if (len <= 0) continue;

        FrameWithFD frame = {
            .fd = args->fd,
            .frame = {0}
        };
        frame.frame.can_id = args->can_id;
        frame.frame.can_dlc = len;
        memcpy(frame.frame.data, buf, len);

        queue_push(args->queue, &frame);

        printf("TTY(fd=%d, CAN ID=0x%X) -> Queue: %d bytes\n", args->fd, args->can_id, (int)len);
    }

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

        struct can_frame frame = {0};
        frame.can_id = 0x200 + item.frame.can_id;
        frame.can_dlc = item.frame.can_dlc;
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

int main() {
    const char* can_interface = "can0";
    queue_init(&queue);
    queue_init(&tty_queue);

    IDSet idSet = collect_can_ID(can_interface);

    printf("Собранные уникальные CAN ID:\n");
    for (int i = 0; i < idSet.count; i++) {
        printf(" - 0x%X\n", idSet.ids[i]);
    }

    PrCodeCanIDSet prCodeSet = collect_product_code(can_interface, &idSet);
    printf("Собранные уникальные  product code + can_id:\n");
    for (int i = 0; i < prCodeSet.count; i++) {
        printf(" - 0x%X", prCodeSet.product_code[i]);
        printf(" - 0x%X\n", prCodeSet.can_id[i]);
    }

    PrCodeModulesSet prCodeModulesSet = open_tty(&prCodeSet);
    printf("Собранные уникальные fd + can_id: %d\n", prCodeModulesSet.count);
    for (int i = 0; i < prCodeModulesSet.count; i++) {
        printf(" fd- 0x%d", prCodeModulesSet.fd[i]);
        printf(" - 0x%X\n", prCodeModulesSet.can_id[i]);
    }
    printf("creating threads:\n");
    pthread_t listener_thread, processor_thread;
    printf("creating threads:\n");
    int res1 = pthread_create(&listener_thread, NULL, listen_can, &prCodeModulesSet);
    int res2 = pthread_create(&processor_thread, NULL, process_data, &queue);
    
    if (res1 != 0) {
        fprintf(stderr, "Ошибка запуска потока listen_can: %s\n", strerror(res1));
    }
    if (res2 != 0) {
        fprintf(stderr, "Ошибка запуска потока process_data: %s\n", strerror(res2));
    }

    pthread_t tty_threads[MAX_TTY_LISTENERS];
    TTYListenerArgs tty_args[MAX_TTY_LISTENERS];

    for (int i = 0; i < prCodeModulesSet.count; i++) {
        tty_args[i].fd = prCodeModulesSet.fd[i];
        tty_args[i].can_id = prCodeModulesSet.can_id[i];
        tty_args[i].queue = &tty_queue;

        pthread_create(&tty_threads[i], NULL, tty_listener_thread, &tty_args[i]);
        printf(" TTY listener thread запущен для fd=%d (CAN ID=0x%X)\n",
            tty_args[i].fd, tty_args[i].can_id);
    }

    pthread_t tty_to_can_thread_id;
    pthread_create(&tty_to_can_thread_id, NULL, tty_to_can_thread, &tty_queue);
    for (int i = 0; i < prCodeModulesSet.count; i++) {
        printf("TTY /dev/ttyS%d: fd=%d, can_id=0x%X\n", 11 + i, prCodeModulesSet.fd[i], prCodeModulesSet.can_id[i]);
    }

    pthread_join(listener_thread, NULL);
    pthread_join(processor_thread, NULL);
    for (int i = 0; i < prCodeModulesSet.count; ++i)
        pthread_join(tty_threads[i], NULL);

    pthread_join(tty_to_can_thread_id, NULL);

    return 0;
}