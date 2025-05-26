#include "can.h"

FrameQueue queue;

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

//function is completed
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

//function is completed
PrCodeCanIDSet collect_product_code(const char* can_interface, IDSet* idSet) {
    PrCodeCanIDSet result = {};  // Инициализируем нулями

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
            printf(" Неверный ID ответа: 0x%X\n", resp.can_id);
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

void* can_listener_thread(void* arg) {
    printf("start listening...\n");
    PrCodeModulesSet* args = (PrCodeModulesSet*)arg; 
    int sock = create_can_socket("can0");
    if (sock < 0) {
        perror("createCanSocket в потоке");
        return NULL;
    }

    struct can_filter filters[MAX_IDS];
    for (int i = 0; i < args->count; i++) {
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

        for (int i = 0; i < args->count; i++) {
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

void* can_to_tty_thread(void* arg) {
    printf("start writing...\n");
    FrameQueue* queue = (FrameQueue*)arg;

    while (1) {

        FrameWithFD item;
        queue_pop(queue, &item);
        printf("\n");
        printf("Обработка CAN ID 0x%X -> tty fd %d: ", item.frame.can_id, item.fd);
        for (int i = 0; i < item.frame.can_dlc; ++i) {
            printf("%02X ", item.frame.data[i]);
        }
        printf("\n");
        unsigned char data[item.frame.can_dlc];

        for (int i=0; i < item.frame.can_dlc;i++) {

            data[i]=(char)item.frame.data[i];
            printf("данные: %02X %02X %d\n", data[i],item.frame.data[i], item.fd);

        }
        if (item.fd != -1) {
            // TODO: обработка frame.data[]
            ssize_t written = write(item.fd, item.frame.data, item.frame.can_dlc);
            if (written < 0) {
                perror("Ошибка записи в tty");
            }
        }
        printf("\n");
    }

    return NULL;
}

