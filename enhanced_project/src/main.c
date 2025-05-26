#include "common.h"
#include "utils.h"
#include "can.h"
#include "tty.h"

#define MAX_TTY_LISTENERS 8

const PrCodesSet MODELS = {
    .product_code = {0x2, 0x3, 0x11},  // Остальные = 0 автоматически
    .count = 2
};

extern FrameQueue queue;
extern FrameQueue tty_queue;

static pthread_t listener_thread;
static pthread_t processor_thread;
static pthread_t tty_to_can_thread_id;
static pthread_t tty_threads[MAX_TTY_LISTENERS];

void init_queues() {
    queue_init(&queue);
    queue_init(&tty_queue);
}

void start_threads(const PrCodeModulesSet* modules) {
    pthread_create(&tty_to_can_thread_id, NULL, tty_to_can_thread, &tty_queue);

    for (int i = 0; i < modules->count; i++) {
        TTYListenerArgs* arg = malloc(sizeof(TTYListenerArgs));
        if (!arg) {
            perror("malloc");
            exit(EXIT_FAILURE);
        }

        arg->fd = modules->fd[i];
        arg->can_id = modules->can_id[i];
        arg->queue = &tty_queue;

        int ret = pthread_create(&tty_threads[i], NULL, tty_listener_thread, arg);
        if (ret != 0) {
            fprintf(stderr, "Ошибка запуска потока tty_listener_thread: %s\n", strerror(ret));
            free(arg);
            continue;
        }
    }

    int res1 = pthread_create(&listener_thread, NULL, can_listener_thread, (void*)modules);
    int res2 = pthread_create(&processor_thread, NULL, can_to_tty_thread, &queue);
    
    if (res1 != 0) {
        fprintf(stderr, "Ошибка запуска потока listen_can: %s\n", strerror(res1));
    }
    if (res2 != 0) {
        fprintf(stderr, "Ошибка запуска потока process_data: %s\n", strerror(res2));
    }
}

void wait_for_threads(int module_count) {
    pthread_join(listener_thread, NULL);
    pthread_join(processor_thread, NULL);

    for (int i = 0; i < module_count; ++i) {
        pthread_join(tty_threads[i], NULL);
    }

    pthread_join(tty_to_can_thread_id, NULL);
}

int main() {
    init_queues();
    IDSet ids = collect_can_ID("can0");
    PrCodeCanIDSet pr_codes = collect_product_code("can0", &ids);
    PrCodeModulesSet prCodeModulesSet = open_tty(&pr_codes);

    start_threads(&prCodeModulesSet);

    wait_for_threads(prCodeModulesSet.count);

    return 0;
}