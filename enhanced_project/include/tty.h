#ifndef TTY_HANDLER_H
#define TTY_HANDLER_H

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <pthread.h>
#include <net/if.h>

#include <stdint.h>
#include <pthread.h>
#include "common.h"
#include "queue.h"
#include "utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Открывает виртуальный последовательный порт.
 *
 * @param n Номер порта (например, 11 для /dev/ttyS11).
 * @return Дескриптор файла при успехе, -1 при ошибке.
 */
int open_port(int n);

/**
 * @brief Открывает TTY-порты для каждого CAN ID, соответствующего поддерживаемым product code.
 *
 * @param CodeSet Указатель на структуру с парами product_code + can_id.
 * @return Структура PrCodeModulesSet с открытыми дескрипторами и CAN ID.
 */
PrCodeModulesSet open_tty(PrCodeCanIDSet* CodeSet);

/**
 * @brief Поток-приемник, читающий данные из TTY и помещающий их в очередь.
 *
 * @param arg Указатель на структуру TTYListenerArgs.
 * @return NULL (поток бесконечный).
 */
void* tty_listener_thread(void* arg);

/**
 * @brief Поток, считывающий данные из очереди и передающий их в CAN-интерфейс.
 *
 * @param arg Указатель на FrameQueue.
 * @return NULL (поток бесконечный).
 */
void* tty_to_can_thread(void* arg);

#ifdef __cplusplus
}
#endif

#endif // TTY_HANDLER_H
