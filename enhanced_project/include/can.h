#ifndef CAN_H
#define CAN_H

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
#include <stdint.h>
#include <time.h>
#include <poll.h>
#include <pthread.h>

#include <stdint.h>

#include "common.h"
#include "queue.h"
#include "utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Проверяет, содержится ли указанный CAN ID в IDSet.
 *
 * @param set Указатель на структуру IDSet.
 * @param id CAN ID для проверки.
 * @return 1, если ID найден; 0 — если нет.
 */
int idset_contains(const IDSet* set, uint16_t id);

/**
 * @brief Добавляет CAN ID в IDSet, если он ещё не добавлен.
 *
 * @param set Указатель на структуру IDSet.
 * @param id CAN ID для добавления.
 * @return 1 при успешном добавлении, 0 — если уже содержится или переполнение.
 */
int idset_add(IDSet* set, uint16_t id);

/**
 * @brief Сканирует CAN-сеть и собирает CAN ID (в диапазоне 0x700–0x77F).
 *
 * @param can_interface Имя интерфейса CAN (например, "can0").
 * @return Структура IDSet с собранными CAN ID.
 */
IDSet collect_can_ID(const char* can_interface);

/**
 * @brief Запрашивает product code по каждому CAN ID через SDO (0x1018:02).
 *
 * @param can_interface Имя интерфейса CAN (например, "can0").
 * @param idSet Указатель на структуру с CAN ID.
 * @return Структура PrCodeCanIDSet с полученными product code.
 */
PrCodeCanIDSet collect_product_code(const char* can_interface, IDSet* idSet);

/**
 * @brief Поток, читающий кадры с CAN-шины и пересылающий их в очередь.
 *
 * @param arg Указатель на структуру PrCodeModulesSet (CAN ID <-> tty fd).
 * @return NULL (бесконечный цикл).
 */
void* can_listener_thread(void* arg);

/**
 * @brief Поток, обрабатывающий кадры из очереди и отправляющий данные в соответствующий tty.
 *
 * @param arg Указатель на структуру FrameQueue.
 * @return NULL (бесконечный цикл).
 */
void* can_to_tty_thread(void* arg);

#ifdef __cplusplus
}
#endif

#endif // UTILS_H
