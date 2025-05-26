#ifndef UTILS_H
#define UTILS_H

#include <linux/can.h>         // struct can_frame
#include <linux/can/raw.h>     // CAN_RAW, etc.
#include <sys/socket.h>        // socket(), bind(), sockaddr
#include <sys/ioctl.h>         // ioctl(), SIOCGIFINDEX
#include <unistd.h>            // close()
#include <stdio.h>             // perror()
#include <string.h>            // strncpy()
#include <stdint.h>            // uint16_t, uint32_t

#include "common.h"


// Прототипы функций

/**
 * @brief Функция сравнения двух uint16_t (для qsort, bsearch и т.п.)
 */
int compare_uint16(const void* a, const void* b);

/**
 * @brief Создаёт и настраивает сокет CAN RAW.
 *
 * @param interface Имя интерфейса, например "can0".
 * @return Дескриптор сокета при успехе, -1 при ошибке.
 */
int create_can_socket(const char* interface);

#endif // UTILS_H
