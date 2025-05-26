# Кросс-компилятор и флаги
CC = /opt/arm-gnu-toolchain-11.3.rel1-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-gcc

CFLAGS = -Wall -Wextra -O2 -std=c11 -Iinclude -pthread

# Каталоги
SRC_DIR = src
INCLUDE_DIR = include
BUILD_DIR = build

# Исходники и объектные файлы
SRCS = $(wildcard $(SRC_DIR)/*.c)
OBJS = $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(SRCS))

# Итоговый бинарник
TARGET = $(BUILD_DIR)/can

.PHONY: all clean

all: $(TARGET)

# Правило сборки финального бинарника
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $^ -o $@

# Правило сборки object-файлов
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Создание каталога build, если его нет
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)