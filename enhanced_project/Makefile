CC = aarch64-none-linux-gnu-gcc
CFLAGS = -Wall -O2
TARGET = can

all: $(TARGET)

$(TARGET): can.c
	$(CC) $(CFLAGS) -o $(TARGET) can.c

clean:
	rm -f $(TARGET)
