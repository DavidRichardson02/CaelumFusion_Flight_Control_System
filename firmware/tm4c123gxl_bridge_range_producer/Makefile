TARGET := tm4c123gxl_bridge_range_producer
PART := TM4C123GH6PM
TIVAWARE_ROOT ?= C:/ti/TivaWare_C_Series-2.2.0.295
PREFIX ?= arm-none-eabi

CC := $(PREFIX)-gcc
OBJCOPY := $(PREFIX)-objcopy
SIZE := $(PREFIX)-size

CFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS += -ffunction-sections -fdata-sections -std=c99 -Wall -Wextra -Os
CFLAGS += -DPART_$(PART) -DTARGET_IS_TM4C123_RB1
CFLAGS += -I$(TIVAWARE_ROOT)

LDFLAGS := -T tm4c123gh6pm.ld -nostartfiles
LDFLAGS += -Wl,--gc-sections -Wl,-Map,$(TARGET).map

LDLIBS := -L$(TIVAWARE_ROOT)/driverlib/gcc -ldriver

OBJS := startup_gcc.o main.o

.PHONY: all clean

all: $(TARGET).bin $(TARGET).elf

$(TARGET).elf: $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)
	$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).bin $(TARGET).map
