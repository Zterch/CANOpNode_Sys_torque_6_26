# CANOpNode_Sys Build System
# Author: System Architect
# Date: 2026-05-12
# Version: 2.0.0 - NiMotion SDK Integration

# 编译器设置
CC = gcc
CFLAGS = -Wall -Wextra -O2 -pthread
CFLAGS += -I./include -I./config -I./utils -I./drivers -I./algorithms
CFLAGS += -I../NimServoSDK-MM-bin-linux-x64/inc
CFLAGS += -D_GNU_SOURCE

# SDK库路径
SDK_PATH = ../NimServoSDK-MM-bin-linux-x64
# 修改Makefile使用本地库
SDK_LIB_PATH = ./lib

# 调试模式
DEBUG = 0
ifeq ($(DEBUG), 1)
    CFLAGS += -g -DDEBUG
else
    CFLAGS += -DNDEBUG
endif

# 链接选项 - 链接SDK库 (注意库名大小写)
LDFLAGS = -pthread -lm -lrt
LDFLAGS += -L$(SDK_LIB_PATH) \
           -lNimServoSDK \
           -lNiMotionUSBCAN \
           -lCANopenMaster \
           -lAbstractMaster \
           -lAbstractCanDevice \
           -lGlobal \
           -lDataBase \
           -lNiMotionCanDevice \
           -lTcpCanDevice
LDFLAGS += -Wl,-rpath,$(SDK_LIB_PATH)

# 目录设置
BUILD_DIR = build
BIN_DIR = bin

# 源文件
SOURCES = main.c \
          utils/logger.c \
          utils/thread_manager.c \
          utils/shared_memory.c \
          algorithms/sine_wave.c \
          algorithms/signal_filter.c \
          algorithms/pid_controller.c \
          algorithms/safety_monitor.c \
          algorithms/gravity_unload.c \
          algorithms/system_check.c \
          drivers/motor_driver.c \
          drivers/power_driver.c \
          drivers/sensor_manager.c \
          drivers/rs485_bus.c \
          drivers/weight_driver.c

# 对象文件
OBJECTS = $(SOURCES:%.c=$(BUILD_DIR)/%.o)

# 目标可执行文件
TARGET = $(BIN_DIR)/CANOpNode_Sys

# 默认目标
.PHONY: all clean dirs debug release

all: dirs $(TARGET)

# 创建目录
dirs:
	@mkdir -p $(BUILD_DIR)/utils
	@mkdir -p $(BUILD_DIR)/drivers
	@mkdir -p $(BUILD_DIR)/algorithms
	@mkdir -p $(BIN_DIR)

# 编译规则
$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@
	@echo "CC $<"

# 链接规则
$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)
	@echo "LINK $@"
	@echo "Build completed: $@"

# 调试模式
debug:
	$(MAKE) DEBUG=1

# 发布模式
release:
	$(MAKE) DEBUG=0

# 清理
clean:
	@rm -rf $(BUILD_DIR)
	@rm -rf $(BIN_DIR)
	@rm -f *.log
	@echo "Clean completed"

# 安装
install: $(TARGET)
	@cp $(TARGET) /usr/local/bin/
	@echo "Installed to /usr/local/bin/"

# 卸载
uninstall:
	@rm -f /usr/local/bin/CANOpNode_Sys
	@echo "Uninstalled"

# 运行主程序（设置LD_LIBRARY_PATH确保SDK能找到所有库）
run: $(TARGET)
	@echo "Running CANOpNode_Sys with LD_LIBRARY_PATH set to SDK directory..."
	@LD_LIBRARY_PATH=$(SDK_LIB_PATH):$$LD_LIBRARY_PATH sudo $(TARGET)

# 帮助
help:
	@echo "CANOpNode_Sys Build System v2.0"
	@echo "==============================="
	@echo "Targets:"
	@echo "  all      - Build the project (default)"
	@echo "  debug    - Build with debug symbols"
	@echo "  release  - Build optimized version"
	@echo "  clean    - Remove all build files"
	@echo "  install  - Install to /usr/local/bin"
	@echo "  uninstall- Remove from /usr/local/bin"
	@echo "  run      - Build and run with sudo"
	@echo "  help     - Show this help message"
