# Зручні команди для homework_16 (BMP388 через I2C).
#
# Працює як окремий CMake-проєкт (не потребує root CMakeLists.txt чи devcontainer-а
# з ROS 2) - лише сам build/ у цій директорії.
#
# Приклади:
#   make build
#   make run
#   make test

SHELL := /bin/bash

BUILD_DIR ?= build
GENERATOR ?= Ninja

.PHONY: help configure build run test clean rebuild

.DEFAULT_GOAL := help

help: ## Показати цей список команд
	@echo "Цілі:"
	@grep -hE '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) \
	  | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-10s\033[0m %s\n", $$1, $$2}'

configure: ## Налаштувати CMake (build/, генератор Ninja; змінні: BUILD_DIR=, GENERATOR=)
	@cmake -S . -B $(BUILD_DIR) -G $(GENERATOR)

build: configure ## Зібрати бібліотеку, i2chardware і тести
	@cmake --build $(BUILD_DIR)

run: build ## Запустити i2chardware (потрібен реальний BMP388 на /dev/i2c-1, адреса 0x77)
	@$(BUILD_DIR)/i2chardware

test: build ## Прогнати юніт-тести (i2chardware_tests)
	@$(BUILD_DIR)/i2chardware_tests

clean: ## Видалити build/
	@rm -rf $(BUILD_DIR)

rebuild: clean build ## clean + build з нуля
