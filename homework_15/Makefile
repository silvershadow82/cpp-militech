# Зручні команди для homework_15.
#
# Запускати ЗСЕРЕДИНИ devcontainer-а.
# Приклади:
#   make build
#   make run

SHELL := /bin/bash

.SHELLFLAGS := -e -o pipefail -c
.ONESHELL:

BUILDIR := build

.PHONY: build run test info help

help: ## Показати цей список команд
	@echo "Цілі:"
	grep -hE '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) \
	  | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-14s\033[0m %s\n", $$1, $$2}'
	echo

.DEFAULT_GOAL := help


check-built:
	@if [ ! -f "$(BUILDIR)/reporter" ]; then
	  echo "Воркспейс не зібраний - спочатку 'make build'." >&2
	  exit 1
	fi

build:
	cmake -S . -B $(BUILDIR) -G Ninja
	for i in $$(seq 1 $(BUILD_RETRIES)); do
	  if cmake --build build; then
	    exit 0
	  fi
	  echo "--- збірка впала, спроба $$i з $(BUILD_RETRIES) ---"
	done
	echo "Збірка не пройшла за $(BUILD_RETRIES) спроб - це вже схоже на справжню помилку." >&2
	exit 1


test: check-built ## Прогнати юніт-тести reporter_tests
	ctest --test-dir "$(BUILDIR)" --output-on-failure

run: check-built ## Запустити reporter
	$(BUILDIR)/reporter


clean: ## Прибрати build/ воркспейсу
	@rm -rf $(BUILDIR)