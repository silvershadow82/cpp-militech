# Швидкі перевірки для homework_11. Це не система збірки - усе будує CMake,
# тут лише короткі команди, щоб не тримати їх у голові.
#
#   make            список цілей
#   make check      усе, що має бути зеленим перед комітом

BUILD_DIR ?= build-check
TSAN_DIR  ?= build-tsan
JOBS      ?= 8
SECS      ?= 20

# Куди слати MAVLink. З devcontainer (--network=host) 127.0.0.1 - це loopback
# віртуалки, а не мака, тому там треба host.docker.internal.
QGC       ?= 127.0.0.1:14550

# Для make checker / make pi-build.
CHECKER   ?= $(HOME)/Downloads/checker-linux-arm64
PLATFORM  ?= linux/arm64
IMAGE     ?= debian:bookworm-slim

.PHONY: help build test tsan checker checker-qgc qgc sim pi-build check clean

help:
	@echo "make build        - зібрати simulation11 ($(BUILD_DIR))"
	@echo "make test         - юніт-тести (зараз їх 0: tests/simulation_tests.cpp порожній)"
	@echo "make tsan         - зібрати з ThreadSanitizer і прогнати місію; падає на гонці"
	@echo "make checker      - MAVLink-чекер у Linux-контейнері, очікується 13/13"
	@echo "make checker-qgc  - те саме + дублювання телеметрії в QGC (QGC=$(QGC))"
	@echo "make qgc          - живий прогін у QGC на цій машині (SECS=$(SECS))"
	@echo "make sim          - лише UART-стенд: друкує PTY і чекає на дрона"
	@echo "make pi-build     - збірка під Linux з libgpiod (USE_GPIOD=ON)"
	@echo "make check        - build + test + tsan + checker"
	@echo "make clean        - прибрати каталоги збірки"
	@echo
	@echo "Змінні: SECS QGC CHECKER PLATFORM IMAGE JOBS BUILD_DIR"

build:
	cmake -S . -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=Debug
	cmake --build $(BUILD_DIR) -j$(JOBS)

test: build
	$(BUILD_DIR)/simulation11_tests

tsan:
	cmake -S . -B $(TSAN_DIR) -DCMAKE_BUILD_TYPE=Debug -DSIMULATION11_TSAN=ON
	cmake --build $(TSAN_DIR) -j$(JOBS)
	tools/run_local.sh $(TSAN_DIR) $(SECS)

checker:
	CHECKER=$(CHECKER) PLATFORM=$(PLATFORM) IMAGE=$(IMAGE) tools/run_checker.sh $(SECS)

checker-qgc:
	CHECKER=$(CHECKER) PLATFORM=$(PLATFORM) IMAGE=$(IMAGE) QGC=$(QGC) tools/run_checker.sh $(SECS)

qgc: build
	tools/run_local.sh $(BUILD_DIR) $(SECS) $(QGC)

sim:
	tools/uart_sim.py 0 $(SECS)

pi-build:
	@docker run --rm --platform $(PLATFORM) -v "$(CURDIR)/..:/src" -w /src/$(notdir $(CURDIR)) $(IMAGE) bash -eu -c '\
	  apt-get update -qq >/dev/null; \
	  apt-get install -y -qq build-essential cmake pkg-config libgpiod-dev >/dev/null 2>&1; \
	  cmake -S . -B build-pi -DUSE_GPIOD=ON -DCMAKE_BUILD_TYPE=Debug > /tmp/c.log 2>&1 || { tail -20 /tmp/c.log; exit 1; }; \
	  for i in 1 2 3; do \
	    cmake --build build-pi -j"$$(nproc)" > /tmp/b.log 2>&1 && break; \
	    grep -q "internal compiler error" /tmp/b.log || { grep -iE error /tmp/b.log | head -20; exit 1; }; \
	  done; \
	  echo "USE_GPIOD=ON build OK, warnings: $$(grep -ciE "warning:" /tmp/b.log)"'

check: build test tsan checker
	@echo "== усе зелено"

clean:
	rm -rf $(BUILD_DIR) $(TSAN_DIR) build-linux build-pi
