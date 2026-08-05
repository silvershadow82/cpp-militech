# Зручні команди для homework_14 (робопес у підземному світі).
#
# Запускати ЗСЕРЕДИНИ devcontainer-а - тут має бути доступний ROS 2.
# Приклади:
#   make build
#   make record SCENARIO=small_rooms
#   make record-all
#   make info SCENARIO=small_rooms
#   make replay SCENARIO=small_rooms

SHELL := /bin/bash

.SHELLFLAGS := -e -o pipefail -c
.ONESHELL:

ROS_DISTRO ?= jazzy
ROS_SETUP := /opt/ros/$(ROS_DISTRO)/setup.bash

WS := robot_ws
BAGS := bags
CONFIG := $(WS)/src/underground_world/config

SCENARIO ?= training_corridor
SCENARIOS := training_corridor small_rooms branching_trench dead_end_bunker

# Затримка перед першою командою руху - дає ros2 bag record час підчепитися.
START_DELAY_MS ?= 3000
RUN_TIMEOUT ?= 180
BUILD_RETRIES ?= 10

.PHONY: help build install test run record record-all info info-all replay status clean distclean check-env check-scenario check-built

help: ## Показати цей список команд
	@echo "Цілі:"
	grep -hE '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) \
	  | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-14s\033[0m %s\n", $$1, $$2}'
	echo
	echo "Змінні:"
	echo "  SCENARIO=<назва>   один із: $(SCENARIOS)  (типово: $(SCENARIO))"
	echo "  FORCE=1            дозволити перезапис наявного bag-а"
	echo "  START_DELAY_MS=N   затримка до першого руху (типово: $(START_DELAY_MS))"

.DEFAULT_GOAL := help

check-env:
	@if [ ! -f "$(ROS_SETUP)" ]; then
	  echo "Не знайдено $(ROS_SETUP)." >&2
	  echo "Ці цілі треба запускати зсередини devcontainer-а з ROS 2 $(ROS_DISTRO)." >&2
	  exit 1
	fi

check-scenario:
	@if [ ! -f "$(CONFIG)/$(SCENARIO).yaml" ]; then
	  echo "Немає сценарію '$(SCENARIO)'. Доступні: $(SCENARIOS)" >&2
	  exit 1
	fi

check-built:
	@if [ ! -f "$(WS)/install/setup.bash" ]; then
	  echo "Воркспейс не зібраний - спочатку 'make build'." >&2
	  exit 1
	fi

build: check-env 
	@source $(ROS_SETUP)
	cd $(WS)
	for i in $$(seq 1 $(BUILD_RETRIES)); do
	  if colcon build --symlink-install; then
	    exit 0
	  fi
	  echo "--- збірка впала, спроба $$i з $(BUILD_RETRIES) ---"
	done
	echo "Збірка не пройшла за $(BUILD_RETRIES) спроб - це вже схоже на справжню помилку." >&2
	exit 1

install: check-env ## Доставити системні залежності пакетів через rosdep
	@source $(ROS_SETUP)
	cd $(WS)
	rosdep install --from-paths src --ignore-src -y

test: check-env check-built ## Прогнати юніт-тести mission_control
	@source $(ROS_SETUP)
	cd $(WS)
	source install/setup.bash
	colcon test --packages-select mission_control
	colcon test-result --verbose

run: check-env check-scenario check-built ## Запустити місію без запису
	@source $(ROS_SETUP)
	source $(WS)/install/setup.bash
	ros2 launch underground_world system.launch.py \
	  scenario:=$(SCENARIO).yaml \
	  start_delay_ms:=$(START_DELAY_MS)

record: check-env check-scenario check-built ## Прогнати місію і записати rosbag
	@source $(ROS_SETUP)
	source $(WS)/install/setup.bash

	out="$(BAGS)/$(SCENARIO)"
	if [ -d "$$out" ]; then
	  if [ -z "$${FORCE:-}" ]; then
	    echo "Bag '$$out' уже існує. Це артефакт здачі - перезапис лише з FORCE=1." >&2
	    exit 1
	  fi
	  echo "FORCE=1: видаляю наявний $$out"
	  rm -rf "$$out"
	fi
	mkdir -p "$(BAGS)"

	log=$$(mktemp)
	reclog=$$(mktemp)

	# Рекордер стартує ПЕРШИМ, інакше він пропустить ранні повідомлення
	# (докладніше -  §6.7).
	ros2 bag record -a -o "$$out" > "$$reclog" 2>&1 &
	recpid=$$!

	ready=0
	for i in $$(seq 1 60); do
	  if grep -qE "Listening for topics|Recording\.\.\." "$$reclog" 2>/dev/null; then
	    ready=1
	    break
	  fi
	  if ! kill -0 $$recpid 2>/dev/null; then
	    echo "Рекордер помер на старті (типовий segfault QEMU):" >&2
	    cat "$$reclog" >&2
	    exit 1
	  fi
	  sleep 0.5
	done
	if [ "$$ready" != "1" ]; then
	  echo "Рекордер не піднявся за 30 с." >&2
	  kill -KILL $$recpid 2>/dev/null || true
	  exit 1
	fi

	echo "=== $(SCENARIO): рекордер готовий, стартую місію ==="
	timeout $(RUN_TIMEOUT) ros2 launch underground_world system.launch.py \
	  scenario:=$(SCENARIO).yaml \
	  start_delay_ms:=$(START_DELAY_MS) 2>&1 | tee "$$log" || true

	# Сигнали не завжди доходять до емульованих процесів, тому ескалація
	# INT -> TERM -> KILL з обмеженим опитуванням, без безумовного wait
	# ( §13.2).
	for sig in INT TERM KILL; do
	  kill -0 $$recpid 2>/dev/null || break
	  kill -$$sig $$recpid 2>/dev/null || true
	  for i in $$(seq 1 20); do
	    kill -0 $$recpid 2>/dev/null || break
	    sleep 0.5
	  done
	done

	# Жорсткий KILL лишає bag без metadata.yaml - відновлюємо з .mcap.
	if [ ! -f "$$out/metadata.yaml" ]; then
	  echo "metadata.yaml відсутній, роблю reindex"
	  ros2 bag reindex "$$out" || true
	fi

	if ! grep -q "world reported SUCCESS" "$$log"; then
	  echo "УВАГА: у лозі немає 'world reported SUCCESS' - bag записаний, але прогін невдалий." >&2
	  exit 1
	fi
	grep "mission finished" "$$log" || true
	echo "=== $(SCENARIO): готово -> $$out ==="

record-all: ## Записати rosbag для всіх чотирьох сценаріїв
	@for s in $(SCENARIOS); do
	  $(MAKE) --no-print-directory record SCENARIO=$$s
	done

info: check-env check-scenario ## Показати ros2 bag info для сценарію
	@source $(ROS_SETUP)
	source $(WS)/install/setup.bash
	ros2 bag info "$(BAGS)/$(SCENARIO)"

info-all: ## Показати ros2 bag info для всіх сценаріїв
	@for s in $(SCENARIOS); do
	  echo "=== $$s ==="
	  $(MAKE) --no-print-directory info SCENARIO=$$s
	done

replay: check-env check-scenario ## Програти записаний rosbag
	@source $(ROS_SETUP)
	source $(WS)/install/setup.bash
	ros2 bag play "$(BAGS)/$(SCENARIO)"

status: check-env check-scenario ## Програти bag і показати потік /student/status
	@source $(ROS_SETUP)
	source $(WS)/install/setup.bash
	out=$$(mktemp)
	# Тип вказуємо явно: echo стартує раніше за програвач і інакше падає
	# з 'Could not determine the type'.
	ros2 topic echo /student/status underground_world/msg/StudentStatus > "$$out" 2>&1 &
	echopid=$$!
	# Знімаємо job control, інакше bash друкує "Killed" після kill нижче.
	disown $$echopid 2>/dev/null || true
	sleep 3
	ros2 bag play "$(BAGS)/$(SCENARIO)" --topics /student/status || true
	sleep 2
	kill -KILL $$echopid 2>/dev/null || true
	echo "повідомлень: $$(grep -c '^state:' "$$out" || true)"
	echo "останній стан: $$(grep '^state:' "$$out" | tail -1)"

clean: ## Прибрати build/install/log воркспейсу
	@rm -rf $(WS)/build $(WS)/install $(WS)/log

distclean: clean ## clean + видалити ВСІ записані bag-и (обережно: це артефакти здачі)
	@if [ -z "$${FORCE:-}" ]; then
	  echo "distclean видалить $(BAGS)/ - артефакти здачі. Повторіть із FORCE=1." >&2
	  exit 1
	fi
	rm -rf $(BAGS)
