# Перевірка ДЗ 13 (antidrone_turret) однією командою.
# Усі цілі виконуються з каталогу homework_13: make build, make test, make demo.
#
# Змінні, які можна перевизначити:
#   ROS_DISTRO=jazzy            дистрибутив ROS 2 (шукається /opt/ros/$ROS_DISTRO/setup.bash)
#   TRACK=all                   трек для запуску: all або *.csv з tracks/
#   LOG_LEVEL=info              info | debug | warn | error
#   DEMO_SECONDS=20             тривалість make demo

SHELL := /bin/bash

ROS_DISTRO ?= jazzy
ROS_SETUP ?= /opt/ros/$(ROS_DISTRO)/setup.bash
WS := robot_ws
PKG := antidrone_turret
TRACK ?= all
LOG_LEVEL ?= info
DEMO_SECONDS ?= 20

# Оточення ROS 2 і оверлей воркспейсу; source потребує bash, тому SHELL вище.
ROS_ENV := source $(ROS_SETUP)
WS_ENV := $(ROS_ENV) && cd $(WS) && source install/setup.bash

.DEFAULT_GOAL := help
.PHONY: help ros-check install build test test-core run run-approach run-far \
        run-low-confidence run-reload demo echo-topics graph trigger tracks clean

help:
	@echo "ДЗ 13 antidrone_turret - команди перевірки:"
	@echo ""
	@echo "  make install            встановити ROS-залежності пакета через rosdep"
	@echo "  make build              colcon build --symlink-install (пакет $(PKG))"
	@echo "  make test               build + colcon test + colcon test-result --verbose"
	@echo "  make test-core          тільки тести логіки контролера (core_controller_test)"
	@echo ""
	@echo "  make run                запустити систему, усі треки послідовно (track:=all)"
	@echo "  make run TRACK=approach_trigger.csv   запустити конкретний трек"
	@echo "  make run-approach       трек approach_trigger.csv (є момент для пострілу)"
	@echo "  make run-far            трек far_flyby_no_trigger.csv (постріл не потрібен)"
	@echo "  make run-low-confidence трек low_confidence_no_trigger.csv (ACTION_IDLE)"
	@echo "  make run-reload         трек reload_pressure.csv (тиск на перезаряджання)"
	@echo "  make demo               запуск на $(DEMO_SECONDS) с і автоматичний вихід"
	@echo ""
	@echo "  make echo-topics        по одному повідомленню з кожного топіка (поки система працює)"
	@echo "  make graph              ros2 node/topic/service/param list"
	@echo "  make trigger            вручну викликати сервіс /actuator/trigger"
	@echo "  make tracks             показати доступні треки"
	@echo "  make clean              видалити build/, install/, log/"
	@echo ""
	@echo "Типова перевірка: make test, далі make demo TRACK=reload_pressure.csv"

ros-check:
	@test -f $(ROS_SETUP) || { \
	  echo "не знайдено $(ROS_SETUP)"; \
	  echo "задайте ROS_DISTRO=<distro> або ROS_SETUP=/шлях/до/setup.bash"; \
	  exit 1; \
	}

install: ros-check
	@$(ROS_ENV) && cd $(WS) && rosdep install --from-paths src --ignore-src -r -y

build: ros-check
	@$(ROS_ENV) && cd $(WS) && colcon build --symlink-install --packages-select $(PKG)

test: build
	@$(WS_ENV) && colcon test --packages-select $(PKG) && colcon test-result --verbose

test-core: build
	@$(WS_ENV) && colcon test --packages-select $(PKG) --ctest-args -R core_controller_test && \
	  colcon test-result --verbose

run: build
	@$(WS_ENV) && ros2 launch $(PKG) system.launch.py track:=$(TRACK) log_level:=$(LOG_LEVEL)

run-approach:
	@$(MAKE) run TRACK=approach_trigger.csv

run-far:
	@$(MAKE) run TRACK=far_flyby_no_trigger.csv

run-low-confidence:
	@$(MAKE) run TRACK=low_confidence_no_trigger.csv

run-reload:
	@$(MAKE) run TRACK=reload_pressure.csv

# timeout -s INT працює як Ctrl-C: launch коректно гасить усі ноди.
demo: build
	@$(WS_ENV) && \
	  timeout -s INT $(DEMO_SECONDS) ros2 launch $(PKG) system.launch.py \
	    track:=$(TRACK) log_level:=$(LOG_LEVEL) || true

# Наступні цілі потребують запущеної системи в іншому терміналі (make run).
echo-topics:
	@$(WS_ENV) && \
	  for topic in /perception/target /turret/status /gimbal/cmd /servo/cmd /actuator/status; do \
	    echo "--- ros2 topic echo $$topic --once"; \
	    timeout 10 ros2 topic echo $$topic --once || echo "(немає повідомлень за 10 с: чи запущено make run?)"; \
	  done

graph:
	@$(WS_ENV) && \
	  echo "--- ros2 node list" && ros2 node list && \
	  echo "--- ros2 topic list" && ros2 topic list && \
	  echo "--- ros2 service list" && ros2 service list && \
	  echo "--- ros2 param list" && ros2 param list

trigger:
	@$(WS_ENV) && \
	  ros2 service call /actuator/trigger $(PKG)/srv/TriggerActuator "{confidence: 0.9, distance_m: 25.0}" && \
	  ros2 topic echo /actuator/status --once

tracks:
	@ls $(WS)/src/$(PKG)/tracks

clean:
	rm -rf $(WS)/build $(WS)/install $(WS)/log
