# Short commands for the coursework. This is not a build system -- CMake builds everything, these
# are just the invocations from README.md so they do not have to be remembered.
#
#   make              list the targets
#   make check        everything that must be green before a commit
#
# Build directories live at the repository root (../build/...), matching README.md.

REPO          := $(abspath $(CURDIR)/..)
BUILD_DIR     ?= $(REPO)/build/coursework
VISION_DIR    ?= $(REPO)/build/vision
JOBS          ?= 8

# SITL. ARDUPILOT_DIR must point at an ArduPilot checkout with SITL built
# (./waf configure --board sitl && ./waf copter); run_sitl_scenario.sh says so if it is unset.
ARDUPILOT_DIR ?=
VENV          ?= $(REPO)/build/sitl-venv
PYTHON        ?= $(VENV)/bin/python
SCENARIO      ?= stationary
TIMEOUT_S     ?= 300

CLANG_FORMAT  ?= clang-format
STYLE         := --style=file:$(REPO)/.devcontainer/.clang-format

# Core.h is hand-formatted and is the one file the formatter must never touch (see README.md).
FORMAT_FILES  := $(shell find include src tests \( -name '*.h' -o -name '*.cpp' \) \
                   ! -path 'include/control/Core.h' 2>/dev/null)

.PHONY: help build build-vision test test-vision sitl sitl-all qgc venv format format-check purity check clean

help:
	@echo "make build         - default build, FOLLOW_WITH_OPENCV=OFF ($(BUILD_DIR))"
	@echo "make build-vision  - OpenCV build, FOLLOW_WITH_OPENCV=ON ($(VISION_DIR))"
	@echo "make test          - unit and integration tests, OFF build"
	@echo "make test-vision   - unit and integration tests, ON build"
	@echo "make sitl          - one SITL scenario against ArduCopter (SCENARIO=$(SCENARIO))"
	@echo "make sitl-all      - all scenarios in config/scenarios, reports a tally"
	@echo "make qgc           - SITL run visible in QGroundControl on udp:14550 (SCENARIO=$(SCENARIO))"
	@echo "make venv          - create $(VENV) with pymavlink, for the pilot stand-in"
	@echo "make format        - clang-format everything except control/Core.h"
	@echo "make format-check  - fail if anything is unformatted"
	@echo "make purity        - fail if OpenCV reached follow_core"
	@echo "make check         - build + test + build-vision + test-vision + format-check + purity"
	@echo "make clean         - remove the build directories"
	@echo
	@echo "Variables: SCENARIO ARDUPILOT_DIR PYTHON JOBS BUILD_DIR VISION_DIR TIMEOUT_S"

build:
	cmake -S . -B $(BUILD_DIR)
	cmake --build $(BUILD_DIR) -j$(JOBS)

build-vision:
	cmake -S . -B $(VISION_DIR) -DFOLLOW_WITH_OPENCV=ON
	cmake --build $(VISION_DIR) -j$(JOBS)

test: build
	ctest --test-dir $(BUILD_DIR) --output-on-failure

test-vision: build-vision
	ctest --test-dir $(VISION_DIR) --output-on-failure

venv:
	python3 -m venv $(VENV)
	$(VENV)/bin/pip install --quiet pymavlink
	@$(PYTHON) -c "import pymavlink; print('pymavlink', pymavlink.__version__, 'in $(VENV)')"

# Starts ArduCopter SITL, follow_app --sim and the pilot stand-in, then checks the run log.
sitl: build
	ARDUPILOT_DIR=$(ARDUPILOT_DIR) PYTHON=$(PYTHON) BUILD_DIR=$(BUILD_DIR) TIMEOUT_S=$(TIMEOUT_S) \
	  tools/run_sitl_scenario.sh config/scenarios/$(SCENARIO).json

sitl-all: build
	@pass=0; fail=0; \
	for s in config/scenarios/*.json; do \
	  if ARDUPILOT_DIR=$(ARDUPILOT_DIR) PYTHON=$(PYTHON) BUILD_DIR=$(BUILD_DIR) TIMEOUT_S=$(TIMEOUT_S) \
	       tools/run_sitl_scenario.sh "$$s"; then pass=$$((pass+1)); \
	  else fail=$$((fail+1)); echo "FAILED: $$s"; fi; \
	done; \
	echo "== $$pass passed, $$fail failed"; \
	[ $$fail -eq 0 ]

# Same run, but with a second MAVLink serial so QGroundControl sees the vehicle. QGC listens on
# udp:14550 by default. SITL waits for the pilot stand-in to connect on tcp:5760 before it runs --
# see README.md, "Watch a run in QGroundControl".
qgc: build
	ARDUPILOT_DIR=$(ARDUPILOT_DIR) PYTHON=$(PYTHON) BUILD_DIR=$(BUILD_DIR) \
	  tools/run_qgc_session.sh config/scenarios/$(SCENARIO).json

format:
	$(CLANG_FORMAT) -i $(STYLE) $(FORMAT_FILES)

format-check:
	@$(CLANG_FORMAT) --dry-run --Werror $(STYLE) $(FORMAT_FILES) && echo "format OK (control/Core.h excluded by design)"

# follow_core must not depend on OpenCV: the default build, the devcontainer gcc-13 build and the
# aarch64 cross-build all lack it. See README.md, "follow_core must stay pure".
purity:
	@if grep -rl opencv include/models include/control include/interfaces/ICameraModel.h \
	     include/interfaces/IByteLink.h include/interfaces/IConfigLoader.h src/models src/control 2>/dev/null; then \
	  echo "^^ OpenCV reached follow_core"; exit 1; \
	else echo "purity OK (no OpenCV in models/, control/ or the interfaces follow_core uses)"; fi

check: build test build-vision test-vision format-check purity
	@echo "== all green"

clean:
	rm -rf $(BUILD_DIR) $(VISION_DIR)
