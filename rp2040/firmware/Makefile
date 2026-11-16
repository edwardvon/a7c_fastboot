FQBN := rp2040:rp2040:waveshare_rp2040_zero
PORT ?= COM4
SKETCH := .
BUILD_DIR := build
DEFINES ?=
CORE_URL := https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json

ARDUINO_CLI  := arduino-cli
ARDUINO_DATA := $(shell $(ARDUINO_CLI) config get directories.data)
PIOASM       := $(firstword $(wildcard \
                  $(ARDUINO_DATA)/packages/rp2040/tools/pqt-pioasm/*/pioasm.exe \
                  $(ARDUINO_DATA)/packages/rp2040/tools/pqt-pioasm/*/pioasm))

.PHONY: all setup build flash clean

all: build

setup:
	$(ARDUINO_CLI) core update-index --additional-urls $(CORE_URL)
	$(ARDUINO_CLI) core install rp2040:rp2040 --additional-urls $(CORE_URL)

a7c_serialboot.pio.h: a7c_serialboot.pio
	$(PIOASM) -o c-sdk $< $@

build: a7c_serialboot.pio.h
	$(ARDUINO_CLI) compile --fqbn $(FQBN) --build-path $(BUILD_DIR) \
		--build-property "compiler.cpp.extra_flags=$(DEFINES)" $(SKETCH)

flash: build
	$(ARDUINO_CLI) upload --fqbn $(FQBN) --port $(PORT) --input-dir $(BUILD_DIR) $(SKETCH)

clean:
	rm -rf $(BUILD_DIR) a7c_serialboot.pio.h
