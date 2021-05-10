ifndef INSTALLED_HOST_DIR
INSTALLED.HOST.DIR=$(BASE.DIR)/installed.host
else
INSTALLED.HOST.DIR=$(INSTALLED_HOST_DIR)
endif

HASH := $(shell git rev-parse --short=10 HEAD)
OS := $(shell uname)
ARCH := $(shell uname -m)
ifeq ($(OS), Linux)
J := $(bc -l << "2*$(shell nproc --all")
endif

$(info building with $(J) threads)
SHELL := /bin/bash

BASE.DIR=$(PWD)
DOWNLOADS.DIR=$(BASE.DIR)/downloads
INSTALLED.TARGET.DIR=$(BASE.DIR)/installed.target
CMAKE.VERSION=3.14.5
CMAKE.URL=https://github.com/Kitware/CMake/archive/v$(CMAKE.VERSION).tar.gz
CMAKE.DIR=$(DOWNLOADS.DIR)/CMake-$(CMAKE.VERSION)
CMAKE.ARCHIVE=v$(CMAKE.VERSION).tar.gz
CMAKE.BIN=$(INSTALLED.HOST.DIR)/bin/cmake
SOURCE.DIR=$(BASE.DIR)/source
BUILD.DIR=$(BASE.DIR)/build
BIN.DIR=$(INSTALLED.HOST.DIR)/bin
LIB.DIR=$(INSTALLED.HOST.DIR)/lib
TESTS.BIN=$(BIN.DIR)/mma8452-test
LIBRARY.DIR=$(BASE.DIR)/source
LIBRARY.BUILD=$(DOWNLOADS.DIR)/build.library

ci: clean bootstrap build

bootstrap: cmake

ctags: .FORCE
	cd $(BASE.DIR) && ctags -R --exclude=.git --exclude=installed.host --exclude=downloads --exclude=documents --exclude=installed.target --exclude=documents  --exclude=build.*  .

build.clean: .FORCE
	rm -rf $(LIBRARY.BUILD)

build: build.clean
	mkdir -p $(LIBRARY.BUILD)
	cd $(LIBRARY.BUILD) && $(CMAKE.BIN) -DCMAKE_PREFIX_PATH=$(INSTALLED.HOST.DIR) -DCMAKE_INSTALL_PREFIX=$(INSTALLED.HOST.DIR) $(LIBRARY.DIR) && make $(J) install

tests: tests.run

tests.run: .FORCE
	LD_LIBRARY_PATH=$(LIB.DIR) $(TESTS.BIN)

tests.gdb: .FORCE
	LD_LIBRARY_PATH=$(LIB.DIR) gdb $(TESTS.BIN)

cmake.fetch: .FORCE
	mkdir -p $(DOWNLOADS.DIR)
	cd $(DOWNLOADS.DIR) && wget $(CMAKE.URL) && cd $(DOWNLOADS.DIR) &&  tar xf $(CMAKE.ARCHIVE)

cmake: cmake.clean cmake.fetch
	cd $(CMAKE.DIR) && ./configure --prefix=$(INSTALLED.HOST.DIR) --no-system-zlib --parallel=$(J)  && make -j$(J) install

cmake.clean: .FORCE
	rm -f $(DOWNLOADS.DIR)/$(CMAKE.ARCHIVE)
	rm -rf $(CMAKE.DIR)

clean: .FORCE
	rm -rf $(INSTALLED.HOST.DIR)
	rm -rf $(DOWNLOADS.DIR)

.FORCE:
