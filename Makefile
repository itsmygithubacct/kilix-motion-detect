PROJECT := kilix-motion-detect
BUILD_DIR ?= build
PREFIX ?= /usr/local
DESTDIR ?=

CC ?= cc
AR ?= ar
INSTALL ?= install

CPPFLAGS += -D_POSIX_C_SOURCE=200809L -Iinclude
WARNINGS := \
	-Wall -Wextra -Wpedantic -Wconversion -Wshadow \
	-Wstrict-prototypes -Wmissing-prototypes -Wformat=2 -Werror
CFLAGS ?= -O2 -g
override CFLAGS += -std=c11 -fPIC $(WARNINGS)
EXTRA_CFLAGS ?=
EXTRA_LDFLAGS ?=
DEPFLAGS := -MMD -MP

STATIC_LIB := $(BUILD_DIR)/lib$(PROJECT).a
SHARED_LIB := $(BUILD_DIR)/lib$(PROJECT).so
OBJECTS := $(BUILD_DIR)/kilix_motion_detect.o

TEST_SOURCES := $(wildcard tests/test_*.c)
TEST_BINARIES := $(patsubst tests/test_%.c,$(BUILD_DIR)/test-%,$(TEST_SOURCES))

BENCH_BIN := $(BUILD_DIR)/kmd-bench
STREAM_BIN := $(BUILD_DIR)/kmd-stream
FUZZ_BIN := $(BUILD_DIR)/fuzz-motion
FUZZ_CASES ?= 10000
SANITIZE_BUILD_DIR ?= $(BUILD_DIR)/sanitize
SANITIZER_FLAGS := -fsanitize=address,undefined -fno-omit-frame-pointer

.PHONY: all test sanitize fuzz bench stream install clean

all: $(STATIC_LIB) $(SHARED_LIB)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: src/%.c include/kilix_motion_detect.h | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(EXTRA_CFLAGS) $(DEPFLAGS) -c $< -o $@

$(STATIC_LIB): $(OBJECTS)
	$(AR) rcs $@ $^

$(SHARED_LIB): $(OBJECTS)
	$(CC) -shared $(LDFLAGS) $(EXTRA_LDFLAGS) $^ $(LDLIBS) -o $@

$(BUILD_DIR)/test-%: tests/test_%.c $(STATIC_LIB) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(EXTRA_CFLAGS) $(LDFLAGS) \
		$(EXTRA_LDFLAGS) $< $(STATIC_LIB) $(LDLIBS) -o $@

test: $(TEST_BINARIES) $(STREAM_BIN)
	@set -e; for binary in $(TEST_BINARIES); do \
		printf '\n== %s ==\n' "$$binary"; \
		"$$binary"; \
	done; \
	printf '\nall test suites passed\n'
	KMD_STREAM=$(abspath $(STREAM_BIN)) sh tests/test_stream.sh

# The cost side of every claim in the README: per-frame time at several
# working heights, with and without contrast normalisation.
bench: $(BENCH_BIN)
	@$(BENCH_BIN)

$(BENCH_BIN): examples/bench.c $(STATIC_LIB) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(EXTRA_CFLAGS) $(LDFLAGS) \
		$(EXTRA_LDFLAGS) $< $(STATIC_LIB) $(LDLIBS) -o $@

# Runs the detector over raw frames on stdin, for measuring against real
# footage rather than synthetic scenes.
stream: $(STREAM_BIN)

$(STREAM_BIN): examples/kmd-stream.c $(STATIC_LIB) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(EXTRA_CFLAGS) $(LDFLAGS) \
		$(EXTRA_LDFLAGS) $< $(STATIC_LIB) $(LDLIBS) -o $@

$(FUZZ_BIN): tests/fuzz_motion.c $(STATIC_LIB) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(EXTRA_CFLAGS) $(LDFLAGS) \
		$(EXTRA_LDFLAGS) $< $(STATIC_LIB) $(LDLIBS) -o $@

fuzz: $(FUZZ_BIN)
	$(FUZZ_BIN) $(FUZZ_CASES)

sanitize:
	@$(MAKE) --no-print-directory BUILD_DIR="$(SANITIZE_BUILD_DIR)" clean
	@$(MAKE) --no-print-directory BUILD_DIR="$(SANITIZE_BUILD_DIR)" \
		EXTRA_CFLAGS="$(SANITIZER_FLAGS)" \
		EXTRA_LDFLAGS="-fsanitize=address,undefined" test fuzz

install: all
	$(INSTALL) -d $(DESTDIR)$(PREFIX)/include
	$(INSTALL) -m 644 include/kilix_motion_detect.h $(DESTDIR)$(PREFIX)/include/
	$(INSTALL) -d $(DESTDIR)$(PREFIX)/lib
	$(INSTALL) -m 644 $(STATIC_LIB) $(DESTDIR)$(PREFIX)/lib/
	$(INSTALL) -m 644 $(SHARED_LIB) $(DESTDIR)$(PREFIX)/lib/

clean:
	rm -rf $(BUILD_DIR)

-include $(OBJECTS:.o=.d)
