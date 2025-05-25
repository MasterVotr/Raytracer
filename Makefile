
CXX := g++
CFLAGS := -std=c++17 -I.
CFLAGSDEBUG := $(CFLAGS) -Wall -Wextra -Wpedantic -fsanitize=address -g

SRC_DIR := src
OBJ_DIR := build
SRC_FILES := $(shell find $(SRC_DIR) -name '*.cc')
OBJ_FILES := $(patsubst $(SRC_DIR)/%.cc,$(OBJ_DIR)/%.o,$(SRC_FILES))
OUT := raytracer.out

.PHONY: compile run clean

compile: $(OUT)

$(OUT): $(OBJ_FILES)
	$(CXX) $(CFLAGS) $^ -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cc
	@mkdir -p $(dir $@)
	$(CXX) $(CFLAGS) -c $< -o $@

run: $(OUT)
	./$(OUT) config.json

clean:
	@rm -rf raytracer.out
	@rm -rf raytracer.out.dSYM
	@rm -rf output.ppm
	@rm -rf $(OBJ_DIR)