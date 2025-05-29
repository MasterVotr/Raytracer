NVCC := nvcc
NVCCFLAGS := -std=c++17 -I.

SRC_DIR := src
OBJ_DIR := build
SRC_FILES := $(shell find $(SRC_DIR) -name '*.cu')
OBJ_FILES := $(patsubst $(SRC_DIR)/%.cu,$(OBJ_DIR)/%.o,$(SRC_FILES))
OUT := raytracer

.PHONY: compile run clean

compile: $(OUT)

$(OUT): $(OBJ_FILES)
	$(NVCC) $(NVCCFLAGS) $^ -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cu
	@mkdir -p $(dir $@)
	$(NVCC) $(NVCCFLAGS) -c $< -o $@

run: $(OUT)
	./$(OUT) config.json

clean:
	@rm -rf $(OUT)
	@rm -rf $(OBJ_DIR)