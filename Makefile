TINYOBJ_INCLUDE_PATH = include/tiny_obj_loader.h

CXX = g++
CFLAGS = -std=c++17
CFLAGSDEBUG = -std=c++17 -Wall -Wextra -Wpedantic -fsanitize=address -g

.PHONY: run clean raytracer

raytracer: src/main.cpp
	$(CXX) $(CFLAGS) src/main.cpp -o raytracer.out

raytracer_debug: src/main.cpp
	$(CXX) $(CFLAGSDEBUG) src/main.cpp -o raytracer.out

run: raytracer
	./raytracer.out config.json

clean:
	@rm -rf raytracer.out
	@rm -rf raytracer.out.dSYM
	@rm -rf output.ppm