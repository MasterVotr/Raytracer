TINYOBJ_INCLUDE_PATH = include/tiny_obj_loader.h

CXX = g++
CFLAGS = -std=c++17
CFLAGSDEBUG = -std=c++17 -Wall -Wextra -Wpedantic -fsanitize=address -g

.PHONY: run clean raytracer

raytracer: raytracer src/main.cpp
	$(CXX) $(CFLAGS) src/main.cpp -o raytracer

raytracer_debug: raytracer src/main.cpp
	$(CXX) $(CFLAGSDEBUG) src/main.cpp -o raytracer

run: raytracer
	./raytracer

clean:
	@rm -rf raytracer
	@rm -rf raytracer.dSYM
	@rm -rf output.ppm