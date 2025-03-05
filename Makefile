TINYOBJ_INCLUDE_PATH = include/tiny_obj_loader.h

CFLAGS = -std=c++17 -I$(TINYOBJ_INCLUDE_PATH) -g

raytracer: src/main.cpp
	g++ $(CFLAGS) src/main.cpp -o raytracer

.PHONY: run clean

run: raytracer
	./raytracer

clean:
	@rm -rf raytracer
	@rm -rf output.ppm