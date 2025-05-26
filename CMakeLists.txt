cmake_minimum_required(VERSION 3.25)

# Only set MSVC runtime library on Windows
if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Release>:Release>")
endif()

project(raytracer LANGUAGES CXX CUDA)

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_ARCHITECTURES native)

file(GLOB_RECURSE SOURCE_FILES "src/*.cu")
file(GLOB_RECURSE HEADER_FILES "src/*.h" "include/*.h" "include/*.hpp")

find_package(CUDAToolkit REQUIRED)

message("CMAKE_CUDA_ARCHITECTURES: ${CMAKE_CUDA_ARCHITECTURES}")
message("CUDAToolKit_LIBRARY_ROOT: ${CUDAToolKit_LIBRARY_ROOT}")
message("CUDAToolKit_INCLUDE_DIRS: ${CUDAToolKit_INCLUDE_DIRS}")
message("CUDAToolKit_LIBRARY_DIR: ${CUDAToolKit_LIBRARY_DIR}")

include_directories(${CUDAToolKit_INCLUDE_DIRS})

add_executable(
    ${PROJECT_NAME}
    "${SOURCE_FILES}"
    "${HEADER_FILES}"
)

target_compile_definitions(${PROJECT_NAME} PUBLIC RESOURCES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/res")
#target_compile_definitions(${PROJECT_NAME} PUBLIC RESOURCES_PATH="$./res")

target_include_directories(${PROJECT_NAME}
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

set_target_properties(
    ${PROJECT_NAME}
    PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
)

target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE
    CUDA::cudart
)

# End of File
