cmake_minimum_required(VERSION 3.10)

project(ex_ray_tracing)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Material
  ${CMAKE_CURRENT_SOURCE_DIR}/include/bdpt
  ${CMAKE_CURRENT_SOURCE_DIR}/include/pt
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rt

)

add_executable(app
	 src/main.cpp
	 src/Vec3.cpp
   src/Ray.cpp
   src/Camera.cpp
)

# Catch2の追加（ヘッダーのみ）
include(FetchContent)
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.5.2
)
FetchContent_MakeAvailable(Catch2)

# テスト用バイナリ
add_executable(Vec3Tests
    tests/test_vec3.cpp
    src/Vec3.cpp
)
target_link_libraries(Vec3Tests PRIVATE Catch2::Catch2WithMain)

enable_testing()
add_test(NAME Vec3Test COMMAND Vec3Tests)

# stb を置いたディレクトリを追加（例: external/stb）
add_library(stb INTERFACE)
target_include_directories(stb INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/external)

# app が唯一の実装を持つ
target_link_libraries(app PRIVATE stb)
target_compile_definitions(app PRIVATE STB_IMAGE_WRITE_IMPLEMENTATION)