cmake_minimum_required(VERSION 3.10)

project(ex_ray_tracing)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Material
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Objects
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Renderer/bdpt
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Renderer/pt
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Renderer/rt

)

add_executable(app
	 src/main.cpp
	 src/Vec3.cpp
	 src/Ray.cpp
   	 src/Camera.cpp
)
add_executable(comparison
	 src/comparePT.cpp
	 src/Vec3.cpp
   	 src/Ray.cpp
   	 src/Camera.cpp
)
add_executable(bdpt
	 src/main_bdpt.cpp
	 src/Vec3.cpp
   	 src/Ray.cpp
   	 src/Camera.cpp
)

# stb を置いたディレクトリを追加（例: external/stb）
add_library(stb INTERFACE)
target_include_directories(stb INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/external)

# app が唯一の実装を持つ
target_link_libraries(app PRIVATE stb)
target_compile_definitions(app PRIVATE STB_IMAGE_WRITE_IMPLEMENTATION)

# comparePT も同様の設定
target_link_libraries(comparison PRIVATE stb)
target_compile_definitions(comparison PRIVATE STB_IMAGE_WRITE_IMPLEMENTATION)

# main_bdpt も同様の設定
target_link_libraries(bdpt PRIVATE stb)
target_compile_definitions(bdpt PRIVATE STB_IMAGE_WRITE_IMPLEMENTATION)