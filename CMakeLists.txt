cmake_minimum_required(VERSION 3.10)

project(renderer)

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

add_executable(raytracing
	 src/Raytracing.cpp
	 src/Vec3.cpp
	 src/Ray.cpp
   	 src/Camera.cpp
)
add_executable(cosweight
	 src/CosWeightPT.cpp
	 src/Vec3.cpp
	 src/Ray.cpp
   	 src/Camera.cpp
)
add_executable(nee
	 src/NEE.cpp
	 src/Vec3.cpp
	 src/Ray.cpp
   	 src/Camera.cpp
)
add_executable(bdpt
	 src/BDPT.cpp
	 src/Vec3.cpp
   	 src/Ray.cpp
   	 src/Camera.cpp
)
add_executable(compPT
	 src/comparePT.cpp
	 src/Vec3.cpp
   	 src/Ray.cpp
   	 src/Camera.cpp
)
add_executable(subtract
	 diff/diff.cpp
)
add_executable(gauss
	 diff/Gaussian_Blur.cpp
)
add_executable(abs
	 diff/abs.cpp
)
add_executable(diff
	 diff/compare.cpp
)
add_executable(pfm2png
	 tools/pfm2png.cpp
)


# stb を置いたディレクトリを追加
add_library(stb INTERFACE)
target_include_directories(stb INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/external)

target_link_libraries(pfm2png PRIVATE stb)
target_compile_definitions(pfm2png PRIVATE STB_IMAGE_WRITE_IMPLEMENTATION)
