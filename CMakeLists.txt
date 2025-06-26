cmake_minimum_required(VERSION 3.10)

project(renderer)

cmake_policy(SET CMP0074 NEW)

# --- Homebrew LLVM を使う ---
set(CMAKE_C_COMPILER  "/opt/homebrew/opt/llvm/bin/clang"   CACHE STRING "" FORCE)
set(CMAKE_CXX_COMPILER "/opt/homebrew/opt/llvm/bin/clang++" CACHE STRING "" FORCE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# libomp のプレフィックスを教える
set(OpenMP_ROOT "/opt/homebrew/opt/libomp")
find_package(OpenMP REQUIRED)

# 自作マクロ: target に OpenMP をくっ付ける
function(link_openmp target)
  target_link_libraries(${target} PRIVATE OpenMP::OpenMP_CXX)
endfunction()

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
link_openmp(raytracing)

add_executable(cos
	src/CosWeightPT.cpp
	src/Vec3.cpp
	src/Ray.cpp
   	src/Camera.cpp
)
link_openmp(cos)
add_executable(nee
	src/NEE.cpp
	src/Vec3.cpp
	src/Ray.cpp
   	src/Camera.cpp
)
link_openmp(nee)
add_executable(bdpt
	src/BDPT.cpp
	src/Vec3.cpp
   	src/Ray.cpp
   	src/Camera.cpp
)
link_openmp(bdpt)
add_executable(compPT
	src/comparePT.cpp
	src/Vec3.cpp
   	src/Ray.cpp
   	src/Camera.cpp
)
link_openmp(compPT)
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

add_executable(test
	test/pathPDF.cpp
	src/Vec3.cpp
   	src/Ray.cpp
   	src/Camera.cpp
)
add_executable(st
	test/stStrategy.cpp
	src/Vec3.cpp
   	src/Ray.cpp
   	src/Camera.cpp
)
link_openmp(st)


# stb を置いたディレクトリを追加
add_library(stb INTERFACE)
target_include_directories(stb INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/external)

target_link_libraries(pfm2png PRIVATE stb)
target_compile_definitions(pfm2png PRIVATE STB_IMAGE_WRITE_IMPLEMENTATION)
