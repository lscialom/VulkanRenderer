cmake_minimum_required(VERSION 3.5)
project(VulkanRenderer CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
endif()

# Output config
set(OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# Exe path
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${OUTPUT_DIRECTORY}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${OUTPUT_DIRECTORY}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${OUTPUT_DIRECTORY}")

# Lib path
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${OUTPUT_DIRECTORY}/archive")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${OUTPUT_DIRECTORY}/archive")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO "${OUTPUT_DIRECTORY}/archive")

# Compile flags
if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Xclang -std=c++17 -Xclang -fcxx-exceptions /fp:fast /arch:AVX -flto -fwhole-program-vtables -Xclang -O0") # /FA for asm, -Rpass-analysis=loop-vectorize for vectorization check
	add_definitions(-DDEBUG)
else()
 	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Xclang -std=c++17 -Xclang -fcxx-exceptions /fp:fast /arch:AVX -flto -fwhole-program-vtables -Xclang -O3") # AVX2 is being slower with Eigen since it prefers 128-bits (rather than 256) packets
	add_definitions(-DNDEBUG)
endif()

message(STATUS "Using ${CMAKE_CXX_COMPILER_ID} in ${CMAKE_BUILD_TYPE} mode")
message(STATUS "Flags: ${CMAKE_CXX_FLAGS}")

# Global includes
#include_directories(${PROJECT_SOURCE_DIR}/include)
#include_directories(${PROJECT_SOURCE_DIR}/deps/eigen)

add_subdirectory(${PROJECT_SOURCE_DIR}/deps EXCLUDE_FROM_ALL)
add_subdirectory(${PROJECT_SOURCE_DIR}/src EXCLUDE_FROM_ALL)

#executable for testing/example
file(GLOB sources main.cpp)

add_executable(TestExe ${sources})
target_include_directories(TestExe PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(TestExe vulkan_renderer)