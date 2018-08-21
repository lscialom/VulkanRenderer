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
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Xclang -std=c++17 -Xclang -fcxx-exceptions -Rpass-analysis=loop-vectorize /fp:fast /arch:AVX -flto -fwhole-program-vtables -Xclang -O0") # /FA for asm
	add_definitions(-DDEBUG)
else()
 	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Xclang -std=c++17 -Xclang -fcxx-exceptions -Rpass-analysis=loop-vectorize /fp:fast /arch:AVX -flto -fwhole-program-vtables -Xclang -O3") # AVX2 is being slower with Eigen since it prefers 128-bits (rather than 256) packets
	add_definitions(-DNDEBUG)
endif()

add_definitions(-DVK_PROTOTYPES)

message(STATUS "Using ${CMAKE_CXX_COMPILER_ID} in ${CMAKE_BUILD_TYPE} mode")
message(STATUS "Flags: ${CMAKE_CXX_FLAGS}")

# GLFW
set(GLFW_DIR deps/glfw) # Set this to point to an up-to-date GLFW repo
option(GLFW_BUILD_EXAMPLES "Build the GLFW example programs" OFF)
option(GLFW_BUILD_TESTS "Build the GLFW test programs" OFF)
option(GLFW_BUILD_DOCS "Build the GLFW documentation" OFF)
option(GLFW_INSTALL "Generate installation target" OFF)
option(GLFW_DOCUMENT_INTERNALS "Include internals in documentation" OFF)
add_subdirectory(${GLFW_DIR} binary_dir EXCLUDE_FROM_ALL)

# Vulkan
find_library(VULKAN_LIBRARY
  NAMES vulkan vulkan-1)

# Setting libraries
set(LIBRARIES "glfw;${VULKAN_LIBRARY}")

# Global includes
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/deps/eigen)

# Generating vulkan renderer library
file(GLOB VULKAN_RENDERER_SRC ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_library(vulkan_renderer SHARED ${VULKAN_RENDERER_SRC})
target_include_directories(vulkan_renderer PRIVATE ${PROJECT_SOURCE_DIR}/src ${GLFW_DIR}/include ${GLFW_DIR}/deps)
target_link_libraries(vulkan_renderer PRIVATE ${LIBRARIES})
set_target_properties(vulkan_renderer PROPERTIES COMPILE_FLAGS "${COMPILE_FLAGS} -DVULKAN_RENDERER_DLL_EXPORTS")

#executable for testing/example
file(GLOB sources main.cpp)

add_executable(TestExe ${sources})
target_link_libraries(TestExe vulkan_renderer)