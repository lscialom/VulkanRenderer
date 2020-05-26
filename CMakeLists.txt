cmake_minimum_required(VERSION 3.5)
project(VulkanRenderer CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
endif()

#Output config
set(OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

#Exe path
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${OUTPUT_DIRECTORY}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${OUTPUT_DIRECTORY}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${OUTPUT_DIRECTORY}")

#Lib path
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${OUTPUT_DIRECTORY}/archive")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${OUTPUT_DIRECTORY}/archive")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO "${OUTPUT_DIRECTORY}/archive")

set(CMAKE_CXX_STANDARD 17)

#Compile flags
if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Xclang -std=c++17 -Xclang -fcxx-exceptions /fp:fast /arch:AVX -Xclang -O0") # /FA for asm, -Rpass-analysis=loop-vectorize for vectorization check
	add_definitions(-DDEBUG)
else()
 	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Xclang -std=c++17 -Xclang -fcxx-exceptions /fp:fast /arch:AVX -Xclang -O3") # AVX2 is being slower with Eigen since it prefers 128-bits (rather than 256) packets
	add_definitions(-DNDEBUG)
endif()

message(STATUS "Using ${CMAKE_CXX_COMPILER_ID} in ${CMAKE_BUILD_TYPE} mode")
message(STATUS "Flags: ${CMAKE_CXX_FLAGS}")

link_directories(libs/${CMAKE_BUILD_TYPE})

#Global includes
#include_directories(${PROJECT_SOURCE_DIR} / include)
#include_directories(${PROJECT_SOURCE_DIR} / deps / eigen)

add_subdirectory(${PROJECT_SOURCE_DIR}/deps EXCLUDE_FROM_ALL)
add_subdirectory(${PROJECT_SOURCE_DIR}/src EXCLUDE_FROM_ALL)

#executable for testing / example
file(GLOB sources main.cpp)

add_executable(TestExe ${sources})
target_include_directories(TestExe PRIVATE ${PROJECT_SOURCE_DIR}/include
                                           ${PROJECT_SOURCE_DIR}/deps/WindowHandler/include
										   ${PROJECT_SOURCE_DIR}/deps/eigen
										   ${PROJECT_SOURCE_DIR}/deps/ExtImGui)

target_link_libraries(TestExe window_handler vulkan_renderer ext_imgui) #Replace with your own window handler if needed

if (WIN32)
	set(ext_imgui_path ${CMAKE_CURRENT_SOURCE_DIR}/libs/${CMAKE_BUILD_TYPE}/ext_imgui.dll)
	set(test_exe_bin_folder ${OUTPUT_DIRECTORY})
	FILE(TO_NATIVE_PATH ${ext_imgui_path} ext_imgui_path)
	FILE(TO_NATIVE_PATH ${test_exe_bin_folder} test_exe_bin_folder)

	add_custom_command(TARGET TestExe
	    # Run after sources have been compiled but before linking the binary or running the librarian or archiver tool of a static library. This is not defined for targets created by the add_custom_target() command
	    PRE_LINK
	    COMMAND copy ${ext_imgui_path} ${test_exe_bin_folder}
	    VERBATIM
	)
else()
	add_custom_command(TARGET TestExe
	    # Run after sources have been compiled but before linking the binary or running the librarian or archiver tool of a static library. This is not defined for targets created by the add_custom_target() command
	    PRE_LINK
	    COMMAND cp "${CMAKE_CURRENT_SOURCE_DIR}/libs/${CMAKE_BUILD_TYPE}/ext_imgui.dll" "${CMAKE_CURRENT_SOURCE_DIR}/bin"
	    VERBATIM
	)
endif (WIN32)

# file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/libs/${CMAKE_BUILD_TYPE}/ext_imgui.dll
#        DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin)
