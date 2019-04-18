#add_definitions(-DVK_PROTOTYPES) # is it necessary ?
add_definitions(-D_CRT_SECURE_NO_WARNINGS)

if (WIN32)
    add_definitions(-DVK_USE_PLATFORM_WIN32_KHR)
	add_definitions(-DNOMINMAX)
endif (WIN32)

# Vulkan
find_library(VULKAN_LIBRARY
  NAMES vulkan vulkan-1 HINTS $ENV{VK_SDK_PATH}/Lib)

# Generating vulkan renderer library
file(GLOB_RECURSE VULKAN_RENDERER_SRC ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_library(vulkan_renderer SHARED ${VULKAN_RENDERER_SRC})

target_include_directories(vulkan_renderer PRIVATE ${PROJECT_SOURCE_DIR}/include
                                                   ${PROJECT_SOURCE_DIR}/src
												   ${PROJECT_SOURCE_DIR}/deps
												   ${PROJECT_SOURCE_DIR}/deps/eigen
												   ${PROJECT_SOURCE_DIR}/deps/glfw/include
												   ${PROJECT_SOURCE_DIR}/deps/vma/include
												   $ENV{VK_SDK_PATH}/Include
												   ${PROJECT_SOURCE_DIR}/resources/shaders/hr/include)

target_link_libraries(vulkan_renderer PRIVATE ${VULKAN_LIBRARY} tinyobjloader)
set_target_properties(vulkan_renderer PROPERTIES COMPILE_FLAGS "${COMPILE_FLAGS} -DVULKAN_RENDERER_DLL_EXPORTS")
