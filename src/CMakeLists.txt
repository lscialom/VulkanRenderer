#add_definitions(-DVK_PROTOTYPES) # is it necessary ?
add_definitions(-D_CRT_SECURE_NO_WARNINGS)

# Vulkan
find_library(VULKAN_LIBRARY
  NAMES vulkan vulkan-1)

# Setting libraries
set(LIBRARIES "glfw;${VULKAN_LIBRARY}")

# Generating vulkan renderer library
file(GLOB VULKAN_RENDERER_SRC ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_library(vulkan_renderer SHARED ${VULKAN_RENDERER_SRC})

target_include_directories(vulkan_renderer PRIVATE ${PROJECT_SOURCE_DIR}/include
                                                   ${PROJECT_SOURCE_DIR}/src
												   ${PROJECT_SOURCE_DIR}/deps
												   ${PROJECT_SOURCE_DIR}/deps/eigen
												   ${PROJECT_SOURCE_DIR}/deps/glfw/include
												   ${PROJECT_SOURCE_DIR}/deps/glfw/deps
												   ${PROJECT_SOURCE_DIR}/deps/vma/include)

target_link_libraries(vulkan_renderer PRIVATE ${LIBRARIES})
set_target_properties(vulkan_renderer PROPERTIES COMPILE_FLAGS "${COMPILE_FLAGS} -DVULKAN_RENDERER_DLL_EXPORTS")