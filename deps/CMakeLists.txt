# disable dependencies related warnings
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-everything")

add_subdirectory(WindowHandler EXCLUDE_FROM_ALL)
add_subdirectory(tinyobjloader)
