cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PROJECT_NAME "server_com")
project(${PROJECT_NAME} VERSION 1.0.0 LANGUAGES C)
add_subdirectory(${CMAKE_SOURCE_DIR}/server)
add_subdirectory(${CMAKE_SOURCE_DIR}/client)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
	set(CMAKE_C_FLAGS
		"${CMAKE_C_FLAGS} -Werror -Wno-implicit-function-declaration -Wno-write-strings -O3 -fsanitize=undefined -fsanitize=address")
endif()

if (CMAKE_BUILD_TYPE MATCHES "Release")
	set(CMAKE_C_FLAGS
		"${CMAKE_C_FLAGS} -Werror -Wno-implicit-function-declaration -Wno-write-strings -O3")
endif()
