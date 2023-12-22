cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PROJECT_NAME "server_com")
set(ADDITIONAL_FLAGS "")
project(${PROJECT_NAME} VERSION 1.0.0 LANGUAGES C)
add_subdirectory(${CMAKE_SOURCE_DIR}/server)
add_subdirectory(${CMAKE_SOURCE_DIR}/client)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
	set(CMAKE_C_FLAGS
		"${CMAKE_C_FLAGS} -Wno-implicit-function-declaration -Wall -Wextra -O0 ${ADDITIONAL_FLAGS}")
	target_link_options(${PROJECT_NAME} BEFORE PUBLIC -fsanitize=undefined PUBLIC "")
endif()

if (CMAKE_BUILD_TYPE MATCHES "Release")
	set(CMAKE_C_FLAGS
		"${CMAKE_C_FLAGS} -Wno-implicit-function-declaration -Wall -Wextra -O3")
endif()
