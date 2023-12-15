set(BINARY_DIR "${CMAKE_SOURCE_DIR}/bin")
set(PUBLIC_LIB_DIR "${CMAKE_SOURCE_DIR}/lib")
set(PROJECT_NAME "client")
project(${PROJECT_NAME} VERSION 1.0.0 LANGUAGES C)
add_executable(
	${PROJECT_NAME}
	client.c
	${PUBLIC_LIB_DIR}/output_format.c
	${PUBLIC_LIB_DIR}/lib_system.c
)
target_include_directories(
	${PROJECT_NAME}
	PUBLIC
	${PUBLIC_LIB_DIR}
)
install(
	TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION ${BINARY_DIR}
)
