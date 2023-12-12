set(BINARY_DIR "${CMAKE_SOURCE_DIR}/bin")
set(SHARED_LIB_DIR "${CMAKE_SOURCE_DIR}/lib")
set(PROJECT_NAME "client")
project(${PROJECT_NAME} VERSION 1.0.0 LANGUAGES C)
add_executable(
	${PROJECT_NAME}
	client.c
	${SHARED_LIB_DIR}/output_format.c
)
target_include_directories(
	${PROJECT_NAME}
	PUBLIC
	${SHARED_LIB_DIR}
)
install(
	TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION ${BINARY_DIR}
)
