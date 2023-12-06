set(BINARY_DIR "${CMAKE_SOURCE_DIR}/bin")
set(PROJECT_NAME "client")
project(${PROJECT_NAME} VERSION 1.0.0 LANGUAGES C)
add_executable(
	${PROJECT_NAME}
	client.c
)
target_include_directories(
)
install(
	TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION ${BINARY_DIR}
)
