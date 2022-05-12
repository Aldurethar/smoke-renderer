project(OpenGLObjects)

add_library(
	${PROJECT_NAME}
	STATIC
	src/OpenGLObjects.cpp
	include/OpenGLObjects.h
)

target_link_libraries(
	${PROJECT_NAME}
	PUBLIC
	glad
)

target_compile_definitions(
	${PROJECT_NAME}
	PUBLIC
	USE_GLAD
)

target_include_directories(
	${PROJECT_NAME}
	PUBLIC
	include
)
