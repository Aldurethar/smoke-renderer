if(ShadersToQRC_INCLUDED)
	return()
endif()
set(ShadersToQRC_INCLUDED ON)

source_group("Shader Files" REGULAR_EXPRESSION "\\.(vert|tesc|tese|geom|frag|comp|glsl)$")

macro(shaders_to_qrc)
	set(EXTRA_ARGS ${ARGN})
	list(LENGTH EXTRA_ARGS NUM_EXTRA_ARGS)
	set(name shaders)
	if(NUM_EXTRA_ARGS GREATER 0)
		list(GET EXTRA_ARGS 0 name)
	endif()
	if(NUM_EXTRA_ARGS GREATER 1)
		list(GET EXTRA_ARGS 1 prefix)
		file(WRITE ${PROJECT_BINARY_DIR}/${name}.qrc "<RCC><qresource prefix=\"${prefix}\">")
	else()
		file(WRITE ${PROJECT_BINARY_DIR}/${name}.qrc "<RCC><qresource>")
	endif()
	get_target_property(_SOURCES ${PROJECT_NAME} SOURCES)
	source_group("Generated Files" FILES ${PROJECT_BINARY_DIR}/shaders.qrc)
	foreach(_SOURCE ${_SOURCES})
		if(_SOURCE MATCHES "\\.(vert|tesc|tese|geom|frag|comp|glsl)$")
			file(APPEND ${PROJECT_BINARY_DIR}/${name}.qrc "<file alias=\"${_SOURCE}\">${PROJECT_SOURCE_DIR}/${_SOURCE}</file>")
		endif()
	endforeach()
	file(APPEND ${PROJECT_BINARY_DIR}/${name}.qrc "</qresource></RCC>")
	target_sources(${PROJECT_NAME} PRIVATE ${PROJECT_BINARY_DIR}/${name}.qrc)
endmacro()
