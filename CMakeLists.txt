# require a sane minimum version of cmake
cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

# build debug by default with single-config generators
# must be done *before* first call to project()
set(CMAKE_BUILD_TYPE_INIT Debug)

# disable in-source builds https://stackoverflow.com/questions/1208681/with-cmake-how-would-you-disable-in-source-builds
# must be done *before* first call to project()
set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

# setup base project
project(SimulationFramework)

# provide a list of config choices for single-config generators when using cmake-gui/ccmake
if(DEFINED CMAKE_BUILD_TYPE)
	set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release MinSizeRel RelWithDebInfo)
endif()

# set up sane default C++ standard settings for all projects
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# tell cmake where to find included .cmake files
list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/CMake")

# setup glad to generate code for OpenGL 3.3
# remember: CACHE INTERNAL implies FORCE!
set(GLAD_API gl=4.5 CACHE INTERNAL "")
set(GLAD_EXPORT ON CACHE INTERNAL "")
set(GLAD_EXTENSIONS GL_EXT_texture_compression_s3tc,GL_EXT_texture_sRGB,GL_EXT_texture_filter_anisotropic CACHE INTERNAL "") # https://www.khronos.org/opengl/wiki/Ubiquitous_Extension
set(GLAD_GENERATOR c CACHE INTERNAL "")
set(GLAD_INSTALL OFF CACHE INTERNAL "")
set(GLAD_NO_LOADER OFF CACHE INTERNAL "")
set(GLAD_PROFILE compatibility CACHE INTERNAL "")
set(GLAD_SPEC gl CACHE INTERNAL "")
set(GLAD_REPRODUCIBLE OFF CACHE INTERNAL "")
add_subdirectory(glad)

#assimp cmake variables
set(ASSIMP_ANDROID_JNIIOSYSTEM OFF CACHE INTERNAL "")
set(ASSIMP_BIN_INSTALL_DIR bin CACHE INTERNAL "")
set(ASSIMP_BUILD_3DS_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_3D_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_3MF_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_AC_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_ASE_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_ASSBIN_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_ASSIMP_VIEW OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_ASSXML_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_B3D_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_BLEND_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_BVH_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_COB_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_COLLADA_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_CSM_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_DXF_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_FBX_IMPORTER ON CACHE INTERNAL "")
set(ASSIMP_BUILD_GLTF_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_HMP_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_IFC_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_IRRMESH_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_IRR_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_LWO_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_LWS_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_MD2_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_MD3_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_MD5_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_MDC_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_MDL_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_MS3D_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_NDO_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_NFF_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_NONFREE_C4D_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_OBJ_IMPORTER ON CACHE INTERNAL "")
set(ASSIMP_BUILD_OFF_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_OGRE_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_OPENGEX_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_PLY_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_Q3BSP_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_Q3D_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_RAW_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_SAMPLES OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_SIB_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_SMD_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_STL_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_TERRAGEN_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_TESTS OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_XGL_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_X_IMPORTER OFF CACHE INTERNAL "")
set(ASSIMP_BUILD_ZLIB OFF CACHE INTERNAL "")
set(ASSIMP_HEADERCHECK OFF CACHE INTERNAL "")
set(ASSIMP_INCLUDE_INSTALL_DIR include CACHE INTERNAL "")
set(ASSIMP_INSTALL_PDB ON CACHE INTERNAL "")
set(ASSIMP_LIBRARY_SUFFIX "" CACHE INTERNAL "")
set(ASSIMP_LIB_INSTALL_DIR lib CACHE INTERNAL "")
set(ASSIMP_NO_EXPORT OFF CACHE INTERNAL "")
set(ASSIMP_OPT_BUILD_PACKAGES OFF CACHE INTERNAL "")
set(ASSIMP_PACKAGE_VERSION 0 CACHE INTERNAL "")
set(DirectX_D3DCOMPILER_LIBRARY DirectX_D3DCOMPILER_LIBRARY-NOTFOUND CACHE INTERNAL "")
set(DirectX_D3DX9_LIBRARY DirectX_D3DX9_LIBRARY-NOTFOUND CACHE INTERNAL "")
set(DirectX_DXERR_LIBRARY DirectX_DXERR_LIBRARY-NOTFOUND CACHE INTERNAL "")
set(DirectX_DXGI_LIBRARY DirectX_DXGI_LIBRARY-NOTFOUND CACHE INTERNAL "")
set(DirectX_DXGUID_LIBRARY DirectX_DXGUID_LIBRARY-NOTFOUND CACHE INTERNAL "")
set(DirectX_INCLUDE_DIR DirectX_INCLUDE_DIR-NOTFOUND CACHE INTERNAL "")
set(DirectX_LIBRARY DirectX_LIBRARY-NOTFOUND CACHE INTERNAL "")
set(CMAKE_INSTALL_PREFIX "C:/Development/Libraries" CACHE INTERNAL "")
add_subdirectory(assimp)

# add OpenGLObjects library
add_subdirectory(OpenGLObjects)

# set up folders for better manageability in IDEs
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_target_properties(
	glad OpenGLObjects assimp
	PROPERTIES
	FOLDER Libraries
)

# setup CMAKE_PREFIX_PATH to use included copy of Eigen by default
list(APPEND CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/eigen/share/eigen3/cmake")
find_package(Eigen3 REQUIRED)

# add an executable target and make it the default debug/startup project on VS
add_executable(${PROJECT_NAME})
set_directory_properties(
	PROPERTIES
	VS_STARTUP_PROJECT ${PROJECT_NAME}
)

if(WIN32)
	# not a console application
	set_property(TARGET ${PROJECT_NAME} PROPERTY WIN32_EXECUTABLE ON)

	# add a pretty executable icon
	target_sources(
		${PROJECT_NAME}
		PRIVATE
		icon.rc
	)
endif()

# set target sources
target_sources(
	${PROJECT_NAME}
	PRIVATE
	main.cpp
	OpenGLWidget.cpp OpenGLWidget.hpp
	OpenGLRenderer.hpp
	GLMainWindow.cpp GLMainWindow.hpp GLMainWindow.ui	
	MyRenderer.cpp MyRenderer.hpp
	MyRendererUtils.hpp
	FileIO.hpp
	constants.hpp	
	shaders/phong_textured.vert shaders/phong_textured.frag
	shaders/phong_color.vert shaders/phong_color.frag
	shaders/depth.vert shaders/depth.frag
	shaders/debug.vert shaders/debug.frag
	shaders/smokeParticle.vert shaders/smokeParticle.frag
	shaders/smokeSlice.vert shaders/smokeSlice.frag
	shaders/deepShadowMap.comp
	shaders/particleCreation.comp
	icon.qrc
	textures.qrc
)

# set target requirements
target_link_libraries(
	${PROJECT_NAME}
	PRIVATE
	Eigen3::Eigen
	glad
	assimp
	OpenGLObjects
)

# set install folder
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# find and link required Qt components
set(_QT_COMPONENTS Core Gui Widgets)
include(SetupQtPrefixPath)
find_package(Qt5 5.10 REQUIRED COMPONENTS ${_QT_COMPONENTS} CONFIG)
foreach(_COMP ${_QT_COMPONENTS})
	target_link_libraries(
		${PROJECT_NAME}
		PRIVATE
		Qt5::${_COMP}
	)
endforeach()

# copy/install required dlls
if(WIN32)
	file(GLOB _ICU_DLLS ${Qt5_DIR}/../../../bin/icu*[0-9].dll)
	set(_DEPENDENCIES)
	foreach(_ICU_DLL ${_ICU_DLLS})
		string(REPLACE ".dll" "$<$<CONFIG:Debug>:d>.dll" _ICU_DLL ${_ICU_DLL})
		list(APPEND _DEPENDENCIES ${_ICU_DLL})
	endforeach()
	foreach(_COMP ${_QT_COMPONENTS})
		list(APPEND _DEPENDENCIES $<TARGET_FILE:Qt5::${_COMP}>)
	endforeach()

	add_custom_command(
		TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} ARGS
		"-DSRC=${_DEPENDENCIES}"
		"-DDST=$<TARGET_FILE_DIR:${PROJECT_NAME}>"
		-P "${PROJECT_SOURCE_DIR}/CMake/InstallFile.cmake"
	)
	add_custom_command(
		TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} ARGS
		"-DSRC=${Qt5_DIR}/../../../plugins/platforms/qwindows$<$<CONFIG:Debug>:d>.dll"
		"-DDST=$<TARGET_FILE_DIR:${PROJECT_NAME}>/platforms"
		-P "${PROJECT_SOURCE_DIR}/CMake/InstallFile.cmake"
	)
	add_custom_command(
		TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} ARGS
		"-DSRC=${Qt5_DIR}/../../../plugins/imageformats/qico$<$<CONFIG:Debug>:d>.dll;${Qt5_DIR}/../../../plugins/imageformats/qjpeg$<$<CONFIG:Debug>:d>.dll"
		"-DDST=$<TARGET_FILE_DIR:${PROJECT_NAME}>/imageformats"
		-P "${PROJECT_SOURCE_DIR}/CMake/InstallFile.cmake"
	)

	
endif()

# setup automatic processing of Qt files
set_target_properties(
	${PROJECT_NAME}
	PROPERTIES
	AUTOMOC ON
	AUTOUIC ON
	AUTORCC ON
)
set_property(GLOBAL PROPERTY AUTOGEN_SOURCE_GROUP "Generated Files")

target_include_directories(
	${PROJECT_NAME}
	PRIVATE
	${PROJECT_BINARY_DIR}
	${PROJECT_SOURCE_DIR}
	assimp/include
)

source_group("Form Files" REGULAR_EXPRESSION "\\.ui$")
source_group("Resource Files" REGULAR_EXPRESSION "\\.q?rc$")

# setup shaders as qrc
include(ShadersToQRC)
shaders_to_qrc()
