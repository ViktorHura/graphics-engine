cmake_minimum_required(VERSION 3.5)

project(engine)

############################################################
# Set compiler flags
############################################################
set(OWN_GXX_FLAGS "-Wall -Wextra -fstack-protector-all -std=c++14")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${OWN_GXX_FLAGS} -pg -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OWN_GXX_FLAGS}")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${OWN_GXX_FLAGS}")

############################################################
# List all sources
############################################################
set(engine_sources easy_image.cc engine.cc ini_configuration.cc l_parser.cc generators/Intro.cpp generators/Utils.cpp generators/LSystem2D.cpp generators/LSystem2D.h vector3d.cc generators/Wireframe.cpp generators/Wireframe.h generators/Bodies3D.cpp generators/Bodies3D.h generators/LSystem3D.cpp generators/LSystem3D.h generators/ZBufferedWireframe.cpp generators/ZBufferedWireframe.h generators/ZBuffer.h generators/ZBufferingTri.cpp generators/ZBufferingTri.h generators/Fractal3D.cpp generators/Fractal3D.h generators/Lighting.cpp generators/Lighting.h)

############################################################
# Create an executable
############################################################
set(exe_name "engine")
add_executable( ${exe_name} ${engine_sources} )
install( TARGETS ${exe_name} DESTINATION ${PROJECT_SOURCE_DIR}/ )
