cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

set(PROJECT uvcut) 
set(CMAKE_CXX_STANDARD_REQUIRED 17)

project(${PROJECT}) 

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

if (NOT OPENGL_FOUNDAND)
	message("OpenGL not found")
endif()

if(NOT OPENGL_GLU_FOUND)
	message("GLU not found")
endif()

set(SOURCE_FILES main.cpp)
add_definitions(-DFREEGLUT_STATIC)

add_executable (${PROJECT} ${SOURCE_FILES})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_link_libraries(${PROJECT} PRIVATE OpenGL::GL PRIVATE OpenGL::GLU)
