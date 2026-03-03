cmake_minimum_required(VERSION 3.10)
project(PoissonSurfaceReconstruction)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# GLEW 경로 설정 (lib 폴더 내부)
set(GLEW_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/lib/glew")
set(GLEW_INCLUDE_DIRS "${GLEW_ROOT}/include")
set(GLEW_LIBRARIES "${GLEW_ROOT}/lib/Release/x64/glew32s.lib")

# GLFW 경로 설정 (lib 폴더 내부)
set(GLFW_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/lib/glfw")
set(GLFW_INCLUDE_DIRS "${GLFW_ROOT}/include")
set(GLFW_LIBRARIES "${GLFW_ROOT}/lib-vc2022/glfw3.lib")

# GLM 경로 설정 (lib 폴더 내부)
set(GLM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/lib/glm")
set(GLM_INCLUDE_DIRS "${GLM_ROOT}")

# OpenGL 찾기
find_package(OpenGL REQUIRED)

# 인클루드 디렉토리 설정
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/includes
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ${GLEW_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${GLM_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
)

# 소스 파일 수집
file(GLOB_RECURSE SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)

file(GLOB_RECURSE HEADERS 
    "${CMAKE_CURRENT_SOURCE_DIR}/includes/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/includes/*.hpp"
)

# 실행 파일 생성
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# 라이브러리 링크
target_link_libraries(${PROJECT_NAME}
    ${GLEW_LIBRARIES}
    ${GLFW_LIBRARIES}
    ${OPENGL_LIBRARIES}
)

# Shader 파일을 빌드 디렉토리로 복사
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_CURRENT_SOURCE_DIR}/shader/vertex.glsl"
    $<TARGET_FILE_DIR:${PROJECT_NAME}>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_CURRENT_SOURCE_DIR}/shader/fragment.glsl"
    $<TARGET_FILE_DIR:${PROJECT_NAME}>
)

# GLEW 정적 링크를 위한 정의
target_compile_definitions(${PROJECT_NAME} PRIVATE GLEW_STATIC)

# Visual Studio에서 폴더 구조 유지
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCES} ${HEADERS})
