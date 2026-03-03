#ifndef COMMON_H
#define COMMON_H

#define GLM_ENABLE_EXPERIMENTAL
#include <GL/glew.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

// 셰이더 파일 읽기
std::string readShaderFile(const char* filePath);

// 셰이더 컴파일
GLuint compileShader(GLenum type, const char* source);

// 셰이더 프로그램 생성
GLuint createShaderProgram(const char* vertexPath, const char* fragmentPath);

// 마우스 위치에 따른 단위 벡터 계산
glm::vec3 calcUnitVecByMousePosition(const glm::vec2& raw, float winW, float winH);

// 트랙볼 회전 계산
glm::mat4 calcTrackball(const glm::vec2& start, const glm::vec2& cur, float winW, float winH);

// GLB Loader (loads mesh and embedded texture)
bool loadGLB(
	const char * path,
	std::vector<glm::vec3> & out_vertices,
	std::vector<glm::vec2> & out_uvs,
	std::vector<glm::vec3> & out_normals,
	GLuint * out_textureID = nullptr
);

#endif // COMMON_H
