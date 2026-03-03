#include "common.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>

// Define STB implementations before including tiny_gltf
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

// Configure tinygltf
#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_NOEXCEPTION
#define JSON_NOEXCEPTION
#include "../lib/tinygltf/tiny_gltf.h"

// 셰이더 파일 읽기
std::string readShaderFile(const char* filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Failed to open shader file: " << filePath << std::endl;
        return "";
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

// 셰이더 컴파일
GLuint compileShader(GLenum type, const char* source) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        std::cerr << "Shader compilation failed:\n" << infoLog << std::endl;
        return 0;
    }
    return shader;
}

// 셰이더 프로그램 생성
GLuint createShaderProgram(const char* vertexPath, const char* fragmentPath) {
    std::string vertexCode = readShaderFile(vertexPath);
    std::string fragmentCode = readShaderFile(fragmentPath);

    if (vertexCode.empty() || fragmentCode.empty()) {
        return 0;
    }

    GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexCode.c_str());
    GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentCode.c_str());

    if (!vertexShader || !fragmentShader) {
        return 0;
    }

    GLuint program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);

    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(program, 512, nullptr, infoLog);
        std::cerr << "Program linking failed:\n" << infoLog << std::endl;
        return 0;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return program;
}

// 마우스 위치에 따른 단위 벡터 계산
glm::vec3 calcUnitVecByMousePosition(const glm::vec2& raw, float winW, float winH) {
	glm::vec2 scr;
	scr.x = std::clamp(raw.x, 0.0F, winW);
	scr.y = std::clamp(raw.y, 0.0F, winH);
	const GLfloat radius = sqrt(winW * winW + winH * winH) / 2.f;
	glm::vec3 v;

	v.x = -1 * (scr.x - winW / 2.0F) / radius;
	v.y = (scr.y - winH / 2.0F) / radius;
	v.z = sqrtf(1.0F - v.x * v.x - v.y * v.y);
	return v;
}

// 트랙볼 회전 계산
glm::mat4 calcTrackball(const glm::vec2& start, const glm::vec2& cur, float winW, float winH) {
	glm::vec3 org = calcUnitVecByMousePosition(start, winW, winH);
	glm::vec3 dst = calcUnitVecByMousePosition(cur, winW, winH);
	glm::quat q = glm::rotation(dst, org);

	float angle = glm::angle(q);
	glm::vec3 axis = glm::axis(q);

	float sensitivity = 2.f;
	angle *= sensitivity;

	q = glm::angleAxis(angle, axis);
	glm::mat4 m = glm::toMat4(q);
	return m;
}

// GLB Loader (loads mesh and embedded texture)
bool loadGLB(
	const char * path,
	std::vector<glm::vec3> & out_vertices,
	std::vector<glm::vec2> & out_uvs,
	std::vector<glm::vec3> & out_normals,
	GLuint * out_textureID
) {
	printf("Loading GLB file %s...\n", path);
	
	tinygltf::Model model;
	tinygltf::TinyGLTF loader;
	std::string err;
	std::string warn;
	
	bool ret = loader.LoadBinaryFromFile(&model, &err, &warn, path);
	
	if (!warn.empty()) {
		printf("Warn: %s\n", warn.c_str());
	}
	
	if (!err.empty()) {
		printf("Error: %s\n", err.c_str());
	}
	
	if (!ret) {
		printf("Failed to parse glTF\n");
		return false;
	}
	
	// Process each mesh in the glTF file
	for (const auto& mesh : model.meshes) {
		for (const auto& primitive : mesh.primitives) {
			// Get vertex positions
			if (primitive.attributes.find("POSITION") != primitive.attributes.end()) {
				const tinygltf::Accessor& accessor = model.accessors[primitive.attributes.at("POSITION")];
				const tinygltf::BufferView& bufferView = model.bufferViews[accessor.bufferView];
				const tinygltf::Buffer& buffer = model.buffers[bufferView.buffer];
				
				const float* positions = reinterpret_cast<const float*>(
					&buffer.data[bufferView.byteOffset + accessor.byteOffset]);
				
				size_t vertexCount = accessor.count;
				
				// Get indices if available
				std::vector<unsigned int> indices;
				if (primitive.indices >= 0) {
					const tinygltf::Accessor& indexAccessor = model.accessors[primitive.indices];
					const tinygltf::BufferView& indexBufferView = model.bufferViews[indexAccessor.bufferView];
					const tinygltf::Buffer& indexBuffer = model.buffers[indexBufferView.buffer];
					
					if (indexAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) {
						const unsigned short* indexData = reinterpret_cast<const unsigned short*>(
							&indexBuffer.data[indexBufferView.byteOffset + indexAccessor.byteOffset]);
						for (size_t i = 0; i < indexAccessor.count; ++i) {
							indices.push_back(indexData[i]);
						}
					} else if (indexAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) {
						const unsigned int* indexData = reinterpret_cast<const unsigned int*>(
							&indexBuffer.data[indexBufferView.byteOffset + indexAccessor.byteOffset]);
						for (size_t i = 0; i < indexAccessor.count; ++i) {
							indices.push_back(indexData[i]);
						}
					}
				}
				
				// Get normals
				const float* normals = nullptr;
				if (primitive.attributes.find("NORMAL") != primitive.attributes.end()) {
					const tinygltf::Accessor& normalAccessor = model.accessors[primitive.attributes.at("NORMAL")];
					const tinygltf::BufferView& normalBufferView = model.bufferViews[normalAccessor.bufferView];
					const tinygltf::Buffer& normalBuffer = model.buffers[normalBufferView.buffer];
					normals = reinterpret_cast<const float*>(
						&normalBuffer.data[normalBufferView.byteOffset + normalAccessor.byteOffset]);
				}
				
				// Get texture coordinates
				const float* texCoords = nullptr;
				if (primitive.attributes.find("TEXCOORD_0") != primitive.attributes.end()) {
					const tinygltf::Accessor& texCoordAccessor = model.accessors[primitive.attributes.at("TEXCOORD_0")];
					const tinygltf::BufferView& texCoordBufferView = model.bufferViews[texCoordAccessor.bufferView];
					const tinygltf::Buffer& texCoordBuffer = model.buffers[texCoordBufferView.buffer];
					texCoords = reinterpret_cast<const float*>(
						&texCoordBuffer.data[texCoordBufferView.byteOffset + texCoordAccessor.byteOffset]);
				}
				
				// Build output arrays based on indices
				if (!indices.empty()) {
					for (unsigned int idx : indices) {
						// Position
						out_vertices.push_back(glm::vec3(
							positions[idx * 3 + 0],
							positions[idx * 3 + 1],
							positions[idx * 3 + 2]
						));
						
						// Normal
						if (normals) {
							out_normals.push_back(glm::vec3(
								normals[idx * 3 + 0],
								normals[idx * 3 + 1],
								normals[idx * 3 + 2]
							));
						} else {
							out_normals.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
						}
						
						// Texture coordinates
						if (texCoords) {
							out_uvs.push_back(glm::vec2(
								texCoords[idx * 2 + 0],
								texCoords[idx * 2 + 1]
							));
						} else {
							out_uvs.push_back(glm::vec2(0.0f, 0.0f));
						}
					}
				} else {
					// No indices, use direct vertex access
					for (size_t i = 0; i < vertexCount; ++i) {
						// Position
						out_vertices.push_back(glm::vec3(
							positions[i * 3 + 0],
							positions[i * 3 + 1],
							positions[i * 3 + 2]
						));
						
						// Normal
						if (normals) {
							out_normals.push_back(glm::vec3(
								normals[i * 3 + 0],
								normals[i * 3 + 1],
								normals[i * 3 + 2]
							));
						} else {
							out_normals.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
						}
						
						// Texture coordinates
						if (texCoords) {
							out_uvs.push_back(glm::vec2(
								texCoords[i * 2 + 0],
								texCoords[i * 2 + 1]
							));
						} else {
							out_uvs.push_back(glm::vec2(0.0f, 0.0f));
						}
					}
				}
			}
		}
	}
	
	printf("Loaded %zu vertices from GLB\n", out_vertices.size());
	
	// Load embedded texture if available and requested
	if (out_textureID != nullptr && !model.textures.empty() && !model.images.empty()) {
		const tinygltf::Texture& tex = model.textures[0];
		const tinygltf::Image& image = model.images[tex.source];
		
		printf("Loading embedded texture: %dx%d, %d channels\n", 
		       image.width, image.height, image.component);
		
		GLuint textureID;
		glGenTextures(1, &textureID);
		glBindTexture(GL_TEXTURE_2D, textureID);
		
		// Determine format based on channels
		GLenum format = GL_RGB;
		if (image.component == 1)
			format = GL_RED;
		else if (image.component == 3)
			format = GL_RGB;
		else if (image.component == 4)
			format = GL_RGBA;
		
		glTexImage2D(GL_TEXTURE_2D, 0, format, image.width, image.height, 
		             0, format, GL_UNSIGNED_BYTE, image.image.data());
		glGenerateMipmap(GL_TEXTURE_2D);
		
		// Set texture parameters
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		
		*out_textureID = textureID;
		printf("Texture loaded from GLB (ID: %u)\n", textureID);
	} else if (out_textureID != nullptr) {
		*out_textureID = 0;
		printf("No embedded texture found in GLB\n");
	}
	
	return true;
}
