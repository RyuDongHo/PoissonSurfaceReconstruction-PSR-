#ifndef SHADER_HPP
#define SHADER_HPP
#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <GL/glew.h>

GLuint LoadShader(const char* vertex_file_path, const char* fragment_file_path);

#endif
