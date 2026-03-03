#version 330 core

layout(location = 0) in vec4 aPos;
layout(location = 1) in vec4 aColor;

out vec4 vColor;

uniform mat4 modelMat;
uniform mat4 viewMat;
uniform mat4 projMat;

void main(void){
	gl_Position = projMat * viewMat * modelMat * aPos;
	vColor = aColor;
}