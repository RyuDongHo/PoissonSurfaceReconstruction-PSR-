#version 330 core

in vec2 vTexCoord;

out vec4 fragColor;

uniform sampler2D textureSampler;
uniform int useTexture;

void main(void){
	if (useTexture == 1) {
		fragColor = texture(textureSampler, vTexCoord);
	} else {
		fragColor = vec4(0.8, 0.8, 0.8, 1.0);
	}
}