#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;

out vec3 FragPos;
out vec3 Normal;
out vec2 TexCoords;
out vec4 FragPosLightSpace;
out vec4 FragPosDSMLightSpace;

uniform mat4 modelViewProjection;
uniform mat4 lightViewMatrix;
uniform mat4 lightProjectionMatrix;
uniform mat4 dsmProjectionMatrix;

void main()
{
	FragPos = vec3(aPos);
	Normal = aNormal;
	TexCoords = aTexCoords;
	FragPosLightSpace = lightProjectionMatrix * lightViewMatrix * vec4(aPos, 1.0);
	FragPosDSMLightSpace = dsmProjectionMatrix * lightViewMatrix * vec4(aPos, 1.0);
	gl_Position = modelViewProjection * vec4((aPos), 1.0);
}
