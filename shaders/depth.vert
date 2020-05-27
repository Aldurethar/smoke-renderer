#version 330 core
layout (location = 0) in vec3 aPos;

out float dep;

uniform mat4 lightSpaceMatrix;

void main()
{
	gl_Position = lightSpaceMatrix * vec4(aPos, 1.0);
	dep = gl_Position.z;
}
