#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 inverseView;
uniform mat4 projection;
uniform mat4 lightSpaceMatrix;
uniform mat4 dsmLightSpaceMatrix;
uniform float smokeNear;
uniform float smokeFar;

out vec3 FragPosWorldSpace;
out vec4 FragPosLightSpace;
out vec4 FragPosDSMLightSpace;

float computeDepth()
{
	float dist = smokeFar - smokeNear;
	return -smokeNear + (aPos.z * dist);
}

void main()
{
	float depth = computeDepth();
	vec4 pos = vec4(aPos.xy, depth, 1.0);
	FragPosWorldSpace = (inverseView * pos).xyz;
	FragPosLightSpace = lightSpaceMatrix * vec4(FragPosWorldSpace, 1.0);
	FragPosDSMLightSpace = dsmLightSpaceMatrix * vec4(FragPosWorldSpace, 1.0);

    gl_Position = projection * pos;
}