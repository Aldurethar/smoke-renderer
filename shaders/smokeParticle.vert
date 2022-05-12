#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in float aDensity;

out float Density;
out vec4 FragPosLightSpace;
out vec4 FragPosDSMLightSpace;

uniform mat4 modelViewProjection;
uniform mat4 lightViewMatrix;
uniform mat4 lightProjectionMatrix;
uniform mat4 dsmProjectionMatrix;

void main()
{
	vec4 FragPosClipSpace = modelViewProjection * vec4(aPos, 1.0);
	float z = FragPosClipSpace.z / FragPosClipSpace.w;
	//gl_PointSize = 30.0 / FragPosClipSpace.z;	
	gl_PointSize = -1000.0 * z;
	//gl_PointSize = 3.0;
	//Most particles are far from being dense enough to see, so we cull them
	if (aDensity <= 0.001){
		gl_PointSize = 0.0;
	}

	Density = aDensity;
	FragPosLightSpace = lightProjectionMatrix * lightViewMatrix * vec4(aPos, 1.0);
	FragPosDSMLightSpace = dsmProjectionMatrix * lightViewMatrix * vec4(aPos, 1.0);
    gl_Position = FragPosClipSpace;
}