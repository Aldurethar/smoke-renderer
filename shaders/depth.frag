#version 330 core
in float dep;

out vec4 color;

void main()
{
	//gl_FragDepth = gl_FragCoord.z;
	color = vec4(vec3(gl_FragCoord.z), 1.0);

}
