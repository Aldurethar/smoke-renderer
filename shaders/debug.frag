#version 330 core
out vec4 FragColor;

in vec3 FragPos;
in vec2 TexCoords;

uniform sampler2DArray debugTexture;

void main()
{           
	vec3 texCo = vec3(TexCoords, 3);
	vec2 texColor = texture(debugTexture, vec3(1.0, 1.0, 4.0)).rg;
	vec4 color = vec4(texColor, 0.0, 1.0);
    FragColor = vec4(color);
}