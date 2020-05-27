#version 330 core
out vec4 FragColor;

in float Density;
in vec4 FragPosLightSpace;
in vec4 FragPosDSMLightSpace;

uniform sampler2D shadowMap;
uniform sampler2DArray deepShadowMap;

uniform vec3 lightColor;

float deepShadowAt(vec3 pos){
	float dep = pos.z;
	vec2 coords = pos.rg * 0.5 + 0.5;
	vec2 nodes[8];
	float result;
	for (int i = 0; i < 8; i++){
		nodes[i] = texture(deepShadowMap, vec3(coords, i)).rg;
	}
	for (int i = 0; i < 7; i++){
		if (nodes[i].x <= dep && nodes[i+1].x > dep){
			float fac = (dep - nodes[i].x) / (nodes[i+1].x - nodes[i].x);
			result = mix(nodes[i].y, nodes[i+1].y, fac);
			return 1 - result;
		}
	}
	if (nodes[7].x <= dep){
		return 1 - nodes[7].y;
	}
	return 0;
}

float ShadowCalculation(vec4 fragPosLightSpace, float bias)
{
	//Perform perspective divide
	vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
	//transform to [0,1] range
	projCoords = projCoords * 0.5 + 0.5;
	//get closest value from depth map
	float closestDepth = texture(shadowMap, projCoords.xy).r;
	//get depth of the current fragment from light's perpective
	float currentDepth = projCoords.z;
	//check wether current frag pos is in shadow
	float shadow = currentDepth - bias > closestDepth ? 1.0 : 0.0;
	//No Shadow beyond depth Buffer reach
	if (fragPosLightSpace.z > 1.0 || fragPosLightSpace.z < -1.0) shadow = 0.0;
	if (fragPosLightSpace.x > 1.0 || fragPosLightSpace.x < -1.0) shadow = 0.0;
	if (fragPosLightSpace.y > 1.0 || fragPosLightSpace.y < -1.0) shadow = 0.0;

	float deepShad = deepShadowAt(FragPosDSMLightSpace.xyz);
	shadow += deepShad;

	return min(1.0, shadow);
}

void main()
{            
	//float texValue = 1 - texture(circleTexture, gl_PointCoord).r;
	float d = 0.5 - length(gl_PointCoord - vec2(0.5));
	
	float density = min(Density, 1.0) * d * 2.0;

	vec3 color = vec3(1.0);

	vec3 ambient = 0.15 * color;

	float bias = 0.0;
	float shadow = ShadowCalculation(FragPosLightSpace, bias);

	vec3 lighting = ambient + (1 - shadow) * lightColor * color;

	FragColor = vec4(lighting, density);
	//FragColor = vec4(1.0, 0.0, 0.0, 1.0);
}