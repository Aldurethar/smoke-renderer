#version 330 core

in vec3 FragPos;
in vec3 Normal;
in vec4 FragPosLightSpace;
in vec4 FragPosDSMLightSpace;

out vec4 FragColor;

uniform vec3 objColor;
uniform sampler2D shadowMap;
uniform sampler2DArray deepShadowMap;

uniform vec3 lightColor;
uniform vec3 lightPos;
uniform vec3 cameraPos;

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

float averageOfSurroundings(vec3 projCoords, float bias, float currentDepth)
{
	float average = 0.0;
	vec2 texelSize = 1.0 / textureSize(shadowMap, 0);
	for (int x = -1; x <= 1; ++x){
		for (int y = -1; y <= 1; ++y){
			float pcfDepth = texture (shadowMap, projCoords.xy + vec2(x, y) * texelSize).r;
			average += currentDepth - bias > pcfDepth? 1.0 : 0.0;
		}
	}
	return average / 9;
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
	float shadow = averageOfSurroundings(projCoords, bias, currentDepth);
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
	vec3 color = objColor;
	vec3 normal = normalize(Normal);

	//Ambient
	vec3 ambient = 0.15 * color;

	//Diffuse
	vec3 lightDir = normalize(lightPos - FragPos);
	float diff = max(dot(normal, lightDir), 0.0);
	vec3 diffuse = diff*lightColor;

	//Specular
	vec3 viewDir = normalize(cameraPos - FragPos);
	vec3 halfwayDir = normalize(lightDir + viewDir);
	float spec = pow(max(dot(normal, halfwayDir), 0.0), 64.0);
	vec3 specular = spec * lightColor;

	//calculate shadow bias based on light angle
	float bias = max(0.05 * (1.0 - dot(normal, lightDir)), 0.005);
	//Calculate Shadow
	float shadow = ShadowCalculation(FragPosLightSpace, bias);
	vec3 lighting = (ambient + (1 - shadow) * (diffuse + specular)) * color;

	FragColor = vec4(lighting, 1.0);
}