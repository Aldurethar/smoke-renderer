#version 330 core
out vec4 FragColor;

in vec3 FragPosWorldSpace;
in vec4 FragPosLightSpace;
in vec4 FragPosDSMLightSpace;

uniform sampler3D smokeData;
uniform sampler2D shadowMap;
uniform sampler2DArray deepShadowMap;
uniform vec3 smokeDims;
uniform vec3 lightPos;
uniform vec3 lightColor;

uniform int numSlices;

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

vec3 toSmokePos(vec3 pos)
{
	vec3 result = pos.xyz * 100;
	result = result + (smokeDims * 0.5);
	result = result / smokeDims;
	return result;
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
	if (projCoords.z > 1.0)	shadow = 0.0;

	float deepShad = deepShadowAt(FragPosDSMLightSpace.xyz);
	shadow += deepShad;

	return min(1.0, shadow);
}

void main()
{           
	float densityFactor = (1 / float(numSlices)) * 100.0;
	
	//Compute the Smoke Density at the Fragment's position
	vec3 FragPosTexSpace = toSmokePos(FragPosWorldSpace);
	float density = texture(smokeData, FragPosTexSpace).r * densityFactor;

	vec3 color = vec3(1.0);

	vec3 ambient = 0.15 * color;

	float bias = 0.0;
	float shadow = ShadowCalculation(FragPosLightSpace, bias);

	vec3 lighting = ambient + (1 - shadow) * lightColor * color;

    FragColor = vec4(lighting, density);	
}