#include <constants.glsl>

vec3 LinearTosRGB(vec3 color)
{
	return pow(color, vec3(1.0 / GAMMA));
}

vec3 SRGBToLinear(vec3 color)
{
	return pow(color, vec3(GAMMA));
}
