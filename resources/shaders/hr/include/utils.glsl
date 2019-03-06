const int nbAttenuationValues = 12;

const int attenuationThresholds[nbAttenuationValues] = int[](7, 13, 20, 32, 50, 65, 100, 160, 200, 325, 600, 3250);
const vec2 attenuationConstants[nbAttenuationValues] = vec2[](vec2(0.7, 1.8),
														vec2(0.35, 0.44),
														vec2(0.22, 0.20),
														vec2(0.14, 0.07),
														vec2(0.09, 0.032),
														vec2(0.07, 0.017),
														vec2(0.045, 0.0075),
														vec2(0.027, 0.0028),
														vec2(0.022, 0.0019),
														vec2(0.014, 0.0007),
														vec2(0.007, 0.0002),
														vec2(0.0014, 0.000007));

vec3 getAttenuationConstants(float lightMaxDist)
{
	int thresholdIndex = nbAttenuationValues - 1;
	for(int i = 0; i<nbAttenuationValues; ++i)
	{
		if(lightMaxDist < attenuationThresholds[i])
			continue;

		thresholdIndex = i;
	}

	if(thresholdIndex == nbAttenuationValues - 1)
		return vec3(1, attenuationConstants[thresholdIndex].x, attenuationConstants[thresholdIndex].y);
	else
	{
		float alpha = (lightMaxDist - attenuationThresholds[thresholdIndex]) / (attenuationThresholds[thresholdIndex + 1] - attenuationThresholds[thresholdIndex]);
		return vec3(1, mix(attenuationConstants[thresholdIndex].x, attenuationConstants[thresholdIndex + 1].x, alpha),
						mix(attenuationConstants[thresholdIndex].y, attenuationConstants[thresholdIndex + 1].y, alpha));
	}
}