#version 450
#extension GL_ARB_separate_shader_objects : enable

#include <constants.glsl>

layout(set = 0, binding = 0) uniform CameraData {
  mat4 view;
  mat4 proj;
  vec3 viewPos;
}
camData;

layout(set = 1, binding = 0) uniform sampler2D[G_BUFFER_SIZE] gBuffer;

layout(set = 2, binding = 0) uniform sampler2D ssaoNoise;
layout(set = 2, binding = 1) uniform ssaoSamplesIn { vec4 ssaoSamples[SSAO_NUM_SAMPLES]; };

layout(push_constant) uniform pushConstant {
  float xExtent;
  float yExtent;
}
u_pushConstant;

layout(location = 0) in vec2 fragUV;

layout(location = 0) out float fragColor;

float computeSSAO(vec3 pos, vec3 normal)
{
  const float radius = 5;
  const float bias = 0.1;
  const vec2 noiseScale = vec2(u_pushConstant.xExtent/SSAO_NOISE_DIM, u_pushConstant.yExtent/SSAO_NOISE_DIM);

  vec3 randomVec = normalize(texture(ssaoNoise, fragUV * noiseScale).xyz);
  vec3 tangent   = normalize(randomVec - normal * dot(randomVec, normal));
  vec3 bitangent = cross(normal, tangent);
  mat3 TBN       = mat3(tangent, bitangent, normal);

  float occlusion = 0.0;
  for(int i = 0; i < SSAO_NUM_SAMPLES; i++)
  {
      // get sample position
      vec3 currentSSAOSample = vec3(ssaoSamples[i]);

      vec3 currentSample = TBN * currentSSAOSample; // From tangent to view-space
      currentSample = pos + currentSample * radius;

      vec4 offset = vec4(currentSample, 1.0);
      offset      = camData.proj * offset;    // from view to clip-space
      offset.xyz /= offset.w;               // perspective divide
      offset.xyz  = offset.xyz * 0.5 + 0.5; // transform to range 0.0 - 1.0    }

      float sampleDepth = (camData.view * texture(gBuffer[POS_BUFFER_INDEX], offset.xy)).z;

      // range check & accumulate
      float rangeCheck = smoothstep(0.0, 1.0, radius / abs(pos.z - sampleDepth));
      occlusion += (sampleDepth >= currentSample.z + bias ? 1.0 : 0.0) * rangeCheck;
  }

  occlusion = 1.0 - (occlusion / SSAO_NUM_SAMPLES);

  return occlusion;
}

void main() {
  // world-space normal
  vec3 wn = texture(gBuffer[NORMAL_BUFFER_INDEX], fragUV).xyz;
  vec3 n = normalize(mat3(camData.view) * wn);

  // world-space position
  vec4 wp = texture(gBuffer[POS_BUFFER_INDEX], fragUV);
  vec3 p = (camData.view * wp).xyz;

  fragColor = computeSSAO(p, n);
}
