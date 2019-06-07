#version 450
#extension GL_ARB_separate_shader_objects : enable

#include <constants.glsl>

layout(set = 0, binding = 0) uniform sampler2D[G_BUFFER_SIZE] gBuffer;

layout(location = 0) in vec2 fragUV;
layout(location = 1) flat in uint currentInstanceId;

layout(location = 0) out vec4 fragColor;

void main() {
  fragColor.a = 1.0;

  if (currentInstanceId != DEPTH_BUFFER_INDEX)
    fragColor.rgb = vec4(texture(gBuffer[currentInstanceId], fragUV)).xyz;
  else
    fragColor.rgb = vec4(texture(gBuffer[currentInstanceId], fragUV)).xxx;

  fragColor.rgb = pow(fragColor.rgb, vec3(1.0f / GAMMA));
}
