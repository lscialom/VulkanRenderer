#version 450
#extension GL_ARB_separate_shader_objects : enable

#include <constants.glsl>

layout(binding = 0) uniform CameraData {
  mat4 view;
  mat4 proj;
  vec3 viewPos;
}
camData;

layout(push_constant) uniform pushConstant {
  mat4 model;
  vec4 color;
}
u_pushConstant;

layout(location = 0) in vec3 fragColor;
layout(location = 1) in vec3 fragPosition;
layout(location = 2) in vec3 fragNormal;

layout(location = POS_BUFFER_INDEX - 1) out vec4 outPos;
layout(location = NORMAL_BUFFER_INDEX - 1) out vec4 outNormal;
layout(location = COLOR_BUFFER_INDEX - 1) out vec4 outColor;

void main() {
  // outNormal.xyz =
  //    normalize(mat3(camData.view) *
  //              fragNormal); // convert normal to view space, u_vm (view
  //                           // matrix), is a rigid body transform.
  // outNormal.w = 1;
  outNormal = vec4(normalize(fragNormal), 1.0);

  // outPos.xyz =
  //    vec3(camData.view * vec4(fragPosition, 1.0)); // position in view space
  // outPos.w = 1;
  outPos = vec4(fragPosition, 1.0); // position in world space

  outColor = u_pushConstant.color;

  // outNormal = vec4(normalize(fragNormal), 1);
  // outPos = vec4(0, 1, 0, 1);
}
