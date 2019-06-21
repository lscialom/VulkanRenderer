#version 450
#extension GL_ARB_separate_shader_objects : enable

#include <colorspace.glsl>

layout(set = 0, binding = 0) uniform sampler2D tDiffuse;
layout(set = 0, binding = 1) uniform sampler2D tAlpha;

layout(set = 1, binding = 0) uniform CameraData {
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

layout(location = 0) in vec3 fragPosition;
layout(location = 1) in vec3 fragNormal;
layout(location = 2) in vec2 fragUV;

layout(location = POS_BUFFER_INDEX - 1) out vec4 outPos;
layout(location = NORMAL_BUFFER_INDEX - 1) out vec4 outNormal;
layout(location = COLOR_BUFFER_INDEX - 1) out vec4 outColor;

void main() {
  // sRGB => Linear conversion
  vec4 userColor = vec4(SRGBToLinear(u_pushConstant.color.rgb), u_pushConstant.color.a);

  vec4 diffuseColor = texture(tDiffuse, fragUV);
  diffuseColor.rgb = SRGBToLinear(diffuseColor.rgb); // TODO Avoid conversion overhead by pushing tDiffuse as R8G8B8A8Srgb ?

  outColor = userColor * diffuseColor;
  outColor.a *= texture(tAlpha, fragUV).r;

  // outNormal.xyz =
  //    normalize(mat3(camData.view) *
  //              fragNormal); // convert normal to view space, u_vm (view
  //                           // matrix), is a rigid body transform.
  // outNormal.w = 1;

  //using alpha for blending. Should use outNormal.w = 1.0f in other shaders
  outNormal = vec4(normalize(fragNormal), outColor.a);

  // outPos.xyz =
  //    vec3(camData.view * vec4(fragPosition, 1.0)); // position in view space
  // outPos.w = 1;

  //using alpha for blending. Should use OutPos.w = 1.0f in other shaders
  outPos = vec4(fragPosition, outColor.a); // position in world space


  // outNormal = vec4(normalize(fragNormal), 1);
  // outPos = vec4(0, 1, 0, 1);
}
