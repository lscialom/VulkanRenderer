#version 450
#extension GL_ARB_separate_shader_objects : enable

// layout(binding = 0) uniform UniformBufferObject { mat4 model; }
// ubo;

layout(push_constant) uniform pushConstant {
  mat4 view;
  mat4 vp;
  mat4 model;
  vec3 color;
  vec3 viewPos;
}
u_pushConstant;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
// layout(location = 1) in vec3 inColor;

layout(location = 0) out vec3 fragColor;
layout(location = 1) out vec3 fragPosition;
layout(location = 2) out vec3 fragNormal;

out gl_PerVertex { vec4 gl_Position; };

void main() {
  gl_Position =
      u_pushConstant.vp * u_pushConstant.model * vec4(inPosition, 1.0);
  fragColor = u_pushConstant.color;

  fragPosition = (u_pushConstant.model * vec4(inPosition, 1.0)).xyz;
  fragNormal = normalize((u_pushConstant.model * vec4(inNormal, 0.0)).xyz);
}
