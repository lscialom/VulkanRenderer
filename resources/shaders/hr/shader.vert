#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 0) uniform UniformBufferObject { mat4 model; }
ubo;

layout(push_constant) uniform pushConstant {
  mat4 vp;
  vec3 color;
}
u_pushConstant;

layout(location = 0) in vec3 inPosition;
// layout(location = 1) in vec3 inColor;

layout(location = 0) out vec3 fragColor;

out gl_PerVertex { vec4 gl_Position; };

void main() {
  gl_Position = u_pushConstant.vp * ubo.model * vec4(inPosition, 1.0);
  fragColor = u_pushConstant.color;
}
