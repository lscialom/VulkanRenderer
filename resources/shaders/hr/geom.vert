#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(set = 2, binding = 0) uniform CameraData {
  mat4 view;
  mat4 proj;
  vec3 viewPos;
}
camData;

layout(push_constant) uniform pushConstant {
  mat4 model;
  vec3 color;
}
u_pushConstant;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUV;
// layout(location = 1) in vec3 inColor;

layout(location = 0) out vec3 fragPosition;
layout(location = 1) out vec3 fragNormal;
layout(location = 2) out vec2 fragUV;

out gl_PerVertex { vec4 gl_Position; };

void main() {
  gl_Position = camData.proj * camData.view * u_pushConstant.model *
                vec4(inPosition, 1.0);

  fragPosition = (u_pushConstant.model * vec4(inPosition, 1.0)).xyz;
  fragNormal = (u_pushConstant.model * vec4(inNormal, 0.0)).xyz;

  fragUV = inUV;
}
