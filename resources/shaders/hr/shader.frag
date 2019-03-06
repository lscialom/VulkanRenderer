#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(push_constant) uniform pushConstant {
  mat4 view;
  mat4 vp;
  mat4 model;
  vec3 color;
  vec3 viewPos;
}
u_pushConstant;

layout(location = 0) in vec3 fragColor;
layout(location = 1) in vec3 fragPosition;
layout(location = 2) in vec3 fragNormal;

layout(location = 0) out vec4 outColor;

void main() {
  // outColor = vec4(fragColor, 1.0);
  // outColor = vec4(fragNormal, 1.0);
  // outColor = vec4(fragPosition, 1.0);

  vec3 n = normalize(mat3(u_pushConstant.view) *
                     fragNormal); // convert normal to view space, u_vm (view
                                  // matrix), is a rigid body transform.
  vec3 p = vec3(u_pushConstant.view *
                vec4(fragPosition, 1.0)); // position in view space

  vec3 v = normalize(-p);          // vector towards eye
  float vdn = max(dot(v, n), 0.0); // the rim-shading contribution

  outColor.a = 1.0;
  outColor.rgb = vec3(vdn) * (fragNormal * 0.5 + 0.5);
  // outColor.rgb = vec3(vdn) * fragNormal;
  // outColor.rgb = vec3(vdn);
  // outColor.rgb = n;
}
