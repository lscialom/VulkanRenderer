#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(location = 0) out vec2 fragUV;

vec2 positions[6] = vec2[](vec2(1, -1), vec2(-1, -1), vec2(-1, 1), vec2(-1, 1),
                           vec2(1, 1), vec2(1, -1));

out gl_PerVertex { vec4 gl_Position; };

void main() {
  gl_Position = vec4(positions[gl_VertexIndex], 0, 1);
  fragUV = positions[gl_VertexIndex] * 0.5f + 0.5f;
}
