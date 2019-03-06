#version 450
#extension GL_ARB_separate_shader_objects : enable

#define NUM_VERTICES 6
#define SPACING 20

layout(push_constant) uniform pushConstant {
  float xExtent;
  float yExtent;
  float ratio;
}
u_pushConstant;

layout(location = 0) out vec2 fragUV;
layout(location = 1) flat out uint currentInstanceId;

// vec2 positions[NUM_VERTICES] = vec2[](vec2(1, -1), vec2(-1, -1), vec2(-1, 1),
// vec2(-1, 1),
//                         vec2(1, 1), vec2(1, -1));

vec2 positions[NUM_VERTICES] = vec2[](vec2(1, 0), vec2(0, 0), vec2(0, 1),
                                      vec2(0, 1), vec2(1, 1), vec2(1, 0));

out gl_PerVertex { vec4 gl_Position; };

vec2 makePosition(vec2 translation, vec2 scale) {

  vec2 extent = vec2(u_pushConstant.xExtent, u_pushConstant.yExtent);

  return vec2(((positions[gl_VertexIndex % NUM_VERTICES] * scale) +
               (translation) / extent.y) *
                  2 -
              1);
}

void main() {

  vec2 extent = vec2(u_pushConstant.xExtent, u_pushConstant.yExtent);

  currentInstanceId = gl_VertexIndex / NUM_VERTICES;

  float yOffset = extent.y * u_pushConstant.ratio + (SPACING * int(bool(currentInstanceId)));
  yOffset *= currentInstanceId;

  gl_Position = vec4(makePosition(vec2(0, yOffset), vec2(u_pushConstant.ratio,
                                                         u_pushConstant.ratio)),
                     0, 1);

  fragUV = positions[gl_VertexIndex % NUM_VERTICES];
}
