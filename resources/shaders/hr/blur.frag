#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(location = 0) in vec2 fragUV;

layout(location = 0) out float fragColor;
  
layout(set = 0, binding = 0) uniform sampler2D ssaoColor;

void main() {
    vec2 texelSize = 1.0 / vec2(textureSize(ssaoColor, 0));
    float result = 0.0;
    for (int x = -2; x < 2; ++x) 
    {
        for (int y = -2; y < 2; ++y) 
        {
            vec2 offset = vec2(float(x), float(y)) * texelSize;
            result += texture(ssaoColor, fragUV + offset).r;
        }
    }

    fragColor = result / (4.0 * 4.0);
}  