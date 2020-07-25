#version 450
#extension GL_ARB_separate_shader_objects : enable

#include <colorspace.glsl>
#include <utils.glsl>

#define BAYER_MATRIX_DIVISOR 64.0f // Because bayer matrix range is from 0 to 63;
#define DITHER_TEX_X_DIM 8.0f

layout(constant_id = 0) const float framebufferSizeX = 0;
layout(constant_id = 1) const float framebufferSizeY = 0;

layout(set = 0, binding = 0) uniform CameraData {
  mat4 view;
  mat4 proj;
  vec3 viewPos;
}
camData;

struct LightDataStruct{
  vec4 vector;
  vec3 color;

  float ambientFactor;

  float maxDist;
};

layout(set = 1, binding = 0) uniform LightUBO {
  LightDataStruct lights[MAX_LIGHTS];
}
lightData;

layout(set = 2, binding = 0) uniform sampler2D[G_BUFFER_SIZE] gBuffer;

layout(set = 3, binding = 0) uniform sampler2D ssaoBlurColor;
layout(set = 4, binding = 0) uniform sampler2D ditherTex;

layout(push_constant) uniform pushConstant {
  uint numLights;
}
u_pushConstant;

layout(location = 0) in vec2 fragUV;

layout(location = 0) out vec4 fragColor;

//TODO Push constant
const float ssaoStrength = 1;
const float exposure = 2.0;
const float shininess = 0.025;
// const float ditherFactor = 768;

void main() {
  // world-space normal
  vec3 wn = texture(gBuffer[NORMAL_BUFFER_INDEX], fragUV).xyz;
  wn = normalize(wn * 2.0 - 1.0);

  vec3 n = normalize(mat3(camData.view) * wn);

  // world-space position
  vec4 wp = vec4(texture(gBuffer[POS_BUFFER_INDEX], fragUV).rgb, 1.0f);
  vec3 p = (camData.view * wp).xyz;

  vec3 v = normalize(-p);          // vector towards eye
  float vdn = max(dot(v, n), 0.0); // the rim-shading contribution

  fragColor.rgb = vec3(0, 0, 0);
  vec3 viewDir = normalize(-p);

  // float shadowFactor = shadow(
  //     wp.xyz, (inverse(camData.view) * vec4(lightData.lights[0].vector.xyz, 1.0)).xyz,
  //     32);

  vec4 diffuseColor = vec4(texture(gBuffer[COLOR_BUFFER_INDEX], fragUV));
  // diffuseColor *= pow(texture(ssaoBlurColor, fragUV).r, ssaoStrength);

  for(int i = 0; i<u_pushConstant.numLights; ++i)
  {
    bool notDirectional = bool(lightData.lights[i].vector.w);

    // avoiding branching
    // if(vector.w == 1) then vector is a position (point light)
    //  so lightDir is vector.xyz - p // 1 - (1 * (1-1)) = 1 <=> 1 * vector.xyz
    //  - p * 1
    //
    // if(vector.w == 0) then vector is a direction (directional light)
    //  so lightDir is -vector.xyz // 0 - (1 * (1-0)) = -1 <=> -1 * vector.xyz - p * 0
    vec3 lightDir = normalize((lightData.lights[i].vector.w - 1 * (1 - lightData.lights[i].vector.w)) * lightData.lights[i].vector.xyz - (p * lightData.lights[i].vector.w));
    vec3 halfDir = normalize(lightDir + viewDir);

    float diff = max(dot(n, lightDir), 0.0);
    vec3 diffuse = diffuseColor.rgb * diff * lightData.lights[i].color;

    float spec = pow(max(dot(n, halfDir), 0.0), 32);
    vec3 specular = shininess * spec * lightData.lights[i].color;

    // No ambient with hdr / tonemapping since exposure performs its job well enough
    // vec3 ambient = diffuseColor.rgb * lightData.lights[i].ambientFactor * lightData.lights[i].color;

    vec3 finalLightColor;
    if (notDirectional) {
      // vec3 attenuationConstants = getAttenuationConstants(lightData.lights[i].maxDist);
      // float constant = attenuationConstants.x;
      // float linear = attenuationConstants.y;
      // float quadratic = attenuationConstants.z;

      // float distance = length(lightData.lights[i].vector.xyz - p);
      // float attenuation = 1.0 / (constant + linear * distance +
      //                            quadratic * (distance * distance));

      float distance = length(p - lightData.lights[i].vector.xyz);
      float attenuation = 1.0 / (distance * distance);

      finalLightColor = (/* ambient +  */diffuse + specular) * attenuation;
    } else
      finalLightColor = /* ambient +  */diffuse + specular;

    fragColor.rgb +=
        finalLightColor;
  }

  fragColor.a = 1.0f;

  float luminance = dot(fragColor.rgb, vec3(0.299, 0.587, 0.114));
  fragColor.rgb *= mix(vec3(pow(texture(ssaoBlurColor, fragUV).r, ssaoStrength)), vec3(1.0), vec3(luminance));

  // fragColor.rgb = vec3(computeSSAO(p, n));
  // fragColor.rgb *= vdn * computeSSAO(p, n);

  // fragColor.rgb *= pow(texture(ssaoBlurColor, fragUV).r, ssaoStrength);

  //tonemap + exposure
  fragColor.rgb *= exposure;
  fragColor.rgb = ACESFitted(fragColor.rgb);

  float noise = texture(ditherTex, gl_FragCoord.xy / vec2(framebufferSizeX, framebufferSizeY) / DITHER_TEX_X_DIM).r;

  // higher mix range = less color banding but more noise
  fragColor.rgb += mix(-0.5/255.0, 1.5/255.0, noise);

  // stronger version for badly calibrated monitors
  // fragColor.rgb += mix(-2.0/255.0, 6.0/255.0, noise);

  // fragColor.rgb *= vec3(texture(ssaoColor, fragUV).r);

  // fragColor.rgb = normalize(vec3(-10.0, 10.0, 0.0) - wp.xyz);
}
