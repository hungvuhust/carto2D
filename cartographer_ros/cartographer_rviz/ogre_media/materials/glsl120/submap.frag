 

#version 120

varying vec2 out_submap_texture_coordinate;

uniform sampler2D u_submap;
uniform float u_alpha;

void main()
{
  vec2 texture_value = texture2D(u_submap, out_submap_texture_coordinate).rg;
  float value = u_alpha * texture_value.r;
  float alpha = u_alpha * texture_value.g;
  gl_FragColor = vec4(value, value, value, alpha);
}
