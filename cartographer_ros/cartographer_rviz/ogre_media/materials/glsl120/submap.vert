 

#version 120

attribute vec4 uv0;

varying vec2 out_submap_texture_coordinate;

void main()
{
  out_submap_texture_coordinate = vec2(uv0);
  gl_Position = ftransform();
}
