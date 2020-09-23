#version 400
layout(location=0) in vec3 position
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
out vec4 colliderColor; // specify a color output to the fragment shader


void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0); //Projection Modify
    colliderColor = vec4(0.1,0.1,0.1,0.2); // Transparent Fragments
}
