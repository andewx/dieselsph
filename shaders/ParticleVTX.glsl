#version 330 core
layout (location = 0) in vec3 aPos; // the position variable has attribute position 0
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
out vec4 fluidVel; // specify a color output to the fragment shader


void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0); // see how we directly give a vec3 to vec4's constructor

    fluidVel = vec4(1.0,1.0,1.0, 1.0); // Blue Points
}
