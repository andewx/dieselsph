#version 330 core
out vec4 FragColor;

in vec4 fluidVel; // the input variable from the vertex shader (same name and same type)

void main()
{
    FragColor = fluidVel;
}
