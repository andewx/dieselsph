#version 400 core
out vec4 fragColor
in vec4 colliderColor; // the input variable from the vertex shader (same name and same type)

void main()
{
    fragColor = colliderColor
}
