#version 330 core
out vec4 Fragment_color;

uniform vec4 ourColor;

void main() {
        Fragment_color = ourColor;
}
