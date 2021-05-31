    #version 330 core
    out vec4 FragColor;
    uniform vec3 ourColor;
    uniform bool use_uniform_color;
    in vec3 color;
    void main()
    {
        if(use_uniform_color)
            FragColor = vec4(ourColor,1);
        else
            FragColor = vec4(color,1);

    } 