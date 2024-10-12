#include <iostream>
#include "MC33.cpp"
#include "grid3d.cpp"
#include "surface.cpp"
#include "../include/chaos.h"

int show(grid3d &o_grid3d, MC33 &o_mc33, surface &o_surface);

int main()
{
	grid3d o_grid3d;
	 o_grid3d.read_dat_file("D:\\PHD\\Data\\dataset-present-492x492x442\\present492x492x442.dat");
	//o_grid3d.set_sphere(3, 3, 3, 0.8f);

	MC33 o_mc33;
	o_mc33.set_grid3d(o_grid3d);

	surface o_surface;
	o_mc33.calculate_isosurface(o_surface, 0);

	o_surface.save_as_obj();

	return 0;
}

int show(grid3d &o_grid3d, MC33 &o_mc33, surface &o_surface)
{
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	GLFWwindow* window = glfwCreateWindow(800, 600, "LearnOpenGL", NULL, NULL);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	// vertex shader
	const char* vertexShaderSource = "#version 330 core\n"
		"layout (location = 0) in vec3 aPos;\n"
		"void main()\n"
		"{\n"
		" gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
		"}\0";
	unsigned int vertexShader;
	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
	glCompileShader(vertexShader);
	// check if vertex shader initialized successfully
	check_shader_compile_status(vertexShader, "ERROR::SHADER:VERTEX::COMPILATION_FAILED\n");

	// fragment shader
	const char* fragmentShaderSource = "#version 330 core\n"
		"out vec4 FragColor;\n"
		"void main()\n"
		"{\n"
		"FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
		"}\n"
		";\n";
	unsigned int fragmentShader;
	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
	glCompileShader(fragmentShader);
	// check if fragment shader initialized successfully
	check_shader_compile_status(fragmentShader, "ERROR::SHADER:FRAGMENT::COMPILATION_FAILED\n");

	// shader program
	unsigned int shaderProgram;
	shaderProgram = glCreateProgram();
	glAttachShader(shaderProgram, vertexShader);
	glAttachShader(shaderProgram, fragmentShader);
	glLinkProgram(shaderProgram);
	// check if link successed
	check_program_link_status(shaderProgram, "ERROR::SHADER::PROGRAM::LINK_FAILED\n");
	glUseProgram(shaderProgram);
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);

	// VAO
	unsigned int VAO;
	glGenVertexArrays(1, &VAO);
	glBindVertexArray(VAO);

	// VBO
	//float vertices[] = {
	//	0.5f, 0.5f, 0.0f, // top right
	//	0.5f, -0.5f, 0.0f, // bottom right
	//	-0.5f, -0.5f, 0.0f, // bottom left
	//	-0.5f, 0.5f, 0.0f // top left
	//};
	float vertices[] = {
		500.0f, 500.0f, 0.0f, // top right
		500.0f, -500.0f, 0.0f, // bottom right
		-500.0f, -500.0f, 0.0f, // bottom left
		-500.0f, 500.0f, 0.0f // top left
	};
	unsigned int VBO;
	glGenBuffers(1, &VBO);
	// copy vertices to buffer for opengl to use later
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
	//glBufferData(GL_ARRAY_BUFFER, 3 * o_surface.get_num_vertices(), o_surface.get_vertices_pointer(), GL_STATIC_DRAW);


	// EBO
	unsigned int indices[] = { // note that we start from 0!
		0, 1, 3, // first triangle
		1, 2, 3 // second triangle
	};
	unsigned int EBO;
	glGenBuffers(1, &EBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
	//glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3 * o_surface.get_num_triangles(), o_surface.get_triangle_index_pointer(), GL_STATIC_DRAW);
	//glBufferData(GL_ELEMENT_ARRAY_BUFFER, 100, o_surface.get_triangle_index_pointer(), GL_STATIC_DRAW);

	// 3. then set our vertex attributes pointers
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	while (!glfwWindowShouldClose(window))
	{
		processInput(window);
		//glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClearColor(0.2f, 0.0f, 0.0f, 1.0f);

		glClear(GL_COLOR_BUFFER_BIT);

		glUseProgram(shaderProgram);
		glBindVertexArray(VAO);
		//glDrawArrays(GL_TRIANGLES, 0, 3);
		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	glfwTerminate();
}
