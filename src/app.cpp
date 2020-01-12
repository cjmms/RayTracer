#include <iostream>
#include <GL\glew.h>
#include <GLFW/glfw3.h>
#include <fstream>
#include <vector>
#include <algorithm>

unsigned int height = 480, width = 640;
struct vec3f {
	float r;
	float g;
	float b;
};

std::vector<vec3f> image;

void gen_ppm_image() {
	std::ofstream ofs("./test.ppm", std::ios::out, std::ios::binary);
	ofs << "P6\n"
		<< width
		<< " "
		<< height 
		<< "\n255\n";

	for (unsigned int i = 0; i < image.size(); ++i) {
		ofs << (unsigned char) (std::min(1.0f, image[i].r) * 255)
			<< (unsigned char) (std::min(1.0f, image[i].g) * 255)
			<< (unsigned char) (std::min(1.0f, image[i].b) * 255);
	}	

	ofs.close();
}


int main() {
	image = std::vector<vec3f>(width * height, { 0.5f, 0.5f, 0.5f });
	gen_ppm_image();
}


//
//int main(void)
//{
//    GLFWwindow* window;
//
//    /* Initialize the library */
//    if (!glfwInit())
//        return -1;
//
//    /* Create a windowed mode window and its OpenGL context */
//    window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
//    if (!window)
//    {
//        glfwTerminate();
//        return -1;
//    }
//
//    /* Make the window's context current */
//    glfwMakeContextCurrent(window);
//
// 
//    /* Loop until the user closes the window */
//    while (!glfwWindowShouldClose(window))
//    {
//        /* Render here */
//        glClear(GL_COLOR_BUFFER_BIT);
//
//        /* Swap front and back buffers */
//        glfwSwapBuffers(window);
//
//        /* Poll for and process events */
//        glfwPollEvents();
//    }
//
//    glfwTerminate();
//    return 0;
//}