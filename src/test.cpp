#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <Eigen/Dense>

//#include <vulkan/vulkan.hpp>

#include <iostream>
#include <chrono>

#include "test.hpp"

using namespace Eigen;

int test()
{
	glfwInit();

	glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
	GLFWwindow* window = glfwCreateWindow(800, 600, "Vulkan window", nullptr, nullptr);

	uint32_t extensionCount = 0;
	vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, nullptr);

	std::cout << extensionCount << " extensions supported" << std::endl;

	Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
	Eigen::Vector4f vec(1, 1, 1, 1);

	Eigen::Vector4f res = matrix * vec;

	//matrix.setRandom();
	//Eigen::Matrix4f matrix1;
	//matrix1.setRandom();

	//auto start = std::chrono::system_clock::now();
	//for (int i = 0; i < 100000000; ++i)
	//{
	//	matrix = matrix * matrix1;
	//}
	//auto end = std::chrono::system_clock::now();

	//std::chrono::duration<double> elapsed_seconds = end - start;
	//std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";

	std::cout << "matrix:\n" << matrix << std::endl;
	std::cout << "vec:\n" << vec << std::endl;
	std::cout << "m * v:\n" << res << std::endl;

	while (!glfwWindowShouldClose(window))
		glfwPollEvents();

	glfwDestroyWindow(window);

	glfwTerminate();

	return 0;
}