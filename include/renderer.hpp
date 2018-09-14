#pragma once

#ifdef VULKAN_RENDERER_DLL_EXPORTS
	#define VULKAN_RENDERER_EXPORTS __declspec(dllexport)
#else
	#define VULKAN_RENDERER_EXPORTS __declspec(dllimport)
#endif

namespace Renderer
{
	enum class PresentMode
	{
		Immediate = 0,
		Mailbox = 1,
		VSync
	};

	VULKAN_RENDERER_EXPORTS void Init(unsigned int width, unsigned int height);

	VULKAN_RENDERER_EXPORTS bool Update();
	VULKAN_RENDERER_EXPORTS void Shutdown();

	VULKAN_RENDERER_EXPORTS void Run(unsigned int width, unsigned int height);

	VULKAN_RENDERER_EXPORTS void SetPresentMode(PresentMode presentMode);
}