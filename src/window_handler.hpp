#pragma once

#include "config.hpp"
#include <cstdint>

namespace WindowHandler
{
	void Init(unsigned int width, unsigned int height);
	bool Update();
	void Shutdown();

	uint32_t GetRequiredInstanceExtensions(const char**& extensions);
}