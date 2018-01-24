#pragma once

#include <SDL2/SDL.h>

#include "ram.h"

constexpr uint16_t WIDTH = 160;
constexpr uint16_t HEIGHT = 144;

class Lcd {
	Ram& ram;

	SDL_Window* window = nullptr;
	SDL_Surface* screenSurface = nullptr;

public:
	Lcd(Ram& ram);
	~Lcd();

	void init();
};