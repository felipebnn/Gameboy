#include <exception>
#include <iostream>

#define SDL_MAIN_HANDLED

#include "core.h"

int main(int argc, char const* args[]) {
	Core core;

	try {
		core.run();
	} catch (std::runtime_error e) {
		std::cerr << e.what() << std::endl;
		return 1;
	}

	return 0;
}