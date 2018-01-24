#include <exception>
#include <iostream>

#define SDL_MAIN_HANDLED

#include "cpu.h"
#include "lcd.h"

int main(int argc, char const* args[]) {
	Cpu cpu;
	Lcd lcd;

	try {
		lcd.init();
		// cpu.loadRom("individual/01-special.gb");
		cpu.loadRom("DMG_ROM.bin");
		// cpu.run();
	} catch (std::runtime_error e) {
		std::cerr << e.what() << std::endl;
		return 1;
	}

	return 0;
}