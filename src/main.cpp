#include <exception>
#include <iostream>

#include "cpu.h"

int main() {
	Cpu cpu;

	try {
		cpu.loadRom("DMG_ROM.bin");
		cpu.run();
	} catch (std::runtime_error e) {
		std::cerr << e.what() << std::endl;
	}
}