#include <exception>
#include <iostream>

#include "cpu.h"

int main() {
	Cpu cpu;

	try {
		cpu.loadRom("tests/cpu_instrs.gb");
		cpu.run();
	} catch (std::exception e) {
		std::cerr << e.what() << std::endl;
	}
}