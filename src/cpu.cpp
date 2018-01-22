#include "cpu.h"

#include <exception>
#include <memory>

void Cpu::loadRom(const std::string& fileName) {
	std::unique_ptr<FILE, decltype(&fclose)> file(fopen(fileName.c_str(), "rb"), &fclose);

	if (!file) {
		throw std::runtime_error("Rom " + fileName + " not found!");
	}

	fread(ram, 1, 0x8000, file.get());
}

void Cpu::init() {
	pc = 0x100;
	sp = 0xFFFE;
}

void Cpu::setByte(size_t addr, char b) {
	ram[addr] = b;

	if (addr >= 0xE000 && addr < 0xFE00) {
		ram[addr - 0x2000] = b;
	}
}

char Cpu::getByte(size_t addr) {
	return ram[addr];
}

void Cpu::runInstruction(char op) {

}

void Cpu::run() {
	init();

	running = true;

	// while (running) {

	// }
}
