#include "ram.h"

#include <memory>
#include <cstdio>

#include "util.h"

uint16_t Ram::remapAddress(uint16_t addr) const {
	if (addr >= 0xE000 && addr < 0xFE00) {
		addr -= 0x2000;
	}

	return addr;
}

void Ram::loadRom(const std::string& fileName) {
	std::unique_ptr<FILE, decltype(&fclose)> file(fopen(fileName.c_str(), "rb"), &fclose);

	if (!file) {
		throw std::runtime_error("Rom " + fileName + " not found!");
	}

	fread(mem, 1, 0x8000, file.get());
}

uint8_t Ram::readMemory8(uint16_t addr) const {
	addr = remapAddress(addr);
	return mem[addr];
}

void Ram::writeMemory8(uint16_t addr, uint8_t x) {
	addr = remapAddress(addr);
	mem[addr] = x;
}

uint16_t Ram::readMemory16(uint16_t addr) const {
	addr = remapAddress(addr);
	return convertTo16(mem[addr], mem[addr+1]);
}

void Ram::writeMemory16(uint16_t addr, uint16_t x) {
	addr = remapAddress(addr);
	convertTo8(x, mem[addr], mem[addr+1]);
}

uint16_t Ram::readMemory16_inverted(uint16_t addr) const {
	addr = remapAddress(addr);
	return convertTo16(mem[addr+1], mem[addr]);
}

void Ram::writeMemory16_inverted(uint16_t addr, uint16_t x) {
	addr = remapAddress(addr);
	convertTo8(x, mem[addr+1], mem[addr]);
}