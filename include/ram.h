#pragma once

#include <string>

class Ram {
	uint8_t mem[0x10000] = {};

	uint16_t remapAddress(uint16_t addr) const;

public:
	void loadRom(const std::string& fileName);

	uint8_t readMemory8(uint16_t addr) const;
	void writeMemory8(uint16_t addr, uint8_t x);

	uint16_t readMemory16(uint16_t addr) const;
	void writeMemory16(uint16_t addr, uint16_t x);

	uint16_t readMemory16_inverted(uint16_t addr) const;
	void writeMemory16_inverted(uint16_t addr, uint16_t x);
};