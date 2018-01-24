#pragma once

#include <set>
#include <string>

constexpr uint8_t Z_FLAG = 128;
constexpr uint8_t N_FLAG = 64;
constexpr uint8_t H_FLAG = 32;
constexpr uint8_t C_FLAG = 16;

class Cpu {
	union {
		struct { //TODO: remove struct {
			uint8_t regs[8];
		};
		struct {
			uint8_t b, c;
			uint8_t d, e;
			uint8_t h, l;
			uint8_t f, a;
		};
	};

	uint16_t pc;
	uint16_t sp;
	
	uint8_t ram[0x10000] = {};

	bool running;

	std::set<uint16_t> breakPoints {
		// 0x0028
		0x0064
	};

	bool stepByStep = false;

	void handleDebugger();

	uint8_t readMemory8(uint16_t addr);
	void writeMemory8(uint16_t addr, uint8_t x);

	uint16_t readMemory16(uint16_t addr);
	void writeMemory16(uint16_t addr, uint16_t x);

	uint16_t readMemory16_inverted(uint16_t addr);
	void writeMemory16_inverted(uint16_t addr, uint16_t x);

	void init();
	uint32_t runInstruction();
	uint32_t runLongInstruction();

	void dumpRegs();

public:
	void loadRom(const std::string& fileName);
	void run();
};
