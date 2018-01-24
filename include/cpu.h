#pragma once

#include <set>
#include <string>

#include "ram.h"

constexpr uint8_t Z_FLAG = 128;
constexpr uint8_t N_FLAG = 64;
constexpr uint8_t H_FLAG = 32;
constexpr uint8_t C_FLAG = 16;

class Cpu {
	Ram& ram;

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

	bool running;

	std::set<uint16_t> breakPoints {
		// 0x0028
		0x0064
	};

	bool stepByStep = false;

	void handleDebugger();

	void init();
	uint32_t runInstruction();
	uint32_t runLongInstruction();

	void dumpRegs();

public:
	Cpu(Ram& ram);
	
	void run();
};
