#pragma once

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
	
	uint8_t ram[0x10000];

	bool running;

	void setByte(uint16_t addr, uint8_t b);
	uint8_t getByte(uint16_t addr);

	void init();
	void runInstruction();
	void runCbInstruction();

public:
	void loadRom(const std::string& fileName);
	void run();
};
