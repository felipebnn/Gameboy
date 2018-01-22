#pragma once

#include <string>

class Cpu {
	char regs[8];
	size_t pc;
	size_t sp;
	
	char ram[0x10000];

	bool running;

	void setByte(size_t addr, char b);
	char getByte(size_t addr);

	void init();
	void runInstruction(char op);

public:
	void loadRom(const std::string& fileName);
	void run();
};
