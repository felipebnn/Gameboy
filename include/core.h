#pragma once

#include "ram.h"
#include "cpu.h"
#include "lcd.h"

class Core {
	Ram ram;
	Cpu cpu;
	Lcd lcd;

public:
	Core();
	void run();
};