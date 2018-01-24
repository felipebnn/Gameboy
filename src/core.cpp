#include "core.h"

Core::Core()
	: cpu(ram)
	, lcd(ram)
{}

void Core::run() {
	ram.loadRom("DMG_ROM.bin");

	lcd.init();
	// cpu.run();
}