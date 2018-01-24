#include "cpu.h"

#include <stdexcept>
#include <memory>

#define dprintf(...) {if (stepByStep) printf(__VA_ARGS__);}

static inline uint16_t convertTo16(uint8_t a, uint8_t b) {
	return a | (b << 8);
}

static inline void convertTo8(uint16_t in, uint8_t& a, uint8_t& b) {
	a = in & 0xff;
	b = (in >> 8) & 0xff;
}

static inline uint8_t calculateCarryBits8(uint8_t a, uint8_t b) {
	uint8_t res = a + b;
	return (a & ~res) | (b & ~res) | (a & b);
}

void Cpu::handleDebugger() {
	if (stepByStep) {
		printf("Command: ");
		char command = getchar();

		if (command != '\n') {
			switch (command) {
				case 'g':
					stepByStep = false;
					break;

				case 'w': {
					uint16_t addr;
					uint8_t value;
					printf("Addr: ");
					scanf("%hx", &addr);
					printf("Value: ");
					scanf("%hhx%*c", &value);
					writeMemory8(addr, value);
					break;
				}

				case 'r': {
					uint16_t addr;
					printf("Addr: ");
					scanf("%hx%*c", &addr);
					printf("Value: %hhx", readMemory8(addr));
					break;
				}

			}

			getchar();
		}
	}

	if (breakPoints.count(pc)) {
		stepByStep = true;
	}
}

void Cpu::loadRom(const std::string& fileName) {
	std::unique_ptr<FILE, decltype(&fclose)> file(fopen(fileName.c_str(), "rb"), &fclose);

	if (!file) {
		throw std::runtime_error("Rom " + fileName + " not found!");
	}

	fread(ram, 1, 0x8000, file.get());
}

void Cpu::init() {
	pc = 0x0;
}

uint8_t Cpu::readMemory8(uint16_t addr) {
	dprintf("readMemory8 %hx %hhx\n", addr, ram[addr]);

	if (addr >= 0xE000 && addr < 0xFE00) {
		addr -= 0x2000;
	}

	return ram[addr];
}

void Cpu::writeMemory8(uint16_t addr, uint8_t x) {
	dprintf("writeMemory8 %hx %hhx\n", addr, x);

	if (addr >= 0xE000 && addr < 0xFE00) {
		addr -= 0x2000;
	}

	ram[addr] = x;
}

uint16_t Cpu::readMemory16(uint16_t addr) {
	dprintf("readMemory16 %hx %hx %hx\n", addr, ram[addr], ram[addr+1]);

	if (addr >= 0xE000 && addr < 0xFE00) {
		addr -= 0x2000;
	}

	return convertTo16(ram[addr], ram[addr+1]);
}

void Cpu::writeMemory16(uint16_t addr, uint16_t x) {
	dprintf("writeMemory16 %hx %hhx\n", addr, x);

	if (addr >= 0xE000 && addr < 0xFE00) {
		addr -= 0x2000;
	}

	convertTo8(x, ram[addr], ram[addr+1]);
}

uint16_t Cpu::readMemory16_inverted(uint16_t addr) {
	dprintf("readMemory16 %hx %hx %hx\n", addr, ram[addr+1], ram[addr]);

	if (addr >= 0xE000 && addr < 0xFE00) {
		addr -= 0x2000;
	}

	return convertTo16(ram[addr+1], ram[addr]);
}

void Cpu::writeMemory16_inverted(uint16_t addr, uint16_t x) {
	dprintf("writeMemory16 %hx %hhx\n", addr, x);

	if (addr >= 0xE000 && addr < 0xFE00) {
		addr -= 0x2000;
	}

	convertTo8(x, ram[addr+1], ram[addr]);
}

uint32_t Cpu::runInstruction() {
	uint8_t opcode = readMemory8(pc++);

	dprintf("Addr %04hx Code %02hhx\n", pc-1, opcode);
	dumpRegs();
	// getchar();

	switch (opcode) {
		case 0x00: //NOP
			dprintf("NOP\n");
			return 4;

		case 0xC3: { //JP nn
			dprintf("JP nn\n");
			pc = readMemory16(pc);
			printf("*** %hx\n", pc);
			return 16;
		}

		// case 0xF3: { //DI //TODO:
		// 	return 4;
		// }

		case 0x06: //LD B, n
		case 0x0E: //LD C, n
		case 0x16: //LD D, n
		case 0x1E: //LD E, n
		case 0x26: //LD H, n
		case 0x2E: //LD L, n
		{ 
			dprintf("LD\n");
			regs[(opcode - 0x06) / 8] = readMemory8(pc++);
			return 8;
		}

		case 0x04: //INC B
		case 0x0C: //INC C
		case 0x14: //INC D
		case 0x1C: //INC E
		case 0x24: //INC H
		case 0x2C: //INC L
		case 0x3C: //INC A
		{
			dprintf("INC\n");
			uint8_t idx = (opcode - 0x4) / 0x8;
			f &= ~(Z_FLAG | N_FLAG | H_FLAG);
			f |= Z_FLAG * (regs[idx] == 0xFF) | H_FLAG * ((regs[idx] & 0xF) != 0);
			++regs[idx];
			return 4;
		}

		case 0x05: //DEC B
		case 0x0D: //DEC C
		case 0x15: //DEC D
		case 0x1D: //DEC E
		case 0x25: //DEC H
		case 0x2D: //DEC L
		case 0x3D: //DEC A
		{
			dprintf("DEC\n");
			uint8_t idx = (opcode - 0x5) / 0x8;
			f &= ~(Z_FLAG | H_FLAG);
			f |= N_FLAG | Z_FLAG * (regs[idx] == 1) | H_FLAG * ((regs[idx]&0xF) < 1);
			--regs[idx];
			return 4;
		}

		case 0x13: //INC DE
		case 0x23: //INC HL
		{
			dprintf("INC\n");
			uint8_t idx = (opcode - 0x3) / 0x10;
			uint16_t nn = convertTo16(regs[2*idx+1], regs[2*idx]) + 1;
			convertTo8(nn, regs[2*idx+1], regs[2*idx]);
			return 8;
		}

		case 0x11: { //LD DE, nn
			dprintf("LD DE\n");
			//TODO: direct assignment
			uint16_t de = readMemory16(pc);
			pc += 2;
			convertTo8(de, d, e);
			return 12;
		}

		case 0x78: // LD A, B
		case 0x79: // LD A, C
		case 0x7A: // LD A, D
		case 0x7B: // LD A, E
		case 0x7C: // LD A, H
		case 0x7D: // LD A, L
		case 0x7F: // LD A, A
		{
			dprintf("LD addr\n");
			a = regs[opcode - 0x78];
			return 4;
		}

		case 0x1A: { //LD A, (DE)
			dprintf("LD A\n");
			uint16_t de = convertTo16(e, d);
			a = readMemory8(de);
			return 8;
		}

		case 0x3E: { //LD A, #
			dprintf("LD A\n");
			a = readMemory8(pc++);
			return 8;
		}

		case 0x18: { //JR n
			dprintf("JR\n");
			int8_t n = readMemory8(pc++);
			pc += n;
			return 12;
		}

		case 0x20: { //JR NZ n
			dprintf("JR NZ\n");
			int8_t n = readMemory8(pc++);

			if (!(f & Z_FLAG)) {
				dprintf(">>>> Jump Taken\n");
				pc += n;
				return 12;
			}

			dprintf(">>>> Jump Not Taken\n");
			return 8;
		}

		case 0x28: { //JR Z n
			dprintf("JR Z\n");
			int8_t n = readMemory8(pc++);

			if (f & Z_FLAG) {
				dprintf(">>>> Jump Taken\n");
				pc += n;
				return 12;
			}

			dprintf(">>>> Jump Not Taken\n");
			return 8;
		}

		case 0x30: { //JR NC n
			dprintf("JR NC\n");
			int8_t n = readMemory8(pc++);

			if (!(f & C_FLAG)) {
				dprintf(">>>> Jump Taken\n");
				pc += n;
				return 12;
			}

			dprintf(">>>> Jump Not Taken\n");
			return 8;
		}

		case 0x38: { //JR C n
			dprintf("JR C\n");
			int8_t n = readMemory8(pc++);

			if (f & C_FLAG) {
				dprintf(">>>> Jump Taken\n");
				pc += n;
				return 12;
			}

			dprintf(">>>> Jump Not Taken\n");
			return 8;
		}

		case 0x21: { //LD hl nn
			dprintf("LD HL nn\n");
			uint16_t hl = readMemory16(pc);
			pc += 2;
			convertTo8(hl, l, h);
			return 12;
		}

		case 0x31: { //LD sp nn
			dprintf("LD sp nn\n");
			sp = readMemory16(pc);
			pc += 2;
			return 12;
		}

		case 0x47: //LD B, A
		case 0x4F: //LD C, A
		case 0x57: //LD D, A
		case 0x5F: //LD E, A
		case 0x67: //LD H, A
		case 0x6F: //LD L, A
		{
			dprintf("LD xx A\n");
			regs[(opcode - 0x47) / 0x8] = a;
			return 4;
		}

		case 0x32: { //LD (HL-), A
			dprintf("LD (HL-) A\n");
			uint16_t hl = convertTo16(l, h);
			writeMemory8(hl--, a);
			convertTo8(hl, l, h);
			return 8;
		}

		case 0x77: { //LD (HL), A
			dprintf("LD (HL) A\n");
			uint16_t hl = convertTo16(l, h);
			writeMemory8(hl, a);
			return 8;
		}

		case 0x22: { //LD (HL+), A
			dprintf("LD (HL+) A\n");
			uint16_t hl = convertTo16(l, h);
			writeMemory8(hl++, a);
			convertTo8(hl, l, h);
			return 8;
		}

		case 0x2A: { //LD A, (HL+)
			dprintf("LD A (HL+)\n");
			uint16_t hl = convertTo16(l, h);
			a = readMemory8(hl++);
			convertTo8(hl, l, h);
			return 8;
		}

		case 0x80: //ADD A, B
		case 0x81: //ADD A, C
		case 0x82: //ADD A, D
		case 0x83: //ADD A, E
		case 0x84: //ADD A, H
		case 0x85: //ADD A, L
		case 0x87: //ADD A, A
		{
			dprintf("ADD A n\n");
			uint8_t idx = opcode - 0x80;

			uint8_t carryBits = calculateCarryBits8(a, regs[idx]);

			f &= ~(Z_FLAG | N_FLAG | H_FLAG | C_FLAG);
			f |= Z_FLAG * (a + regs[idx] == 0x00) | C_FLAG * ((carryBits >> 7) & 1) | H_FLAG * ((carryBits >> 3) & 1);

			a += regs[idx];
			return 4;
		}

		case 0x90: // SUB B
		case 0x91: // SUB C
		case 0x92: // SUB D
		case 0x93: // SUB E
		case 0x94: // SUB H
		case 0x95: // SUB L
		case 0x97: // SUB A
		{
			dprintf("SUB\n");
			uint8_t idx = opcode - 0x90;

			f &= ~(Z_FLAG | H_FLAG | C_FLAG);
			f |= N_FLAG | Z_FLAG * (a == regs[idx]) | H_FLAG * ((a&0xF) >= (regs[idx]&0xf)) | C_FLAG * (a >= regs[idx]);

			a -= regs[idx];
			return 4;
		}

		case 0xA0: //AND B
		case 0xA1: //AND C
		case 0xA2: //AND D
		case 0xA3: //AND E
		case 0xA4: //AND H
		case 0xA5: //AND L
		case 0xA7: //AND A
		{
			dprintf("AND\n");

			a &= regs[opcode - 0xA0];

			f &= ~(Z_FLAG | N_FLAG | C_FLAG);
			f |= H_FLAG | Z_FLAG * (a == 0);
			return 4;
		}

		case 0xAF: { //XOR A A
			dprintf("XOR A A\n");
			f &= ~(N_FLAG | H_FLAG | C_FLAG);
			f |= Z_FLAG;

			a = 0;
			return 4;
		}

		case 0xCB: { //2 byte instr
			return runLongInstruction();
		}

		case 0xCD: { //CALL nn
			dprintf("CALL\n");
			uint16_t nn = readMemory16(pc);
			dprintf(">> Call function %hx\n", nn);
			pc += 2;
			
			sp -= 2;
			writeMemory16_inverted(sp, pc);
			dprintf("   Save ret addr %hx\n", pc);
			pc = nn;
			return 24;
		}

		case 0xC9: { //RET
			dprintf("RET\n");
			pc = readMemory16_inverted(sp);
			sp += 2;
			
			dprintf(">> Return to %hx\n", pc);
			return 16;
		}

		case 0xE0: { //LDH (n), A
			dprintf("LDH (n) A\n");
			uint8_t n = readMemory8(pc++);
			writeMemory8(0xFF00 + n, a);
			return 12;
		}

		case 0xF0: { //LDH A, (n)
			dprintf("LDH A (n)\n");
			uint8_t n = readMemory8(pc++);
			a = readMemory8(0xFF00 + n);
			return 12;
		}

		case 0xE2: { //LDH
			dprintf("LDH C\n");
			writeMemory8(0xFF00 + c, a);
			return 8;
		}

		// case 0x02: //LD (BC), A
		// case 0x77: //LD (HL), A

		case 0x12: { //LD (DE), A
			dprintf("LDH (DE) A\n");
			uint16_t de = convertTo16(e, d);
			writeMemory8(de, a);
			return 8;
		}

		case 0x37: { //SCF
			f &= ~(N_FLAG | H_FLAG);
			f |= C_FLAG;
			return 4;
		}

		case 0xC5: //PUSH BC
		case 0xD5: //PUSH DE
		case 0xE5: //PUSH HL
		case 0xF5: //PUSH AF
		{
			dprintf("PUSH\n");
			uint8_t idx = (opcode - 0xC5) / 0x10;
			writeMemory8(--sp, regs[idx+1]);
			writeMemory8(--sp, regs[idx]);
			return 16;
		}

		case 0xC1: //POP BC
		case 0xD1: //POP DE
		case 0xE1: //POP HL
		case 0xF1: //POP AF
		{
			dprintf("POP\n");
			uint8_t idx = (opcode - 0xC1) / 0x10;
			regs[idx] = readMemory8(sp++);
			regs[idx+1] = readMemory8(sp++);
			return 12;
		}

		case 0x17: { //RLA
			dprintf("RLA\n");
			bool cf = f & C_FLAG;

			f &= ~(Z_FLAG | N_FLAG | H_FLAG | C_FLAG);
			f |= Z_FLAG * (a == 0x80) | C_FLAG * (a & 0x80);

			a = (a << 1) | cf;
			return 4;
		}

		case 0xFE: { //CP A, #
			dprintf("CP A, n\n");
			uint8_t n = readMemory8(pc++);

			f &= ~(Z_FLAG | H_FLAG | C_FLAG);
			f |= N_FLAG | Z_FLAG * (a == n) | H_FLAG * ((a&0xF) < (n&0xf)) | C_FLAG * (a < n);

			a -= n;
			return 8;
		}

		case 0xEA: { //LD (nn), A
			dprintf("LD (nn), A\n");
			uint16_t nn = readMemory16(pc);
			pc += 2;
			writeMemory8(nn, a);
			return 16;
		}

		default: {
			char buffer[256];
			sprintf(buffer, "Unknown opcode %02hhx", opcode);
			throw std::runtime_error(buffer);
		}
	}
}

uint32_t Cpu::runLongInstruction() {
	uint8_t opcode = readMemory8(pc++);

	dprintf(" opcode %02hhx\n", opcode);
	// getchar();

	switch (opcode) {
		case 0x7C: { //BIT 7 H
			dprintf("BIT 7 H\n");
			f &= ~(Z_FLAG | N_FLAG);
			f |= (Z_FLAG * !(h & (1 << 7))) | H_FLAG;
			return 8;
		}

		case 0x11: { //RL C
			dprintf("RL C\n");
			bool cf = f & C_FLAG;

			f &= ~(Z_FLAG | N_FLAG | H_FLAG | C_FLAG);
			f |= Z_FLAG * (c == 0x80) | C_FLAG * ((c & 0x80) != 0);

			c = (c << 1) | cf;
			return 8;
		}

		default: {
			char buffer[256];
			sprintf(buffer, "Unknown opcode cb%02hhx", opcode);
			throw std::runtime_error(buffer);
		}
	}
}


void Cpu::dumpRegs() {
	dprintf("=========\n");
	dprintf("A %02hhx F %02hhx\n", a, f);
	dprintf("B %02hhx C %02hhx\n", b, c);
	dprintf("D %02hhx E %02hhx\n", d, e);
	dprintf("H %02hhx L %02hhx\n", h, l);
	dprintf("SP %04hx\n", sp);
	dprintf("=========\n");
}

void Cpu::run() {
	init();

	running = true;

	while (running) {
		handleDebugger();
		runInstruction();
	}
}
