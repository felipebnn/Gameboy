#include "cpu.h"

#include <exception>
#include <memory>

static inline uint16_t convertTo16(uint8_t a, uint8_t b) {
	return a | (b << 8);
}

static inline void convertTo8(uint16_t in, uint8_t& a, uint8_t& b) {
	a = in & 0xff;
	b = (in >> 8) & -0xff;
}

static inline uint8_t calculateCarryBits8(uint8_t a, uint8_t b) {
	uint8_t res = a + b;
	return (a & ~res) | (b & ~res) | (a & b);
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

void Cpu::setByte(uint16_t addr, uint8_t b) {
	ram[addr] = b;

	if (addr >= 0xE000 && addr < 0xFE00) {
		ram[addr - 0x2000] = b;
	}
}

uint8_t Cpu::getByte(uint16_t addr) {
	return ram[addr];
}

void Cpu::runInstruction() {
	uint8_t opcode = ram[pc++];

	printf("addr %04hx opcode %02hhx\n", pc-1, opcode);
	// getchar();

	switch (opcode) {
		case 0x00: //NOP
			return;

		case 0x06: //LD B, d8
		case 0x0E: //LD C, d8
		{ 
			ram[(opcode - 0x06) / 8] = ram[pc++];
			break;
		}

		case 0x05: //DEC B
		case 0x3d: //DEC A
		{
			uint8_t idx = (opcode - 0x5) / 0x8;
			f |= N_FLAG | Z_FLAG * (regs[idx] == 1) | H_FLAG * ((regs[idx] & 0xF) == 0);
			++regs[idx];
			break;
		}

		case 0x0C: { //INC C
			f &= ~N_FLAG;
			f |= Z_FLAG * (c == 0xFF) | H_FLAG * ((c & 0xF) != 0);
			++c;
			break;
		}

		case 0x13: //INC DE
		case 0x23: //INC HL
		{
			uint8_t idx = (opcode - 0x3) / 0x10;
			uint16_t nn = convertTo16(regs[2*idx+1], regs[2*idx]) + 1;
			convertTo8(nn, regs[2*idx+1], regs[2*idx]);
			break;
		}

		case 0x11: { //LD DE, nn
			uint16_t de = convertTo16(ram[pc], ram[pc+1]);
			pc += 2;

			convertTo8(de, d, e);
		}

		case 0x7B: { //LD A, E
			a = e;
			break;
		}

		case 0x1A: { //LD A, (DE)
			uint16_t de = convertTo16(e, d);

			a = getByte(de);
			break;
		}

		case 0x3E: { //LD A, #
			a = ram[pc++];
			break;
		}

		case 0x20: { //JR NZ
			uint8_t n = ram[pc++];

			if (!(f & Z_FLAG)) {
				printf(">> Jumped\n");
				pc += n;
			} else printf(">> Didn't jumped\n");
			break;
		}

		case 0x28: { //JR Z
			uint8_t n = ram[pc++];

			if (f & Z_FLAG) {
				printf(">> Jumped\n");
				pc += n;
			} else printf(">> Didn't jumped\n");
			break;
		}

		case 0x30: { //JR NC
			uint8_t n = ram[pc++];

			if (!(f & C_FLAG)) {
				printf(">> Jumped\n");
				pc += n;
			} else printf(">> Didn't jumped\n");
			break;
		}

		case 0x38: { //JR NC
			uint8_t n = ram[pc++];

			if (f & C_FLAG) {
				printf(">> Jumped\n");
				pc += n;
			} else printf(">> Didn't jumped\n");
			break;
		}

		case 0x21: { //LD hl nn
			uint16_t hl = convertTo16(ram[pc], ram[pc+1]);
			pc += 2;

			convertTo8(hl, l, h);
			break;
		}

		case 0x31: { //LD sp nn
			sp = convertTo16(ram[pc], ram[pc+1]);
			printf("LOAD SP %hx\n", sp);
			pc += 2;
			break;
		}

		case 0x4F: { //LD C, A
			c = a;
			break;
		}

		case 0x32: { //LD (HL-), A
			uint16_t hl = convertTo16(l, h);
			setByte(hl--, a);
			convertTo8(hl, l, h);
			break;
		}

		case 0x77: { //LD (HL), A
			uint16_t hl = convertTo16(l, h);
			setByte(hl, a);
			break;
		}

		case 0x22: { //LD (HL+), A
			uint16_t hl = convertTo16(l, h);
			setByte(hl++, a);
			convertTo8(hl, l, h);
			break;
		}

		case 0x80: { //ADD A, B
			uint8_t carryBits = calculateCarryBits8(a, b);

			f &= ~N_FLAG;
			f |= (C_FLAG * ((carryBits >> 7) & 1)) | (H_FLAG * ((carryBits >> 3) & 1));

			a += b;
			break;
		}

		case 0xAF: { //XOR A A
			f &= ~(N_FLAG | H_FLAG | C_FLAG);
			f |= Z_FLAG;

			a = 0;
			break;
		}

		case 0xCB: { //2 byte instr
			runCbInstruction();
			break;
		}

		case 0xCD: { //CALL nn
			uint16_t nn = convertTo16(ram[pc], ram[pc+1]);
			printf(">> Call function %hx\n", nn);
			pc += 2;
			

			sp -= 2;
			convertTo8(pc, ram[sp+1], ram[sp]);
			printf("   Save ret addr %hx\n", pc);
			pc = nn;
			break;
		}

		case 0xC9: { //RET
			pc = convertTo16(ram[sp+1], ram[sp]);
			sp += 2;
			
			printf(">> Return to %hx\n", pc);
			break;
		}

		case 0xE0: { //LDH (a8), A
			uint8_t n = ram[pc++];
			setByte(0xFF00 + n, a);
			break;
		}

		case 0xE2: { //LDH
			setByte(0xFF00 + c, a);
			break;
		}

		case 0xC5: { //PUSH BC //TODO: check
			ram[--sp] = c;
			ram[--sp] = b;
			break;
		}

		case 0xC1: { //POP BC //TODO: check
			c = ram[sp++];
			b = ram[sp++];
			break;
		}

		case 0x17: { //RLA
			bool cf = f & C_FLAG;

			f &= ~(N_FLAG | H_FLAG);
			f |= Z_FLAG * (a == 0x80) | C_FLAG * (a & 0x80);

			a = (a << 1) | cf;
			break;
		}

		case 0xFE: { //CP A, #
			uint8_t n = ram[pc++];

			f |= N_FLAG | Z_FLAG * (a == n) | H_FLAG * ((a&0xF) < (n&0xf)) | C_FLAG * (a < n);

			a -= n;
			break;
		}

		case 0xEA: { //LD (nn), A
			uint16_t nn = convertTo16(ram[pc], ram[pc+1]);
			pc += 2;
			ram[nn] = a;
			break;
		}

		default: {
			char buffer[256];
			sprintf(buffer, "Unknown opcode %02hhx", opcode);
			throw std::runtime_error(buffer);
		}
	}
}

void Cpu::runCbInstruction() {
	uint8_t opcode = ram[pc++];

	printf(" opcode %02hhx\n", opcode);
	// getchar();

	switch (opcode) {
		case 0x7C: { //BIT 7 H
			f &= ~N_FLAG;
			f |= (Z_FLAG * !(h & (1 << 7))) | H_FLAG;
			break;
		}

		case 0x11: { //RL C
			bool cf = f & C_FLAG;

			f &= ~(N_FLAG | H_FLAG);
			f |= Z_FLAG * (c == 0x80) | C_FLAG * (c & 0x80);

			c = (c << 1) | cf;
			break;
		}

		default: {
			char buffer[256];
			sprintf(buffer, "Unknown opcode cb%02hhx", opcode);
			throw std::runtime_error(buffer);
		}
	}
}


void Cpu::run() {
	init();

	running = true;

	while (running) {
		runInstruction();
	}
}
