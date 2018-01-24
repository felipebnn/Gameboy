#pragma once

#include <cstdint>

inline uint16_t convertTo16(uint8_t a, uint8_t b) {
	return a | (b << 8);
}

inline void convertTo8(uint16_t in, uint8_t& a, uint8_t& b) {
	a = in & 0xff;
	b = (in >> 8) & 0xff;
}

inline uint8_t calculateCarryBits8(uint8_t a, uint8_t b) {
	uint8_t res = a + b;
	return (a & ~res) | (b & ~res) | (a & b);
}
