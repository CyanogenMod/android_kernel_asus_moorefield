#ifndef _vied_bit_h
#define _vied_bit_h

/** \brief compute the minimum number of bits to encode a value */
static inline unsigned int vied_minimum_bits(
		unsigned int value)
{
	int result = 0;

	if (value == 0) {
		return 0;
	}

	value -= 1;
	while (value) {
		result++;
		value >>= 1;
	}

	return result;
}

/** \brief compute the byte index containing a certain bit where the LSB is in byte 0 */
static inline unsigned int vied_bitpos_to_bytepos(
		unsigned int bitpos)
{
	return (bitpos + 7) / 8;
}

/** \brief compute the least significant bit position of a certain byte where the LSB is in byte 0 */
static inline unsigned int vied_bytepos_to_bitpos(
		unsigned int bytepos)
{
	return bytepos * 8;
}

/** \brief get a slice of bits from a value */
static inline unsigned int vied_bit_slice(
		unsigned int value,
		unsigned int lsb,
		unsigned int numbits)
{
	unsigned int mask = (1 << numbits) - 1;
	return (value >> lsb) & mask;
}

/** \brief get a slice of bytes from a value */
static inline unsigned int vied_byte_slice(
		unsigned int value,
		unsigned int byte0,
		unsigned int numbytes)
{
	unsigned int mask = (1 << (numbytes * 8)) - 1;
	return (value >> (byte0 * 8)) & mask;
}

/** \brief shift a value and perform bitwise OR operation with another value */
static inline unsigned int vied_bit_shift_OR(
		unsigned int val1,
		unsigned int numbits,
		unsigned int val2)
{
	return (val1 << numbits) | vied_bit_slice(val2, 0, numbits);
}

/** \brief set a bit in a value */
static inline unsigned int vied_bit_set_bit(
		unsigned int value,
		unsigned int lsb)
{
	return value | (1 << lsb);
}

/** \brief clear a bit in a value */
static inline unsigned int vied_bit_clear_bit(
		unsigned int value,
		unsigned int lsb)
{
	return value & (~(1 << lsb));
}

#endif				/* _vied_bit_h */
