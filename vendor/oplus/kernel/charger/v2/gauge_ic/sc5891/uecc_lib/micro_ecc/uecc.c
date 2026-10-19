// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024-2026 Oplus. All rights reserved.
 */

#include "uecc.h"
#include "uecc_vli.h"

#ifndef U_ECC_RNG_MAX_TRIES
#define U_ECC_RNG_MAX_TRIES 64
#endif

#if UECC_ENABLE_VLI_API
#define U_ECC_VLI_API
#else
#define U_ECC_VLI_API static
#endif

#if (U_ECC_PLATFORM == U_ECC_AVR) || \
	(U_ECC_PLATFORM == U_ECC_ARM) || \
	(U_ECC_PLATFORM == U_ECC_ARM_THUMB) || \
	(U_ECC_PLATFORM == U_ECC_ARM_THUMB2)
#define CONCATX(a, ...) a ## __VA_ARGS__
#define CONCAT(a, ...) CONCATX(a, __VA_ARGS__)

#undef STR
#define STRX(a) #a
#define STR(a) STRX(a)

#define EVAL(...)  EVAL1(EVAL1(EVAL1(EVAL1(__VA_ARGS__))))
#define EVAL1(...) EVAL2(EVAL2(EVAL2(EVAL2(__VA_ARGS__))))
#define EVAL2(...) EVAL3(EVAL3(EVAL3(EVAL3(__VA_ARGS__))))
#define EVAL3(...) EVAL4(EVAL4(EVAL4(EVAL4(__VA_ARGS__))))
#define EVAL4(...) __VA_ARGS__

#define DEC_1  0
#define DEC_2  1
#define DEC_3  2
#define DEC_4  3
#define DEC_5  4
#define DEC_6  5
#define DEC_7  6
#define DEC_8  7
#define DEC_9  8
#define DEC_10 9
#define DEC_11 10
#define DEC_12 11
#define DEC_13 12
#define DEC_14 13
#define DEC_15 14
#define DEC_16 15
#define DEC_17 16
#define DEC_18 17
#define DEC_19 18
#define DEC_20 19
#define DEC_21 20
#define DEC_22 21
#define DEC_23 22
#define DEC_24 23
#define DEC_25 24
#define DEC_26 25
#define DEC_27 26
#define DEC_28 27
#define DEC_29 28
#define DEC_30 29
#define DEC_31 30
#define DEC_32 31

#define DEC(N) CONCAT(DEC_, N)

#define SECOND_ARG(_, val, ...) val
#define SOME_CHECK_0 ~, 0
#define GET_SECOND_ARG(...) SECOND_ARG(__VA_ARGS__, SOME, )
#define SOME_OR_0(N) GET_SECOND_ARG(CONCAT(SOME_CHECK_, N))

#define EMPTY(...)
#define DEFER(...) __VA_ARGS__ EMPTY()

#define REPEAT_NAME_0() REPEAT_0
#define REPEAT_NAME_SOME() REPEAT_SOME
#define REPEAT_0(...)
#define REPEAT_SOME(N, stuff) DEFER(CONCAT(REPEAT_NAME_, SOME_OR_0(DEC(N))))()(DEC(N), stuff) stuff
#define REPEAT(N, stuff) EVAL(REPEAT_SOME(N, stuff))

#define REPEATM_NAME_0() REPEATM_0
#define REPEATM_NAME_SOME() REPEATM_SOME
#define REPEATM_0(...)
#define REPEATM_SOME(N, macro) macro(N) \
	DEFER(CONCAT(REPEATM_NAME_, SOME_OR_0(DEC(N))))()(DEC(N), macro)
#define REPEATM(N, macro) EVAL(REPEATM_SOME(N, macro))
#endif

#include "platform_specific.inc"

#if (U_ECC_WORD_SIZE == 1)
#if U_ECC_SUPPORTS_SECP160R1
#define U_ECC_MAX_WORDS 21		 /* Due to the size of curve_n. */
#endif
#if (U_ECC_SUPPORTS_SECP192R1 || U_ECC_SUPPORTS_SECP192K1)
#undef U_ECC_MAX_WORDS
#define U_ECC_MAX_WORDS 24
#endif
#if U_ECC_SUPPORTS_SECP224R1
#undef U_ECC_MAX_WORDS
#define U_ECC_MAX_WORDS 28
#endif
#if (U_ECC_SUPPORTS_SECP256R1 || U_ECC_SUPPORTS_SECP256K1)
#undef U_ECC_MAX_WORDS
#define U_ECC_MAX_WORDS 32
#endif
#elif (U_ECC_WORD_SIZE == 4)
#if U_ECC_SUPPORTS_SECP160R1
#define U_ECC_MAX_WORDS 6		 /* Due to the size of curve_n. */
#endif
#if (U_ECC_SUPPORTS_SECP192R1 || U_ECC_SUPPORTS_SECP192K1)
#undef U_ECC_MAX_WORDS
#define U_ECC_MAX_WORDS 6
#endif
#if U_ECC_SUPPORTS_SECP224R1
#undef U_ECC_MAX_WORDS
#define U_ECC_MAX_WORDS 7
#endif
#if (U_ECC_SUPPORTS_SECP256R1 || U_ECC_SUPPORTS_SECP256K1)
#undef U_ECC_MAX_WORDS
#define U_ECC_MAX_WORDS 8
#endif
#elif (U_ECC_WORD_SIZE == 8)
#if U_ECC_SUPPORTS_SECP160R1
#define U_ECC_MAX_WORDS 3
#endif
#if (U_ECC_SUPPORTS_SECP192R1 || U_ECC_SUPPORTS_SECP192K1)
#undef U_ECC_MAX_WORDS
#define U_ECC_MAX_WORDS 3
#endif
#if U_ECC_SUPPORTS_SECP224R1
#undef U_ECC_MAX_WORDS
#define U_ECC_MAX_WORDS 4
#endif
#if (U_ECC_SUPPORTS_SECP256R1 || U_ECC_SUPPORTS_SECP256K1)
#undef U_ECC_MAX_WORDS
#define U_ECC_MAX_WORDS 4
#endif
#endif /* U_ECC_WORD_SIZE */

#define BITS_TO_WORDS(num_bits) ((num_bits + ((U_ECC_WORD_SIZE * 8) - 1)) / (U_ECC_WORD_SIZE * 8))

struct u_ecc_curve_t {
	wordcount_t num_words;
	wordcount_t num_bytes;
	bitcount_t num_n_bits;
	u_ecc_word_t p[U_ECC_MAX_WORDS];
	u_ecc_word_t n[U_ECC_MAX_WORDS];
	u_ecc_word_t G[U_ECC_MAX_WORDS * 2];
	u_ecc_word_t b[U_ECC_MAX_WORDS];
	void (*double_jacobian)(u_ecc_word_t *X1, u_ecc_word_t *Y1, u_ecc_word_t *Z1, u_ecc_curve curve);
#if U_ECC_SUPPORT_COMPRESSED_POINT
	void (*mod_sqrt)(u_ecc_word_t *a, u_ecc_curve curve);
#endif
	void (*x_side)(u_ecc_word_t *result, const u_ecc_word_t *x, u_ecc_curve curve);
#if (U_ECC_OPTIMIZATION_LEVEL > 0)
	void (*mmod_fast)(u_ecc_word_t *result, u_ecc_word_t *product);
#endif
};

#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN
static void uecc_bcopy(uint8_t *dst,
		       const uint8_t *src,
		       unsigned num_bytes)
{
	while (0 != num_bytes) {
		num_bytes--;
		dst[num_bytes] = src[num_bytes];
	}
}
#endif

static cmpresult_t u_ecc_vli_cmp_unsafe(const u_ecc_word_t *left, const u_ecc_word_t *right,
				       wordcount_t num_words);

#if (U_ECC_PLATFORM == U_ECC_ARM || U_ECC_PLATFORM == U_ECC_ARM_THUMB || \
	 U_ECC_PLATFORM == U_ECC_ARM_THUMB2)
#include "asm_arm.inc"
#endif

#if (U_ECC_PLATFORM == U_ECC_AVR)
#include "asm_avr.inc"
#endif

#if DEFAULT_RNG_DEFINED
static u_ecc_rng_function g_rng_function = &default_rng;
#else
static u_ecc_rng_function g_rng_function = 0;
#endif

#ifndef WITH_ROM
void u_ecc_set_rng(u_ecc_rng_function rng_function)
{
	g_rng_function = rng_function;
}

#else
void u_ecc_set_rng(u_ecc_rng_function rng_function)
{
	rom_u_ecc_set_rng(rng_function);
}

#endif

#ifndef WITH_ROM
u_ecc_rng_function u_ecc_get_rng(void)
{
	return g_rng_function;
}

#else
u_ecc_rng_function u_ecc_get_rng(void)
{
	return rom_u_ecc_get_rng();
}

#endif

int u_ecc_curve_private_key_size(u_ecc_curve curve)
{
	return BITS_TO_BYTES(curve->num_n_bits);
}

int u_ecc_curve_public_key_size(u_ecc_curve curve)
{
	return 2 * curve->num_bytes;
}

#ifndef asm_clear
U_ECC_VLI_API void u_ecc_vli_clear(u_ecc_word_t *vli, wordcount_t num_words)
{
	wordcount_t i;

	for (i = 0; i < num_words; ++i)
		vli[i] = 0;
}
#endif /* !asm_clear */

/* Constant-time comparison to zero - secure way to compare long integers */
/* Returns 1 if vli == 0, 0 otherwise. */
U_ECC_VLI_API u_ecc_word_t u_ecc_vli_is_zero(const u_ecc_word_t *vli, wordcount_t num_words)
{
	u_ecc_word_t bits = 0;
	wordcount_t i;

	for (i = 0; i < num_words; ++i)
		bits |= vli[i];

	return bits == 0;
} /* u_ecc_vli_is_zero */

/* Returns nonzero if bit 'bit' of vli is set. */
U_ECC_VLI_API u_ecc_word_t u_ecc_vli_test_bit(const u_ecc_word_t *vli, bitcount_t bit)
{
	return vli[bit >> UECC_WORD_BITS_SHIFT] & ((u_ecc_word_t)1 << (bit & UECC_WORD_BITS_MASK));
}

/* Counts the number of words in vli. */
static wordcount_t vli_num_digits(const u_ecc_word_t *vli, const wordcount_t max_words)
{
	wordcount_t i;

	/* Search from the end until we find a non-zero digit.
	 * We do it in reverse because we expect that most digits will be nonzero. */
	for (i = max_words - 1; (i >= 0) && (vli[i] == 0); --i);

	return i + 1;
} /* vli_num_digits */

/* Counts the number of bits required to represent vli. */
U_ECC_VLI_API bitcount_t u_ecc_vli_num_bits(const u_ecc_word_t *vli, const wordcount_t max_words)
{
	u_ecc_word_t i;
	u_ecc_word_t digit;

	wordcount_t num_digits = vli_num_digits(vli, max_words);

	if (num_digits == 0)
		return 0;

	digit = vli[num_digits - 1];
	for (i = 0; digit; ++i)
		digit >>= 1;

	return ((bitcount_t)(num_digits - 1) << UECC_WORD_BITS_SHIFT) + i;
} /* u_ecc_vli_num_bits */
#ifndef asm_set
U_ECC_VLI_API void u_ecc_vli_set(u_ecc_word_t *dest, const u_ecc_word_t *src, wordcount_t num_words)
{
	wordcount_t i;

	for (i = 0; i < num_words; ++i)
		dest[i] = src[i];
}
#endif /* asm_set */
/* Sets dest = src. */

/* Returns sign of left - right. */
static cmpresult_t u_ecc_vli_cmp_unsafe(const u_ecc_word_t *left,
				       const u_ecc_word_t *right,
				       wordcount_t num_words)
{
	wordcount_t i;

	for (i = num_words - 1; i >= 0; --i) {
		if (left[i] > right[i])
			return 1;
		else if (left[i] < right[i])
			return -1;
	}
	return 0;
} /* u_ecc_vli_cmp_unsafe */

/* Constant-time comparison function - secure way to compare long integers */
/* Returns one if left == right, zero otherwise. */
U_ECC_VLI_API u_ecc_word_t u_ecc_vli_equal(const u_ecc_word_t *left,
					const u_ecc_word_t *right,
					wordcount_t num_words)
{
	u_ecc_word_t diff = 0;
	wordcount_t i;

	for (i = num_words - 1; i >= 0; --i)
		diff |= (left[i] ^ right[i]);

	return diff == 0;
} /* u_ecc_vli_equal */

U_ECC_VLI_API u_ecc_word_t u_ecc_vli_sub(u_ecc_word_t *result, const u_ecc_word_t *left,
				      const u_ecc_word_t *right, wordcount_t num_words);

/* Returns sign of left - right, in constant time. */
U_ECC_VLI_API cmpresult_t u_ecc_vli_cmp(const u_ecc_word_t *left,
				      const u_ecc_word_t *right,
				      wordcount_t num_words)
{
	u_ecc_word_t tmp[U_ECC_MAX_WORDS];
	u_ecc_word_t neg = !!u_ecc_vli_sub(tmp, left, right, num_words);
	u_ecc_word_t equal = u_ecc_vli_is_zero(tmp, num_words);

	return !equal - 2 * neg;
}

#ifndef asm_rshift1
/* Computes vli = vli >> 1. */
U_ECC_VLI_API void u_ecc_vli_rshift1(u_ecc_word_t *vli, wordcount_t num_words)
{
	u_ecc_word_t *end = vli;
	u_ecc_word_t carry = 0;

	vli += num_words;
	while (vli-- > end) {
		u_ecc_word_t temp = *vli;
		*vli = (temp >> 1) | carry;
		carry = temp << (UECC_WORD_BITS - 1);
	}
} /* u_ecc_vli_rshift1 */

#endif /* !asm_rshift1 */

#ifndef asm_add
/* Computes result = left + right, returning carry. Can modify in place. */
U_ECC_VLI_API u_ecc_word_t u_ecc_vli_add(u_ecc_word_t *result,
				      const u_ecc_word_t *left,
				      const u_ecc_word_t *right,
				      wordcount_t num_words)
{
	u_ecc_word_t carry = 0;
	wordcount_t i;

	for (i = 0; i < num_words; ++i) {
		u_ecc_word_t sum = left[i] + right[i] + carry;
		if (sum != left[i])
			carry = (sum < left[i]);

		result[i] = sum;
	}
	return carry;
} /* u_ecc_vli_add */

#endif /* asm_add */

#ifndef asm_sub
/* Computes result = left - right, returning borrow. Can modify in place. */
U_ECC_VLI_API u_ecc_word_t u_ecc_vli_sub(u_ecc_word_t *result,
				      const u_ecc_word_t *left,
				      const u_ecc_word_t *right,
				      wordcount_t num_words)
{
	u_ecc_word_t borrow = 0;
	wordcount_t i;

	for (i = 0; i < num_words; ++i) {
		u_ecc_word_t diff = left[i] - right[i] - borrow;
		if (diff != left[i])
			borrow = (diff > left[i]);

		result[i] = diff;
	}
	return borrow;
} /* u_ecc_vli_sub */

#endif /* !asm_sub */

#ifndef asm_mult
#if (U_ECC_SQUARE_FUNC && !asm_square) || \
	(U_ECC_SUPPORTS_SECP256K1 && (U_ECC_OPTIMIZATION_LEVEL > 0) && \
	((U_ECC_WORD_SIZE == 1) || (U_ECC_WORD_SIZE == 8)))
static void muladd(u_ecc_word_t  a,
		   u_ecc_word_t  b,
		   u_ecc_word_t *r0,
		   u_ecc_word_t *r1,
		   u_ecc_word_t *r2)
{
#if U_ECC_WORD_SIZE == 8 && !SUPPORTS_INT128
	uint64_t a0 = a & 0xffffffffull;
	uint64_t a1 = a >> 32;
	uint64_t b0 = b & 0xffffffffull;
	uint64_t b1 = b >> 32;

	uint64_t i0 = a0 * b0;
	uint64_t i1 = a0 * b1;
	uint64_t i2 = a1 * b0;
	uint64_t i3 = a1 * b1;

	uint64_t p0, p1;

	i2 += (i0 >> 32);
	i2 += i1;
	if (i2 < i1) { /* overflow */
		i3 += 0x100000000ull;
	}

	p0 = (i0 & 0xffffffffull) | (i2 << 32);
	p1 = i3 + (i2 >> 32);

	*r0 += p0;
	*r1 += (p1 + (*r0 < p0));
	*r2 += ((*r1 < p1) || (*r1 == p1 && *r0 < p0));
#else
	u_ecc_dword_t p = (u_ecc_dword_t)a * b;
	u_ecc_dword_t r01 = ((u_ecc_dword_t)(*r1) << UECC_WORD_BITS) | *r0;
	r01 += p;
	*r2 += (r01 < p);
	*r1 = r01 >> UECC_WORD_BITS;
	*r0 = (u_ecc_word_t)r01;
#endif
} /* muladd */

#endif /* muladd needed */
#endif

#ifndef asm_mult
U_ECC_VLI_API void u_ecc_vli_mult(u_ecc_word_t *result,
				const u_ecc_word_t *left,
				const u_ecc_word_t *right,
				wordcount_t num_words)
{
	u_ecc_word_t r0 = 0;
	u_ecc_word_t r1 = 0;
	u_ecc_word_t r2 = 0;
	wordcount_t i, k;

	/* Compute each digit of result in sequence, maintaining the carries. */
	for (k = 0; k < num_words; ++k) {
		for (i = 0; i <= k; ++i)
			muladd(left[i], right[k - i], &r0, &r1, &r2);

		result[k] = r0;
		r0 = r1;
		r1 = r2;
		r2 = 0;
	}
	for (k = num_words; k < num_words * 2 - 1; ++k) {
		for (i = (k + 1) - num_words; i < num_words; ++i)
			muladd(left[i], right[k - i], &r0, &r1, &r2);

		result[k] = r0;
		r0 = r1;
		r1 = r2;
		r2 = 0;
	}
	result[num_words * 2 - 1] = r0;
} /* u_ecc_vli_mult */

#endif /* !asm_mult */

#if U_ECC_SQUARE_FUNC

#if !asm_square
static void mul2add(u_ecc_word_t a,
		    u_ecc_word_t b,
		    u_ecc_word_t *r0,
		    u_ecc_word_t *r1,
		    u_ecc_word_t *r2)
{
#if U_ECC_WORD_SIZE == 8 && !SUPPORTS_INT128
	uint64_t a0 = a & 0xffffffffull;
	uint64_t a1 = a >> 32;
	uint64_t b0 = b & 0xffffffffull;
	uint64_t b1 = b >> 32;

	uint64_t i0 = a0 * b0;
	uint64_t i1 = a0 * b1;
	uint64_t i2 = a1 * b0;
	uint64_t i3 = a1 * b1;

	uint64_t p0, p1;

	i2 += (i0 >> 32);
	i2 += i1;
	if (i2 < i1) /* overflow */
		i3 += 0x100000000ull;

	p0 = (i0 & 0xffffffffull) | (i2 << 32);
	p1 = i3 + (i2 >> 32);

	*r2 += (p1 >> 63);
	p1 = (p1 << 1) | (p0 >> 63);
	p0 <<= 1;

	*r0 += p0;
	*r1 += (p1 + (*r0 < p0));
	*r2 += ((*r1 < p1) || (*r1 == p1 && *r0 < p0));
#else
	u_ecc_dword_t p = (u_ecc_dword_t)a * b;
	u_ecc_dword_t r01 = ((u_ecc_dword_t)(*r1) << UECC_WORD_BITS) | *r0;
	*r2 += (p >> (UECC_WORD_BITS * 2 - 1));
	p *= 2;
	r01 += p;
	*r2 += (r01 < p);
	*r1 = r01 >> UECC_WORD_BITS;
	*r0 = (u_ecc_word_t)r01;
#endif
} /* mul2add */

U_ECC_VLI_API void u_ecc_vli_square(u_ecc_word_t * result,
				  const u_ecc_word_t *left,
				  wordcount_t num_words)
{
	u_ecc_word_t r0 = 0;
	u_ecc_word_t r1 = 0;
	u_ecc_word_t r2 = 0;

	wordcount_t i, k;

	for (k = 0; k < num_words * 2 - 1; ++k) {
		u_ecc_word_t min = (k < num_words ? 0 : (k + 1) - num_words);
		for (i = min; (i <= k) && (i <= k - i); ++i) {
			if (i < k - i)
				mul2add(left[i], left[k - i], &r0, &r1, &r2);
			else
				muladd(left[i], left[k - i], &r0, &r1, &r2);
		}
		result[k] = r0;
		r0 = r1;
		r1 = r2;
		r2 = 0;
	}

	result[num_words * 2 - 1] = r0;
} /* u_ecc_vli_square */

#endif /* !asm_square */

#else /* U_ECC_SQUARE_FUNC */

#if UECC_ENABLE_VLI_API
U_ECC_VLI_API void u_ecc_vli_square(u_ecc_word_t *result,
				  const u_ecc_word_t *left,
				  wordcount_t num_words)
{
	u_ecc_vli_mult(result, left, left, num_words);
}

#endif /* UECC_ENABLE_VLI_API */

#endif /* U_ECC_SQUARE_FUNC */

/* Computes result = (left + right) % mod.
 * Assumes that left < mod and right < mod, and that result does not overlap mod. */
U_ECC_VLI_API void u_ecc_vli_mod_add(u_ecc_word_t *result,
				  const u_ecc_word_t *left,
				  const u_ecc_word_t *right,
				  const u_ecc_word_t *mod,
				  wordcount_t num_words)
{
	u_ecc_word_t carry = u_ecc_vli_add(result, left, right, num_words);

	if (carry || u_ecc_vli_cmp_unsafe(mod, result, num_words) != 1) {
		/* result > mod (result = mod + remainder), so subtract mod to get remainder. */
		u_ecc_vli_sub(result, result, mod, num_words);
	}
}

/* Computes result = (left - right) % mod.
 * Assumes that left < mod and right < mod, and that result does not overlap mod. */
U_ECC_VLI_API void u_ecc_vli_mod_sub(u_ecc_word_t *result,
				  const u_ecc_word_t *left,
				  const u_ecc_word_t *right,
				  const u_ecc_word_t *mod,
				  wordcount_t num_words)
{
	u_ecc_word_t l_borrow = u_ecc_vli_sub(result, left, right, num_words);

	if (l_borrow) {
		/* In this case, result == -diff == (max int) - diff. Since -x % d == d - x,
		 * we can get the correct result from result + mod (with overflow). */
		u_ecc_vli_add(result, result, mod, num_words);
	}
}

/* Computes result = product % mod, where product is 2N words long. */
/* Currently only designed to work for curve_p or curve_n. */
U_ECC_VLI_API void u_ecc_vli_mmod(u_ecc_word_t *result,
				u_ecc_word_t *product,
				const u_ecc_word_t *mod,
				wordcount_t num_words)
{
	u_ecc_word_t mod_multiple[2 * U_ECC_MAX_WORDS];
	u_ecc_word_t tmp[2 * U_ECC_MAX_WORDS];
	u_ecc_word_t *v[2] = { tmp, product };
	u_ecc_word_t index;

	/* Shift mod so its highest set bit is at the maximum position. */
	bitcount_t shift = (num_words * 2 * UECC_WORD_BITS) - u_ecc_vli_num_bits(mod, num_words);
	wordcount_t word_shift = shift / UECC_WORD_BITS;
	wordcount_t bit_shift = shift % UECC_WORD_BITS;
	u_ecc_word_t carry = 0;

	u_ecc_vli_clear(mod_multiple, word_shift);
	if (bit_shift > 0) {
		for (index = 0; index < (u_ecc_word_t)num_words; ++index) {
			mod_multiple[word_shift + index] = (mod[index] << bit_shift) | carry;
			carry = mod[index] >> (UECC_WORD_BITS - bit_shift);
		}
	}
	else {
		u_ecc_vli_set(mod_multiple + word_shift, mod, num_words);
	}

	for (index = 1; shift >= 0; --shift) {
		u_ecc_word_t borrow = 0;
		wordcount_t i;
		for (i = 0; i < num_words * 2; ++i) {
			u_ecc_word_t diff = v[index][i] - mod_multiple[i] - borrow;
			if (diff != v[index][i]) {
				borrow = (diff > v[index][i]);
			}
			v[1 - index][i] = diff;
		}
		index = !(index ^ borrow); /* Swap the index if there was no borrow */
		u_ecc_vli_rshift1(mod_multiple, num_words);
		mod_multiple[num_words - 1] |= mod_multiple[num_words] << (UECC_WORD_BITS - 1);
		u_ecc_vli_rshift1(mod_multiple + num_words, num_words);
	}
	u_ecc_vli_set(result, v[index], num_words);
} /* u_ecc_vli_mmod */

/* Computes result = (left * right) % mod. */
U_ECC_VLI_API void u_ecc_vli_mod_mult(u_ecc_word_t *result,
				   const u_ecc_word_t *left,
				   const u_ecc_word_t *right,
				   const u_ecc_word_t *mod,
				   wordcount_t num_words)
{
	u_ecc_word_t product[2 * U_ECC_MAX_WORDS];

	u_ecc_vli_mult(product, left, right, num_words);
	u_ecc_vli_mmod(result, product, mod, num_words);
}

U_ECC_VLI_API void u_ecc_vli_mod_mult_fast(u_ecc_word_t *result,
					const u_ecc_word_t *left,
					const u_ecc_word_t *right,
					u_ecc_curve curve)
{
	u_ecc_word_t product[2 * U_ECC_MAX_WORDS];

	u_ecc_vli_mult(product, left, right, curve->num_words);
#if (U_ECC_OPTIMIZATION_LEVEL > 0)
	curve->mmod_fast(result, product);
#else
	u_ecc_vli_mmod(result, product, curve->p, curve->num_words);
#endif
} /* u_ecc_vli_mod_mult_fast */

#if U_ECC_SQUARE_FUNC

#if UECC_ENABLE_VLI_API
/* Computes result = left^2 % mod. */
U_ECC_VLI_API void u_ecc_vli_mod_square(u_ecc_word_t *result,
				     const u_ecc_word_t *left,
				     const u_ecc_word_t *mod,
				     wordcount_t num_words)
{
	u_ecc_word_t product[2 * U_ECC_MAX_WORDS];

	u_ecc_vli_square(product, left, num_words);
	u_ecc_vli_mmod(result, product, mod, num_words);
}

#endif /* UECC_ENABLE_VLI_API */

U_ECC_VLI_API void u_ecc_vli_mod_square_fast(u_ecc_word_t *result,
					  const u_ecc_word_t *left,
					  u_ecc_curve curve)
{
	u_ecc_word_t product[2 * U_ECC_MAX_WORDS];

	u_ecc_vli_square(product, left, curve->num_words);
#if (U_ECC_OPTIMIZATION_LEVEL > 0)
	curve->mmod_fast(result, product);
#else
	u_ecc_vli_mmod(result, product, curve->p, curve->num_words);
#endif
} /* u_ecc_vli_mod_square_fast */

#else /* U_ECC_SQUARE_FUNC */

#if UECC_ENABLE_VLI_API
U_ECC_VLI_API void u_ecc_vli_mod_square(u_ecc_word_t *result,
				     const u_ecc_word_t *left,
				     const u_ecc_word_t *mod,
				     wordcount_t num_words)
{
	u_ecc_vli_mod_mult(result, left, left, mod, num_words);
}

#endif /* UECC_ENABLE_VLI_API */

U_ECC_VLI_API void u_ecc_vli_mod_square_fast(u_ecc_word_t *result,
					  const u_ecc_word_t *left,
					  u_ecc_curve curve)
{
	u_ecc_vli_mod_mult_fast(result, left, left, curve);
}

#endif /* U_ECC_SQUARE_FUNC */

#define EVEN(vli) (!(vli[0] & 1))
static void vli_mod_inv_update(u_ecc_word_t *uv,
			      const u_ecc_word_t *mod,
			      wordcount_t num_words)
{
	u_ecc_word_t carry = 0;

	if (!EVEN(uv))
		carry = u_ecc_vli_add(uv, uv, mod, num_words);

	u_ecc_vli_rshift1(uv, num_words);
	if (carry)
		uv[num_words - 1] |= HIGH_BIT_SET;
} /* vli_mod_inv_update */

/* Computes result = (1 / input) % mod. All VLIs are the same size.
 * See "From Euclid's GCD to Montgomery Multiplication to the Great Divide" */
U_ECC_VLI_API void u_ecc_vli_mod_inv(u_ecc_word_t *result,
				  const u_ecc_word_t *input,
				  const u_ecc_word_t *mod,
				  wordcount_t num_words)
{
	u_ecc_word_t a[U_ECC_MAX_WORDS], b[U_ECC_MAX_WORDS], u[U_ECC_MAX_WORDS], v[U_ECC_MAX_WORDS];
	cmpresult_t cmpResult;

	if (u_ecc_vli_is_zero(input, num_words)) {
		u_ecc_vli_clear(result, num_words);
		return;
	}

	u_ecc_vli_set(a, input, num_words);
	u_ecc_vli_set(b, mod, num_words);
	u_ecc_vli_clear(u, num_words);
	u[0] = 1;
	u_ecc_vli_clear(v, num_words);
	cmpResult = u_ecc_vli_cmp_unsafe(a, b, num_words);
	while (cmpResult != 0) {
		if (EVEN(a)) {
			u_ecc_vli_rshift1(a, num_words);
			vli_mod_inv_update(u, mod, num_words);
		}
		else if (EVEN(b)) {
			u_ecc_vli_rshift1(b, num_words);
			vli_mod_inv_update(v, mod, num_words);
		}
		else if (cmpResult > 0) {
			u_ecc_vli_sub(a, a, b, num_words);
			u_ecc_vli_rshift1(a, num_words);
			if (u_ecc_vli_cmp_unsafe(u, v, num_words) < 0) {
				u_ecc_vli_add(u, u, mod, num_words);
			}
			u_ecc_vli_sub(u, u, v, num_words);
			vli_mod_inv_update(u, mod, num_words);
		}
		else {
			u_ecc_vli_sub(b, b, a, num_words);
			u_ecc_vli_rshift1(b, num_words);
			if (u_ecc_vli_cmp_unsafe(v, u, num_words) < 0) {
				u_ecc_vli_add(v, v, mod, num_words);
			}
			u_ecc_vli_sub(v, v, u, num_words);
			vli_mod_inv_update(v, mod, num_words);
		}
		cmpResult = u_ecc_vli_cmp_unsafe(a, b, num_words);
	}
	u_ecc_vli_set(result, u, num_words);
} /* u_ecc_vli_mod_inv */

/* ------ Point operations ------ */

#include "curve_specific.inc"

/* Returns 1 if 'point' is the point at infinity, 0 otherwise. */
#define ecc_point_is_zero(point, curve) u_ecc_vli_is_zero((point), (curve)->num_words * 2)

/* Point multiplication algorithm using Montgomery's ladder with co-Z coordinates.
 * From http://eprint.iacr.org/2011/338.pdf
 */

/* Modify (x1, y1) => (x1 * z^2, y1 * z^3) */
static void apply_z(u_ecc_word_t *X1,
		    u_ecc_word_t *Y1,
		    const u_ecc_word_t * const Z,
		    u_ecc_curve curve)
{
	u_ecc_word_t t1[U_ECC_MAX_WORDS];

	u_ecc_vli_mod_square_fast(t1, Z, curve);	/* z^2 */
	u_ecc_vli_mod_mult_fast(X1, X1, t1, curve); /* x1 * z^2 */
	u_ecc_vli_mod_mult_fast(t1, t1, Z, curve);  /* z^3 */
	u_ecc_vli_mod_mult_fast(Y1, Y1, t1, curve); /* y1 * z^3 */
}

/* P = (x1, y1) => 2P, (x2, y2) => P' */
static void XYcZ_initial_double(u_ecc_word_t *X1,
				u_ecc_word_t *Y1,
				u_ecc_word_t *X2,
				u_ecc_word_t *Y2,
				const u_ecc_word_t * const initial_Z,
				u_ecc_curve curve)
{
	u_ecc_word_t z[U_ECC_MAX_WORDS];
	wordcount_t num_words = curve->num_words;

	if (initial_Z) {
		u_ecc_vli_set(z, initial_Z, num_words);
	} else {
		u_ecc_vli_clear(z, num_words);
		z[0] = 1;
	}

	u_ecc_vli_set(X2, X1, num_words);
	u_ecc_vli_set(Y2, Y1, num_words);

	apply_z(X1, Y1, z, curve);
	curve->double_jacobian(X1, Y1, z, curve);
	apply_z(X2, Y2, z, curve);
} /* XYcZ_initial_double */

/* Input P = (x1, y1, Z), Q = (x2, y2, Z)
 * Output P' = (x1', y1', Z3), P + Q = (x3, y3, Z3)
 * or P => P', Q => P + Q
 */
static void XYcZ_add(u_ecc_word_t *X1,
		     u_ecc_word_t *Y1,
		     u_ecc_word_t *X2,
		     u_ecc_word_t *Y2,
		     u_ecc_curve curve)
{
	/* t1 = X1, t2 = Y1, t3 = X2, t4 = Y2 */
	u_ecc_word_t t5[U_ECC_MAX_WORDS];
	wordcount_t num_words = curve->num_words;

	u_ecc_vli_mod_sub(t5, X2, X1, curve->p, num_words);	/* t5 = x2 - x1 */
	u_ecc_vli_mod_square_fast(t5, t5, curve);			/* t5 = (x2 - x1)^2 = A */
	u_ecc_vli_mod_mult_fast(X1, X1, t5, curve);		/* t1 = x1*A = B */
	u_ecc_vli_mod_mult_fast(X2, X2, t5, curve);		/* t3 = x2*A = C */
	u_ecc_vli_mod_sub(Y2, Y2, Y1, curve->p, num_words);	/* t4 = y2 - y1 */
	u_ecc_vli_mod_square_fast(t5, Y2, curve);			/* t5 = (y2 - y1)^2 = D */

	u_ecc_vli_mod_sub(t5, t5, X1, curve->p, num_words); /* t5 = D - B */
	u_ecc_vli_mod_sub(t5, t5, X2, curve->p, num_words); /* t5 = D - B - C = x3 */
	u_ecc_vli_mod_sub(X2, X2, X1, curve->p, num_words); /* t3 = C - B */
	u_ecc_vli_mod_mult_fast(Y1, Y1, X2, curve);	/* t2 = y1*(C - B) */
	u_ecc_vli_mod_sub(X2, X1, t5, curve->p, num_words); /* t3 = B - x3 */
	u_ecc_vli_mod_mult_fast(Y2, Y2, X2, curve);	/* t4 = (y2 - y1)*(B - x3) */
	u_ecc_vli_mod_sub(Y2, Y2, Y1, curve->p, num_words); /* t4 = y3 */

	u_ecc_vli_set(X2, t5, num_words);
} /* XYcZ_add */

/* Input P = (x1, y1, Z), Q = (x2, y2, Z)
 * Output P + Q = (x3, y3, Z3), P - Q = (x3', y3', Z3)
 * or P => P - Q, Q => P + Q
 */
static void XYcZ_addC(u_ecc_word_t *X1,
		      u_ecc_word_t *Y1,
		      u_ecc_word_t *X2,
		      u_ecc_word_t *Y2,
		      u_ecc_curve curve)
{
	/* t1 = X1, t2 = Y1, t3 = X2, t4 = Y2 */
	u_ecc_word_t t5[U_ECC_MAX_WORDS];
	u_ecc_word_t t6[U_ECC_MAX_WORDS];
	u_ecc_word_t t7[U_ECC_MAX_WORDS];
	wordcount_t num_words = curve->num_words;

	u_ecc_vli_mod_sub(t5, X2, X1, curve->p, num_words); /* t5 = x2 - x1 */
	u_ecc_vli_mod_square_fast(t5, t5, curve);		  /* t5 = (x2 - x1)^2 = A */
	u_ecc_vli_mod_mult_fast(X1, X1, t5, curve);	  /* t1 = x1*A = B */
	u_ecc_vli_mod_mult_fast(X2, X2, t5, curve);	  /* t3 = x2*A = C */
	u_ecc_vli_mod_add(t5, Y2, Y1, curve->p, num_words); /* t5 = y2 + y1 */
	u_ecc_vli_mod_sub(Y2, Y2, Y1, curve->p, num_words); /* t4 = y2 - y1 */

	u_ecc_vli_mod_sub(t6, X2, X1, curve->p, num_words); /* t6 = C - B */
	u_ecc_vli_mod_mult_fast(Y1, Y1, t6, curve);	/* t2 = y1 * (C - B) = E */
	u_ecc_vli_mod_add(t6, X1, X2, curve->p, num_words); /* t6 = B + C */
	u_ecc_vli_mod_square_fast(X2, Y2, curve);		  /* t3 = (y2 - y1)^2 = D */
	u_ecc_vli_mod_sub(X2, X2, t6, curve->p, num_words); /* t3 = D - (B + C) = x3 */

	u_ecc_vli_mod_sub(t7, X1, X2, curve->p, num_words); /* t7 = B - x3 */
	u_ecc_vli_mod_mult_fast(Y2, Y2, t7, curve);	/* t4 = (y2 - y1)*(B - x3) */
	u_ecc_vli_mod_sub(Y2, Y2, Y1, curve->p, num_words); /* t4 = (y2 - y1)*(B - x3) - E = y3 */

	u_ecc_vli_mod_square_fast(t7, t5, curve);		  /* t7 = (y2 + y1)^2 = F */
	u_ecc_vli_mod_sub(t7, t7, t6, curve->p, num_words); /* t7 = F - (B + C) = x3' */
	u_ecc_vli_mod_sub(t6, t7, X1, curve->p, num_words); /* t6 = x3' - B */
	u_ecc_vli_mod_mult_fast(t6, t6, t5, curve);	/* t6 = (y2+y1)*(x3' - B) */
	u_ecc_vli_mod_sub(Y1, t6, Y1, curve->p, num_words); /* t2 = (y2+y1)*(x3' - B) - E = y3' */

	u_ecc_vli_set(X1, t7, num_words);
} /* XYcZ_addC */

/* result may overlap point. */
static void ecc_point_mult(u_ecc_word_t *result,
			  const u_ecc_word_t *point,
			  const u_ecc_word_t *scalar,
			  const u_ecc_word_t *initial_Z,
			  bitcount_t num_bits,
			  u_ecc_curve curve)
{
	/* R0 and R1 */
	u_ecc_word_t Rx[2][U_ECC_MAX_WORDS];
	u_ecc_word_t Ry[2][U_ECC_MAX_WORDS];
	u_ecc_word_t z[U_ECC_MAX_WORDS];
	bitcount_t i;
	u_ecc_word_t nb;
	wordcount_t num_words = curve->num_words;

	u_ecc_vli_set(Rx[1], point, num_words);
	u_ecc_vli_set(Ry[1], point + num_words, num_words);

	XYcZ_initial_double(Rx[1], Ry[1], Rx[0], Ry[0], initial_Z, curve);

	for (i = num_bits - 2; i > 0; --i) {
		nb = !u_ecc_vli_test_bit(scalar, i);
		XYcZ_addC(Rx[1 - nb], Ry[1 - nb], Rx[nb], Ry[nb], curve);
		XYcZ_add(Rx[nb], Ry[nb], Rx[1 - nb], Ry[1 - nb], curve);
	}

	nb = !u_ecc_vli_test_bit(scalar, 0);
	XYcZ_addC(Rx[1 - nb], Ry[1 - nb], Rx[nb], Ry[nb], curve);

	/* Find final 1/Z value. */
	u_ecc_vli_mod_sub(z, Rx[1], Rx[0], curve->p, num_words); /* X1 - X0 */
	u_ecc_vli_mod_mult_fast(z, z, Ry[1 - nb], curve);		/* Yb * (X1 - X0) */
	u_ecc_vli_mod_mult_fast(z, z, point, curve);		/* xP * Yb * (X1 - X0) */
	u_ecc_vli_mod_inv(z, z, curve->p, num_words);		/* 1 / (xP * Yb * (X1 - X0)) */
	/* yP / (xP * Yb * (X1 - X0)) */
	u_ecc_vli_mod_mult_fast(z, z, point + num_words, curve);
	u_ecc_vli_mod_mult_fast(z, z, Rx[1 - nb], curve); /* Xb * yP / (xP * Yb * (X1 - X0)) */
	/* End 1/Z calculation */

	XYcZ_add(Rx[nb], Ry[nb], Rx[1 - nb], Ry[1 - nb], curve);
	apply_z(Rx[0], Ry[0], z, curve);

	u_ecc_vli_set(result, Rx[0], num_words);
	u_ecc_vli_set(result + num_words, Ry[0], num_words);
} /* ecc_point_mult */

static u_ecc_word_t regularize_k(const u_ecc_word_t * const k,
				u_ecc_word_t *k0,
				u_ecc_word_t *k1,
				u_ecc_curve curve)
{
	wordcount_t num_n_words = BITS_TO_WORDS(curve->num_n_bits);
	bitcount_t num_n_bits = curve->num_n_bits;
	u_ecc_word_t carry = u_ecc_vli_add(k0, k, curve->n, num_n_words)
						|| (num_n_bits < ((bitcount_t)num_n_words * U_ECC_WORD_SIZE * 8)
							&& u_ecc_vli_test_bit(k0, num_n_bits));

	u_ecc_vli_add(k1, k0, curve->n, num_n_words);
	return carry;
} /* regularize_k */

/* Generates a random integer in the range 0 < random < top.
 * Both random and top have num_words words. */
U_ECC_VLI_API int u_ecc_generate_random_int(u_ecc_word_t *random,
					  const u_ecc_word_t *top,
					  wordcount_t num_words)
{
	u_ecc_word_t mask = (u_ecc_word_t)-1;
	u_ecc_word_t tries;
	bitcount_t num_bits = u_ecc_vli_num_bits(top, num_words);

	if (!g_rng_function)
		return 0;

	for (tries = 0; tries < U_ECC_RNG_MAX_TRIES; ++tries) {
		if (!g_rng_function((uint8_t *)random, num_words * U_ECC_WORD_SIZE))
			return 0;

		random[num_words - 1] &= mask >> ((bitcount_t)(num_words * U_ECC_WORD_SIZE * 8 - num_bits));
		if (!u_ecc_vli_is_zero(random, num_words)
			&& u_ecc_vli_cmp(top, random, num_words) == 1)
			return 1;
	}
	return 0;
} /* u_ecc_generate_random_int */

static u_ecc_word_t ecc_point_compute_public_key(u_ecc_word_t *result,
					       u_ecc_word_t *private_key,
					       u_ecc_curve curve)
{
	u_ecc_word_t tmp1[U_ECC_MAX_WORDS];
	u_ecc_word_t tmp2[U_ECC_MAX_WORDS];
	u_ecc_word_t *p2[2] = { tmp1, tmp2 };
	u_ecc_word_t *initial_Z = 0;
	u_ecc_word_t carry;

	/* Regularize the bitcount for the private key so that attackers cannot use a side channel
	 * attack to learn the number of leading zeros. */
	carry = regularize_k(private_key, tmp1, tmp2, curve);

	/* If an RNG function was specified, try to get a random initial Z value to improve
	 * protection against side-channel attacks. */
	if (g_rng_function) {
		if (!u_ecc_generate_random_int(p2[carry], curve->p, curve->num_words))
			return 0;

		initial_Z = p2[carry];
	}
	ecc_point_mult(result, curve->G, p2[!carry], initial_Z, curve->num_n_bits + 1, curve);

	if (ecc_point_is_zero(result, curve))
		return 0;

	return 1;
} /* ecc_point_compute_public_key */

#if U_ECC_WORD_SIZE == 1

U_ECC_VLI_API void u_ecc_vli_native_to_bytes(uint8_t *bytes,
					 int num_bytes,
					 const uint8_t *native)
{
	wordcount_t i;

	for (i = 0; i < num_bytes; ++i)
		bytes[i] = native[(num_bytes - 1) - i];
}

U_ECC_VLI_API void u_ecc_vli_bytes_to_native(uint8_t *native,
					 const uint8_t *bytes,
					 int num_bytes)
{
	u_ecc_vli_native_to_bytes(native, num_bytes, bytes);
}

#else

U_ECC_VLI_API  __attribute__((used)) void u_ecc_vli_native_to_bytes(uint8_t *bytes,
								int num_bytes,
								const u_ecc_word_t *native)
{
	int i;

	for (i = 0; i < num_bytes; ++i) {
		unsigned b = num_bytes - 1 - i;
		bytes[i] = native[b / U_ECC_WORD_SIZE] >> (8 * (b % U_ECC_WORD_SIZE));
	}
}

U_ECC_VLI_API  __attribute__((used)) void u_ecc_vli_bytes_to_native(u_ecc_word_t *native,
								const uint8_t *bytes,
								int num_bytes)
{
	int i;

	u_ecc_vli_clear(native, (num_bytes + (U_ECC_WORD_SIZE - 1)) / U_ECC_WORD_SIZE);
	for (i = 0; i < num_bytes; ++i) {
		unsigned b = num_bytes - 1 - i;
		native[b / U_ECC_WORD_SIZE] |=
			(u_ecc_word_t)bytes[i] << (8 * (b % U_ECC_WORD_SIZE));
	}
} /* u_ecc_vli_bytes_to_native */

#endif /* U_ECC_WORD_SIZE */

int u_ecc_make_key(uint8_t *public_key,
		  uint8_t *private_key,
		  u_ecc_curve curve)
{
#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN
	u_ecc_word_t *_private = (u_ecc_word_t *)private_key;
	u_ecc_word_t *_public = (u_ecc_word_t *)public_key;
#else
	u_ecc_word_t _private[U_ECC_MAX_WORDS];
	u_ecc_word_t _public[U_ECC_MAX_WORDS * 2];
#endif
	u_ecc_word_t tries;

	for (tries = 0; tries < U_ECC_RNG_MAX_TRIES; ++tries) {
		if (!u_ecc_generate_random_int(_private, curve->n, BITS_TO_WORDS(curve->num_n_bits)))
			return 0;

		if (ecc_point_compute_public_key(_public, _private, curve)) {
#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN == 0
			u_ecc_vli_native_to_bytes(private_key, BITS_TO_BYTES(curve->num_n_bits), _private);
			u_ecc_vli_native_to_bytes(public_key, curve->num_bytes, _public);
			u_ecc_vli_native_to_bytes(
				public_key + curve->num_bytes, curve->num_bytes, _public + curve->num_words);
#endif
			return 1;
		}
	}
	return 0;
} /* u_ecc_make_key */

int u_ecc_shared_secret(const uint8_t *public_key,
		       const uint8_t *private_key,
		       uint8_t *secret,
		       u_ecc_curve curve)
{
	u_ecc_word_t _public[U_ECC_MAX_WORDS * 2];
	u_ecc_word_t _private[U_ECC_MAX_WORDS];

	u_ecc_word_t tmp[U_ECC_MAX_WORDS];
	u_ecc_word_t *p2[2] = { _private, tmp };
	u_ecc_word_t *initial_Z = 0;
	u_ecc_word_t carry;
	wordcount_t num_words = curve->num_words;
	wordcount_t num_bytes = curve->num_bytes;

#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN
	uecc_bcopy((uint8_t *)_private, private_key, num_bytes);
	uecc_bcopy((uint8_t *)_public, public_key, num_bytes * 2);
#else
	u_ecc_vli_bytes_to_native(_private, private_key, BITS_TO_BYTES(curve->num_n_bits));
	u_ecc_vli_bytes_to_native(_public, public_key, num_bytes);
	u_ecc_vli_bytes_to_native(_public + num_words, public_key + num_bytes, num_bytes);
#endif

	/* Regularize the bitcount for the private key so that attackers cannot use a side channel
	 * attack to learn the number of leading zeros. */
	carry = regularize_k(_private, _private, tmp, curve);

	/* If an RNG function was specified, try to get a random initial Z value to improve
	 * protection against side-channel attacks. */
	if (g_rng_function) {
		if (!u_ecc_generate_random_int(p2[carry], curve->p, num_words))
			return 0;

		initial_Z = p2[carry];
	}

	ecc_point_mult(_public, _public, p2[!carry], initial_Z, curve->num_n_bits + 1, curve);
#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN
	uecc_bcopy((uint8_t *)secret, (uint8_t *)_public, num_bytes);
#else
	u_ecc_vli_native_to_bytes(secret, num_bytes, _public);
#endif
	return !ecc_point_is_zero(_public, curve);
} /* u_ecc_shared_secret */

#if U_ECC_SUPPORT_COMPRESSED_POINT
void u_ecc_compress(const uint8_t *public_key, uint8_t *compressed, u_ecc_curve curve)
{
	wordcount_t i;

	for (i = 0; i < curve->num_bytes; ++i)
		compressed[i + 1] = public_key[i];

#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN
	compressed[0] = 2 + (public_key[curve->num_bytes] & 0x01);
#else
	compressed[0] = 2 + (public_key[curve->num_bytes * 2 - 1] & 0x01);
#endif
} /* u_ecc_compress */

void u_ecc_decompress(const uint8_t *compressed, uint8_t *public_key, u_ecc_curve curve)
{
#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN
	u_ecc_word_t *point = (u_ecc_word_t *)public_key;
#else
	u_ecc_word_t point[U_ECC_MAX_WORDS * 2];
#endif
	u_ecc_word_t *y = point + curve->num_words;
#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN
	uecc_bcopy(public_key, compressed + 1, curve->num_bytes);
#else
	u_ecc_vli_bytes_to_native(point, compressed + 1, curve->num_bytes);
#endif
	curve->x_side(y, point, curve);
	curve->mod_sqrt(y, curve);

	if ((y[0] & 0x01) != (compressed[0] & 0x01))
		u_ecc_vli_sub(y, curve->p, y, curve->num_words);

#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN == 0
	u_ecc_vli_native_to_bytes(public_key, curve->num_bytes, point);
	u_ecc_vli_native_to_bytes(public_key + curve->num_bytes, curve->num_bytes, y);
#endif
} /* u_ecc_decompress */

#endif /* U_ECC_SUPPORT_COMPRESSED_POINT */

U_ECC_VLI_API int u_ecc_valid_point(const u_ecc_word_t *point, u_ecc_curve curve)
{
	u_ecc_word_t tmp1[U_ECC_MAX_WORDS];
	u_ecc_word_t tmp2[U_ECC_MAX_WORDS];
	wordcount_t num_words = curve->num_words;

	/* The point at infinity is invalid. */
	if (ecc_point_is_zero(point, curve))
		return 0;

	/* x and y must be smaller than p. */
	if (u_ecc_vli_cmp_unsafe(curve->p, point, num_words) != 1
		|| u_ecc_vli_cmp_unsafe(curve->p, point + num_words, num_words) != 1)
		return 0;

	u_ecc_vli_mod_square_fast(tmp1, point + num_words, curve);
	curve->x_side(tmp2, point, curve); /* tmp2 = x^3 + ax + b */

	/* Make sure that y^2 == x^3 + ax + b */
	return (int)(u_ecc_vli_equal(tmp1, tmp2, num_words));
} /* u_ecc_valid_point */

int u_ecc_valid_public_key(const uint8_t *public_key, u_ecc_curve curve)
{
#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN
	u_ecc_word_t *_public = (u_ecc_word_t *)public_key;
#else
	u_ecc_word_t _public[U_ECC_MAX_WORDS * 2];
#endif

#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN == 0
	u_ecc_vli_bytes_to_native(_public, public_key, curve->num_bytes);
	u_ecc_vli_bytes_to_native(
		_public + curve->num_words, public_key + curve->num_bytes, curve->num_bytes);
#endif
	return u_ecc_valid_point(_public, curve);
} /* u_ecc_valid_public_key */

int u_ecc_compute_public_key(const uint8_t *private_key, uint8_t *public_key, u_ecc_curve curve)
{
#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN
	u_ecc_word_t *_private = (u_ecc_word_t *)private_key;
	u_ecc_word_t *_public = (u_ecc_word_t *)public_key;
#else
	u_ecc_word_t _private[U_ECC_MAX_WORDS];
	u_ecc_word_t _public[U_ECC_MAX_WORDS * 2];
#endif

#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN == 0
	u_ecc_vli_bytes_to_native(_private, private_key, BITS_TO_BYTES(curve->num_n_bits));
#endif

	/* Make sure the private key is in the range [1, n-1]. */
	if (u_ecc_vli_is_zero(_private, BITS_TO_WORDS(curve->num_n_bits)))
		return 0;

	if (u_ecc_vli_cmp(curve->n, _private, BITS_TO_WORDS(curve->num_n_bits)) != 1)
		return 0;

	/* Compute public key. */
	if (!ecc_point_compute_public_key(_public, _private, curve))
		return 0;

#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN == 0
	u_ecc_vli_native_to_bytes(public_key, curve->num_bytes, _public);
	u_ecc_vli_native_to_bytes(
		public_key + curve->num_bytes, curve->num_bytes, _public + curve->num_words);
#endif
	return 1;
} /* u_ecc_compute_public_key */

/* -------- ECDSA code -------- */

static void bits2int(u_ecc_word_t *native,
		     const uint8_t *bits,
		     unsigned bits_size,
		     u_ecc_curve curve)
{
	unsigned num_n_bytes = BITS_TO_BYTES(curve->num_n_bits);
	unsigned num_n_words = BITS_TO_WORDS(curve->num_n_bits);
	int shift;
	u_ecc_word_t carry;
	u_ecc_word_t *ptr;

	if (bits_size > num_n_bytes)
		bits_size = num_n_bytes;

	u_ecc_vli_clear(native, num_n_words);
#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN
	uecc_bcopy((uint8_t *)native, bits, bits_size);
#else
	u_ecc_vli_bytes_to_native(native, bits, bits_size);
#endif
	if (bits_size * 8 <= (unsigned)curve->num_n_bits)
		return;

	shift = bits_size * 8 - curve->num_n_bits;
	carry = 0;
	ptr = native + num_n_words;
	while (ptr-- > native) {
		u_ecc_word_t temp = *ptr;
		*ptr = (temp >> shift) | carry;
		carry = temp << (UECC_WORD_BITS - shift);
	}

	/* Reduce mod curve_n */
	if (u_ecc_vli_cmp_unsafe(curve->n, native, num_n_words) != 1)
		u_ecc_vli_sub(native, native, curve->n, num_n_words);
} /* bits2int */

static int u_ecc_sign_with_k_internal(const uint8_t *private_key,
				     const uint8_t *message_hash,
				     unsigned hash_size,
				     u_ecc_word_t *k,
				     uint8_t *signature,
				     u_ecc_curve curve)
{
	u_ecc_word_t tmp[U_ECC_MAX_WORDS];
	u_ecc_word_t s[U_ECC_MAX_WORDS];
	u_ecc_word_t *k2[2] = {tmp, s};
	u_ecc_word_t *initial_Z = 0;
#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN
	u_ecc_word_t *p = (u_ecc_word_t *)signature;
#else
	u_ecc_word_t p[U_ECC_MAX_WORDS * 2];
#endif
	u_ecc_word_t carry;
	wordcount_t num_words = curve->num_words;
	wordcount_t num_n_words = BITS_TO_WORDS(curve->num_n_bits);
	bitcount_t num_n_bits = curve->num_n_bits;

	/* Make sure 0 < k < curve_n */
	if (u_ecc_vli_is_zero(k, num_words) || u_ecc_vli_cmp(curve->n, k, num_n_words) != 1)
		return 0;

	carry = regularize_k(k, tmp, s, curve);
	/* If an RNG function was specified, try to get a random initial Z value to improve
	 * protection against side-channel attacks. */
	if (g_rng_function) {
		if (!u_ecc_generate_random_int(k2[carry], curve->p, num_words))
			return 0;
		initial_Z = k2[carry];
	}

	ecc_point_mult(p, curve->G, k2[!carry], initial_Z, num_n_bits + 1, curve);
	if (u_ecc_vli_is_zero(p, num_words))
		return 0;

	/* If an RNG function was specified, get a random number
	 * to prevent side channel analysis of k. */
	if (!g_rng_function) {
		u_ecc_vli_clear(tmp, num_n_words);
		tmp[0] = 1;
	} else if (!u_ecc_generate_random_int(tmp, curve->n, num_n_words)) {
		return 0;
	}

	/* Prevent side channel analysis of u_ecc_vli_mod_inv() to determine
	 * bits of k / the private key by premultiplying by a random number */
	u_ecc_vli_mod_mult(k, k, tmp, curve->n, num_n_words); /* k' = rand * k */
	u_ecc_vli_mod_inv(k, k, curve->n, num_n_words);	   /* k = 1 / k' */
	u_ecc_vli_mod_mult(k, k, tmp, curve->n, num_n_words); /* k = 1 / k */

#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN == 0
	u_ecc_vli_native_to_bytes(signature, curve->num_bytes, p); /* store r */
#endif

#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN
	uecc_bcopy((uint8_t *)tmp, private_key, BITS_TO_BYTES(curve->num_n_bits));
#else
	u_ecc_vli_bytes_to_native(tmp, private_key, BITS_TO_BYTES(curve->num_n_bits)); /* tmp = d */
#endif

	s[num_n_words - 1] = 0;
	u_ecc_vli_set(s, p, num_words);
	u_ecc_vli_mod_mult(s, tmp, s, curve->n, num_n_words); /* s = r*d */

	bits2int(tmp, message_hash, hash_size, curve);
	u_ecc_vli_mod_add(s, tmp, s, curve->n, num_n_words); /* s = e + r*d */
	u_ecc_vli_mod_mult(s, s, k, curve->n, num_n_words);  /* s = (e + r*d) / k */
	if (u_ecc_vli_num_bits(s, num_n_words) > (bitcount_t)curve->num_bytes * 8)
		return 0;

#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN
	uecc_bcopy((uint8_t *)signature + curve->num_bytes, (uint8_t *)s, curve->num_bytes);
#else
	u_ecc_vli_native_to_bytes(signature + curve->num_bytes, curve->num_bytes, s);
#endif
	return 1;
} /* u_ecc_sign_with_k_internal */

/* For testing - sign with an explicitly specified k value */
int u_ecc_sign_with_k(const uint8_t *private_key,
		     const uint8_t *message_hash,
		     unsigned hash_size,
		     const uint8_t *k,
		     uint8_t *signature,
		     u_ecc_curve curve)
{
	u_ecc_word_t k2[U_ECC_MAX_WORDS];
	bits2int(k2, k, BITS_TO_BYTES(curve->num_n_bits), curve);
	return u_ecc_sign_with_k_internal(private_key, message_hash, hash_size, k2, signature, curve);
}

#ifndef WITH_ROM
int u_ecc_sign(const uint8_t *private_key,
	      const uint8_t *message_hash,
	      unsigned hash_size,
	      uint8_t *signature,
	      u_ecc_curve curve)
{
	u_ecc_word_t k[U_ECC_MAX_WORDS];
	u_ecc_word_t tries;

	for (tries = 0; tries < U_ECC_RNG_MAX_TRIES; ++tries) {
		if (!u_ecc_generate_random_int(k, curve->n, BITS_TO_WORDS(curve->num_n_bits)))
			return 0;

		if (u_ecc_sign_with_k_internal(private_key, message_hash, hash_size, k, signature, curve))
			return 1;
	}
	return 0;
} /* u_ecc_sign */

#else
int u_ecc_sign(const uint8_t *private_key,
	      const uint8_t *message_hash,
	      unsigned hash_size,
	      uint8_t *signature,
	      u_ecc_curve curve)
{
	return rom_u_ecc_sign(private_key, message_hash, hash_size, signature, curve);
}

#endif

/* Compute an HMAC using K as a key (as in RFC 6979). Note that K is always
 * the same size as the hash result size. */
static void hmac_init(const struct u_ecc_hash_context *hash_context, const uint8_t *K)
{
	uint8_t *pad = hash_context->tmp + 2 * hash_context->result_size;
	unsigned i;

	for (i = 0; i < hash_context->result_size; ++i)
		pad[i] = K[i] ^ 0x36;

	for (; i < hash_context->block_size; ++i)
		pad[i] = 0x36;

	hash_context->init_hash(hash_context);
	hash_context->update_hash(hash_context, pad, hash_context->block_size);
} /* hmac_init */

static void hmac_update(const struct u_ecc_hash_context *hash_context,
			const uint8_t * message,
			unsigned message_size)
{
	hash_context->update_hash(hash_context, message, message_size);
}

static void hmac_finish(const struct u_ecc_hash_context *hash_context,
			const uint8_t *K,
			uint8_t *result)
{
	uint8_t *pad = hash_context->tmp + 2 * hash_context->result_size;
	unsigned i;

	for (i = 0; i < hash_context->result_size; ++i)
		pad[i] = K[i] ^ 0x5c;

	for (; i < hash_context->block_size; ++i)
		pad[i] = 0x5c;

	hash_context->finish_hash(hash_context, result);

	hash_context->init_hash(hash_context);
	hash_context->update_hash(hash_context, pad, hash_context->block_size);
	hash_context->update_hash(hash_context, result, hash_context->result_size);
	hash_context->finish_hash(hash_context, result);
} /* hmac_finish */

/* V = HMAC_K(V) */
static void update_v(const struct u_ecc_hash_context *hash_context, uint8_t *K, uint8_t *V)
{
	hmac_init(hash_context, K);
	hmac_update(hash_context, V, hash_context->result_size);
	hmac_finish(hash_context, K, V);
}

/* Deterministic signing, similar to RFC 6979. Differences are:
 * We just use H(m) directly rather than bits2octets(H(m))
 *	(it is not reduced modulo curve_n).
 * We generate a value for k (aka T) directly rather than converting endianness.
 *
 * Layout of hash_context->tmp: <K> | <V> | (1 byte overlapped 0x00 or 0x01) / <HMAC pad> */
int u_ecc_sign_deterministic(const uint8_t *private_key,
			    const uint8_t *message_hash,
			    unsigned hash_size,
			    const struct u_ecc_hash_context *hash_context,
			    uint8_t * signature,
			    u_ecc_curve curve)
{
	uint8_t *K = hash_context->tmp;
	uint8_t *V = K + hash_context->result_size;
	wordcount_t num_bytes = curve->num_bytes;
	wordcount_t num_n_words = BITS_TO_WORDS(curve->num_n_bits);
	bitcount_t num_n_bits = curve->num_n_bits;
	u_ecc_word_t tries;
	unsigned i;

	for (i = 0; i < hash_context->result_size; ++i) {
		V[i] = 0x01;
		K[i] = 0;
	}

	/* K = HMAC_K(V || 0x00 || int2octets(x) || h(m)) */
	hmac_init(hash_context, K);
	V[hash_context->result_size] = 0x00;
	hmac_update(hash_context, V, hash_context->result_size + 1);
	hmac_update(hash_context, private_key, num_bytes);
	hmac_update(hash_context, message_hash, hash_size);
	hmac_finish(hash_context, K, K);

	update_v(hash_context, K, V);

	/* K = HMAC_K(V || 0x01 || int2octets(x) || h(m)) */
	hmac_init(hash_context, K);
	V[hash_context->result_size] = 0x01;
	hmac_update(hash_context, V, hash_context->result_size + 1);
	hmac_update(hash_context, private_key, num_bytes);
	hmac_update(hash_context, message_hash, hash_size);
	hmac_finish(hash_context, K, K);

	update_v(hash_context, K, V);

	for (tries = 0; tries < U_ECC_RNG_MAX_TRIES; ++tries) {
		u_ecc_word_t T[U_ECC_MAX_WORDS];
		uint8_t *T_ptr = (uint8_t *)T;
		wordcount_t T_bytes = 0;
		for (;;) {
			update_v(hash_context, K, V);
			for (i = 0; i < hash_context->result_size; ++i) {
				T_ptr[T_bytes++] = V[i];
				if (T_bytes >= num_n_words * U_ECC_WORD_SIZE)
					goto filled;
			}
		}
filled:
		if ((bitcount_t)num_n_words * U_ECC_WORD_SIZE * 8 > num_n_bits) {
			u_ecc_word_t mask = (u_ecc_word_t)-1;
			T[num_n_words - 1] &=
				mask >> ((bitcount_t)(num_n_words * U_ECC_WORD_SIZE * 8 - num_n_bits));
		}

		if (u_ecc_sign_with_k_internal(private_key, message_hash, hash_size, T, signature, curve))
			return 1;

		/* K = HMAC_K(V || 0x00) */
		hmac_init(hash_context, K);
		V[hash_context->result_size] = 0x00;
		hmac_update(hash_context, V, hash_context->result_size + 1);
		hmac_finish(hash_context, K, K);

		update_v(hash_context, K, V);
	}
	return 0;
} /* u_ecc_sign_deterministic */

static bitcount_t smax(bitcount_t a, bitcount_t b)
{
	return a > b ? a : b;
}

int u_ecc_verify(const uint8_t *public_key,
		const uint8_t *message_hash,
		unsigned hash_size,
		const uint8_t *signature,
		u_ecc_curve curve)
{
	u_ecc_word_t u1[U_ECC_MAX_WORDS], u2[U_ECC_MAX_WORDS];
	u_ecc_word_t z[U_ECC_MAX_WORDS];
	u_ecc_word_t sum[U_ECC_MAX_WORDS * 2];
	u_ecc_word_t rx[U_ECC_MAX_WORDS];
	u_ecc_word_t ry[U_ECC_MAX_WORDS];
	u_ecc_word_t tx[U_ECC_MAX_WORDS];
	u_ecc_word_t ty[U_ECC_MAX_WORDS];
	u_ecc_word_t tz[U_ECC_MAX_WORDS];
	const u_ecc_word_t *points[4];
	const u_ecc_word_t *point;
	bitcount_t num_bits;
	bitcount_t i;

#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN
	u_ecc_word_t *_public = (u_ecc_word_t *)public_key;
#else
	u_ecc_word_t _public[U_ECC_MAX_WORDS * 2];
#endif
	u_ecc_word_t r[U_ECC_MAX_WORDS], s[U_ECC_MAX_WORDS];
	wordcount_t num_words = curve->num_words;
	wordcount_t num_n_words = BITS_TO_WORDS(curve->num_n_bits);

	rx[num_n_words - 1] = 0;
	r[num_n_words - 1] = 0;
	s[num_n_words - 1] = 0;

#if U_ECC_VLI_NATIVE_LITTLE_ENDIAN
	uecc_bcopy((uint8_t *)r, signature, curve->num_bytes);
	uecc_bcopy((uint8_t *)s, signature + curve->num_bytes, curve->num_bytes);
#else
	u_ecc_vli_bytes_to_native(_public, public_key, curve->num_bytes);
	u_ecc_vli_bytes_to_native(
		_public + num_words, public_key + curve->num_bytes, curve->num_bytes);
	u_ecc_vli_bytes_to_native(r, signature, curve->num_bytes);
	u_ecc_vli_bytes_to_native(s, signature + curve->num_bytes, curve->num_bytes);
#endif

	/* r, s must not be 0. */
	if (u_ecc_vli_is_zero(r, num_words) || u_ecc_vli_is_zero(s, num_words))
		return 0;

	/* r, s must be < n. */
	if (u_ecc_vli_cmp_unsafe(curve->n, r, num_n_words) != 1
		|| u_ecc_vli_cmp_unsafe(curve->n, s, num_n_words) != 1)
		return 0;

	/* Calculate u1 and u2. */
	u_ecc_vli_mod_inv(z, s, curve->n, num_n_words); /* z = 1/s */
	u1[num_n_words - 1] = 0;
	bits2int(u1, message_hash, hash_size, curve);
	u_ecc_vli_mod_mult(u1, u1, z, curve->n, num_n_words); /* u1 = e/s */
	u_ecc_vli_mod_mult(u2, r, z, curve->n, num_n_words); /* u2 = r/s */

	/* Calculate sum = G + Q. */
	u_ecc_vli_set(sum, _public, num_words);
	u_ecc_vli_set(sum + num_words, _public + num_words, num_words);
	u_ecc_vli_set(tx, curve->G, num_words);
	u_ecc_vli_set(ty, curve->G + num_words, num_words);
	u_ecc_vli_mod_sub(z, sum, tx, curve->p, num_words); /* z = x2 - x1 */
	XYcZ_add(tx, ty, sum, sum + num_words, curve);
	u_ecc_vli_mod_inv(z, z, curve->p, num_words); /* z = 1/z */
	apply_z(sum, sum + num_words, z, curve);

	/* Use Shamir's trick to calculate u1*G + u2*Q */
	points[0] = 0;
	points[1] = curve->G;
	points[2] = _public;
	points[3] = sum;
	num_bits = smax(u_ecc_vli_num_bits(u1, num_n_words),
					u_ecc_vli_num_bits(u2, num_n_words));

	point = points[(!!u_ecc_vli_test_bit(u1, num_bits - 1)) |
			((!!u_ecc_vli_test_bit(u2, num_bits - 1)) << 1)];
	u_ecc_vli_set(rx, point, num_words);
	u_ecc_vli_set(ry, point + num_words, num_words);
	u_ecc_vli_clear(z, num_words);
	z[0] = 1;

	for (i = num_bits - 2; i >= 0; --i) {
		u_ecc_word_t index;
		curve->double_jacobian(rx, ry, z, curve);

		index = (!!u_ecc_vli_test_bit(u1, i)) | ((!!u_ecc_vli_test_bit(u2, i)) << 1);
		point = points[index];
		if (point) {
			u_ecc_vli_set(tx, point, num_words);
			u_ecc_vli_set(ty, point + num_words, num_words);
			apply_z(tx, ty, z, curve);
			u_ecc_vli_mod_sub(tz, rx, tx, curve->p, num_words); /* Z = x2 - x1 */
			XYcZ_add(tx, ty, rx, ry, curve);
			u_ecc_vli_mod_mult_fast(z, z, tz, curve);
		}
	}

	u_ecc_vli_mod_inv(z, z, curve->p, num_words); /* Z = 1/Z */
	apply_z(rx, ry, z, curve);

	/* v = x1 (mod n) */
	if (u_ecc_vli_cmp_unsafe(curve->n, rx, num_n_words) != 1)
		u_ecc_vli_sub(rx, rx, curve->n, num_n_words);

	/* Accept only if v == r. */
	return (int)(u_ecc_vli_equal(rx, r, num_words));
} /* u_ecc_verify */

#if UECC_ENABLE_VLI_API

unsigned u_ecc_curve_num_words(u_ecc_curve curve)
{
	return curve->num_words;
}

unsigned u_ecc_curve_num_bytes(u_ecc_curve curve)
{
	return curve->num_bytes;
}

unsigned u_ecc_curve_num_bits(u_ecc_curve curve)
{
	return curve->num_bytes * 8;
}

unsigned u_ecc_curve_num_n_words(u_ecc_curve curve)
{
	return BITS_TO_WORDS(curve->num_n_bits);
}

unsigned u_ecc_curve_num_n_bytes(u_ecc_curve curve)
{
	return BITS_TO_BYTES(curve->num_n_bits);
}

unsigned u_ecc_curve_num_n_bits(u_ecc_curve curve)
{
	return curve->num_n_bits;
}

const u_ecc_word_t *u_ecc_curve_p(u_ecc_curve curve)
{
	return curve->p;
}

const u_ecc_word_t *u_ecc_curve_n(u_ecc_curve curve)
{
	return curve->n;
}

const u_ecc_word_t *u_ecc_curve_G(u_ecc_curve curve)
{
	return curve->G;
}

const u_ecc_word_t *u_ecc_curve_b(u_ecc_curve curve)
{
	return curve->b;
}

#if U_ECC_SUPPORT_COMPRESSED_POINT
void u_ecc_vli_mod_sqrt(u_ecc_word_t *a, u_ecc_curve curve)
{
	curve->mod_sqrt(a, curve);
}

#endif

void u_ecc_vli_mmod_fast(u_ecc_word_t *result, u_ecc_word_t *product, u_ecc_curve curve)
{
#if (U_ECC_OPTIMIZATION_LEVEL > 0)
	curve->mmod_fast(result, product);
#else
	u_ecc_vli_mmod(result, product, curve->p, curve->num_words);
#endif
}

void u_ecc_point_mult(u_ecc_word_t *result,
		     const u_ecc_word_t *point,
		     const u_ecc_word_t *scalar,
		     u_ecc_curve curve)
{
	u_ecc_word_t tmp1[U_ECC_MAX_WORDS];
	u_ecc_word_t tmp2[U_ECC_MAX_WORDS];
	u_ecc_word_t *p2[2] = { tmp1, tmp2 };
	u_ecc_word_t carry = regularize_k(scalar, tmp1, tmp2, curve);

	ecc_point_mult(result, point, p2[!carry], 0, curve->num_n_bits + 1, curve);
}

#endif /* UECC_ENABLE_VLI_API */
