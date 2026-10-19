// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024-2026 Oplus. All rights reserved.
 */
#ifndef __SHA256_H__
#define __SHA256_H__
#include <linux/types.h>

#define SHA256_BLOCKLEN		64 /* size of message block buffer */
#define SHA256_DIGESTLEN	32 /* size of digest in uint8_t */
#define SHA256_DIGESTINT	8  /* size of digest in uint32_t */

struct sha256_ctx_t {
	uint64_t len;				/* processed message length */
	uint32_t h[SHA256_DIGESTINT];		/* hash state */
	uint8_t buf[SHA256_BLOCKLEN];		/* message block buffer */
};

void sha256_final(struct sha256_ctx_t *s, uint8_t *md);
void sha256_update(struct sha256_ctx_t *s, const uint8_t *m, uint32_t len);
void calculate_sha256(uint8_t *message, uint32_t len, uint8_t *hash);

#endif /* __SHA256_H__ */
