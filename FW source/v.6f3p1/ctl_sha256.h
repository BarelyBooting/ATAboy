// ATAboy Control: SHA-256 in software (FIPS 180-4), for the request digest
// the replay table and status.presence key on (SPEC-v1 3.5) and for the
// digests in some results. Pure C. It does not use the RP2350's SHA-256
// engine: obs.c owns that engine and its DMA channels for the per-sector
// hashes, and a request is a few hundred bytes.
#ifndef CTL_SHA256_H
#define CTL_SHA256_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint32_t h[8];
    uint64_t bytes;
    uint8_t  block[64];
    uint32_t used;
} ctl_sha256_t;

void ctl_sha256_init(ctl_sha256_t *s);
void ctl_sha256_add(ctl_sha256_t *s, const void *p, size_t n);
void ctl_sha256_done(ctl_sha256_t *s, uint8_t out[32]);
void ctl_sha256(const void *p, size_t n, uint8_t out[32]);

#endif
