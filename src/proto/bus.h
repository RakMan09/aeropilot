/* Avionics bus transport: packs a telemetry frame into a stream of ARINC-429
 * words (two payload bytes per word, each word emitted little-endian on the
 * wire). This is the byte stream that flows over the simulated bus link.
 * Pure C11, no RTOS dependency. */
#ifndef AEROPILOT_BUS_H
#define AEROPILOT_BUS_H

#include <stddef.h>
#include <stdint.h>

/* Wire bytes produced per input frame byte (2 bytes -> one 4-byte word). */
#define BUS_WIRE_BYTES_PER_FRAME(frame_len) (((frame_len) / 2u) * 4u)

/* Encode frame (frame_len must be even) into ARINC-429 words written as
 * little-endian bytes in out. Returns the number of wire bytes written, or 0
 * if out_cap is insufficient or frame_len is odd. */
size_t bus_encode_frame(const uint8_t *frame, size_t frame_len,
                        uint8_t *out, size_t out_cap);

/* Decode a contiguous run of ARINC-429 wire words back into frame bytes.
 * in_len must be a multiple of 4. Writes recovered bytes to frame_out and
 * returns the number of bytes recovered; increments *parity_errors for each
 * word that fails parity/label validation (those words are skipped). */
size_t bus_decode_words(const uint8_t *in, size_t in_len,
                        uint8_t *frame_out, size_t frame_cap,
                        int *parity_errors);

#endif /* AEROPILOT_BUS_H */
