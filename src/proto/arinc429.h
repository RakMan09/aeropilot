/* Simulated ARINC-429 word codec used to model the avionics data bus.
 *
 * An ARINC-429 word is 32 bits:
 *   bits 0-7   Label (8 bits, bit-reversed relative to the octal notation)
 *   bits 8-9   SDI  (source/destination identifier)
 *   bits 10-28 Data (19 bits)
 *   bits 29-30 SSM  (sign/status matrix)
 *   bit  31    Parity (odd)
 *
 * The telemetry frame bytes ride the bus as a sequence of these words, two
 * payload bytes per word. Parity gives per-word error detection on top of the
 * frame CRC. Pure C11, no RTOS dependency. */
#ifndef AEROPILOT_ARINC429_H
#define AEROPILOT_ARINC429_H

#include <stddef.h>
#include <stdint.h>

/* Octal label 0311 marks a telemetry payload word on our bus. */
#define ARINC429_LABEL_TELEMETRY 0311u

/* Assemble a word from its logical fields and append odd parity. */
uint32_t arinc429_encode(uint8_t label, uint8_t sdi, uint32_t data19,
                         uint8_t ssm);

/* Return non-zero if the word carries valid odd parity. */
int arinc429_parity_ok(uint32_t word);

uint8_t  arinc429_label(uint32_t word); /* Logical (de-reversed) label. */
uint8_t  arinc429_sdi(uint32_t word);
uint32_t arinc429_data(uint32_t word);
uint8_t  arinc429_ssm(uint32_t word);

/* Encode a 2-byte telemetry payload into a labelled, parity-checked word. */
uint32_t arinc429_encode_payload(uint16_t two_bytes);

/* Decode a telemetry payload word. Returns 0 on success (parity + label OK)
 * and writes the two payload bytes; returns negative on error. */
int arinc429_decode_payload(uint32_t word, uint16_t *two_bytes);

#endif /* AEROPILOT_ARINC429_H */
