// Host-side replacements for the nrf_cc310 library, shared with the nRF52840 machine.
#ifndef HW_ARM_NRF52840_CC310_H
#define HW_ARM_NRF52840_CC310_H

void nrf52840_cc310_patch(uint8_t *image, uint32_t len);
bool nrf52840_cc310_call(uint32_t frame, uint32_t number, uint32_t exc_return, uint32_t *result);

#endif
