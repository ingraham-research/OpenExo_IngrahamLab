#ifndef HEEL_FSR_CONFIG_H
#define HEEL_FSR_CONFIG_H

/**
 * @brief Whether a heel FSR is physically installed.
 *
 * On the Teensy: read once (cached) from [Sensors] heelFsrPresent in /config.ini; defaults
 * to false (no heel FSR) if the section/key/file is missing. Gates ALL heel FSR use —
 * calibration, refinement, ground-strike detection, and the passive analog read — so
 * switching between heel / no-heel hardware needs only an SD-card edit, no reflash.
 *
 * On the Nano (no SD card): always returns false.
 *
 * Declared for both MCUs so shared headers (e.g. uart_commands.h) compile on each. On the
 * Teensy, optionally pre-warm the cache by calling this once in setup() after the config is
 * parsed, to keep the SD read out of the control loop.
 */
bool heel_fsr_present();

#endif
