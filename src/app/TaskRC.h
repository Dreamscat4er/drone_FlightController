#pragma once
#include "Shared.h"

/**
 * @brief Start the RC input pipeline (PPM ISR + RC task).
 *
 * - Attaches the PPM GPIO interrupt to capture pulse widths.
 * - Creates/uses a FreeRTOS queue for ISR->task communication.
 * - Launches TaskRC that assembles channels into RCState and updates @ref gRC.
 *
 * Pins and timing come from Config.h. Thread safety via @ref gMutexRC.
 */
void startTaskRC();
