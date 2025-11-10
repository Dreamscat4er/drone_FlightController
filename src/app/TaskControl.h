#pragma once
#include "Shared.h"

/**
 * @brief Start the control loop task (cascaded PID + motor mixing).
 *
 * - Runs at CTRL_PERIOD_MS (200 Hz).
 * - Reads @ref gRC and @ref gAtt, computes angle→rate→motor outputs.
 * - Writes ESC commands via LEDC (PWM1_PIN..PWM4_PIN).
 * - Enforces failsafe (invalid RC or low throttle).
 */
void startTaskControl();
