#pragma once
#include "Shared.h"

/**
 * @brief Start the logging/diagnostics task.
 *
 * - Periodically (LOG_PERIOD_MS) prints snapshots of @ref gRC and @ref gAtt.
 * - Low priority to avoid impacting real-time control.
 */
void startTaskLog();
