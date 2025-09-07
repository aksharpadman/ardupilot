#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_CUSTOMCONTROL LOG_CCPD_MSG

#define LOG_STRUCTURE_FROM_CUSTOM_CONTROL \
    {LOG_CCPD_MSG, sizeof(log_PID), \
      "CPDP", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS, true },
      