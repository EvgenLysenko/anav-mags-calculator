#ifndef __LOGGING_H__
#define __LOGGING_H__

#include "loguru/loguru.hpp"

#define logInfo(...) LOG_F(INFO, __VA_ARGS__)
#define logWarning(...) LOG_F(WARNING, __VA_ARGS__)
#define logError(...) LOG_F(ERROR, __VA_ARGS__)

#endif
