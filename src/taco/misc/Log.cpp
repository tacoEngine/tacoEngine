// tacoEngine (c) Nikolas Wipper 2024-2025

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "Log.h"

#include <log/log.h>

#include <cstdarg>

void taco::TraceImpl(const char *inFMT, ...) {
    // Format the message
    va_list list;
    va_start(list, inFMT);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), inFMT, list);
    va_end(list);

    logging::Logger::Info(std::string("[jolt]: ") + buffer);
}

bool taco::AssertFailedImpl(const char *inExpression, const char *inMessage, const char *inFile, unsigned int inLine) {
    logging::Logger::Error(std::string("[jolt]: Assert failed: ") + inMessage, inFile, inLine, inExpression);

    return true;
}
