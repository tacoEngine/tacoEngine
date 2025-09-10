// tacoEngine (c) Nikolas Wipper 2024-2025

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef LOG_H
#define LOG_H

namespace taco {

void TraceImpl(const char *inFMT, ...);
bool AssertFailedImpl(const char *inExpression, const char *inMessage, const char *inFile, unsigned int inLine);

}

#endif //LOG_H
