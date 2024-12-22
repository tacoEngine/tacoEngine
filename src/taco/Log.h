// tacoEngine (c) Nikolas Wipper 2024

#ifndef LOG_H
#define LOG_H

namespace taco {

void TraceImpl(const char *inFMT, ...);
bool AssertFailedImpl(const char *inExpression, const char *inMessage, const char *inFile, unsigned int inLine);

}

#endif //LOG_H
