#pragma once

#ifndef __SYNTHESIS__

#define nsprintf(fmt, ...) std::printf(fmt, ##__VA_ARGS__); std::fflush(stdout)

#else

template <typename... Args>
inline void nsprintf(const char*, Args&&...) {}

#endif