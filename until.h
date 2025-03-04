#pragma once

template <typename T>
inline T clamp(T val, T low, T high) {
    return max(min(val, high), low);
}