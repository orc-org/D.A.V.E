#pragma once
#include <raylib.h>

/*
    This header file stores all the global constants that we are using

    they are defined in one place so that they can be used accross files and also be easily found/edited
*/

namespace UIConfig {

    inline bool SHOW_DEBUG = true;

    // window dimensions (aspect ratio)
    constexpr int BASE_W = 1920 * 2; // assuming dual monitor setup
    constexpr int BASE_H = 1080;
    constexpr float HALF_BASE_WIDTH  = BASE_W * 0.5f;
    constexpr float HALF_BASE_HEIGHT = BASE_H * 0.5f;
}