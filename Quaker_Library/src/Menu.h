#ifndef MENU_H
#define MENU_H

#include "MenuName.h"
#include <array> // Include array for cursor positions

constexpr size_t MAX_CURSOR_POSITIONS = 10; // Define the maximum number of possible cursor positions

struct Menu
{
    MenuName menuName;                                       // Attached Menu enum
    uint8_t maxPosition;                                     // Number of selectable positions
    uint8_t initalPosition;                                  // Initial cursor position
    uint8_t positionIndex;                                   // Current cursor position
    std::array<uint8_t, MAX_CURSOR_POSITIONS> positionArray; // Array of indices for cursor positions

    // Constructor
    Menu(MenuName menuName, uint8_t maxPosition, const std::array<uint8_t, MAX_CURSOR_POSITIONS> &positionArray)
        : menuName(menuName), maxPosition(maxPosition), initalPosition(0), positionIndex(0), positionArray(positionArray) {}
};

#endif
