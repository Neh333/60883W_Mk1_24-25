#include "include.hpp"
#include <functional>

void winPointRed();
void winPointBlue();

void leftSideRed();
void leftSideBlue();

void rightSideRed();
void rightSideBlue();

void leftElimRed();
void leftElimBlue();

void rightElimRed();
void rightElimBlue();

void skills();

void nothing();
void tune();

extern std::function<void()> autos[AUTO_NUMBER];

