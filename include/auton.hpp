#include "include.hpp"
#include <functional>

void winPointRed();
void winPointBlue();

void ringSideRed();
void ringSideBlue();

void goalSideRed();
void goalSideBlue();

void ringElimRed();
void ringElimBlue();

void goalElimRed();
void goalElimBlue();

void skills();
void tune();

extern std::function<void()> autos[AUTO_NUMBER];

