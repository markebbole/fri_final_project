#include "MazeProblem.h"

#include <iostream>

bool MazeProblem::solved() const {
    const MazeWorldState *s = world->getCurrentState();

    bool done = isFinal(s);
    delete s;

    return done;
}

bool MazeProblem::isFinal(const State *s) const {
    const MazeWorldState *s_linear = dynamic_cast<const MazeWorldState *>(s);

    if(s_linear == NULL)
        std::cerr << "MazeProblem: state not from MazeWorld" << std::endl;

    // TODO: fix this
    unsigned char grid[10][10] = MazeWorld::getGrid();

    return grid[s_linear->xpos][s_linear->ypos] == 'e';
}

double MazeProblem::r(const State* s, const Action *a, const State *s_prime) {
    if(isFinal(s_prime))
      return 100.;

    return 0.;
}