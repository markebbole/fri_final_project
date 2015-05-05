#include "MazeProblem.h"

#include <iostream>

bool MazeProblem::solved() const {
    const MazeWorldState *s = world->getCurrentState();
    //std::cout << "solved: s_clone: " << std::hex << (long)s << std::endl;
    bool done = isFinal(s);
    //std::cout << "deleting " << std::hex << (long)s << std::endl;
    delete s;

    return done;
}

bool MazeProblem::isFinal(const State *s) const {
    const MazeWorldState *s_linear = dynamic_cast<const MazeWorldState *>(s);
    //std::cout << "FINAL STATE" << std::endl;
    if(s_linear == NULL)
        std::cerr << "MazeProblem: state not from MazeWorld" << std::endl;

    // TODO: fix this
    return world->getEndPosX() == s_linear->ypos && world->getEndPosY() == s_linear->xpos;
}

double MazeProblem::r(const State* s, const Action *a, const State *s_prime) {
    if(isFinal(s_prime))
      return 100.;

    return -1.;
}