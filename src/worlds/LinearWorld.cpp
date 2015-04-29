#include "LinearWorld.h"

#include <iostream>

using namespace std;

bool LinearWorldState::lessThen(const State* other) const {
    const LinearWorldState *s = dynamic_cast<const LinearWorldState*>(other);

    if(s == NULL)
        cerr << " LinearWorldState: the states are not comparable" << endl;

    return pos < s->pos;
}

std::list<const Action*> LinearWorldState::availableActions() const {
    return actions;
}

LinearWorldState *LinearWorldState::clone() const {
    return new LinearWorldState(*this);
}

bool LinearWorldAction::lessThen(const Action *other) const {
    const LinearWorldAction *a = dynamic_cast<const LinearWorldAction*>(other);

    if(a == NULL)
        cerr << " LinearWorldAction: the actions are not comparable" << endl;

    return movement < a->movement;

}


LinearWorld::LinearWorld(unsigned int max_position) :
    max_pos(max_position),
    actions(),
    currentState(0,actions) {    //initial state
 
    actions.push_back(new LinearWorldAction(true));
    actions.push_back(new LinearWorldAction(false));
}

LinearWorldState *LinearWorld::getCurrentState() const {
    return new LinearWorldState(currentState);
}

void LinearWorld::applyAction(const Action *a) {
    const LinearWorldAction *act = dynamic_cast<const LinearWorldAction*>(a);

    if(act == NULL)
        cerr << "LinearWorld: can't apply an action from another domain" << endl;

    if((currentState.pos != 0 || act->getMovement() >= 0) && (currentState.pos != max_pos || act->getMovement() <= 0))
        currentState.pos += act->getMovement();
}

void LinearWorld::reset() {
    currentState.pos = 0;
}

LinearWorld::~LinearWorld() {
  list<const Action*>::iterator act = actions.begin();
  for(; act != actions.end(); ++act)
    delete *act;
}


