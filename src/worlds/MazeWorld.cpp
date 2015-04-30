#include "MazeWorld.h"

#include <iostream>

using namespace std;

bool MazeWorldState::lessThen(const State* other) const {
    const MazeWorldState *s = dynamic_cast<const MazeWorldState*>(other);

    if(s == NULL)
        cerr << " MazeWorldState: the states are not comparable" << endl;

    if(ypos < s->ypos)
        return TRUE;
    else if(ypos == s->ypos)
        return xpos < s->ypos;
    else
        return FALSE;
}

std::list<const Action*> MazeWorldState::availableActions() const {
    return actions;
}

MazeWorldState *MazeWorldState::clone() const {
    return new MazeWorldState(*this);
}

bool MazeWorldAction::lessThen(const Action *other) const {
    const MazeWorldAction *a = dynamic_cast<const MazeWorldAction*>(other);

    if(a == NULL)
        cerr << " MazeWorldAction: the actions are not comparable" << endl;

    return movement < a->movement;

}


MazeWorld::MazeWorld(unsigned char [][] grid, int xSize, int ySize) :
    grid(grid)
    endPosX(xSize),
    endPosY(ySize),
    actions(),
    currentState(0, 0, 's', actions) {    //initial state
 
    actions.push_back(new MazeWorldAction(0));
    actions.push_back(new MazeWorldAction(1));
    actions.push_back(new MazeWorldAction(2));
}

MazeWorldState *MazeWorld::getCurrentState() const {
    return new MazeWorldState(currentState);
}

void MazeWorld::applyAction(const Action *a) {
    const MazeWorldAction *act = dynamic_cast<const MazeWorldAction*>(a);

    if(act == NULL)
        cerr << "MazeWorld: can't apply an action from another domain" << endl;

    if(act->getMovement() == 0) {
        // Move forward if possible
        switch(currentState.direction) {
            case 'n': 
                if(currentState.ypos - 1 >= 0 && grid[currentState.xpos][currentState.ypos - 1] != 'x')
                    currentState.ypos -= 1;
            break;
            case 's':
                if(currentState.ypos + 1 < endPosY && grid[currentState.xpos][currentState.ypos + 1] != 'x')
                    currentState.ypos += 1;
            break;
            case 'e':
                if(currentState.xpos + 1 < endPosX && grid[currentState.xpos + 1][currentState.ypos] != 'x')
                    currentState.xpos += 1;
            break;
            case 'w':
                if(currentState.xpos - 1 >= 0 && grid[currentState.xpos - 1][currentState.ypos] != 'x')
                    currentState.xpos -= 1;
            break;
        }
    } else if(act->getMovement() == 1) {
        // rotate right
        switch(currentState.direction) {
            case 'n': currentState.direction = 'e'; break;
            case 's': currentState.direction = 'w'; break;
            case 'e': currentState.direction = 's'; break;
            case 'w': currentState.direction = 'n'; break;
        }
    } else if(act->getMovement() == 2) {
        // rotate left
        switch(currentState.direction) {
            case 'n': currentState.direction = 'w'; break;
            case 's': currentState.direction = 'e'; break;
            case 'e': currentState.direction = 'n'; break;
            case 'w': currentState.direction = 's'; break;
        }
    }

    if((currentState.pos != 0 || act->getMovement() >= 0) && (currentState.pos != max_pos || act->getMovement() <= 0))
        currentState.pos += act->getMovement();
}

void MazeWorld::reset() {
    currentState.xpos = 0;
    currentState.ypos = 0;
}

MazeWorld::~MazeWorld() {
    list<const Action*>::iterator act = actions.begin();
    for(; act != actions.end(); ++act)
        delete *act;
}


