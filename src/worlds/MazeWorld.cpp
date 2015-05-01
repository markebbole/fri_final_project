#include "MazeWorld.h"

#include <iostream>

using namespace std;

#define GRID_SIZE 10
/*static unsigned char grid[GRID_SIZE][GRID_SIZE] = {
    {'x','x','x','s','x','x','x','x','e','x'},
    {'x','x','x','o','x','x','x','x','o','x'},
    {'x','x','x','o','x','x','x','x','o','x'},
    {'x','x','x','o','x','x','x','x','o','x'},
    {'x','x','x','o','x','x','x','x','o','x'},
    {'o','o','o','o','o','o','o','o','o','x'},
    {'x','x','x','o','x','x','x','x','o','x'},
    {'x','x','x','o','x','x','x','x','o','x'},
    {'x','x','x','o','x','x','x','x','o','x'},
    {'x','x','x','o','x','x','x','x','o','x'}
};*/

bool MazeWorldState::lessThen(const State* other) const {
    const MazeWorldState *s = dynamic_cast<const MazeWorldState*>(other);

    if(s == NULL)
        cerr << " MazeWorldState: the states are not comparable" << endl;

    if(ypos < s->ypos)
        return true;
    else if(ypos == s->ypos)
        return xpos < s->ypos;
    else
        return false;
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

// unsigned char [10][10] grid, int xSize, int ySize
MazeWorld::MazeWorld(unsigned char** grid) : grid(grid), actions(), currentState(0, 0, 'n', actions) {    //initial state


    for(unsigned int r = 0; r < GRID_SIZE; ++r) {
        for(unsigned int c = 0; c < GRID_SIZE; ++c) {
            if(grid[r][c] == 'e') {
                endPosX = c;
                endPosY = r;
            }
            else if(grid[r][c] == 's') {
                startPosX = c;
                startPosY = r;
                currentState.xpos = c;
                currentState.ypos = r;
                currentState.direction = 's';
            }
        }
    }

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
                if(currentState.ypos  > 0 && grid[currentState.xpos][currentState.ypos - 1] != 'x')
                    currentState.ypos -= 1;
            break;

            case 's':
                if(currentState.ypos < GRID_SIZE-1 && grid[currentState.xpos][currentState.ypos + 1] != 'x')
                    currentState.ypos += 1;
            break;
            case 'e':
                if(currentState.xpos < GRID_SIZE -1 && grid[currentState.xpos + 1][currentState.ypos] != 'x')
                    currentState.xpos += 1;
            break;
            case 'w':
                if(currentState.xpos  > 0 && grid[currentState.xpos - 1][currentState.ypos] != 'x')
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
}

unsigned char** MazeWorld::getGrid() const {
    return this->grid;
}

void MazeWorld::reset() {
    currentState.xpos = startPosX;
    currentState.ypos = startPosY;
    currentState.direction = 's';
}

MazeWorld::~MazeWorld() {
    list<const Action*>::iterator act = actions.begin();
    for(; act != actions.end(); ++act)
        delete *act;
}


