#include "MazeWorld.h"

#include <iostream>

using namespace std;

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

    if(ypos < s->ypos) {
        return true;
    }

    if(ypos == s->ypos && xpos < s->xpos) {
        return true;
    }

    if(ypos == s->ypos && xpos == s->xpos && direction < s->direction) {
        return true;
    }

    return false;
}

std::list<const Action*> MazeWorldState::availableActions() const {
    return actions;
}

MazeWorldState *MazeWorldState::clone() const {
    //std::cout << "CLONING: " << std::hex <<(long)this << std::endl;
    return new MazeWorldState(*this);
}

bool MazeWorldAction::lessThen(const Action *other) const {
    const MazeWorldAction *a = dynamic_cast<const MazeWorldAction*>(other);

    if(a == NULL)
        cerr << " MazeWorldAction: the actions are not comparable" << endl;

    return movement < a->movement;

}

// unsigned char [10][10] grid, int xSize, int ySize
MazeWorld::MazeWorld(unsigned char** grid, int size) : grid(grid), size(size), actions(), currentState(0, 0, 'n', actions) {    //initial state

    //takes in a 2d grid of chars, finds e and s
    for(unsigned int r = 0; r < size; ++r) {
        for(unsigned int c = 0; c < size; ++c) {
            //std::cout << grid[r][c] << std::endl;
            //endPosX and endPosY are the positions of the finish
            if(grid[r][c] == 'e') {
                endPosX = c;
                endPosY = r;
            }
            else if(grid[r][c] == 's') { //set start position, set initial current state
                startPosX = c;
                startPosY = r;
                currentState.xpos = c;
                currentState.ypos = r;
                currentState.direction = 's'; //arbitrary initial direction
            }
        }
    }

    actions.push_back(new MazeWorldAction(0));
    actions.push_back(new MazeWorldAction(1));
    actions.push_back(new MazeWorldAction(2));
    /*for(std::list<const Action*>::iterator p = actions.begin(); p != actions.end(); p++) {
        std::cout << ((MazeWorldAction*)*p)->getMovement() << std::endl;
    }*/
}

MazeWorldState *MazeWorld::getCurrentState() const {
    //std::cout << "GET CURRENT STATE" << std::endl;
    return currentState.clone();
}

void MazeWorld::applyAction(const Action *a) {
    //std::cout << "APPLY ACTION" << std::endl;
    const MazeWorldAction *act = dynamic_cast<const MazeWorldAction*>(a);
    //std::cout << act->getMovement() << std::endl;
    if(act == NULL)
        cerr << "MazeWorld: can't apply an action from another domain" << endl;

    if(act->getMovement() == 0) {
        // Move forward if possible
        //std::cout << currentState.direction << std::endl;
        //std::cout << "MOVE FORWARD" << std::endl;
        //std::cout << currentState.xpos << currentState.ypos << size << std::endl;
        //std::cout << grid[currentState.ypos][currentState.xpos] << std::endl;
        switch(currentState.direction) {
            
            case 'n': 
                if(currentState.ypos  > 0 && grid[currentState.ypos-1][currentState.xpos] != 'x') {
                    //std::cout << "MOVE NORTH" << std::endl;
                    currentState.ypos -= 1;
                }
            break;

            case 's':
                if(currentState.ypos < size-1 && grid[currentState.ypos+1][currentState.xpos] != 'x') {
                    //std::cout << "MOVE SOUTH" << std::endl;
                    currentState.ypos += 1;
                }
            break;
            case 'e':
                if(currentState.xpos < size -1 && grid[currentState.ypos][currentState.xpos+1] != 'x'){
                    //std::cout << "MOVE EAST" << std::endl;
                    currentState.xpos += 1;
                }
            break;
            case 'w':
                if(currentState.xpos  > 0 && grid[currentState.ypos][currentState.xpos-1] != 'x') {
                    //std::cout << "MOVE WEST" << std::endl;
                    currentState.xpos -= 1;
                }
            break;
            default:
                std::cout << "ERRRRRROR" << std::endl;
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

//grid no longer static. A grid belongs to the world
unsigned char** MazeWorld::getGrid() const {
    std::cout << "GET GRID" << std::endl;
    return this->grid;
}

void MazeWorld::reset() {
    currentState.xpos = startPosX;
    currentState.ypos = startPosY;
    currentState.direction = 's';
    std::cout << "RESET" << std::endl;
}

MazeWorld::~MazeWorld() {
    //std::cout << "GET DECONS" << std::endl;
    list<const Action*>::iterator act = actions.begin();
    for(; act != actions.end(); ++act)
        delete *act;
}


