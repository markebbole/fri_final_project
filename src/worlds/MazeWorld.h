#ifndef MazeWorld_h__guard
#define MazeWorld_h__guard

#include "../World.h"
#include "../State.h"
#include "../Action.h"
#include <string>

struct MazeWorldState : public State {
  
    MazeWorldState(unsigned int xpos, unsigned int ypos, unsigned char direction, const std::list<const Action*>& actions) 
    : xpos(xpos), ypos(ypos), direction(direction), actions(actions) {}
    //make x and y position  
    unsigned int xpos;
    unsigned int ypos;
    unsigned char direction;
    //used for map? 
    bool lessThen(const State* other) const;   
    std::list<const Action*> availableActions() const;

    MazeWorldState *clone() const;
  
    private:
        const std::list<const Action*> &actions;
};

struct MazeWorldAction : public Action{
    explicit MazeWorldAction(int moveDir) : movement(moveDir) {}
    // let 0 = forward
    // 1 = right
    // 2 = left
    int getMovement()  const { return movement;}
    bool lessThen(const Action *other) const;
    Action *clone() const { return new MazeWorldAction(*this);}
  
    operator std::string() const {return (movement == 0)? std::string("forward") : (movement == 1) ? std::string("right") : std::string("left");} 
 
    private:
        int movement;
};


struct MazeWorld : public World {
  
    //replace max_position with 2d array grid
    //added 2nd param size, assuming always using square grid
    explicit MazeWorld(unsigned char** grid);

    MazeWorldState* getCurrentState() const;

    void applyAction(const Action *a);

    void reset();
    //used to get the end coordinates
    unsigned int getEndPosX() const {return endPosY;}
    unsigned int getEndPosY() const {return endPosX;}
    unsigned char** getGrid() const;
    ~MazeWorld();

    private:
        unsigned int endPosX;
        unsigned int endPosY;
        unsigned int startPosX;
        unsigned int startPosY;
        std::list<const Action*> actions;
        MazeWorldState currentState;
        unsigned char** grid;
        int size;
};
#endif
