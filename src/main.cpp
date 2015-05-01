#include <iostream>

#include "worlds/MazeWorld.h"
#include "worlds/MazeProblem.h"
#include "agents/SarsaAgent.h"
#include "agents/FileValueObserver.h"
#include <stdlib.h>
#include <fstream>

using namespace std;

const int num_episodes = 10000;

int main(int argc, char **argv) {
  //std::cout << "HIEIEIEIEIE" << std::endl;
  /*char grid[10][10] = {
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

  char grid[5][5]  = {
    {'x', 's', 'o', 'x', 'x'},
    {'x', 'o', 'x', 'o', 'x'},
    {'x', 'o', 'x', 'x', 'x'},
    {'x', 'e', 'x', 'x', 'x'},
    {'x', 'x', 'x', 'x', 'x'}
  };

  //std::cout << "HI" << std::endl;
  unsigned char** grid_p = (unsigned char**)malloc(10*sizeof(grid[0]));
  for(int i = 0; i < 5; i++) {
    unsigned char* grid_p_tmp = (unsigned char*)malloc(sizeof(grid[0]));
    for(int j = 0; j < 5; j++) {
      grid_p_tmp[j] = grid[i][j];
    }
    grid_p[i] = grid_p_tmp;
  }
  
  MazeWorld *world = new MazeWorld(grid_p, 5);
  
  Problem *problem= new MazeProblem(world);
  
  SarsaAgent::Params p;
  p.epsilon = .8;
  p.alpha = 0.6;
  
  SarsaAgent *agent= new SarsaAgent(problem,world,p);
  
  State *initial_state = new MazeWorldState(1,2,'s', list<const Action*>()); //don't care about available actions in this case
  State *final_state = new MazeWorldState(2, 0, 'e', list<const Action*>()); 
  State *looking_down = new MazeWorldState(1, 0, 's', list<const Action*>());
  State *looking_east = new MazeWorldState(1, 0, 'e', list<const Action*>());
  FileValueObserver value_initial("value_initial.txt", initial_state);
  FileValueObserver value_final("value_final.txt", final_state);
  FileValueObserver value_down("value_down.txt", looking_down);
  FileValueObserver value_east("value_east.txt", looking_east);
  agent->addValueObserver(&value_initial);
  agent->addValueObserver(&value_final);
  agent->addValueObserver(&value_down);
  agent->addValueObserver(&value_east);
  
  ifstream storedExperience("exp.txt");
  
  agent->readFrom(storedExperience);
  storedExperience.close();
  
  for(int i=0; i<num_episodes; ++i) {
    int total = 0;
    while (!problem->solved()) {
      
      agent->act();
      /*MazeWorldState* s = world->getCurrentState();
      std::cout << s->xpos << " " << s->ypos << " " << s->direction << std::endl;
      delete s;*/
      total ++;
    }
    std::cout << "FINISHED" << std::endl;
    ofstream experienceFile("exp.txt");
    agent->writeTo(experienceFile);
    experienceFile.close();
    std::cout << "took " << total << " tries" << " at " << p.epsilon << std::endl;
    p.epsilon = p.epsilon * .9999;
    p.alpha = p.alpha *.99999;
    world->reset();
    
  }
  //std::cout << "GOT HERE" << std::endl;
  
  delete agent;
  //std::cout << "deleted agent" << std::endl;
  delete world;
  delete problem;
}
