#include <iostream>

#include "worlds/LinearWorld.h"
#include "worlds/LinearProblem.h"
#include "agents/SarsaAgent.h"
#include "agents/FileValueObserver.h"

#include <fstream>

using namespace std;

const int num_episodes = 500;

int main(int argc, char **argv) {
  
  
  const int max_state = 10;
  
  LinearWorld *world = new LinearWorld(max_state);
  
  Problem *problem= new LinearProblem(world);
  
  SarsaAgent::Params p;
  p.epsilon = 0.1;
  p.alpha = 0.3;
  
  SarsaAgent *agent= new SarsaAgent(problem,world,p);
  
  State *initial_state = new LinearWorldState(1, list<const Action*>()); //don't care about available actions in this case
  State *final_state = new LinearWorldState(max_state-1, list<const Action*>()); 
  FileValueObserver value_initial("value_initial.txt", initial_state);
  FileValueObserver value_final("value_final.txt", final_state);
  agent->addValueObserver(&value_initial);
  agent->addValueObserver(&value_final);
  
  ifstream storedExperience("exp.txt");
  
  agent->readFrom(storedExperience);
  storedExperience.close();
  
  for(int i=0; i<num_episodes; ++i) {
    
    while (!problem->solved()) {
      
      agent->act();
      
    }
    
    ofstream experienceFile("exp.txt");
    agent->writeTo(experienceFile);
    experienceFile.close();
    
    world->reset();
    
  }
  
  delete agent;
  delete world;
  delete problem;
}
