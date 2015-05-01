#include "FileValueObserver.h"

#include "../State.h"

#include <algorithm>
#include <iterator>
#include <iostream>
using namespace std;

FileValueObserver::FileValueObserver(const std::string& fileName, State *desired_state ) : 
      file(fileName.c_str()),
      desired_state(desired_state) {}


void FileValueObserver::valueChanged(const State* s, const std::vector<double>& old_values, const std::vector<double>& new_values) {
  if(!s->lessThen(desired_state) && !desired_state->lessThen(s)) {//same state!
   //std::cout << "YO" << std::endl;
   copy(new_values.begin(), new_values.end(),ostream_iterator<double>(file, " "));
   file << endl;
  }
}

FileValueObserver::~FileValueObserver() {
  file.close();
  delete desired_state;
}

