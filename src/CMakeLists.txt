add_subdirectory(worlds)
add_subdirectory(agents)


set ( learnrl_SRC  ${learnrl_SRC}
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
  PARENT_SCOPE
)
