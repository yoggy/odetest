cmake_minimum_required (VERSION 2.8)
    
project (odetest02)
    
add_executable (odetest02 odetest02.cpp)
target_link_libraries(odetest02 ode ncurses)
