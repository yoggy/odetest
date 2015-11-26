cmake_minimum_required (VERSION 2.8)
    
project (odetest01)
add_executable (odetest01 odetest01.cpp)
target_link_libraries(odetest01 ode)

project (odetest02)
add_executable (odetest02 odetest02.cpp)
target_link_libraries(odetest02 ode ncurses)

project (odetest03)
add_executable (odetest03 odetest03.cpp)
target_link_libraries(odetest03 ode ncurses)
