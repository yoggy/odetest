cmake_minimum_required (VERSION 2.8)
    
project (odetest01)
    
add_executable (odetest01 odetest01.cpp)
target_link_libraries(odetest01 ode)
