#include <iostream>
#include "position_algorithm.h"


int main(){
    float x, y;
    PositionAlgorithm alg(20.76355, 16.50985, 7.835, 15, 180, 50.3, -111);
    while( 1 > 0)
    {
        std::cout<<"Please enter X value: ";
        std::cin>>x;
        std::cout<<"Please enter Y value: ";
        std::cin>>y;
        alg.calc_arm_pos_vertically(x, y);
        std::cout<<alg.alfa<<std::endl;
    }
    return 0;
}