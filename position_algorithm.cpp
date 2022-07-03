#include "position_algorithm.h"
#include <iostream>
#include <math.h>

float PositionAlgorithm::rad_to_deg(float radians)
{
    radians *= 180;
    radians /= M_PI;
    return radians;
}


float PositionAlgorithm::deg_to_rad(float degrees)
{
    degrees *= M_PI;
    degrees /= 180;
    return degrees;
}

void PositionAlgorithm::update_base_robot_angles()
{
    alfa = rad_to_deg(rad_alfa);
    beta = rad_to_deg(rad_beta);
    theta = rad_to_deg(rad_theta);
}

void PositionAlgorithm::update_robot_adapted_base_robot_angles()
{
    r_alfa = abs(alfa-shoulder_j_offset);
    r_beta = abs(beta-elbow_j_offset);
    r_theta = abs(theta-pitch_j_offset);
}

float PositionAlgorithm::calc_arm_pos_horizontally(float x, float y)
{   
    float new_x = x - cd_sect;
    float new_y = y - h;

    ac_sect = sqrt(pow(new_x, 2) + pow(new_y, 2));

    float alfa2 = atan(new_y/new_x);
    float alfa1 = acos((pow(ab_sect, 2)+pow(ac_sect, 2)-pow(bc_sect, 2))/(2*ab_sect*ac_sect));
    rad_alfa = alfa1+alfa2;
    rad_beta = acos((pow(ab_sect, 2)+pow(bc_sect, 2)-pow(ac_sect, 2))/(2*ab_sect*bc_sect));
    rad_theta = M_PI - (rad_alfa+rad_beta);

    update_base_robot_angles();
    update_robot_adapted_base_robot_angles();

    d[0], d[1] = x, y;
    c[0], c[1] = new_x, new_y;
    float b_x = sqrt(pow(ab_sect,2)/(1+pow(tan(rad_alfa), 2)));
    float b_y = tan(rad_alfa) * b_x;

    if(90<alfa && alfa < 180)
    {
        b_x *= -1;
        b_y *= -1;
    }

    b[0], b[1] = b_x, b_y;
    a[1] = h;
}

float PositionAlgorithm::calc_arm_pos_vertically(float x, float y)
{
    float new_y = y - h;
    float new_y2 = new_y + cd_sect;

    ac_sect = sqrt(pow(x, 2)+pow(new_y2, 2));
    float alfa1 = acos((pow(ab_sect, 2)+pow(ac_sect, 2)-pow(bc_sect, 2))/(2*ab_sect*ac_sect));
    float alfa2 = atan(new_y2/x);

    rad_alfa = alfa1 + alfa2;
    rad_beta = acos((pow(ab_sect, 2)+pow(bc_sect, 2)-pow(ac_sect, 2))/(2*ab_sect*bc_sect));
    rad_theta = (M_PI/2) - (rad_alfa+rad_beta);
    
    update_base_robot_angles();
    update_robot_adapted_base_robot_angles();

    d[0], d[1] = x, y;
    c[0], c[1] = x, y+cd_sect;
    float b_x = sqrt((pow(ab_sect, 2)/(1+pow(tan(rad_alfa), 2))));
    float b_y = tan(rad_alfa)*b_x;

    if(90 < alfa && alfa < 180)
    {
        b_x *= -1;
        b_y *= -1;
    }
    b[0], b[1] = b_x, b_y;
    a[1] = h;
}