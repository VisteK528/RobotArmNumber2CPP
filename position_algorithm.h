#include <iostream>

class PositionAlgorithm
{
    private:
        float rad_to_deg(float radians);
        float deg_to_rad(float degrees);
        void update_base_robot_angles();
        void update_robot_adapted_base_robot_angles();
    public:
        float ab_sect;
        float bc_sect;
        float cd_sect;
        float ac_sect;
        float h;

        float a[2] = {0, 0};
        float b[2] = {0, 0};
        float c[2] = {0, 0};
        float d[2] = {0, 0};

        float alfa = 0;
        float beta = 0;
        float theta = 0;
        
        float rad_alfa = 0;
        float rad_beta = 0;
        float rad_theta = 0;

        float r_alfa = 0;
        float r_beta = 0;
        float r_theta = 0;

        float shoulder_j_offset;
        float elbow_j_offset;
        float pitch_j_offset;

        float calc_arm_pos_horizontally(float x, float y);
        float calc_arm_pos_vertically(float x, float y);
        float calc_arm_pos_horizontally_adapted(float x, float y);
        float calc_arm_pos_vertically_adapted(float x, float y);

        //Constructor
        PositionAlgorithm(float shoulder_len, float elbow_len, float effector_len, float base_height,
         float shoulder_joint_offset = 0, float elbow_joint_offset = 0, float pitch_joint_offset = 0)
         {
            ab_sect = shoulder_len;
            bc_sect = elbow_len;
            cd_sect = effector_len;
            h = base_height;

            shoulder_j_offset = shoulder_joint_offset;
            elbow_j_offset = elbow_joint_offset;
            pitch_j_offset = pitch_joint_offset;
         }

};