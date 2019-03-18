#ifndef ROBOTPOSE_H
#define ROBOTPOSE_H


class RobotPose
{
public:
    RobotPose();
    RobotPose(float _x, float _y, float _z);

    float x;
    float y;
    float z;
};

#endif // ROBOTPOSE_H
