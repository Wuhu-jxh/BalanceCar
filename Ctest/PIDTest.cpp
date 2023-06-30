//
// Created by 神奇bug在哪里 on 6/26/23.
//
#include <iostream>
#include "PidContorl.h"
#include <vector>
int main()
{
    std::cout << "PID Test will start!" << std::endl;
    std::vector<float> en_L;
    std::vector<float> en_R;
    std::vector<float> targetSpeed,targetAngle;
    std::vector<_MPU6050_DATA> input_MPU6050;
    std::vector<Angle> input_angle;
    std::vector<result> output;
    std::vector<result> output_expected;
   ///在这里运行PID代码并且检查代码的返回值是否正确
    std::cout << "PID Test will end!" << std::endl;
    return 0;
}