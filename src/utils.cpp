/*
 * Author: Jackson Allen
 * Filename: utils.cpp
 *
 * */

#include <utils.hpp>

bool calculateJointAngles(const Eigen::Matrix4d &Pdes)
{
    double nx = Pdes(0, 0);
    double ny = Pdes(1, 0);
    double nz = Pdes(2, 0);

    double ox = Pdes(0, 1);
    double oy = Pdes(1, 1);
    double oz = Pdes(2, 1);

    double ax = Pdes(0, 2);
    double ay = Pdes(1, 2);
    double az = Pdes(2, 2);

    double x = Pdes(0, 3);
    double y = Pdes(1, 3);
    double z = Pdes(2, 3);

    // Solve for θ₁
    double A = x;
    double B = -y;
    double C = -d4;
    double D = sqrt(pow(A, 2) + pow(B, 2));
    // Clamp C/D to [-1, 1] to prevent complex results from asin
    double ratio = checkTol(std::clamp(C / D, -1.0, 1.0)); // Clamp to valid range
    double theta1 = checkTol(M_PI - asin(ratio) - atan2(B / D, A / D));
    if (theta1 > M_PI)
    {
        theta1 = checkTol(-2 * M_PI + theta1);
    }
    double cos1 = checkTol(cos(theta1));
    double sin1 = checkTol(sin(theta1));
    double theta1deg = checkTol(180 / M_PI * theta1);
    std::cout << "θ₁ = " << theta1deg << " deg" << std::endl;

    // Solve for θ₂₃₄
    double theta234 = checkTol(atan2(az, ax * cos1 + ay * sin1));
    double cos234 = checkTol(cos(theta234));
    double sin234 = checkTol(sin(theta234));
    double theta234deg = checkTol(180 / M_PI * theta234);
    std::cout << "θ₂₃₄ = " << theta234deg << " deg" << std::endl;

    // Solve for θ₃
    double cos3 = (pow((x * cos1 + y * sin1 - cos234 * l4), 2) + pow((z - sin234 * l4), 2) - pow(l2, 2) - pow(l3, 2)) / (2 * l2 * l3);
    cos3 = checkTol(std::clamp(cos3, -1.0, 1.0));
    double sin3 = checkTol(sqrt(1 - pow(cos3, 2)));
    double theta3;
    if (theta234 >= 0)
    {
        theta3 = atan(sin3 / cos3);
    }
    else
    {
        theta3 = atan(-sin3 / cos3);
    }
    double theta3deg = 180 / M_PI * theta3;
    std::cout << "θ₃ = " << theta3deg << " deg" << std::endl;

    // Solve for θ₂
    double sin2 = checkTol((cos3 * l3 + l2) * (z - sin(theta234) * l4) - sin3 * l3 * (x * cos(theta1) + y * sin(theta1) - cos(theta234) * l4));
    double cos2 = checkTol((cos3 * l3 + l2) * (x * cos(theta1) + y * sin(theta1) - cos(theta234) * l4) + sin3 * l3 * (z - sin(theta234) * l4));
    double theta2 = checkTol(atan2(sin2, cos2));
    double theta2deg = 180 / M_PI * theta2;
    std::cout << "θ₂ = " << theta2deg << " deg" << std::endl;

    // Solve for θ₄
    double theta4 = checkTol(theta234 - theta2 - theta3);
    double theta4deg = 180 / M_PI * theta4;
    std::cout << "θ₄ = " << theta4deg << " deg" << std::endl;

    // Solve for θ₅
    double sin5 = checkTol(nx * sin(theta1) - ny * cos(theta1));
    double cos5 = checkTol(ox * sin(theta1) - oy * cos(theta1));
    double theta5 = checkTol(atan2(sin5 , cos5));
    double theta5deg = 180 / M_PI * theta5;
    std::cout << "θ₅ = " << theta5deg << " deg" << std::endl;

    return true;
}

double checkTol(double val)
{
    if ((val < epsilon) & (val > -epsilon))
    {
        return 0;
    }
    else
    {
        return val;
    }
}
