#include "monotonecubicinterpolation.h"
#include <Eigen/Dense>
#include <iostream>


int main()
{
    // test the MonotoneCubicInterpolation
    colvecX rs = lspb(0, 1.0, 15.0, 0.1);
    std::cout << "test interpolate with lspb" << std::endl;

    Eigen::MatrixXd input;
    input.resize(6, 3);
    input << 0, 30, 0,
        0, 30, 0,
        0, 30, 0,
        0, 30, 0,
        0, 30, 0,
        0, 30, 0;
    Eigen::MatrixXd output = mcspline(input, rs);
    std::cout << "xs" << std::endl<< rs << std::endl;
    std::cout << "ys" << std::endl << output.transpose() << std::endl;

    system("pause");
    return 1;
}
