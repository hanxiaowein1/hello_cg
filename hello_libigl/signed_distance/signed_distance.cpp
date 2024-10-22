#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <iostream>
#include "igl/readOFF.h"
#include "igl/signed_distance.h"

int main(int argc, char *argv[])
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::readOFF("D:\\Library\\libigl\\build\\_deps\\libigl_tutorial_tata-src\\bunny.off", V, F);

    Eigen::Vector3d m = V.colwise().minCoeff();
    Eigen::Vector3d M = V.colwise().maxCoeff();

    std::cout << m << std::endl;
    std::cout << M << std::endl;

    Eigen::MatrixXd P;
    int sample = 3;
    P.resize(std::pow(sample - 1, 3), 3);
    for(int z = 1; z < sample; z++)
    {
        double z_sample = m.z() + double(z) * (M.z() - m.z()) / double(sample);
        for(int y = 1; y < sample; y++)
        {
            double y_sample = m.y() + double(y) * (M.y() - m.y()) / double(sample);
            for(int x = 1; x < sample; x++)
            {
                double x_sample = m.x() + double(x) * (M.x() - m.x()) / double(sample);
                int count = (z - 1) * (sample - 1) * (sample - 1) + (y - 1) * (sample - 1) + x - 1;
                //int count = (x + 1) * (y + 1) * (z + 1) - 1;
                //std::cout << count << std::endl;
                P(count, 0) = x_sample;
                P(count, 1) = y_sample;
                P(count, 2) = z_sample;
            }
        }
    }
    std::cout << P << std::endl;
    Eigen::VectorXi I;
    Eigen::MatrixXd N,C;
    Eigen::VectorXd S;
    igl::SignedDistanceType type = igl::SIGNED_DISTANCE_TYPE_PSEUDONORMAL;
    igl::signed_distance(P, V, F, type, S, I, C, N);
    std::cout << "signed distance is " << std::endl;
    std::cout << S << std::endl;

    return 0;
}