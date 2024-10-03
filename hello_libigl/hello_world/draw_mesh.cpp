#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "igl/readOFF.h"
#include "igl/opengl/glfw/Viewer.h"
#include <iostream>

Eigen::MatrixXd V;
Eigen::MatrixXi F;

int main(int argc, char *argv[])
{
    std::cout << "123" << std::endl;
    igl::readOFF("D:\\Library\\libigl\\build\\_deps\\libigl_tutorial_tata-src\\bunny.off", V, F);

    // Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.launch();

    return 0;
}