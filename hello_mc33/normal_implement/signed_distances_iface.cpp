#include "signed_distances_iface.h"
#include "Eigen/Dense"
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include "igl/readOFF.h"
#include "igl/signed_distance.h"
#include "tensor_flat.pb.h"
#include <format>
#include <fstream>
/**
 * @brief 
 * 
 * @param nx how many points do you want to generate in the x direction
 * @param ny 
 * @param nz 
 * @param mesh_path (off type mesh path)
 * @param save_path (output path for signed distance)
 */
void generate_signed_distance(const int& nx, const int& ny, const int &nz, const std::string& mesh_path, const std::string& save_path)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    std::string off_path = mesh_path;
    igl::readOFF(off_path, V, F);
    Eigen::Vector3d m = V.colwise().minCoeff();
    std::cout << m << std::endl;
    Eigen::Vector3d M = V.colwise().maxCoeff();
    std::cout << M << std::endl;
    Eigen::MatrixXd P;
	P.resize(nx * ny * nz, 3);
	for(unsigned int k = 0; k < nz; k++)
	{
		double z_axis = m.z() + double(k) * (M.z() - m.z()) / double(nz - 1);
		for(unsigned int j = 0; j < ny; j++)
		{
			double y_axis = m.y() + double(j) * (M.y() - m.y()) / double(ny - 1);
			for(unsigned int i = 0; i < nx; i++)
			{
				double x_axis = m.x() + double(i) * (M.x() - m.x()) / double(nx - 1);
				unsigned int count = k * ny * nx + j * nx + i;
				P(count, 0) = x_axis;
				P(count, 1) = y_axis;
				P(count, 2) = z_axis;
			}
		}
	}

    Eigen::VectorXi I;
    Eigen::MatrixXd N,C;
    Eigen::VectorXd S;
    igl::SignedDistanceType type = igl::SIGNED_DISTANCE_TYPE_PSEUDONORMAL;
    igl::signed_distance(P, V, F, type, S, I, C, N);

    GOOGLE_PROTOBUF_VERIFY_VERSION;
    Tensor3DFlat tensor;
    tensor.set_dim_x(nx);
    tensor.set_dim_y(ny);
    tensor.set_dim_z(nz);

    for(const auto& val: S)
    {
        tensor.add_values(val);
    }

    std::ofstream ofs(save_path, std::ios::binary);
    if(!ofs)
    {
        std::string err_msg = std::format("cannot open file {} for writing!", save_path);
        std::cerr << err_msg << std::endl;
        throw std::exception(err_msg.c_str());
    }
    if(!tensor.SerializeToOstream(&ofs))
    {
        std::string err_msg = std::format("Failed to serialize tensor to {} file", save_path);
        std::cerr << err_msg << std::endl;
        throw std::exception(err_msg.c_str());
    }
    ofs.close();
    google::protobuf::ShutdownProtobufLibrary();
}

Eigen::VectorXd get_signed_distance(int &nx, int &ny, int &nz, const std::string& sd_path)
{
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    Tensor3DFlat tensor;
    std::ifstream ifs(sd_path, std::ios::binary);
    if(!ifs)
    {
        std::string err_msg = std::format("cannot open file {} for reading", sd_path);
        std::cerr << err_msg << std::endl;
        throw std::exception(err_msg.c_str());
    }

    if(!tensor.ParseFromIstream(&ifs))
    {
        std::string err_msg = std::format("failed to parse from {} to variable", sd_path);
        std::cerr << err_msg << std::endl;
        throw std::exception(err_msg.c_str());
    }
    nx = tensor.dim_x();
    ny = tensor.dim_y();
    nz = tensor.dim_z();

    Eigen::VectorXd signed_distances;
    signed_distances.resize(nx * ny * nz);

    for(int i = 0; i < tensor.values_size(); i++)
    {
        signed_distances[i] = tensor.values(i);
    }

    google::protobuf::ShutdownProtobufLibrary();

    return signed_distances;
}
