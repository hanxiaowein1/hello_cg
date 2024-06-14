// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <limits>
#include <iostream>
#include <stdio.h>
#include "gtest/gtest.h"


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Triangle& t)
{
    auto _v = t.toVector4();
    // std::cout << "x: " << x << " y: "<< y << " " << "(" << _v[0][0] << "," << _v[0][1] << ")" << "(" << _v[1][0] << "," << _v[1][1] << ")" << "(" << _v[2][0] << "," << _v[2][1] << ")"<<std::endl;
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // assume (x, y) is U, v0 is A, v1 is B, v2 is C
    // 1. A->B * A->U
    Eigen::Vector3f AB {_v[1][0] - _v[0][0], _v[1][1] - _v[0][1], 0};
    Eigen::Vector3f AU {x - _v[0][0], y - _v[0][1], 0};
    auto dot_product1 = AB.cross(AU)[2];
    // 2. B->C * B->U
    Eigen::Vector3f BC {_v[2][0] - _v[1][0], _v[2][1] - _v[1][1], 0};
    Eigen::Vector3f BU {x - _v[1][0], y - _v[1][1], 0};
    auto dot_product2 = BC.cross(BU)[2];
    // 3. C->A * C->U
    Eigen::Vector3f CA {_v[0][0] - _v[2][0], _v[0][1] - _v[2][1], 0};
    Eigen::Vector3f CU {x - _v[2][0], y - _v[2][1], 0};
    auto dot_product3 = CA.cross(CU)[2];
    // check if dot product is all plus or minus

    // std::cout<<dot_product1<<", "<<dot_product2<<", "<<dot_product3<<std::endl;
    if(dot_product1 > 0 && dot_product2 > 0 && dot_product3 > 0)
    {
        // std::cout << "x: " << x << " y: "<< y << "is inside " << "(" << _v[0][0] << "," << _v[0][1] << ")" << "(" << _v[1][0] << "," << _v[1][1] << ")" << "(" << _v[2][0] << "," << _v[2][1] << ")";
        return true;
    }
    // if(dot_product1 < 0 && dot_product2 < 0 && dot_product3 < 0)
    // {
    //     return true;
    // }
    return false;
}

#if GTEST_ENABLED

class GlobalTest
{
    ~GlobalTest() = default;
    void SetUp() {};
    void TearDown() {};
};

TEST(GlobalTest, InsideTriangleTest)
{
    Triangle t;
    Eigen::Vector3f a_point {0, 0, 1};
    Eigen::Vector3f b_point {1, 0, 1};
    Eigen::Vector3f c_point {0, 1, 1};
    t.setVertex(0, a_point);
    t.setVertex(1, b_point);
    t.setVertex(2, c_point);

    auto ret = insideTriangle(0.1f, 0.1f, t);
    ASSERT_EQ(ret, true);
    ret = insideTriangle(0.5f, 0.6f, t);
    ASSERT_EQ(ret, false);
}
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}
#endif


static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // std::cout<< "triangle, A:" << v[0][0] << "," << v[0][1] << "," << v[0][2] << "B:"<< v[1][0] << "," << v[1][1] << "," << v[1][2] << "C:"<< v[2][0] << "," << v[2][1] << "," << v[2][2]<<std::endl;

    // TODO : Find out the bounding box of current triangle.
    // get max x, y, and min x, y
    float max_x = std::numeric_limits<float>::min();
    float max_y = std::numeric_limits<float>::min();
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    for(const auto &point: v)
    {
        if(point[0] < min_x)
        {
            min_x = point[0];
        }
        if(point[1] < min_y)
        {
            min_y = point[1];
        }
        if(point[0] > max_x)
        {
            max_x = point[0];
        }
        if(point[1] > max_y)
        {
            max_y = point[1];
        }
    }
    // so bounding box is:
    int max_x_i = std::ceil(max_x);
    int max_y_i = std::ceil(max_y);
    int min_x_i = std::floor(min_x);
    int min_y_i = std::floor(min_y);
    // printf("max_x_i: %d, max_y_i: %d, min_x_i: %d, min_y_i: %d\n", max_x_i, max_y_i, min_x_i, min_y_i);
    // iterate through the pixel and find if the current pixel is inside the triangle
    // auto z_buffer = Eigen::MatrixXf::Constant(max_x_i - min_x_i, max_y_i - min_y_i, std::numeric_limits<float>::max());
    for(int x = min_x_i; x < max_x_i; x++)
    {
        for(int y = min_y_i; y < max_y_i; y++)
        {
            float center_x_f = float(x) + 0.5f;
            float center_y_f = float(y) + 0.5f;
            if(insideTriangle(center_x_f, center_y_f, t))
            {
                // If so, use the following code to get the interpolated z value.
                auto[alpha, beta, gamma] = computeBarycentric2D(center_x_f, center_y_f, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                if(this->depth_buf[(x - min_x_i) * height + (y - min_y_i)] >= z_interpolated)
                {
                    Eigen::Vector3f point {x, y, 1.0f};
                    auto color = t.getColor();
                    set_pixel(point, color);
                    this->depth_buf[(x - min_x_i) * height + (y - min_y_i)] = z_interpolated;
                }
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::max());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on