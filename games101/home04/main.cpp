#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

// template<typename T>
// T cv_point_distance(cv::Point_<T> &a, cv::Point_<T>& b)
// {
//     auto distance = std::sqrt()
// }

cv::Point2f get_t_point_between_two_points(const cv::Point2f &a, const cv::Point2f &b, float t)
{
    cv::Point2f t_point{0.0f, 0.0f};
    t_point.x = a.x + t * (b.x - a.x);
    t_point.y = a.y + t * (b.y - a.y);
    return t_point;
}

std::vector<cv::Point2f> middle_recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
    std::vector<cv::Point2f> the_next_control_points;
    for(int i = 0; i < control_points.size() - 1; i++)
    {
        auto t_point = get_t_point_between_two_points(control_points[i], control_points[i + 1], t);
        the_next_control_points.emplace_back(std::move(t_point));
    }
    return the_next_control_points;
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    std::vector<cv::Point2f> middle_control_points = middle_recursive_bezier(control_points, t);
    while(middle_control_points.size() != 1)
    {
        middle_control_points = middle_recursive_bezier(middle_control_points, t);
    }
    auto ret = middle_control_points[0];
    return ret;
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    int steps = 1000;
    for(int i = 0; i < steps; i++)
    {
        float t = float(i) / 100.0f;
        auto point = recursive_bezier(control_points, t);
        // window[point.x, point.y] = 
        cv::Vec3b& color = window.at<cv::Vec3b>(point.y, point.x);
        color[0] = 0;
        color[1] = 0;
        color[2] = 255;
    }
}

int main() 
{
    cv::Mat window = cv::Mat(1500, 1000, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            //   bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
