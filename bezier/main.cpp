// 此处插入编写的代码
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include "matplotlibcpp.h"

#include <assert.h>

using namespace std;
namespace plt = matplotlibcpp;
template <typename T> // // TODO: 后续改成模板类--已完成
struct Point
{
    T x;
    T y;
    Point() = default;
    Point(const T x_, const T y_) : x(x_), y(y_) {}
    Point(const Point &other)
    {
        x = other.x;
        y = other.y;
    }
    Point &operator=(const Point &other)
    {
        if (this == &other)
        {
            return *this;
        }
        x = other.x;
        y = other.y;

        return *this;
    }
};

int factorial(const unsigned int n)
{
    if (n == 0 || n == 1)
    {
        return 1;
    }

    return n * factorial(n - 1);
}
//n个数里取i个数的方法
int Cni(const unsigned int n, const unsigned int i)
{
    if (i == 0 || n == i)
    {
        return 1;
    }

    unsigned int up = factorial(n);
    unsigned int down = factorial(i) * factorial(n - i);
    return up / down;
}

template <typename T>
vector<Point<T>> Bezier(const vector<Point<T>> &origin_point, const int path_point_number)
{
    int point_number = (int)origin_point.size();
    vector<Point<T>> res;

    double iter = 1.0 / (path_point_number - 1.0); // FIXME: 这里出错了，用的都是int型，没有办法转成double,1.0写成了1.淦，搞了半天。
    int order = point_number - 1;

    for (int i = 0; i <= path_point_number - 1; i++)
    {
        Point<T> tmp_point(0.0, 0.0);
        float t = i * iter;
        vector<double> coefficient(point_number);
        for (int m = 0; m <= order; m++)
        {
            double tmp_coefficient;
            tmp_coefficient = Cni(order, m) * powf(t, (float)m) * powf(1.0 - t, (float)(order - m));
            coefficient[m] = tmp_coefficient;
        }
        for (int j = 0; j <= order; j++)
        {
            tmp_point.x += coefficient[j] * origin_point[j].x;

            tmp_point.y += coefficient[j] * origin_point[j].y;
        }
        res.push_back(tmp_point);
    }

    return res;
}

int main()
{
    vector<vector<double>> origin_point_vec = {{0.0, 0.0}, {0.0, 2.0},  {2.0, 2.0}, {2.0, 6.0}};
    int number_points = (int)origin_point_vec.size();
    vector<Point<double>> origin_point(number_points);
    vector<double> x(number_points), y(number_points); // 原始点
    for (int i = 0; i < number_points; i++)
    {
        origin_point[i].x = origin_point_vec[i][0];
        x[i] = origin_point_vec[i][0];
        origin_point[i].y = origin_point_vec[i][1];
        y[i] = origin_point_vec[i][1];
    }

    //vector<double> origin_y(number_points);
    int path_point_number = 101;
    vector<Point<double>> res = Bezier(origin_point, path_point_number);
    vector<double> x_output(path_point_number);
    vector<double> y_output(path_point_number);

    for (int j = 0; j < path_point_number; j++)
    {
        x_output[j] = res[j].x;
        // cout<<"x["<<j<<"]"<<res[j].x<<endl;
        // cout<<"y["<<j<<"]"<<res[j].y<<endl;

        y_output[j] = res[j].y;
    }

    plt::plot(x, y, "r-", x_output, y_output, "b-");
    plt::grid(true);

    //plt::set_aspect(0.5);
    plt::set_aspect_equal();

    // show plots
    plt::show();
    return 0;
}

// g++ main.cpp  -o bezier  -std=c++11 -I/usr/include/python2.7 -lpython2.7
// 解释：-o modern指生成可执行文件的名字；-std=c++11 为c++的版本  -I/usr/include/python2.7  -I 为include，意思是把python2.7include进来
// -lpython2.7 -l 是link的缩写，意思是把可执行文件与python2.7链接起来。



/**
 * 1.前轮转向，后轮驱动；单向行驶，货物放在车上的，不是挂车。
 * 2. 方案先不变，后续再考虑降成本的事；
 * 3. 6月底只在室外。
 * 4. 6月演示的场地，图与视频需要。
 * 5. 拉货地点需要再确定。
 * 6. 整车控制器，差速转向。应用层软件。
 * 7.转向和制动一起，驱动电机还没定。*/
