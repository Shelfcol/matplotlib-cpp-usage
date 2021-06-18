#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include "/home/nio/matplotlib-cpp-usage/bg_test/include/bg_test/matplotlibcpp.h"
#include <vector>
#include <string>
using namespace std;
namespace plt = matplotlibcpp;

typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::linestring<point> linestring;
typedef boost::geometry::model::polygon<point> polygon;

//
void PolyGonsVis(boost::geometry::model::multi_polygon<polygon>& mul_geo)
{
    string  color_string[]={"g-"};
    vector<string>  color_vec{color_string,color_string+sizeof(color_string)/sizeof(string)};
    for(int i=0;i<mul_geo.size();++i)
    {
        size_t point_size=boost::geometry::num_points(mul_geo[i]);

        vector<double> x;
        vector<double> y;
        x.reserve(point_size);
        y.reserve(point_size);
        boost::geometry::for_each_point(mul_geo[i],[&](auto& p){
            x.push_back(p.x());
            y.push_back(p.y());
        });
        plt::plot(x, y,color_vec[i%color_vec.size()]);
    }
}

void LineStringsVis( boost::geometry::model::multi_linestring<linestring>& mul_geo)
{
    
    //线型 线方式： - 实线 :点线 -. 虚点线 - - 波折线。
    // 线型 点方式： . 圆点 +加号 * 星号 x x形 o 小圆
    // 颜色： y黄； r红； g绿； b蓝； w白； k黑； m紫； c青.
    // string  color_string[]={"r-","b-","g-"};
    string  color_string[]={"r-"};

    vector<string>  color_vec{color_string,color_string+sizeof(color_string)/sizeof(string)};
    for(int i=0;i<mul_geo.size();++i)
    {
        size_t point_size=boost::geometry::num_points(mul_geo[i]);

        vector<double> x;
        vector<double> y;
        x.reserve(point_size);
        y.reserve(point_size);
        boost::geometry::for_each_point(mul_geo[i],[&](auto& p){
            x.push_back(p.x());
            y.push_back(p.y());
        });
        plt::plot(x, y,color_vec[color_vec.size()-1-i%color_vec.size()]);
    }
}

//多条线生成可行驶区域
void GenDriveableArea(boost::geometry::model::multi_linestring<linestring> &ml,double line_width,boost::geometry::model::multi_polygon<polygon> &result)
{
    // Declare the symmetric distance strategy
    boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(line_width/2);

    // Declare other strategies
    boost::geometry::strategy::buffer::side_straight side_strategy;
    boost::geometry::strategy::buffer::join_round join_strategy;
    boost::geometry::strategy::buffer::end_flat end_strategy;
    boost::geometry::strategy::buffer::point_circle point_strategy;

    boost::geometry::buffer(ml, result,
                distance_strategy, side_strategy,
                join_strategy, end_strategy, point_strategy);
}

int main()
{
    boost::geometry::model::multi_linestring<linestring> ml;
    boost::geometry::read_wkt("MULTILINESTRING((3 5,5 10,7 5),(7 7,11 10,15 7,19 10),(7.2 7.2,11.2 10.2,15.2 7.2,19.2 10.2))", ml);
    boost::geometry::model::multi_polygon<polygon> result;

    GenDriveableArea(ml,2.9,result);

    LineStringsVis( ml);
    PolyGonsVis(result);
    plt::grid(true);

    //plt::set_aspect(0.5);
    plt::set_aspect_equal();

    // show plots
    plt::show();
    return 0;
}
