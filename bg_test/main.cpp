#include <iostream>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::linestring<point> linestring;
typedef boost::geometry::model::polygon<point> polygon;

int main()
{
    // Calculate the intersects of a cartesian polygon

    linestring line1, line2,line3;

    boost::geometry::read_wkt("linestring(2 2,3 3)", line1);
    boost::geometry::read_wkt("linestring(2 1,1 2,4 0)", line2);
	boost::geometry::read_wkt("linestring(2 1,1 2,4 0)", line3);

	boost::geometry::model::multi_linestring<linestring>  ml;
    bool b = boost::geometry::intersects(line1, line2);
    std::cout << "Intersects: " << (b ? "YES" : "NO") << std::endl;
	if(b)
	{
		std::deque<point> inter_p_deq;
		boost::geometry::intersection(line1, line2, inter_p_deq);

		std::cout<<"("<<inter_p_deq.front().x()<<", "<<inter_p_deq.front().y()<<")"<<std::endl;
	}

	std::deque<point> inter_p_deq;
	boost::geometry::intersection(line1, line2, inter_p_deq);
	std::cout<<inter_p_deq.size();

    return 0;
}
