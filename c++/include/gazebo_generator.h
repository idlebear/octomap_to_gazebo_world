//
// Created by bjgilhul on 7/17/19.
//

#include <boost/format.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

namespace gazebo_generator {

    typedef std::pair<double,double> Point;
    typedef std::pair<Point, Point> Segment;
    typedef std::vector<Segment> Polygon;

    class GazeboGenerator {
    public:
        GazeboGenerator( const std::string& gazeboDest, const std::string& contourDest,
                double height, double width,
                double wallHeight = 2.5, double wallWidth = 0.2,
                const std::string& worldName = "default" ) :
            contourFileName(contourDest), gazeboFileName(gazeboDest),
            worldName( worldName ), height(height), width(width),
            wallHeight(wallHeight), wallWidth(wallWidth) {
        };
        ~GazeboGenerator() = default;

        void
        addPolygon( const Polygon& poly ) {
            polygons.emplace_back( poly );
        }

        void
        write( double scale = 1 );

    protected:
        void writeHeader( double scale );
        void writeFooter();
        void writeToDatFile( const Polygon& poly, double scale );
        void writeToWorldFile( int& wallNum, Polygon poly, double scale );
        void writeWall( const Segment& segment, int wallNum, double scale );

        inline Point
        mid( const Segment& segment ) const {
            return { (segment.first.first + segment.second.first )/2.0, (segment.first.second + segment.second.second)/2.0};
        };

        inline double
        angle( const Point& a, const Point& b ) const {
            return atan2( b.second - a.second, b.first - a.first );
        }

        inline double
        length( const Segment& segment ) const {
            auto dx = segment.first.first - segment.second.first;
            auto dy = segment.first.second - segment.second.second;
            return sqrt( dx * dx + dy * dy );
        }


    private:
        std::string worldName;
        double height, width;
        double wallHeight, wallWidth;

        std::string contourFileName;
        std::string gazeboFileName;

        std::vector<Polygon> polygons;

        std::ofstream contourFile;
        std::ofstream gazeboFile;
    };
}

