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
        GazeboGenerator( const std::string& gazeboDest,
                double height, double width,
                double wallHeight = 2.5, double wallWidth = 0.2,
                const std::string& worldName = "default" ) :
            worldName( worldName ),
            height(height), width(width),
            wallHeight(wallHeight),
            wallWidth(wallWidth),
            gazeboFileName(gazeboDest) {};
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

        const double mapOccupiedThreshold = 0.65;
        const double mapFreeThreshold = 0.2;
        const int negateMap = 0;
        const double mapOriginX = 0;
        const double mapOriginY = 0;
        const double mapOriginZ = 0;
        const double mapResolution = 1;

        inline void
        writeYAML() {
            auto  yamlFile = std::ofstream( gazeboFileName + ".yaml", std::ios::out | std::ios::ate | std::ios::trunc );
            yamlFile << "image: " << gazeboFileName << ".pgm" << std::endl;
            yamlFile << "resolution: " << mapResolution << std::endl;
            yamlFile << "origin: [" << mapOriginX << ", " << mapOriginY << ", "  << mapOriginZ << " ]" << std::endl;
            yamlFile << "negate: " << std::to_string(negateMap) << std::endl;
            yamlFile << "occupied_thresh: " << mapOccupiedThreshold << std::endl;
            yamlFile << "free_thresh: " << mapFreeThreshold << std::endl;
            yamlFile << std::endl;
            yamlFile.close();
        }


    private:
        std::string worldName;
        double height, width;
        double wallHeight, wallWidth;

        std::string contourFileName;
        std::string gazeboFileName;

        std::vector<Polygon> polygons;

        std::ofstream gazeboFile;
    };
}

