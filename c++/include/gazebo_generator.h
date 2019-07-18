//
// Created by bjgilhul on 7/17/19.
//

#include <boost/format.hpp>
#include <fstream>
#include <iostream>
#include <string>

#include "VisibilityPolygon.h"

namespace V = Visibility;
using namespace std;

namespace GazeboGen {

    class World {
    public:
        World( const string& gazeboDest, const string& contourDest ) : contourFile("foo"), gazeboFile("gazebo") {
            contourFileName = contourDest + ".dat";
            gazeboFileName = gazeboDest + ".world";
        };

        ~World() {
        }

        void
        writeToDatFile( const V::Polygon& poly, double scale = 1 ) {
            // TODO: Polygons are only writing their exterior ring here -- should clean this up to add any
            //  holes that are included (need to know the proper format for gazebo...)
            auto points = V::convertToExteriorPoints( poly );
            contourFile << points.size() << std::endl;
            for( auto const& pt : points ) {
                // TODO: implement scaling here (was commented out)
                contourFile << pt.x() << " " << pt.y() << std::endl;
            }
        }

        void
        write( vector<V::Polygon> polygons, double height, double width ) {
            gazeboFile = ofstream( gazeboFileName.c_str(), ios::out | ios::ate | ios::trunc );
            contourFile = ofstream( contourFileName.c_str(), ios::out | ios::ate | ios::trunc );
            writeHeader( height, width );
            int wallNum = 0;
            for( auto const& poly: polygons ) {
                writeToDatFile( poly, 1 );
                writeToWorldFile( wallNum, poly, 1 );
            }
            writeFooter();
            gazeboFile.close();
            contourFile.close();
        }


        const char *header = R"(
<?xml version='1.0'?>
<sdf version='1.4'>
    <world name='1_wall'>
        <light name='sun' type='directional'>
            <cast_shadows>1</cast_shadows>
            <pose>0 0 10 0 -0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <attenuation>
            <range>1000</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
        </light>
        <model name='ground_plane'>
            <static>1</static>
            <link name='link'>
                <collision name='collision'>
                <geometry>
                    <plane>
                        <normal>0 0 1</normal>
                        <size>%1% %2%</size>
                    </plane>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>100</mu>
                            <mu2>50</mu2>
                        </ode>
                    </friction>
                    <bounce/>
                        <contact>
                            <ode/>
                        </contact>
                </surface>
                <max_contacts>10</max_contacts>
                </collision>
            <visual name='visual'>
                <cast_shadows>0</cast_shadows>
                <geometry>
                    <plane>
                        <normal>0 0 1</normal>
                        <size>%1% %2%</size>
                    </plane>
                </geometry>
                <material>
                    <script>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    <name>Gazebo/Grey</name>
                    </script>
                </material>
            </visual>
            <velocity_decay>
                <linear>0</linear>
                <angular>0</angular>
            </velocity_decay>
            <self_collide>0</self_collide>
                <kinematic>0</kinematic>
                <gravity>1</gravity>
            </link>
        </model>
        <physics type='ode'>
            <max_step_size>0.01</max_step_size>
            <real_time_factor>1</real_time_factor>
            <real_time_update_rate>100</real_time_update_rate>
            <gravity>0 0 -9.8</gravity>
        </physics>
        <scene>
            <ambient>0.4 0.4 0.4 1</ambient>
            <background>0.7 0.7 0.7 1</background>
            <shadows>1</shadows>
        </scene>
        )";


        const char * footer = R"(
    </world>
</sdf>
        )";

        const char * wallTemplate = R"(
        <model name='Wall_$index'>
            <static>1</static>
            <link name='Wall_%1!'>
                <collision name='Wall_Collision_$index'>
                    <geometry>
                        <box>
                            <size>%2% %3% %4%</size>
                        </box>
                    </geometry>
                    <pose>0 0 1.25 0 -0 0</pose>
                </collision>
                <visual name='Wall_Visual_$index'>
                    <pose>0 0 1.25 0 -0 0</pose>
                    <geometry>
                        <box>
                            <size>%2% %3% %4%</size>
                        </box>
                    </geometry>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>Gazebo/Grey</name>
                        </script>
                    </material>
                </visual>
                <velocity_decay>
                    <linear>0</linear>
                    <angular>0</angular>
                </velocity_decay>
                <pose>%5% %6% 0 0 0 %7%</pose>
            </link>
        </model>)";

        void
        writeHeader( double height, double width ) {
            auto formattedHeader = boost::format( header ) % height % width;
            gazeboFile << formattedHeader << endl;
        }

        void
        writeFooter() {
            gazeboFile << footer << endl;
        }

        inline
        V::Point mid( V::Segment segment ) const {
            return { (segment.first.x() + segment.second.x() ) / 2, (segment.first.y() + segment.second.y() ) / 2};
        };

        void
        writeWall( V::Segment segment, int wallNum, double scale = 1 ) {
            auto x1 = segment.first.x() * scale;
            auto y1 = segment.first.y() * scale;

            auto x2 = segment.second.x() * scale;
            auto y2 = segment.second.x() * scale;

            V::Point midp = mid( segment );

            auto wallWidth = 0.2;
            auto wallHeight = 2.5;
            auto wallDesc = boost::format( wallTemplate ) % wallNum % V::length( segment ) % wallWidth % wallHeight % midp.x() % midp.y()
                    % V::angle(segment.first, segment.second);
            gazeboFile << wallDesc << endl;
        };


        void
        writeToWorldFile( int& wallNum, V::Polygon poly, double scale = 1) {
            auto segments = V::convertToSegments( poly );
            for( auto const& segment: segments ) {
                writeWall( segment, wallNum, scale );
                wallNum++;
            }
        }

    private:
        string contourFileName;
        string gazeboFileName;

        ofstream contourFile;
        ofstream gazeboFile;

    };
}


//
//# todo: Make this take a .yaml file as an argument.
//# Improve documentation
//# pass contours using ros?
//# ignore small contours?
//# add tsp stuff
//if __name__ == '__main__':
//    occupancy_grid_name = rospy.get_param("occupancy_grid_name")
//    occupancy_grid_yaml = occupancy_grid_name + '.yaml'
//
//    print occupancy_grid_yaml
//    with open(occupancy_grid_yaml, 'r') as f:
//        configs = yaml.load(f)
//
//    scale = configs["resolution"]
//    map_name = configs["image"]
//    occupancy_grid_file = os.path.dirname(occupancy_grid_yaml) + '/' + map_name
//
//    output_contour_filename = rospy.get_param("contour_file")
//    output_gazebo_filename = rospy.get_param("gazebo_world_file")
//
//    # min_perimeter = 50
//    # epsilon = 4
//    # img = cv2.imread('FloorPlanImages/e5_rescaled2_inverted2.png')
//    img = cv2.imread(occupancy_grid_file)
//    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
//    ret, thresh = cv2.threshold(imgray, 206, 255, cv2.THRESH_BINARY)
//    contours, hierarchy = cv2.findContours(
//        thresh,
//        cv2.RETR_TREE,
//        cv2.CHAIN_APPROX_SIMPLE
//    )
//
//    contour_file = open(output_contour_filename, 'w+')
//    gazebo_file = open(output_gazebo_filename, 'w+')
//
//    writeGazeboAndDatFiles(contours, scale, contour_file, gazebo_file)
//    closeWorldFile(gazebo_file)
//    contour_file.close()
//
