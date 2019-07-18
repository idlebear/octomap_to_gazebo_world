//
// Created by bjgilhul on 7/18/19.
//

#include <iostream>
#include <ros/ros.h>
#include <string>
#include <VisibilityPolygon.h>

#include "gazebo_generator.h"


int
main( int argc, char** argv ) {

    // Init ROS
    ros::init( argc, argv, "GazeboGen" );
    ros::NodeHandle n;

    string gazeboFileName;
    n.getParam( "gazebo_world_file", gazeboFileName );
    string contourFileName;
    n.getParam( "contour_file", contourFileName );

    GazeboGen::World world( gazeboFileName, contourFileName );

    vector<V::Polygon> obs;
    V::Polygon p {{ V::Point( 100, 100 ), V::Point( 200, 100 ),V::Point( 200, 200 ),V::Point( 100, 200 ), V::Point( 100, 100 ) } };
    obs.emplace_back( p );
    world.write( obs, 2000, 2000 );
}

