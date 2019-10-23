//
// Created by bjgilhul on 7/18/19.
//

#include "gazebo_generator.h"

namespace gazebo_generator {

    const double basePlateMultiple = 4.0;

    void
    GazeboGenerator::write( double scale ) {
        gazeboFile = std::ofstream( gazeboFileName + ".world", std::ios::out | std::ios::ate | std::ios::trunc );
        writeHeader( scale );
        int wallNum = 0;
        for( auto const& poly: polygons ) {
            writeWallToWorldFile( wallNum, poly, scale );
        }
        int pylonNum = 0;
        for( auto const& p: pylons ) {
            writePylonToWorldFile( pylonNum, p.first, p.second, scale );
            pylonNum++;
        }
        writeFooter();
        gazeboFile.close();

        writeYAML();
    }

    const char *header = R"(
<?xml version='1.0'?>
<sdf version='1.6'>
    <world name='%1%'>
        <light name='sun' type='directional'>
        <cast_shadows>1</cast_shadows>
        <pose frame=''>0 0 10 0 -0 0</pose>
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


    <!-- Only one ROS interface plugin is required per world, as any other plugin can connect a Gazebo
         topic to a ROS topic (or vise versa). -->
    <plugin name="ros_interface_plugin" filename="librotors_gazebo_ros_interface_plugin.so"/>


    <model name='ground_plane'>
        <static>1</static>
        <link name='link'>
            <collision name='collision'>
                <geometry>
                    <plane>
                    <normal>0 0 1</normal>
                    <size>%2% %3%</size>
                    </plane>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>100</mu>
                            <mu2>50</mu2>
                        </ode>
                        <torsional>
                            <ode/>
                        </torsional>
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
                        <size>%2% %3%</size>
                    </plane>
                </geometry>
                <material>
                    <script>
                        <uri>file://media/materials/scripts/gazebo.material</uri>
                        <name>Gazebo/Green</name>
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
            <enable_wind>0</enable_wind>
        </link>
    </model>

    <physics name='default_physics' default='0' type='ode'>
        <max_step_size>0.01</max_step_size>
        <real_time_factor>1</real_time_factor>
        <real_time_update_rate>100</real_time_update_rate>
    </physics>

    <scene>
        <ambient>0.4 0.4 0.4 1</ambient>
        <background>0.7 0.7 0.7 1</background>
        <shadows>1</shadows>
    </scene>

    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <audio>
        <device>default</device>
    </audio>

    <wind/>

    <spherical_coordinates>
        <surface_model>EARTH_WGS84</surface_model>
        <latitude_deg>0</latitude_deg>
        <longitude_deg>0</longitude_deg>
        <elevation>0</elevation>
        <heading_deg>0</heading_deg>
    </spherical_coordinates>

)";


    const char * footer = R"(
    </world>
</sdf>
)";

    const char * wallTemplate = R"(

        <model name='Wall_%1%'>
            <static>1</static>
            <link name='link'>
                <collision name='Wall_Collision_%1%'>
                    <geometry>
                        <box>
                            <size>%2% %3% %4%</size>
                        </box>
                    </geometry>
                    <pose frame=''>%5% %6% %7% 0 -0 %8%</pose>
                    <max_contacts>10</max_contacts>
                    <surface>
                        <contact>
                            <ode/>
                        </contact>
                        <bounce/>
                        <friction>
                            <torsional>
                                <ode/>
                            </torsional>
                            <ode/>
                        </friction>
                    </surface>
                </collision>
                <visual name='Wall_Visual_%1%'>
                    <pose frame=''>%5% %6% %7% 0 -0 %8%</pose>
                    <geometry>
                        <box>
                           <size>%2% %3% %4%</size>
                        </box>
                    </geometry>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>Gazebo/Blue</name>
                        </script>
                    </material>
                </visual>
                <velocity_decay>
                    <linear>0</linear>
                    <angular>0</angular>
                </velocity_decay>
                <self_collide>0</self_collide>
                <enable_wind>0</enable_wind>
                <kinematic>0</kinematic>
            </link>
        </model>

)";

    const char* pylonTemplate = R"(
        <model name='Construction Cone_%1%'>
            <static>0</static>
            <link name='link'>
                <collision name='collision'>
                    <geometry>
                        <mesh>
                            <scale>1 1 1</scale>
                            <uri>model://construction_cone/meshes/construction_cone.dae</uri>
                        </mesh>
                    </geometry>
                    <max_contacts>10</max_contacts>
                    <surface>
                        <contact>
                            <ode/>
                        </contact>
                        <bounce/>
                        <friction>
                            <torsional>
                                <ode/>
                            </torsional>
                            <ode/>
                        </friction>
                    </surface>
                </collision>
                <visual name='CC_%1%'>
                    <geometry>
                        <mesh>
                        <scale>1 1 1</scale>
                        <uri>model://construction_cone/meshes/construction_cone.dae</uri>
                        </mesh>
                    </geometry>
                </visual>
                <self_collide>0</self_collide>
                <inertial>
                    <inertia>
                        <ixx>1</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>1</iyy>
                        <iyz>0</iyz>
                        <izz>1</izz>
                    </inertia>
                    <mass>1</mass>
                </inertial>
                <enable_wind>0</enable_wind>
                <kinematic>0</kinematic>
            </link>
            <pose frame=''>%2% %3% 0 0 -0 0</pose>
        </model>
)";

    void
    GazeboGenerator::writeHeader( double scale ) {
        auto formattedHeader = boost::format( header ) % worldName % (height * scale * basePlateMultiple)
                % (width * scale * basePlateMultiple);
        gazeboFile << formattedHeader << std::endl;
    }

    void
    GazeboGenerator::writeFooter() {
        gazeboFile << footer << std::endl;
    }

    void
    GazeboGenerator::writeWallToWorldFile( int& wallNum, Polygon poly, double scale ) {
        for( auto const& segment: poly ) {
            Point midp = mid( segment );

            auto wallDesc = boost::format( wallTemplate ) % wallNum
                            % (length( segment ) * scale) % (wallWidth * scale) % (wallHeight * scale)
                            % (midp.first * scale) % (midp.second * scale) % (wallHeight * scale / 2.0)
                            % angle(segment.first, segment.second);
            gazeboFile << wallDesc << std::endl;

            wallNum++;
        }
    }


    void
    GazeboGenerator::writePylonToWorldFile(int pylonNum, double x, double y, double scale) {
        auto pylonDesc = boost::format( pylonTemplate ) % pylonNum % x % y;
        gazeboFile << pylonDesc << std::endl;
    }
}
