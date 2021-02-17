#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include "geographic_msgs/GeoPointStamped.h"
#include "geographic_msgs/GeoPoint.h"
#include "marine_msgs/NavEulerStamped.h"
#include "project11/gz4d_geo.h"
#include "project11_transformations/LatLongToEarth.h"
#include "project11_transformations/LatLongToMap.h"
#include "project11_transformations/EarthToLatLong.h"
#include "project11_transformations/MapToLatLong.h"
#include <sensor_msgs/NavSatFix.h>

double LatOrigin  = 0.0;
double LongOrigin = 0.0;

gz4d::LocalENU geoReference;

tf2::Quaternion earth_to_map_rotation;

bool initializedLocalReference = false;

static tf2_ros::TransformBroadcaster * broadcaster = nullptr;

ros::Publisher position_map_pub;
ros::Publisher origin_pub;

geographic_msgs::GeoPointStamped last_gps_position;
marine_msgs::NavEulerStamped last_heading;

sensor_msgs::NavSatFix last_posmv_position;
marine_msgs::NavEulerStamped last_posmv_orientation;

ros::ServiceServer wgs84_to_map_service;
ros::ServiceServer map_to_wgs84_service;

ros::NodeHandle *node;

std::string base_link_prefix = "";

bool ll2map(project11_transformations::LatLongToMap::Request &req, project11_transformations::LatLongToMap::Response &res)
{
    gz4d::GeoPointLatLong p_ll(req.wgs84.position.latitude,req.wgs84.position.longitude,req.wgs84.position.altitude);
    gz4d::GeoPointECEF p_ecef(p_ll);
    
    gz4d::Point<double> position = geoReference.toLocal(p_ecef);
    
    res.map.header.frame_id = "map";
    res.map.header.stamp = req.wgs84.header.stamp;
    res.map.point.x = position[0];
    res.map.point.y = position[1];
    res.map.point.z = position[2];
    return true;
}

bool map2ll(project11_transformations::MapToLatLong::Request &req, project11_transformations::MapToLatLong::Response &res)
{
    gz4d::Point<double> position(req.map.point.x,req.map.point.y,req.map.point.z);
    auto latlon = geoReference.toLatLong(position);
    res.wgs84.position.latitude = latlon[0];
    res.wgs84.position.longitude = latlon[1];
    res.wgs84.position.altitude = latlon[2];
    
    res.wgs84.header.frame_id = "wgs84";
    res.wgs84.header.stamp = req.map.header.stamp;
    return true;
}

bool ll2earth(project11_transformations::LatLongToEarth::Request &req, project11_transformations::LatLongToEarth::Response &res)
{
    gz4d::GeoPointLatLong p_ll(req.wgs84.position.latitude,req.wgs84.position.longitude,req.wgs84.position.altitude);
    gz4d::GeoPointECEF p_ecef(p_ll);
    res.earth.header.frame_id = "earth";
    res.earth.header.stamp = req.wgs84.header.stamp;
    res.earth.point.x = p_ecef[0];
    res.earth.point.y = p_ecef[1];
    res.earth.point.z = p_ecef[2];
    return true;
}

bool earth2ll(project11_transformations::EarthToLatLong::Request &req, project11_transformations::EarthToLatLong::Response &res)
{
    gz4d::GeoPointECEF p_ecef(req.earth.point.x,req.earth.point.y,req.earth.point.z);
    gz4d::GeoPointLatLong p_ll(p_ecef);
    
    res.wgs84.header.frame_id = "wgs84";
    res.wgs84.header.stamp = req.earth.header.stamp;
    
    res.wgs84.position.latitude = p_ll[0];
    res.wgs84.position.longitude = p_ll[1];
    res.wgs84.position.altitude = p_ll[2];
    return true;

}

void initializeLocalReference(ros::Time stamp)
{
    gz4d::GeoPointLatLong gr(LatOrigin,LongOrigin,0.0);
    geoReference = gz4d::LocalENU(gr);

    geometry_msgs::TransformStamped transformStamped_earth_to_map;
    transformStamped_earth_to_map.header.stamp = stamp;
    transformStamped_earth_to_map.header.frame_id = "earth";
    transformStamped_earth_to_map.child_frame_id = "map";
    
    gz4d::GeoPointECEF originECEF(gr);
    transformStamped_earth_to_map.transform.translation.x = originECEF[0];
    transformStamped_earth_to_map.transform.translation.y = originECEF[1];
    transformStamped_earth_to_map.transform.translation.z = originECEF[2];
    
    tf2::Quaternion longQuat;
    longQuat.setRPY(0.0,0.0,(LongOrigin+90.0)*M_PI/180.0);
    tf2::Quaternion latQuat;
    latQuat.setRPY((90-LatOrigin)*M_PI/180.0,0.0,0.0);
    earth_to_map_rotation = longQuat*latQuat;
    
    transformStamped_earth_to_map.transform.rotation = tf2::toMsg(earth_to_map_rotation);
    
    broadcaster->sendTransform(transformStamped_earth_to_map);
    
    
    wgs84_to_map_service = node->advertiseService("wgs84_to_map",ll2map);
    map_to_wgs84_service = node->advertiseService("map_to_wgs84",map2ll);
    
    initializedLocalReference = true;
}

marine_msgs::NavEulerStamped const & getLatestOrientation()
{
    if(last_heading.header.stamp-last_posmv_orientation.header.stamp > ros::Duration(0.5))
        return last_heading;
    return last_posmv_orientation;
}

void updatePosition()
{
    double latitude, longitude, altitude;
    ros::Time stamp;
    
    if(last_gps_position.header.stamp-last_posmv_position.header.stamp > ros::Duration(0.5))
    {
        latitude = last_gps_position.position.latitude;
        longitude = last_gps_position.position.longitude;
        altitude = last_gps_position.position.altitude;
        stamp = last_gps_position.header.stamp;
    }
    else
    {
        latitude = last_posmv_position.latitude;
        longitude = last_posmv_position.longitude;
        altitude = last_posmv_position.altitude;
        stamp = last_posmv_position.header.stamp;
    }
    
    if(!initializedLocalReference)
    {
        LatOrigin = latitude;
        LongOrigin = longitude;
        initializeLocalReference(stamp);
    }
    
    gz4d::Point<double> position = geoReference.toLocal(gz4d::GeoPointECEF(gz4d::GeoPointLatLong(latitude,longitude,altitude)));
    
    geometry_msgs::TransformStamped map_to_north_up_base_link;
    map_to_north_up_base_link.header.stamp = stamp;
    map_to_north_up_base_link.header.frame_id = "map";
    map_to_north_up_base_link.child_frame_id = base_link_prefix+"north_up_base_link";
    map_to_north_up_base_link.transform.translation.x = position[0];
    map_to_north_up_base_link.transform.translation.y = position[1];
    map_to_north_up_base_link.transform.translation.z = position[2];
    map_to_north_up_base_link.transform.rotation.w = 1.0;
    broadcaster->sendTransform(map_to_north_up_base_link);


    const marine_msgs::NavEulerStamped &orientation = getLatestOrientation();
    ROS_DEBUG_STREAM("orientation (heading degrees:) " << orientation.orientation.heading);
    tf2::Quaternion rq;
    rq.setRPY(0.0,0.0,(90-orientation.orientation.heading)*M_PI/180.0);
    //rq.setEuler((90-orientation.orientation.heading)*M_PI/180.0,0.0,0.0);
    ROS_DEBUG_STREAM("quaternion: " << rq.getAngle());

    geometry_msgs::PoseStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = "map";
    ps.pose.position.x = position[0];
    ps.pose.position.y = position[1];
    ps.pose.position.z = position[2];
    ps.pose.orientation.w = rq.getW();
    ps.pose.orientation.x = rq.getX();
    ps.pose.orientation.y = rq.getY();
    ps.pose.orientation.z = rq.getZ();

    position_map_pub.publish(ps);
}

void updateOrientation()
{
    const marine_msgs::NavEulerStamped &orientation = getLatestOrientation();
    
    geometry_msgs::TransformStamped north_up_base_link_to_level_base_link;
    north_up_base_link_to_level_base_link.header.stamp = orientation.header.stamp;
    north_up_base_link_to_level_base_link.header.frame_id = base_link_prefix+"north_up_base_link";
    north_up_base_link_to_level_base_link.child_frame_id = base_link_prefix+"level_base_link";
    tf2::Quaternion heading_quat;
    heading_quat.setRPY(0.0,0.0,(90-orientation.orientation.heading)*M_PI/180.0);
    north_up_base_link_to_level_base_link.transform.rotation = tf2::toMsg(heading_quat);
    broadcaster->sendTransform(north_up_base_link_to_level_base_link);
    
    geometry_msgs::TransformStamped level_base_link_to_pitched_base_link;
    level_base_link_to_pitched_base_link.header.stamp = orientation.header.stamp;
    level_base_link_to_pitched_base_link.header.frame_id = base_link_prefix+"level_base_link";
    level_base_link_to_pitched_base_link.child_frame_id = base_link_prefix+"pitched_base_link";
    tf2::Quaternion pitch_quat;
    pitch_quat.setRPY(0.0,-orientation.orientation.pitch*M_PI/180.0,0.0);
    level_base_link_to_pitched_base_link.transform.rotation = tf2::toMsg(pitch_quat);
    broadcaster->sendTransform(level_base_link_to_pitched_base_link);
    
    geometry_msgs::TransformStamped pitched_base_link_to_base_link;
    pitched_base_link_to_base_link.header.stamp = orientation.header.stamp;
    pitched_base_link_to_base_link.header.frame_id = base_link_prefix+"pitched_base_link";
    pitched_base_link_to_base_link.child_frame_id = base_link_prefix+"base_link";
    tf2::Quaternion roll_quat;
    roll_quat.setRPY(orientation.orientation.roll*M_PI/180.0,0.0,0.0);
    pitched_base_link_to_base_link.transform.rotation = tf2::toMsg(roll_quat);
    broadcaster->sendTransform(pitched_base_link_to_base_link);
}

void posmvPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& inmsg)
{
    last_posmv_position = *inmsg;
    updatePosition();
}

void posmvOrientationCallback(const marine_msgs::NavEulerStamped::ConstPtr & inmsg)
{
    last_posmv_orientation = *inmsg;
    updateOrientation();
}

void positionCallback(const geographic_msgs::GeoPointStamped::ConstPtr& inmsg)
{
    last_gps_position = *inmsg;
    if(last_gps_position.header.stamp-last_posmv_position.header.stamp > ros::Duration(0.5))
        updatePosition();
}


void headingCallback(const marine_msgs::NavEulerStamped::ConstPtr& inmsg)
{
    last_heading = *inmsg;
    if(last_heading.header.stamp - last_posmv_orientation.header.stamp > ros::Duration(0.5))
        updateOrientation();
}

void originCallback(const ros::WallTimerEvent& event)
{
    if(initializedLocalReference)
    {
        geographic_msgs::GeoPoint gp;
        gp.latitude = LatOrigin;
        gp.longitude = LongOrigin;
        origin_pub.publish(gp);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "project11_transformation_node");
    
    node = new ros::NodeHandle();
    
    std::string role;
    node->param<std::string>("project11/role", role, "unknown");
    std::cerr << "role: " << role << std::endl;

    node->param<std::string>("project11/base_link_prefix", base_link_prefix, "");
    std::cerr << "base_link_prefix: " << base_link_prefix << std::endl;
    
    broadcaster = new tf2_ros::TransformBroadcaster;

    ros::Subscriber psub;
    ros::Subscriber hsub;
    ros::Subscriber posmv_position_sub;
    ros::Subscriber posmv_orientation_sub;

    if(role == "operator" || role == "observer")
    {
        psub = node->subscribe("position",10,positionCallback);
        hsub = node->subscribe("heading",10,headingCallback);
        posmv_position_sub = node->subscribe("posmv/position",10,posmvPositionCallback);
        posmv_orientation_sub = node->subscribe("posmv/orientation",10,posmvOrientationCallback);
    }
    else
    {    
        psub = node->subscribe("position",10,positionCallback);
        hsub = node->subscribe("heading",10,headingCallback);
        posmv_position_sub = node->subscribe("posmv/position",10,posmvPositionCallback);
        posmv_orientation_sub = node->subscribe("posmv/orientation",10,posmvOrientationCallback);
    }
    
    position_map_pub = node->advertise<geometry_msgs::PoseStamped>("position_map",10);
    origin_pub = node->advertise<geographic_msgs::GeoPoint>("project11/origin",1,true);

    ros::WallTimer originTimer = node->createWallTimer(ros::WallDuration(1.0),originCallback);
    
    ros::ServiceServer wgs84_to_earth_service = node->advertiseService("wgs84_to_earth",ll2earth);
    ros::ServiceServer earth_to_wgs84_service = node->advertiseService("earth_to_wgs84",earth2ll);

    ros::spin();
    return 0;
}
