#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geographic_msgs/GeoPointStamped.h"
#include "geographic_msgs/GeoPoint.h"
#include "marine_msgs/NavEulerStamped.h"
#include "project11/gz4d_geo.h"
#include <geodesy/utm.h>
#include "project11_transformations/LatLongToEarth.h"
#include "project11_transformations/LatLongToMap.h"
#include <nav_msgs/Odometry.h>


double LatOrigin  = 0.0;
double LongOrigin = 0.0;

gz4d::geo::LocalENU<> geoReference;
geodesy::UTMPoint geoReferenceUTM;

tf::Quaternion earth_to_map_rotation;

bool initializedLocalReference = false;

static tf2_ros::StaticTransformBroadcaster *static_broadcaster = nullptr;
static tf2_ros::TransformBroadcaster * broadcaster = nullptr;

ros::Publisher position_map_pub;
ros::Publisher position_utm_pub;
//ros::Publisher position_ecef_pub;
ros::Publisher origin_pub;
ros::Publisher odom_pub;

double heading = 0.0;
ros::Time last_heading_timestamp;
double rotation_speed = 0.0;

gz4d::Point<double> last_position;
ros::Time last_timestamp;

ros::ServiceServer map_service;

ros::NodeHandle *node;

bool ll2map(project11_transformations::LatLongToMap::Request &req, project11_transformations::LatLongToMap::Response &res)
{
    gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> p_ll(req.wgs84.position.latitude,req.wgs84.position.longitude,req.wgs84.position.altitude);
    gz4d::geo::Point<double, gz4d::geo::WGS84::ECEF> p_ecef(p_ll);

    gz4d::Point<double> position = geoReference.toLocal(p_ecef);
    
    res.map.header.frame_id = "map";
    res.map.header.stamp = req.wgs84.header.stamp;
    res.map.point.x = position[0];
    res.map.point.y = position[1];
    res.map.point.z = position[2];
    return true;
}


void initializeLocalReference(const geographic_msgs::GeoPointStamped::ConstPtr& inmsg)
{
    gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> gr(LatOrigin,LongOrigin,0.0);
    geoReference = gz4d::geo::LocalENU<>(gr);
    geographic_msgs::GeoPoint gp;
    gp = inmsg->position;
    geodesy::fromMsg(gp,geoReferenceUTM);
    
    geometry_msgs::TransformStamped static_transformStamped_map_to_utm;

    static_transformStamped_map_to_utm.header.stamp = ros::Time::now();
    static_transformStamped_map_to_utm.header.frame_id = "map";
    static_transformStamped_map_to_utm.child_frame_id = "utm";
    static_transformStamped_map_to_utm.transform.translation.x = -geoReferenceUTM.easting;
    static_transformStamped_map_to_utm.transform.translation.y = -geoReferenceUTM.northing;
    static_transformStamped_map_to_utm.transform.rotation.w = 1.0;
    
    static_broadcaster->sendTransform(static_transformStamped_map_to_utm);

    geometry_msgs::TransformStamped static_transformStamped_earth_to_map;
    static_transformStamped_earth_to_map.header.stamp = ros::Time::now();
    static_transformStamped_earth_to_map.header.frame_id = "earth";
    static_transformStamped_earth_to_map.child_frame_id = "map";
    
    gz4d::geo::Point<double, gz4d::geo::WGS84::ECEF> originECEF(gr);
    static_transformStamped_earth_to_map.transform.translation.x = originECEF[0];
    static_transformStamped_earth_to_map.transform.translation.y = originECEF[1];
    static_transformStamped_earth_to_map.transform.translation.z = originECEF[2];
    
    tf::Quaternion longQuat = tf::createQuaternionFromRPY(0.0,0.0,(LongOrigin+90.0)*M_PI/180.0);
    tf::Quaternion latQuat = tf::createQuaternionFromRPY((90-LatOrigin)*M_PI/180.0,0.0,0.0);
    earth_to_map_rotation = longQuat*latQuat;
    
    quaternionTFToMsg(earth_to_map_rotation,static_transformStamped_earth_to_map.transform.rotation);
    
    
    static_broadcaster->sendTransform(static_transformStamped_earth_to_map);
    
    geometry_msgs::TransformStamped map_to_odom;
    map_to_odom.header.stamp = ros::Time::now();
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    map_to_odom.transform.rotation.w = 1.0;
    static_broadcaster->sendTransform(map_to_odom);

    //todo: read offsets from parameter server
    geometry_msgs::TransformStamped base_link_to_mbes;
    base_link_to_mbes.header.stamp = ros::Time::now();
    base_link_to_mbes.header.frame_id = "base_link";
    base_link_to_mbes.child_frame_id = "mbes";
    base_link_to_mbes.transform.rotation.w = 1.0;

    static_broadcaster->sendTransform(base_link_to_mbes);
    
    map_service = node->advertiseService("wgs84_to_map",ll2map);

    
    initializedLocalReference = true;
}

void positionCallback(const geographic_msgs::GeoPointStamped::ConstPtr& inmsg)
{
    if(!initializedLocalReference)
    {
        LatOrigin = inmsg->position.latitude;
        LongOrigin = inmsg->position.longitude;
        initializeLocalReference(inmsg);
    }
    //double t = inmsg->header.stamp.toSec();
    
    gz4d::Point<double> position = geoReference.toLocal(gz4d::geo::Point<double,gz4d::geo::WGS84::ECEF>(gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon>(inmsg->position.latitude,inmsg->position.longitude,0.0)));
    
    geometry_msgs::TransformStamped odom_to_base_link;
    odom_to_base_link.header.stamp = inmsg->header.stamp;
    odom_to_base_link.header.frame_id = "odom";
    odom_to_base_link.child_frame_id = "base_link";
    odom_to_base_link.transform.translation.x = position[0];
    odom_to_base_link.transform.translation.y = position[1];
    odom_to_base_link.transform.translation.z = position[2];
    auto rq =  tf::createQuaternionFromYaw((90-heading)*M_PI/180.0);
    odom_to_base_link.transform.rotation.w = rq.getW();
    odom_to_base_link.transform.rotation.x = rq.getX();
    odom_to_base_link.transform.rotation.y = rq.getY();
    odom_to_base_link.transform.rotation.z = rq.getZ();
    broadcaster->sendTransform(odom_to_base_link);

    geometry_msgs::PoseStamped ps;
    ps.header.stamp = inmsg->header.stamp;
    ps.header.frame_id = "map";
    ps.pose.position.x = position[0];
    ps.pose.position.y = position[1];
    ps.pose.position.z = position[2];
    ps.pose.orientation.w = rq.getW();
    ps.pose.orientation.x = rq.getX();
    ps.pose.orientation.y = rq.getY();
    ps.pose.orientation.z = rq.getZ();

    position_map_pub.publish(ps);
    
    
//     gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> p_ll(inmsg->position.latitude,inmsg->position.longitude,inmsg->position.altitude);
//     gz4d::geo::Point<double, gz4d::geo::WGS84::ECEF> p_ecef(p_ll);
//     geometry_msgs::PoseStamped ps_ecef;
//     ps_ecef.header.stamp = inmsg->header.stamp;
//     ps_ecef.header.frame_id = "earth";
//     ps_ecef.pose.position.x = p_ecef[0];
//     ps_ecef.pose.position.y = p_ecef[1];
//     ps_ecef.pose.position.z = p_ecef[2];
//     
//     quaternionTFToMsg(earth_to_map_rotation*rq,ps_ecef.pose.orientation);
//     
//     position_ecef_pub.publish(ps_ecef);
    if(last_timestamp.isValid())
    {
        nav_msgs::Odometry odom;
        odom.header.stamp = inmsg->header.stamp;
        odom.header.frame_id = "odom";
        odom.pose.pose = ps.pose;
        odom.child_frame_id = "base_link";
        ros::Duration dt = inmsg->header.stamp - last_timestamp;
        odom.twist.twist.linear.x = (position[0] - last_position[0])/dt.toSec();
        odom.twist.twist.linear.y = (position[1] - last_position[1])/dt.toSec();
        odom.twist.twist.angular.z = rotation_speed;
        odom_pub.publish(odom);
    }
    
    last_position = position;
    last_timestamp = inmsg->header.stamp;

    geodesy::UTMPoint p_utm;
    geodesy::fromMsg(inmsg->position,p_utm);
    ps.header.frame_id = "utm";
    ps.pose.position.x = p_utm.easting;
    ps.pose.position.y = p_utm.northing;
    ps.pose.position.z = p_utm.altitude;
    
    position_utm_pub.publish(ps);
    
}

bool ll2earth(project11_transformations::LatLongToEarth::Request &req, project11_transformations::LatLongToEarth::Response &res)
{
     gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> p_ll(req.wgs84.position.latitude,req.wgs84.position.longitude,req.wgs84.position.altitude);
     gz4d::geo::Point<double, gz4d::geo::WGS84::ECEF> p_ecef(p_ll);
     res.earth.header.frame_id = "earth";
     res.earth.header.stamp = req.wgs84.header.stamp;
     res.earth.point.x = p_ecef[0];
     res.earth.point.y = p_ecef[1];
     res.earth.point.z = p_ecef[2];
     return true;
}




void headingCallback(const marine_msgs::NavEulerStamped::ConstPtr& inmsg)
{
    //double t = inmsg->header.stamp.toSec();
    if(last_heading_timestamp.isValid())
    {
        rotation_speed = ((inmsg->orientation.heading - heading)/(inmsg->header.stamp - last_heading_timestamp).toSec())*M_PI/180.0;
    }
    
    heading = inmsg->orientation.heading;
    last_heading_timestamp = inmsg->header.stamp;
    
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
    node->param<std::string>("/project11/role", role, "unknown");
    std::cerr << "role: " << role << std::endl;
    
    static_broadcaster = new tf2_ros::StaticTransformBroadcaster;
    broadcaster = new tf2_ros::TransformBroadcaster;

    ros::Subscriber psub;
    ros::Subscriber hsub;

    if(role == "operator" || role == "observer")
    {
        psub = node->subscribe("/udp/position",10,positionCallback);
        hsub = node->subscribe("/udp/heading",10,headingCallback);
    }
    else
    {    
        psub = node->subscribe("/position",10,positionCallback);
        hsub = node->subscribe("/heading",10,headingCallback);
    }
    
    position_map_pub = node->advertise<geometry_msgs::PoseStamped>("/position_map",10);
    position_utm_pub = node->advertise<geometry_msgs::PoseStamped>("/position_utm",10);
    //position_ecef_pub = n.advertise<geometry_msgs::PoseStamped>("/position_ecef",10);
    origin_pub = node->advertise<geographic_msgs::GeoPoint>("/origin",1);
    odom_pub = node->advertise<nav_msgs::Odometry>("odom",50);

    ros::WallTimer originTimer = node->createWallTimer(ros::WallDuration(1.0),originCallback);
    
    ros::ServiceServer service = node->advertiseService("wgs84_to_earth",ll2earth);

    ros::spin();
    return 0;
}
