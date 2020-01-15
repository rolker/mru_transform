#ifndef LOCAL_SERVICES_H
#define LOCAL_SERVICES_H

#include <ros/ros.h>
#include "geographic_msgs/GeoPoint.h"
#include "geometry_msgs/Point.h"
#include "project11/gz4d_geo.h"
#include <cmath>

namespace project11
{
    /// Provide the same transformations available as ROS services, but in the local process to reduce overhead.
    class Transformations
    {
    public:
        Transformations(ros::NodeHandle &node):m_origin(NAN,NAN,0.0)
        {
            m_origin_sub = node.subscribe("/origin", 1, &Transformations::originCallback, this);
        }
        
        bool haveOrigin() const
        {
            return !isnan(m_origin[0]) && !isnan(m_origin[1]);
        }
        
        geometry_msgs::Point wgs84_to_map(geographic_msgs::GeoPoint const &position)
        {
            geometry_msgs::Point ret;
            if(haveOrigin())
            {
                gz4d::GeoPointLatLong p_ll(position.latitude, position.longitude, position.altitude);
                gz4d::GeoPointECEF p_ecef(p_ll);
        
                gz4d::Point<double> position_map = m_geoReference.toLocal(p_ecef);
                ret.x = position_map[0];
                ret.y = position_map[1];
                ret.z = position_map[2];
            }
            return ret;
        }
        
        geographic_msgs::GeoPoint map_to_wgs84(geometry_msgs::Point const &point)
        {
            geographic_msgs::GeoPoint ret;
            if(haveOrigin())
            {
                gz4d::Point<double> position(point.x, point.y, point.z);
                auto latlon = m_geoReference.toLatLong(position);
                ret.latitude = latlon[0];
                ret.longitude = latlon[1];
                ret.altitude = latlon[2];
            }
            return ret;
        }

    private:
        void originCallback(const geographic_msgs::GeoPoint::ConstPtr& msg)
        {
            if (m_origin[0] != msg->latitude || m_origin[1] != msg->longitude)
            {
                m_origin[0] = msg->latitude;
                m_origin[1] = msg->longitude;
                m_geoReference = gz4d::LocalENU(m_origin);
            }
        }
        
        
        ros::Subscriber m_origin_sub;
        gz4d::GeoPointLatLong m_origin;
        gz4d::LocalENU m_geoReference;
    };
}

#endif
