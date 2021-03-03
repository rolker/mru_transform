#ifndef LOCAL_SERVICES_H
#define LOCAL_SERVICES_H

#include <ros/ros.h>
#include "geographic_msgs/GeoPoint.h"
#include "geometry_msgs/Point.h"
#include "project11/utils.h"
#include <cmath>

namespace project11
{
    /// Provide the same transformations available as ROS services, but in the local process to reduce overhead.
    class Transformations
    {
    public:
        Transformations(ros::NodeHandle &node):m_origin(NAN,NAN,0.0)
        {
            m_origin_sub = node.subscribe("project11/origin", 1, &Transformations::originCallback, this);
        }

        Transformations():m_origin(NAN,NAN,0.0)
        {
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
                LatLongDegrees p_ll;
                fromMsg(position, p_ll);
                ECEF p_ecef(p_ll);
        
                Point position_map = m_geoReference.toLocal(p_ecef);
                toMsg(position_map, ret);
            }
            return ret;
        }
        
        geographic_msgs::GeoPoint map_to_wgs84(geometry_msgs::Point const &point)
        {
            geographic_msgs::GeoPoint ret;
            if(haveOrigin())
            {
                Point position(point.x, point.y, point.z);
                LatLongDegrees latlon = m_geoReference.toLatLong(position);
                toMsg(latlon, ret);
            }
            return ret;
        }
        
        LatLongDegrees const &origin() const
        {
            return m_origin;
        }

    private:
        void originCallback(const geographic_msgs::GeoPoint::ConstPtr& msg)
        {
            if (m_origin[0] != msg->latitude || m_origin[1] != msg->longitude)
            {
                m_origin[0] = msg->latitude;
                m_origin[1] = msg->longitude;
                m_geoReference = ENUFrame(m_origin);
            }
        }
        
        ros::Subscriber m_origin_sub;
        LatLongDegrees m_origin;
        ENUFrame m_geoReference;
    };
}

#endif
