#ifndef PCL_MSG_REPEATER_H
#define PCL_MSG_REPEATER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class PointcloudMessageRepeater
{
public:
    PointcloudMessageRepeater();
    void pointcloudMsgCallback( const sensor_msgs::PointCloud2ConstPtr& msg );
    void repeatTimerCallback( const ros::TimerEvent& e );

private:
    void appendPoints( sensor_msgs::PointCloud2& dst, const sensor_msgs::PointCloud2& src );

    ros::NodeHandle m_nodeHandle;

    ros::Subscriber m_sourceSub;
    ros::Publisher m_repeatPub;
    ros::Timer m_repeatTimer;

    std::string m_srcTopic;
    std::string m_repeatTopic;

    unsigned int m_receivedMessages;
    double m_repeatInterval;
    sensor_msgs::PointCloud2 m_repeatCache;
};

#endif // PCL_MSG_REPEATER_H
