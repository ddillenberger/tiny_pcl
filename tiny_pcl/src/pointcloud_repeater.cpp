#include "pointcloud_repeater.h"

//-------------------------------------------------------------------------------------------------
PointcloudMessageRepeater::PointcloudMessageRepeater():m_nodeHandle("~")
{
    m_nodeHandle.getParam("srcTopic",m_srcTopic);
    m_nodeHandle.getParam("repeatInterval",m_repeatInterval);
    m_repeatTopic = m_srcTopic + "/repeat";

    m_sourceSub = m_nodeHandle.subscribe(m_srcTopic,5,&PointcloudMessageRepeater::pointcloudMsgCallback,this);
    m_repeatPub = m_nodeHandle.advertise<sensor_msgs::PointCloud2>(m_repeatTopic,5);
    m_repeatTimer = m_nodeHandle.createTimer(ros::Duration(m_repeatInterval),&PointcloudMessageRepeater::repeatTimerCallback,this);
}

//-------------------------------------------------------------------------------------------------
void PointcloudMessageRepeater::pointcloudMsgCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    m_receivedMessages++;

    if( m_receivedMessages == 1 )
        m_repeatCache = *msg;
    else
        appendPoints(m_repeatCache,*msg);
}

//-------------------------------------------------------------------------------------------------
void PointcloudMessageRepeater::repeatTimerCallback(const ros::TimerEvent &e)
{
    if(m_receivedMessages == 0)
        return;

    m_repeatCache.header.stamp = ros::Time::now();
    m_repeatPub.publish(m_repeatCache);
}

//-------------------------------------------------------------------------------------------------
void PointcloudMessageRepeater::appendPoints(sensor_msgs::PointCloud2 &dst, const sensor_msgs::PointCloud2 &src)
{
    const std::size_t oldSize = dst.data.size();
    dst.data.resize( dst.data.size() + src.data.size() );
    memcpy( &dst.data[oldSize], &src.data[0], src.data.size() );

    dst.width += src.width;
    dst.row_step = dst.width * dst.point_step;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_message_repeater");
    new PointcloudMessageRepeater();
    ros::spin();
    return 0;
}
