#ifndef TEST_PCL_TRANSPORT_H
#define TEST_PCL_TRANSPORT_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class PointcloudTransportTestNode
{
public:
    PointcloudTransportTestNode();
    void loadTestData();
    void pointcloudMsgCallback( const sensor_msgs::PointCloud2ConstPtr& msg );
    void sendTimer( const ros::TimerEvent& e );
    sensor_msgs::PointCloud2 createPclMsg();

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Timer pub_timer_;

    std::vector< std::vector<double> > testdata_;
};

#endif
