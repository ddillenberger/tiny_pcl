#ifndef POINTCLOUD_TRANSPORT_NODE_H
#define POINTCLOUD_TRANSPORT_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tiny_pcl/CPointCloud2.h>
#include <tiny_pcl/tinypcl.h>

//! @class PointcloudTransportNode
//! @brief Compressed pointcloud transports over networks
//! Node 1 reads sensor_msgs/Pointcloud2 messages from topic A, sends
//! compressed version over topic B, Node 2 decompresses data from topic
//! B and republished uncompressed form on topic C
class PointcloudTransportNode
{
public:
    enum CompressorMode
    {
        UNDEFINED,
        COMPRESS,
        DECOMPRESS
    };

    //! Initialize, subscribe, advertise and read parameters
    PointcloudTransportNode();

    //! Callback for source topic
    void srcTopicCallback ( const sensor_msgs::PointCloud2ConstPtr& msg );

    //! Callback for compressed transport topic
    void transTopicCallback (const tiny_pcl::CPointCloud2ConstPtr& cmsg );

    //! Check if a subscriber is listening, only do work if a subscriber is connected
    void checkSubscriber(const ros::TimerEvent& e);

    CompressorMode getMode() const  {return mode_;}

private:
    //! Parse channel parameter
    void parseChannelsParam();

    //! Parse scale factor parameter
    void parseScaleFactorsParam();

    //! Copy container setup
    void copyContainerSetup(std::vector<tiny_pcl::CPointField>& dst);

    //! Standard PCL container setup 3xfloat: X,Y,Z
    void containerSetup();

    //! Container setup from CPointCloud2 msg
    void containerSetup(const tiny_pcl::CPointCloud2ConstPtr& msg );

    //! ZLIB compression of buffer
    const std::vector<unsigned char> &compressBuffer(const std::vector<uint64_t> &data );

    //! ZLIB decompression of buffer
    void decompressBuffer(const std::vector<uint8_t> &src, std::vector<uint64_t>& dst , std::size_t dst_size);

    //! compression Node or decompression node?
    CompressorMode mode_;

    //! ZLIB compression enabled?
    bool zlib_compression_;

    //! scale factor of selected channels
    std::vector<double> scale_factor_;

    //! scale factor of selected channels
    std::set<std::size_t> selected_channels_;

    //! Private node handle
    ros::NodeHandle nh_;

    //! source/original topic
    std::string src_topic_;

    //! transport topic
    std::string transport_topic_;

    //! Destinantion/Republishing topic
    std::string dst_topic_;

    //! Dst topic suffix
    std::string dst_topic_suffix_;

    //! Subscriber src topic
    ros::Subscriber src_topic_sub_;

    //! Publisher transport topic
    ros::Publisher transport_topic_pub_;

    //! Subscriber src topic
    ros::Subscriber transport_topic_sub_;

    //! Publisher destination topic
    ros::Publisher dst_topic_pub_;

    //! Timer for subscription check
    ros::Timer check_subscriber_tmr_;

    //! Differential encoded pointcloud data container
    TinyPCL::PCLContainer container_;

    //! Channel data specification of pointcloud
    TinyPCL::DataSpec dataspec_;

    //! Pointcloud counter for auto data probe
    std::size_t pclCounter;

    //! Temporary buffer for compression
    std::vector<unsigned char> compression_buffer_;

    //! Temporary buffer for pointdata
    std::vector<int64_t> pointdata_buffer_;
};

#endif
