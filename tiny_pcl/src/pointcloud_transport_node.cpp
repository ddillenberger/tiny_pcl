#include "pointcloud_transport_node.h"
#include <zlib.h>
#include "stringutil.h"
#include "readwriteutil.h"
#include "stopwatch.h"

//-----------------------------------------------------------------------------
PointcloudTransportNode::PointcloudTransportNode():nh_("~")
{
    pclCounter=0;
    nh_.param("src_topic",src_topic_,std::string(""));
    std::string mode;
    nh_.param("mode",mode,std::string(""));
    nh_.param("use_zlib",zlib_compression_,true);
    nh_.param("dst_topic_suffix",dst_topic_suffix_,std::string(""));

    parseChannelsParam();
    parseScaleFactorsParam();
    containerSetup();

    if(mode == "compress") {
        mode_=COMPRESS;
        std::cout << "Compression mode selected" << std::endl;
    } else if( mode == "decompress" ) {
        mode_=DECOMPRESS;
        std::cout << "Decompression mode selected" << std::endl;
    } else {
        std::cerr << "No valid mode (" << mode << ") selected: Exiting." << std::endl;
        return;
    }

    if( src_topic_=="" )
    {
        mode_=UNDEFINED;
        std::cerr << "No topic set: Exiting." << std::endl;
        return;
    }

    transport_topic_ = src_topic_ + "/transport";
    dst_topic_ = src_topic_ + "/compressed_transport";
    if(dst_topic_suffix_!="")
        dst_topic_ += "_" + dst_topic_suffix_;

    if(mode_==COMPRESS)
    {
        src_topic_sub_ = nh_.subscribe( src_topic_, 2 ,&PointcloudTransportNode::srcTopicCallback, this );
        transport_topic_pub_ = nh_.advertise<tiny_pcl::CPointCloud2>( transport_topic_, 1 );
    }
    if(mode_==DECOMPRESS)
    {
        transport_topic_sub_ = nh_.subscribe(transport_topic_, 2, &PointcloudTransportNode::transTopicCallback, this );
        dst_topic_pub_ = nh_.advertise<sensor_msgs::PointCloud2>( dst_topic_, 1 );
    }
}

//-----------------------------------------------------------------------------
void PointcloudTransportNode::srcTopicCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if( transport_topic_pub_.getNumSubscribers() == 0 )
        return;

    StopWatch totalSW(true);
    StopWatch taskSW;

    container_.clear();
    pointdata_buffer_.clear();

    // Iterate over all points
    const unsigned int pointstep=msg->point_step;
    const unsigned int totalPointDataSize=msg->width*msg->height*msg->point_step;
    pointdata_buffer_.reserve(msg->width*msg->height*selected_channels_.size());
    const std::vector<unsigned char>& pointdata = msg->data;

    taskSW.start();
    for( unsigned int i=0; i<totalPointDataSize; i+=pointstep )
    {
        std::size_t selChRead = 0;
        for( std::size_t j=0; j<msg->fields.size(); j++ )
        {
            if( selected_channels_.count(j) == 0 )
                continue;
           int64_t v = readAndScaleType(&pointdata[i+msg->fields[j].offset],scale_factor_[selChRead],msg->fields[j].datatype);
           pointdata_buffer_.push_back(v);
           selChRead++;
        }
    }
    std::cout << "Reading points from Pointcloud2 msg took " << taskSW.getTimeSum() << " ms" << std::endl;

    if(pclCounter%10 == 0)
    {
        taskSW.resetStart();
        container_.adjustByProbeData(pointdata_buffer_);
        std::cout << "adjustByProbeData took " << taskSW.getTimeSum() << " ms" << std::endl;
    }
    pclCounter++;

    taskSW.resetStart();
    const std::size_t channelCount = selected_channels_.size();
    const std::size_t pointcount = pointdata_buffer_.size()/channelCount;
    for( std::size_t i=0; i<pointcount; i++ )
        container_.writeVector(&pointdata_buffer_[i*channelCount]);
    std::cout << "Differential encoding of points took " << taskSW.getTimeSum() << " ms" << std::endl;


    std::size_t bit_size = container_.bitSize();
    const std::vector<uint64_t>& de_data = container_.getData();
    std::cout << "Differential encoded size " << (de_data.size()*8/1024) << " KB" << std::endl;
    if( zlib_compression_ )
    {
        taskSW.resetStart();
        const std::vector<uint8_t> cde_data = compressBuffer(de_data);
        std::cout << "ZLib compression took " << taskSW.getTimeSum() << " ms" << std::endl;
        std::cout << "ZLib compressed size " << (cde_data.size()/1024) << " KB" << std::endl;
    }

    // Prepare Message
    tiny_pcl::CPointCloud2 cmsg;
    cmsg.header.frame_id=msg->header.frame_id;
    cmsg.height=msg->height;
    cmsg.width=msg->width;

    for( std::size_t i : selected_channels_ )
        cmsg.fields.push_back(msg->fields[i]);
    copyContainerSetup(cmsg.cfields);

    cmsg.is_dense=msg->is_dense;
    cmsg.zlib_compressed = zlib_compression_;
    cmsg.bit_size = bit_size;
    cmsg.aligned_data_size = de_data.size()*sizeof(uint64_t);

    if( zlib_compression_ )
    {
        const std::vector<uint8_t> cde_data = compressBuffer(de_data);
        cmsg.data.resize(cde_data.size());
        memcpy(&cmsg.data[0],&cde_data[0],cde_data.size());
    }
    else
    {
        cmsg.data.resize(de_data.size()*sizeof(uint64_t));
        memcpy(&cmsg.data[0],&de_data[0],cmsg.data.size());
    }

    int pointstep_new = 0;
    for( std::size_t i=0; i<msg->fields.size(); i++ )
        if(selected_channels_.count(i) != 0)
            pointstep_new += sizeofPCLDatatype(msg->fields[i].datatype);
    std::cout << "Compressed points from " << (pointstep_new*pointcount/1024) << " KB to " << (cmsg.data.size()/1024) << " KB (ratio " << ((cmsg.data.size())/float(pointstep_new*pointcount)*100.0f) << " %)" << std::endl;


    transport_topic_pub_.publish(cmsg);
    std::cout << "Total compression time " << totalSW.getTimeSum() << " ms" << std::endl;
}

//-----------------------------------------------------------------------------
void PointcloudTransportNode::transTopicCallback(const tiny_pcl::CPointCloud2ConstPtr &cmsg)
{
    if( dst_topic_pub_.getNumSubscribers()==0 )
    {
        transport_topic_sub_.shutdown();
        check_subscriber_tmr_ = nh_.createTimer(ros::Duration(0.1), &PointcloudTransportNode::checkSubscriber, this );
        return;
    }

    StopWatch totalSW(true);
    sensor_msgs::PointCloud2 dstmsg;
    std::vector<uint64_t> de_data;
    if( cmsg->zlib_compressed )
        decompressBuffer(cmsg->data,de_data,cmsg->aligned_data_size);
    else
    {
        de_data.resize(cmsg->aligned_data_size/8);
        memcpy( &de_data[0], &cmsg->data[0], cmsg->aligned_data_size );
    }
    container_.clear();
    containerSetup(cmsg);
    container_.setData(de_data,cmsg->bit_size);

    std::size_t pointstep = 0;
    for( sensor_msgs::PointField pf: cmsg->fields )
        pointstep += sizeofPCLDatatype(pf.datatype);

    dstmsg.data.resize(cmsg->width*cmsg->height*pointstep);
    std::vector<int64_t> buf;
    buf.resize(selected_channels_.size());

    for( std::size_t i=0; i<dstmsg.data.size(); i+=pointstep )
    {
        if(! container_.readVector(&buf[0]) )
            std::cout << "Something went wrong :( less data than exspected" << std::endl;

        for( std::size_t j=0; j<cmsg->fields.size(); j++ )
            writeAndScaleType(&dstmsg.data[i+cmsg->fields[j].offset],buf[j],scale_factor_[j],cmsg->fields[j].datatype);
    }

    dstmsg.header.frame_id=cmsg->header.frame_id;
    dstmsg.height=cmsg->height;
    dstmsg.width=cmsg->width;
    dstmsg.fields=cmsg->fields;
    dstmsg.is_bigendian=false;
    dstmsg.point_step=pointstep;
    dstmsg.row_step=dstmsg.height*dstmsg.width*dstmsg.point_step;
    dstmsg.is_dense=cmsg->is_dense;

    dst_topic_pub_.publish(dstmsg);
    std::cout << "Total decompression time " << totalSW.getTimeSum() << " ms" << std::endl;
}

//-----------------------------------------------------------------------------
const std::vector<unsigned char>& PointcloudTransportNode::compressBuffer(const std::vector<uint64_t> &data)
{
    size_t buf_size=int(1.01*data.size()*sizeof(uint64_t)+1024);
    if( compression_buffer_.size()<buf_size )
        compression_buffer_.resize(size_t(buf_size));
    int r=compress((unsigned char*)&compression_buffer_[0],&buf_size,(const unsigned char*)&data[0],data.size()*sizeof(uint64_t));
    if( r!=Z_OK )
        std::cerr << "ERROR while Z compression!" << std::endl;
    compression_buffer_.resize(buf_size);
    return compression_buffer_;
}

//-----------------------------------------------------------------------------
void PointcloudTransportNode::decompressBuffer(const std::vector<uint8_t>& src, std::vector<uint64_t> &dst, std::size_t dst_size)
{
    // Decompress data via zlib
    unsigned long int buf_size = dst_size;
    dst.resize(size_t(dst_size/sizeof(uint64_t)));
    int r=uncompress((unsigned char*) &dst[0],&buf_size,&src[0],src.size());
    if( r!=Z_OK || buf_size != dst_size )
    {
        std::cerr << "ERROR: Couldn't Z decompress." << std::endl;
        return;
    }
}

//-----------------------------------------------------------------------------
void PointcloudTransportNode::checkSubscriber(const ros::TimerEvent &e)
{
    if( dst_topic_pub_.getNumSubscribers()>0 )
    {
        transport_topic_sub_ = nh_.subscribe(transport_topic_, 2, &PointcloudTransportNode::transTopicCallback, this );
        check_subscriber_tmr_.stop();
    }
}

//-----------------------------------------------------------------------------
void PointcloudTransportNode::containerSetup()
{
    TinyPCL::DataSpec ds;
    TinyPCL::DiffTypeSpec dts;
    dts.absValBits=32;
    dts.dstBits=16;
    dts.otype=TinyPCL::FLOAT32;
    dts.srcBits=32;
    for( std::size_t i=0; i<selected_channels_.size(); i++ )
        ds.spec.push_back(dts);
    container_.setDataSpec(ds);
}

//-----------------------------------------------------------------------------
void PointcloudTransportNode::containerSetup(const tiny_pcl::CPointCloud2ConstPtr& msg )
{
    TinyPCL::DataSpec ds;
    for( std::size_t i=0; i<msg->cfields.size(); i++ )
    {
        TinyPCL::DiffTypeSpec dts;
        //dts.otype = (unsigned char) (msg->cfields[i].oType);
        dts.absValBits = msg->cfields[i].absValBits;
        dts.srcBits = msg->cfields[i].srcBits;
        dts.dstBits = msg->cfields[i].dstBits;
        ds.spec.push_back(dts);
    }
    container_.setDataSpec(ds);
}

//-----------------------------------------------------------------------------
void PointcloudTransportNode::parseChannelsParam()
{
    using std::string;
    string p;
    nh_.param("channels",p,string("0 1 2"));
    std::vector<string> selCh = str_explode(p," ");
    for( string s : selCh )
        selected_channels_.insert(std::stoi(s));
}

//-----------------------------------------------------------------------------
void PointcloudTransportNode::parseScaleFactorsParam()
{
    using std::string;
    string p;
    nh_.param("selected_channels",p,string("500 500 500"));
    std::vector<string> selCh = str_explode(p," ");
    for( string s : selCh )
        scale_factor_.push_back(std::stoi(s));
    while(scale_factor_.size()<selected_channels_.size())
        scale_factor_.push_back(1);
}

//-----------------------------------------------------------------------------
void PointcloudTransportNode::copyContainerSetup(std::vector<tiny_pcl::CPointField> &dst)
{
    const std::vector<TinyPCL::DiffTypeSpec>& ds = container_.getDataSpec().spec;
    for( TinyPCL::DiffTypeSpec dts : ds )
    {
        tiny_pcl::CPointField cpf;
        cpf.absValBits = dts.absValBits;
        cpf.dstBits = dts.dstBits;
        cpf.srcBits = dts.srcBits;
        cpf.oType = dts.otype;
        dst.push_back(cpf);
    }
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "PointcloudTransportNode", ros::init_options::AnonymousName);
    PointcloudTransportNode* ptn = new PointcloudTransportNode();
    if( ptn->getMode() != PointcloudTransportNode::UNDEFINED )
      ros::spin();
    else
      return -1;
    return 0;
}
