#include "test_pcl_transport.h"
#include <iostream>
#include <fstream>

//-----------------------------------------------------------------------------
std::vector<std::string> str_explode(const std::string& text, const std::string& separators)
{
        std::vector<std::string> words;
        size_t n     = text.length ();
        size_t start = text.find_first_not_of (separators);

        while (start < n)
        {
                size_t stop = text.find_first_of (separators, start);
                if (stop > n) stop = n;
                words.push_back (text.substr (start, stop-start));
                start = text.find_first_not_of (separators, stop+1);
        };
        return words;
}

//-----------------------------------------------------------------------------
std::vector<double> loadTestFile(std::string filename)
{
    std::ifstream xyzstr(filename.c_str());

    std::vector<double> xyzdata;
    while (xyzstr.good() &&!xyzstr.eof())
    {
        char buf[1024];
        xyzstr.getline(buf,sizeof(buf));
        std::vector<std::string> xyzStr=str_explode(buf," ");
        if(xyzStr.size()<3)
            continue;
        for( int i=0; i<3; i++)
        {
            double v = atof(xyzStr[i].c_str());
            xyzdata.push_back(v);
        }
    }

    return xyzdata;
}

//-----------------------------------------------------------------------------
PointcloudTransportTestNode::PointcloudTransportTestNode():nh_("~")
{
    loadTestData();
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sample_points",10);
    sub_ = nh_.subscribe("/sample_points/compressed_transport",10,&PointcloudTransportTestNode::pointcloudMsgCallback, this);
    pub_timer_ = nh_.createTimer(ros::Duration(1.0), &PointcloudTransportTestNode::sendTimer, this );
}

//-----------------------------------------------------------------------------
void PointcloudTransportTestNode::loadTestData()
{
    for( unsigned int n=0; n<5; n++ )
    {
        char xyzfilename[512];
        sprintf(xyzfilename, "velodyne-pcl-%u.xyz", n);

        std::cout << "Loading " << xyzfilename << std::endl;
        std::vector<double> xyzdata = loadTestFile(xyzfilename);
        testdata_.push_back(xyzdata);
    }
}

//-----------------------------------------------------------------------------
void PointcloudTransportTestNode::pointcloudMsgCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    std::cout << "received msg" << std::endl;
    std::vector<float> msgdata;
    std::vector<double>& testdata = testdata_[0];

    msgdata.resize(msg->data.size()/sizeof(float));
    float* pFloat = (float*) &msg->data[0];

    for( std::size_t i=0; i<msgdata.size(); i++ )
        msgdata[i] = pFloat[i];

    if( msgdata.size() != testdata.size() )
    {
        std::cout << "point counts don't match" << std::endl;
        return;
    }

    std::size_t mismatches = 0;
    std::size_t zerocount = 0;
    for( std::size_t i=0; i<msgdata.size(); i++ )
    {
        double d = msgdata[i]-testdata[i];
        if( d>0.002 )
            mismatches++;
        if(msgdata[i] == 0)
            zerocount++;
    }
    std::cout << "no of mismatches: " << mismatches << std::endl;
    std::cout << "no of zeros: " << zerocount << std::endl;
}

//-----------------------------------------------------------------------------
void PointcloudTransportTestNode::sendTimer(const ros::TimerEvent &e)
{
    std::vector<double>& testdata = testdata_[0];
    const std::size_t pointcount = testdata.size()/3;

    sensor_msgs::PointCloud2 msg = createPclMsg();
    msg.width = pointcount;
    msg.row_step = pointcount*12;

    msg.data.resize(12*pointcount);
    float* pfloat = (float*) &msg.data[0];

    for( std::size_t i=0; i<pointcount*3; i++ )
        pfloat[i] = testdata[i];

    msg.header.stamp = ros::Time::now();
    pub_.publish(msg);
    std::cout << "sending msg" << std::endl;
}

//-----------------------------------------------------------------------------
sensor_msgs::PointCloud2 PointcloudTransportTestNode::createPclMsg()
{
    sensor_msgs::PointCloud2 msg;
    sensor_msgs::PointField pf;
    pf.datatype=pf.FLOAT32;
    pf.count=1;

    pf.name="x";
    pf.offset=0;
    msg.fields.push_back(pf);

    pf.name="y";
    pf.offset=4;
    msg.fields.push_back(pf);

    pf.name="z";
    pf.offset=8;
    msg.fields.push_back(pf);

    msg.height = 1;
    msg.is_bigendian=false;
    msg.is_dense=true;
    msg.point_step=12;

    return msg;
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "PointcloudTransportTestNode", ros::init_options::AnonymousName);
    new PointcloudTransportTestNode();
    ros::spin();
    return 0;
}
