#ifndef PCLCONTAINER_H
#define PCLCONTAINER_H
#include <tiny_pcl/bitdatamem.h>
#include <tiny_pcl/datatypes.h>
#include <iostream>

namespace TinyPCL
{

//! @class PCLContainer
//! @brief Container for a differential compressed pointclouds
//! Useful for storing or sending pointclouds over a network
class PCLContainer
{
public:
    //! Initialize container
    PCLContainer();

    //! Set data of container, typically for reading
    //! @param data, 64bit aligned data
    //! @param size size of data in bits
    void setData( const std::vector<uint64_t>& data, int64_t size ) {m_data.setData(data,size); }

    //! Get data of container, typically for storing or sending them over a network
    //! @returns raw differential encoded data as 64bit aligned vector
    const std::vector<uint64_t>& getData() const {return m_data.getData();}

    //! Get size of container in bits
    //! @returns size of container in bits
    uint64_t bitSize() const {return m_data.size();}

    //! Set vector/point specification of container
    //! @param spec Specification / Content description
    void setDataSpec( const TinyPCL::DataSpec& spec );

    //! Get vector/point specification of container
    //! @returns Specification / Content description
    const TinyPCL::DataSpec& getDataSpec() const {return m_spec;}

    //! Write vector of pointcloud data
    //! @param pointer to data as signed 64 bit values
    void writeVector(int64_t *data );

    //! Read vector of pointcloud data
    //! @param pointer where to store data as signed 64 bit values
    //! @returns true on success, false in case of end of data
    bool readVector( int64_t* data );

    //! Clear container data, specification is not cleared
    void clear();

    //! Enable debug output, if compiled with debug support
    void setDebug( bool debug ) { m_debug = debug; m_data.setDebug(debug); }

    //! Calculate a multiple sum histograms which answers how many values fit into certain bit values (value < 2^n)
    //! @param data Pointer to data as signed 64 bit values
    //! @returns histogram for each component of vector in the data specification
    std::vector< std::vector<unsigned int> > probeDataHistogram( const std::vector<int64_t> &data );

    //! Calculate different variants from histograms: How many bits/value are needed if a point-component is encoded with a differential N bit encoding
    //! @param hist Sum Histogram calculated by probeDataHistogram
    //! @returns Bits/value for each component for each (N bit) encoding option
    std::vector< std::vector<float> > probeDataBitsPerValue( const std::vector< std::vector<unsigned int> >&hist );

    //! Calls probeDataHistogram and probeDataBitsPerValue and prints the results to a given stream
    void printDataProbe(const std::vector<int64_t>& data, bool printHist = false, bool printBitsPerVal = true, std::ostream &out = std::cout);

    //! Automatically adjust data specification by setting the best N-bit encoding for each point-component
    //! This is done by calling probeDataHistogram and probeDataBitsPerValue and analysing the results
    //! @param data Pointer to data as signed 64 bit values
    void adjustByProbeData(const std::vector<int64_t> &data );

protected:
    //! Precalculated power of 2 values
    std::vector<int64_t> m_maxVal;

    //! Bit storage container
    BitDataMem m_data;

    //! Vector/Point specification
    TinyPCL::DataSpec m_spec;

    //! Last values written, used for differential encoding
    std::vector<int64_t> m_lastValuesWritten;

    //! Flag if first values are written
    bool m_firstValuesWritten;

    //! Last values read, used for differential decoding
    std::vector<int64_t> m_lastValuesRead;

    //! Debug flag if compiled with debug support
    bool m_debug;
};
}

#endif // PCLCONTAINER_H
