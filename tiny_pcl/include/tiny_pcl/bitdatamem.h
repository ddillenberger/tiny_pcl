#ifndef BITDATAMEM_H
#define BITDATAMEM_H
#include <vector>
#include <cstdint>

namespace TinyPCL
{

//! @class BitDataMem
//! @brief sequential storage container which
//!        can store values with arbitrary bit lengths <48 bit
class BitDataMem
{
public:
    //! Construct and initialize container
    BitDataMem();

    //! Write value to container
    //! @param src value, make sure that value is < 2^(bitLen-1)
    //! @param bit length of value
    void write(int64_t src, unsigned int bitLen);


    //! Read value from container
    //! @param dst value
    //! @param bit length of value
    //! @returns true on sucess, false if end of data reached
    bool read(int64_t& dst, unsigned int bitLen);

    //! Get size of container data in bits
    //! @returns size of container  data in bits
    int64_t size() const {return m_writePos;}

    //! Get container bit data aligned to 64 bits
    //! @returns container bit data aligned to 64 bits
    const std::vector<uint64_t>& getData() const {return m_data;}

    //! Set container bit data, the data must be aligned to 64 bit
    //! @param data aligned to 64 bit
    //! @param actual data length in bits
    void setData( const std::vector<uint64_t>& data, int64_t size );

    //! Set read position, no check is performed if pos is beyond data size.
    //! Following reads will fail in that case.
    //! @param read position
    void setReadPos( uint64_t pos ) { m_readPos = pos; }

    //! Clear and reset container
    void clear();

    //! Enable debugging messages if class is compiled with DEBUG_ENABLED 1
    void setDebug( bool debug ) {m_debug = debug;}

    //! Dump container data in hexadecimal format to std::cout
    void dumpData();

    //! Dump single value in hexadecimal format to std::cout
    void dumpData(int64_t d);

protected:
    //! Container data in a 64bit aligned vector
    std::vector<uint64_t> m_data;

    //! Precalculated power of 2 values
    std::vector<int64_t> m_maxVal;

    //! Write position in bits
    uint64_t m_writePos;

    //! Read position in bits
    uint64_t m_readPos;

    //! Debug output enabled if compiled with DEBUG_ENABLED 1
    bool m_debug;
};
}

#endif // BITDATACONTAINER_H
