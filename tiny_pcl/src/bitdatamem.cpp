#include <tiny_pcl/bitdatamem.h>
#include <iostream>
#include <iomanip>
#include <bitset>
#include <cassert>

#define DEBUG_ENABLED 0

namespace TinyPCL
{

//-----------------------------------------------------------------------------
BitDataMem::BitDataMem()
{
    m_writePos = 0;
    m_readPos = 0;
    m_maxVal.resize(49);
    m_maxVal[0] = 1;
    for( int i=1; i<49; i++ )
        m_maxVal[i] = m_maxVal[i-1]*2;
}

//-----------------------------------------------------------------------------
void BitDataMem::write(int64_t src, unsigned int bitLen)
{
    assert(bitLen<=64);
    if(src<0)
        src = m_maxVal[bitLen] + src;

#if DEBUG_ENABLED
    if( m_debug )
        std::cout << "Write " << src << " with bitLen " << bitLen << std::endl;
#endif

    std::size_t oldVecSize = m_data.size();
    std::size_t newVecSize = (m_writePos+bitLen)/64;
    if( (m_writePos+bitLen)%64 != 0)
        newVecSize++;
    m_data.resize(newVecSize,0);

    std::size_t dstpos = oldVecSize-1;
    if( m_writePos%64 == 0 )
        dstpos++;

    unsigned int spaceAfterWritePos = 64-(m_writePos%64);
    int leftShift = spaceAfterWritePos-bitLen;
    if( leftShift >= 0 )
    {
        uint64_t ls = leftShift;
        m_data[dstpos] = m_data[dstpos] | (src << ls);
    }
    else
    {
        uint64_t ls = -leftShift;
        m_data[dstpos] = m_data[dstpos] | (src >> ls);
    }

    if( bitLen > spaceAfterWritePos )
    {
        dstpos++;
        m_data[dstpos] = src << (64-bitLen+spaceAfterWritePos);
    }

    m_writePos += bitLen;
}

//-----------------------------------------------------------------------------
bool BitDataMem::read(int64_t& dst, unsigned int bitLen)
{
    assert(bitLen<=64);
    if( m_readPos+bitLen > m_writePos)
        return false;

    dst = 0;
    std::size_t srcpos = m_readPos/64;
    uint64_t relReadPos = m_readPos%64;

    unsigned int bitsAfterReadPos = 64-relReadPos;

    if(bitsAfterReadPos>=bitLen)
    {
        dst = (m_data[srcpos] & (0xFFFFFFFFFFFFFFFF >> relReadPos)) >> (bitsAfterReadPos-bitLen);
    }
    else
    {
        dst = (m_data[srcpos] & (0xFFFFFFFFFFFFFFFF >> relReadPos)) << (bitLen-bitsAfterReadPos);
        dst = dst | (m_data[srcpos+1] >> (64-bitLen+bitsAfterReadPos));
    }

    if(dst>(m_maxVal[bitLen-1]-1))
        dst -= m_maxVal[bitLen];

#if DEBUG_ENABLED
    if( m_debug )
        std::cout << "Read " << dst << " with bitLen " << bitLen << std::endl;
#endif

    m_readPos += bitLen;
    return true;
}

//-----------------------------------------------------------------------------
void BitDataMem::clear()
{
    m_readPos = 0;
    m_writePos = 0;
    m_data.clear();
}

//-----------------------------------------------------------------------------
void BitDataMem::setData(const std::vector<uint64_t> &data, int64_t size)
{
    m_data = data;
    m_writePos = size;
    m_readPos = 0;
}

//-----------------------------------------------------------------------------
void BitDataMem::dumpData()
{
    if( m_data.empty() )
        return;

    uint8_t* c = (uint8_t*) &m_data[0];
    std::cout << "BitDataMemory Dump:" << std::endl;

    for( std::size_t i=0; i<m_data.size()*8; i++ )
    {
        if( i%8 == 0)
            std::cout << " | ";
        std::cout << std::hex << std::setfill('0') << std::setw(2) << ((int) *c) << " ";
        c++;
    }
    std::cout << std::dec << std::endl << std::endl;
}

//-----------------------------------------------------------------------------
void BitDataMem::dumpData(int64_t d)
{
    uint8_t* c = (uint8_t*) &d;

    for( std::size_t i=0; i<8; i++ )
    {
        if( i%8 == 0)
            std::cout << " | ";
        std::cout << std::hex << std::setfill('0') << std::setw(2) << ((int) *c) << " ";
        c++;
    }
    std::cout << std::dec << std::endl;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------


}
