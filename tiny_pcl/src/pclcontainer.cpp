#include <tiny_pcl/pclcontainer.h>
#include <cstdlib>
#include <iostream>

namespace TinyPCL
{
//-----------------------------------------------------------------------------
PCLContainer::PCLContainer()
{
    m_maxVal.resize(49);
    m_maxVal[0] = 1;
    for( int i=1; i<49; i++ )
        m_maxVal[i] = m_maxVal[i-1]*2;
    m_firstValuesWritten = false;
}

//-----------------------------------------------------------------------------
void PCLContainer::setDataSpec(const DataSpec &spec)
{
    m_spec = spec;
    m_lastValuesWritten.resize(m_spec.spec.size());
    m_lastValuesRead.resize(m_spec.spec.size());
    m_data = BitDataMem();
    m_firstValuesWritten = false;
}

//-----------------------------------------------------------------------------
void PCLContainer::writeVector(int64_t* data)
{
    if(!m_firstValuesWritten)
    {
        for( std::size_t i=0; i<m_spec.spec.size(); i++ )
        {
            const unsigned int dstBits = m_spec.spec[i].dstBits;
            const int64_t markerVal = -m_maxVal[dstBits-1];

            m_data.write(markerVal,dstBits);
            m_data.write(data[i],m_spec.spec[i].absValBits);
            m_lastValuesWritten[i] = data[i];
        }
        m_firstValuesWritten = true;
        return;
    }

    for( std::size_t i=0; i<m_spec.spec.size(); i++ )
    {
        const unsigned int dstBits = m_spec.spec[i].dstBits;
        const int64_t markerVal = -m_maxVal[dstBits-1];

        int32_t d = data[i]-m_lastValuesWritten[i];
        m_lastValuesWritten[i] = data[i];
        if( abs(d)>(m_maxVal[dstBits-1]-1) )
        {
            m_data.write(markerVal,dstBits);
            m_data.write(data[i],m_spec.spec[i].absValBits);
        }
        else
        {
            m_data.write(d,dstBits);
        }
    }
}

//-----------------------------------------------------------------------------
bool PCLContainer::readVector(int64_t *data)
{
    for( std::size_t i=0; i<m_spec.spec.size(); i++ )
    {
        int64_t d;
        const unsigned int dstBits = m_spec.spec[i].dstBits;
        const int64_t markerVal = -m_maxVal[dstBits-1];

        //std::cout << "dstBits " << dstBits << std::endl;
        //std::cout << "markerVal " << markerVal << std::endl;

        if(!m_data.read(d,dstBits))
            return false;
        //std::cout << "d " << d << std::endl;

        if( d == markerVal )
        {
            m_data.read(d,m_spec.spec[i].absValBits);
            data[i] = d;
            m_lastValuesRead[i] = d;
        }
        else
        {
            data[i] = m_lastValuesRead[i]+d;
            m_lastValuesRead[i] = m_lastValuesRead[i]+d;
        }
    }
    return true;
}

//-----------------------------------------------------------------------------
std::vector< std::vector<unsigned int> > PCLContainer::probeDataHistogram(const std::vector<int64_t> &data)
{
    std::vector< std::vector<unsigned int> > hist;
    hist.resize(m_spec.spec.size());
    for( std::size_t i=0; i<m_spec.spec.size(); i++ )
        hist[i].resize(49,0);

    const unsigned int specSize = m_spec.spec.size();

    for( std::size_t i=specSize; i<data.size(); i++ )
    {
        const unsigned int modSpec = i%specSize;
        const int d = abs(data[i]-data[i-specSize]);

        for( std::size_t j=0; j<49; j++)
        {
            if( d<m_maxVal[j] )
            {
                hist[modSpec][j]++;
                break;
            }
        }
    }
    return hist;
}

//-----------------------------------------------------------------------------
std::vector< std::vector<float> > PCLContainer::probeDataBitsPerValue(const std::vector<std::vector<unsigned int> > &hist)
{
    const unsigned int specSize = m_spec.spec.size();

    std::vector< std::vector<float> > bitsPerValue;
    bitsPerValue.resize(specSize);
    for( std::size_t i=0; i<specSize; i++ )
        bitsPerValue[i].resize(48,0);

    for( std::size_t i=0; i<specSize; i++ )
    {
        for( int j=2; j<49; j++ )
        {
            unsigned int diffCount = 0;
            unsigned int absCount = 0;
            for( int k=0; k<49; k++ )
            {
                if(k<j)
                    diffCount += hist[i][k];
                else
                    absCount += hist[i][k];
            }
            const unsigned int totalSize = (m_spec.spec[i].absValBits+j)*absCount+diffCount*j;
            bitsPerValue[i][j] = (totalSize/float(diffCount+absCount));
        }
    }
    return bitsPerValue;
}

//-----------------------------------------------------------------------------
void PCLContainer::adjustByProbeData(const std::vector<int64_t> &data)
{
    std::vector< std::vector<float> > bitsPerVal = probeDataBitsPerValue(probeDataHistogram(data));
    const unsigned int specSize = m_spec.spec.size();

    for( std::size_t i=0; i<specSize; i++ )
    {
        std::size_t minIndex = 2;
        float minVal = bitsPerVal[i][2];
        for( std::size_t j=2; j<49; j++ )
        {
            if( bitsPerVal[i][j] < minVal )
            {
                minVal = bitsPerVal[i][j];
                minIndex = j;
            }
        }
        m_spec.spec[i].dstBits = minIndex;
    }
}

//-----------------------------------------------------------------------------
void PCLContainer::printDataProbe(const std::vector<int64_t> &data, bool printHist, bool printBitsPerVal, std::ostream& out)
{
    std::vector< std::vector<unsigned int> > hist = probeDataHistogram(data);
    const unsigned int specSize = m_spec.spec.size();

    if( printHist )
        for( std::size_t i=0; i<specSize; i++ )
        {
            out << "Histogram for dim " << i << std::endl;
            out << "---------------------------------------------------------------------------------" << std::endl;
            for( int j=0; j<49; j++ )
            {
                out << " x<" << m_maxVal[j] << " : " << hist[i][j] << std::endl;
            }
            out << std::endl;
        }


    std::vector< std::vector<float> > bitsPerValue = probeDataBitsPerValue(hist);

    if( printBitsPerVal )
        for( std::size_t i=0; i<specSize; i++ )
        {
            out << "Size calculation for dim " << i << std::endl;
            out << "---------------------------------------------------------------------------------" << std::endl;
            for( int j=2; j<49; j++ )
            {
                unsigned int diffCount = 0;
                unsigned int absCount = 0;
                for( int k=0; k<49; k++ )
                {
                    if(k<j)
                        diffCount += hist[i][k];
                    else
                        absCount += hist[i][k];
                }
                out.precision(10);
                const unsigned int totalSize = (m_spec.spec[i].absValBits+j)*absCount+diffCount*j;
                out << "Size for " << j << " bit diff storage: " << (totalSize/8.0f) << " (" << (totalSize/float(diffCount+absCount)) << " bits/value)" <<  std::endl;
            }
            out << std::endl;
        }
}

//-----------------------------------------------------------------------------
void PCLContainer::clear()
{
    m_data.clear();
    m_firstValuesWritten = false;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------


}
