#ifndef READUTIL_H
#define READUTIL_H

#include <cstdint>
#include <cmath>

template<class T>
int64_t readAndScale( const void* pdata, float scale_factor )
{
    T* pval = (T*)  pdata;
    int64_t r = int64_t(floor((*pval)*scale_factor+0.5f));
    return r;
}

template<class T>
void writeAndScale( void* pdata, int64_t value, float scale_factor )
{
    T* pval = (T*) pdata;
    *pval = value/scale_factor;
}

int64_t readAndScaleType(const void *pdata, float scale_factor, unsigned int type);
int64_t writeAndScaleType(void *pdata, int64_t value, float scale_factor, unsigned int type);
std::size_t sizeofPCLDatatype( unsigned int type );

#endif // READUTIL_H
