#ifndef DATATYPES_H
#define DATATYPES_H

#include <vector>

namespace TinyPCL
{
    enum DataType
    {
        FLOAT32,
        FLOAT64,
        UINT64,
        INT64,
        UINT32,
        INT32,
        UINT16,
        INT16,
        UINT8,
        INT8
    };

    struct DiffTypeSpec
    {
        DataType otype;
        unsigned int srcBits;
        unsigned int absValBits;
        unsigned int dstBits;
    };

    struct DataSpec
    {
        std::vector< DiffTypeSpec > spec;
    };

}

#endif
