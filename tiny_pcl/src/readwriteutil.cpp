#include "readwriteutil.h"
#include <string>

int64_t readAndScaleType(const void *pdata, float scale_factor, unsigned int type)
{
    switch( type )
    {
    case 1:
        return readAndScale<int8_t>(pdata,scale_factor);
        break;
    case 2:
        return readAndScale<uint8_t>(pdata,scale_factor);
        break;
    case 3:
        return readAndScale<int16_t>(pdata,scale_factor);
        break;
    case 4:
        return readAndScale<uint16_t>(pdata,scale_factor);
        break;
    case 5:
        return readAndScale<int32_t>(pdata,scale_factor);
        break;
    case 6:
        return readAndScale<uint32_t>(pdata,scale_factor);
        break;
    case 7:
        return readAndScale<float>(pdata,scale_factor);
        break;
    case 8:
        return readAndScale<double>(pdata,scale_factor);
        break;
    default:
        throw std::string("Wrong type!");
    }
    return 0;
}

int64_t writeAndScaleType(void *pdata, int64_t value, float scale_factor, unsigned int type)
{
    switch( type )
    {
    case 1:
        writeAndScale<int8_t>(pdata,value,scale_factor);
        break;
    case 2:
        writeAndScale<uint8_t>(pdata,value,scale_factor);
        break;
    case 3:
        writeAndScale<int16_t>(pdata,value,scale_factor);
        break;
    case 4:
        writeAndScale<uint16_t>(pdata,value,scale_factor);
        break;
    case 5:
        writeAndScale<int32_t>(pdata,value,scale_factor);
        break;
    case 6:
        writeAndScale<uint32_t>(pdata,value,scale_factor);
        break;
    case 7:
        writeAndScale<float>(pdata,value,scale_factor);
        break;
    case 8:
        writeAndScale<double>(pdata,value,scale_factor);
        break;
    default:
        throw std::string("Wrong type!");
    }
    return 0;
}

std::size_t sizeofPCLDatatype( unsigned int type )
{
    switch( type )
    {
    case 1:
        return 1;
        break;
    case 2:
        return 1;
        break;
    case 3:
        return 2;
        break;
    case 4:
        return 2;
        break;
    case 5:
        return 4;
        break;
    case 6:
        return 4;
        break;
    case 7:
        return 4;
        break;
    case 8:
        return 8;
        break;
    default:
        throw std::string("Wrong type!");
    }
    return 0;
}



