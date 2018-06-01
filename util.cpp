#include "util.h"

/*!
Build a float from a uint8_t array
Public function defined in util.h
*/
float Util_parseFloat(uint8_t *pArray)
{
    return (Util_buildFloat(pArray[0], pArray[1], pArray[2], pArray[3]));
}

/*!
 Break and buffer a float value - LSB first
 Public function defined in util.h
 */
float Util_buildFloat(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
    float ret = 0;
    uint8_t * pF = (uint8_t * ) & ret;
    *pF++ = b0;
    *pF++ = b1;
    *pF++ = b2;
    *pF++ = b3;
    return ret;
}
/*
 * @brief: Break and buffer a float value - LSB first
 * @return: pointer to last buffer
 */
uint8_t * Util_bufferFloat(uint8_t * pBuf, float val)
{
    uint8_t * pF = (uint8_t *) &val;
    *pBuf++ = *pF++;
    *pBuf++ = *pF++;
    *pBuf++ = *pF++;
    *pBuf++ = *pF++;
    return (pBuf);
}
/*!
 Build a uint32_t out of 4 uint8_t variables
 Public function defined in util.h
 */
uint32_t Util_buildUint32(uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3)
{
    return ((uint32_t)((uint32_t)((byte0) & 0x00FF) +
                      ((uint32_t)((byte1) & 0x00FF) <<  8) +
                      ((uint32_t)((byte2) & 0x00FF) << 16) +
                      ((uint32_t)((byte3) & 0x00FF) << 24)));
}

/*!
 Build a uint32_t from a uint8_t array
 Public function defined in util.h
 */
uint32_t Util_parseUint32(uint8_t *pArray)
{
    return (Util_buildUint32(pArray[0], pArray[1], pArray[2], pArray[3]));
}
/*!
 Pull 1 uint8_t out of a uint32_t
 Public function defined in util.h
 */
uint8_t Util_breakUint32(uint32_t var, int byteNum)
{
    return (uint8_t)((uint32_t)(((var) >> ((byteNum) * 8)) & 0x00FF));
}
/*!
 Break and buffer a uint32_t value - LSB first
 Public function defined in util.h
 */
uint8_t * Util_bufferUint32(uint8_t *pBuf, uint32_t val)
{
    *pBuf++ = Util_breakUint32(val, 0);
    *pBuf++ = Util_breakUint32(val, 1);
    *pBuf++ = Util_breakUint32(val, 2);
    *pBuf++ = Util_breakUint32(val, 3);

    return (pBuf);
}
