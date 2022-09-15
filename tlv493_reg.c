#include <stdio.h>
#include "tlv493_reg.h"

const RegMask_t regMasks[] = {
    { REGMASK_READ, 0, 0xFF, 0 },           // R_BX1
    { REGMASK_READ, 4, 0xF0, 4 },           // R_BX2
    { REGMASK_READ, 1, 0xFF, 0 },           // R_BY1
    { REGMASK_READ, 4, 0x0F, 0 },           // R_BY2
    { REGMASK_READ, 2, 0xFF, 0 },           // R_BZ1
    { REGMASK_READ, 5, 0x0F, 0 },           // R_BZ2
    { REGMASK_READ, 3, 0xF0, 4 },           // R_TEMP1
    { REGMASK_READ, 6, 0xFF, 0 },           // R_TEMP2
    { REGMASK_READ, 3, 0x0C, 2 },           // R_FRAMECOUNTER
    { REGMASK_READ, 3, 0x03, 0 },           // R_CHANNEL
    { REGMASK_READ, 5, 0x10, 4 },           // R_POWERDOWNFLAG
    { REGMASK_READ, 7, 0x18, 3 },           // R_RES1
    { REGMASK_READ, 8, 0xFF, 0 },           // R_RES2
    { REGMASK_READ, 9, 0x1F, 0 },           // R_RES3
    { REGMASK_WRITE, 1, 0x80, 7 },          // W_PARITY
    { REGMASK_WRITE, 1, 0x60, 5 },          // W_ADDR
    { REGMASK_WRITE, 1, 0x04, 2 },          // W_INT
    { REGMASK_WRITE, 1, 0x02, 1 },          // W_FAST
    { REGMASK_WRITE, 1, 0x01, 0 },          // W_LOWPOWER
    { REGMASK_WRITE, 3, 0x80, 7 },          // W_TEMP_EN
    { REGMASK_WRITE, 3, 0x40, 6 },          // W_LOWPOWER
    { REGMASK_WRITE, 3, 0x20, 5 },          // W_PARITY_EN
    { REGMASK_WRITE, 1, 0x18, 3 },          // W_RES1
    { REGMASK_WRITE, 2, 0xFF, 0 },          // W_RES2
    { REGMASK_WRITE, 3, 0x1F, 0 },          // W_RES3
    { REGMASK_WRITE, 0, 0xFF, 0 }           // REGMASK_MAX
};

AccessModes_t accModes[] = {
    { 0, 0, 0, 1000 },              // POWERDOWNMODE
    { 0, 1, 1, 10 },                // LOWPOWERMODE
    { 1, 1, 1, 10 }                 // MASTERCONTROLLEDMODE
};

uint8_t getFromRegs(const RegMask_t *p_mask, uint8_t *pData)
{
    return ((pData[p_mask->byteAdress] & p_mask->bitMask) >> p_mask->shift);
}

uint8_t setToRegs(const RegMask_t *p_mask, uint8_t *pData, uint8_t data)
{
    if(p_mask->rw == REGMASK_WRITE)
    {
        uint8_t regval = pData[p_mask->byteAdress];
        regval &= ~(p_mask->bitMask);
        regval |= (data << p_mask->shift) & p_mask->bitMask;
        pData[p_mask->byteAdress] = regval;
    }
    return 0;
}

void setRegBits(uint8_t *data, Registers_e id, uint8_t value)
{
    if(id < REGMASK_MAX)
    {
        setToRegs(&regMasks[id], data, value);
    }
}

uint8_t getRegBits(uint8_t *data, Registers_e id)
{
    uint8_t val = 0;
    if(id < REGMASK_MAX)
    {
        const RegMask_t *l_mask = &regMasks[id];
        if(l_mask->rw == REGMASK_READ)
        {
            val = getFromRegs(l_mask, data);
        }
        else
        {
            val = getFromRegs(l_mask, data);
        }
    }
    return val;
}

AccessModes_t* GetAccModes(AccessModes_e mode)
{
    return &accModes[mode];
}

int16_t concate_value(uint8_t upperByte, uint8_t lowerByte, bool flag)
{
        int16_t value=0x0000;   //16-bit signed integer for 12-bit values of sensor
        if(!flag)
        {
                value=upperByte<<8;
                value|=(lowerByte & 0x0F)<<4;
        }
        else
        {
                value=(upperByte & 0x0F)<<12;
                value|=lowerByte<<4;
        }
        value >>= 4;                              //shift left so that value is a signed 12 bit integer
        
        return value;
}