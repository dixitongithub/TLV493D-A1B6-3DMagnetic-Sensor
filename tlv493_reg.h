/* 
 * File:   tlv493_reg.h
 * Author: dixit
 *
 * Created on 27 August, 2022, 5:42 PM
 */

#ifndef TLV493_REG_H
#define	TLV493_REG_H

#ifdef	__cplusplus
extern "C" {
#endif

    #include <stdio.h>
    #include <stdint.h>
    #include <stdbool.h>
    
    #define REGMASK_READ    0
    #define REGMASK_WRITE   1

    typedef struct
    {
        uint8_t rw;
        uint8_t byteAdress;
        uint8_t bitMask;
        uint8_t shift;
    } RegMask_t;
    
    typedef struct
    {
        uint8_t fast;
        uint8_t lp;
        uint8_t lpPeriod;
        uint16_t measurementTime;
    } AccessModes_t;

    typedef enum ACCESSMODE_e
    {
        POWERDOWNMODE = 0,
        LOWPOWERMODE,
        MASTERCONTROLMODE
    } AccessModes_e;
    
    typedef enum REGISTERS_e
    {
        R_BX1 = 0,
        R_BX2, 
        R_BY1, 
        R_BY2, 
        R_BZ1, 
        R_BZ2, 
        R_TEMP1, 
        R_TEMP2, 
        R_FRAMECOUNTER, 
        R_CHANNEL,
        R_POWERDOWNFLAG, 
        R_RES1,
        R_RES2,
        R_RES3,
        W_PARITY,
        W_ADDR,
        W_INT,
        W_FAST,
        W_LOWPOWER,
        W_TEMP_NEN,
        W_LP_PERIOD,
        W_PARITY_EN,
        W_RES1,
        W_RES2,
        W_RES3,
        REGMASK_MAX        
    } Registers_e;
    
    uint8_t getRegBits(uint8_t *data, Registers_e id);
    void setRegBits(uint8_t *data, Registers_e id, uint8_t value);
    AccessModes_t* GetAccModes(AccessModes_e mode);
    int16_t concate_value(uint8_t msb, uint8_t lsb, bool reversed);
#ifdef	__cplusplus
}
#endif

#endif	/* TLV493_REG_H */

