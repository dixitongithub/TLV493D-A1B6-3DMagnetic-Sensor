/* 
 * File:   tlv493.c
 * Author: dixit
 *
 * Created on 24 August, 2022
 */

#include "tlv493.h"
#include <util/delay.h>

/*
 * Static function definations
 */

bool Tlv493d_SetAccessMode(TLV493D_dev *dev, AccessModes_e mode)
{
    const AccessModes_t *modes = GetAccModes(mode);
    
    setRegBits(dev->regWriteData, W_FAST, modes->fast);
    setRegBits(dev->regWriteData, W_LOWPOWER, modes->lp);
    setRegBits(dev->regWriteData, W_LP_PERIOD, modes->lpPeriod);
    
    dev->mode = mode;
    
    return true;
}

bool Tlv493d_ReadOut(TLV493D_dev *dev, bool runtime)
{
    uint8_t dataCnt = TLV493D_READ_SIZE;
    uint8_t buff[TLV493D_READ_SIZE] = {0};
    uint8_t ret = 0;
    if(runtime)
        dataCnt = TLV493D_READDATA_SIZE;
    
    ret = I2C_GetData(dev->addr, buff, dataCnt);
    
    if(ret == dataCnt)
    {
        for (uint8_t idx = 0; idx < dataCnt; idx++)
            dev->regReadData[idx] = buff[idx];
        return true;
    }
    return true;
}

bool Tlv493d_WriteOut(TLV493D_dev *dev)
{
    uint8_t retCount = 0;
    
    retCount = I2C_SendData(dev->addr, dev->regWriteData, TLV493D_WRITE_SIZE);
    
    return retCount == TLV493D_WRITE_SIZE? true: false;
}

bool Tlv493d_SetDefaultConfiguration(TLV493D_dev *dev)
{
    // Set read-reserved bits to write-reserved
    setRegBits(dev->regWriteData, W_RES1, getRegBits(dev->regReadData, R_RES1));
    setRegBits(dev->regWriteData, W_RES2, getRegBits(dev->regReadData, R_RES2));
    setRegBits(dev->regWriteData, W_RES3, getRegBits(dev->regReadData, R_RES3));
    
    // Disable Parity calculation
    setRegBits(dev->regWriteData, W_PARITY_EN, 0);
    
    // Set Access mode
    Tlv493d_SetAccessMode(dev, TLV493D_DEFAULTMODE);
    
    // Set conversion mode - 12-Bit default
    Tlv493d_SetConversionMode(dev, TLV493D_DEFAULTCONVMODE);
    
    // Disable Temperature calculation
    Tlv493d_EnableDisableTemp(dev, false, false);
    
    return true;
}

/*
 * Global function definations
 */

/*
    * @Name: Tlv493d_Init
    * @Description: This function initializes the TLV493 Sensor data & descriptors
    * @Param: dev - TLV493 Sensor device node
    *         addr - I2C address to be configured for the provided node
    * @Return: None
*/
void Tlv493d_Init(TLV493D_dev *dev, uint8_t addr)
{
    I2C_Init();
    dev->addr = addr;
    dev->mode = POWERDOWNMODE;
    for (uint8_t idx = 0; idx < TLV493D_READ_SIZE; idx++)
        dev->regReadData[idx] = 0;
    for (uint8_t idx = 0; idx < TLV493D_WRITE_SIZE; idx++)
        dev->regWriteData[idx] = 0;    
}

/*
    * @Name: Tlv493d_Reset
    * @Description: This API resets the I2C bus and initializes the sensor
    * @Param: dev - TLV493 Sensor device node
    *         addr - I2C address to be configured for the provided node
    * @Return: None
*/
void Tlv493d_Reset(TLV493D_dev *dev, uint8_t addr)
{
    /*
    uint8_t buff[1];
    
    // Reset I2C bus
    if (addr == TLV493D_ADDR1)
        buff[0] = 0xFF;
    else
        buff[0] = 0x00;
    
    //I2C_SendData(0x00, buff, 1);
    _delay_ms(100);
    */
    //re-initialize sensor data
    Tlv493d_Init(dev, addr);
}

/*
    * @Name: Tlv493d_SetConversionMode
    * @Description: This API is to set the conversion mode of data to 8 or 12 bit.
    * @Param: dev - TLV493 Sensor device node
    *         mode - 0 = 8 Bit conversion
    *                1 = 12 Bit conversion
    * @Return: None
*/
void Tlv493d_SetConversionMode(TLV493D_dev* dev, ConversionMode_e mode)
{
    dev->ConvMode = mode;
}

/*
    * @Name: Tlv493d_EnableDisableTemp
    * @Description: This API enables/disables the temperature reading control on the sensor
    * @Param: dev - TLV493 Sensor device node
    *         flag - 0 = Off
    *                1 = On
    *         writeop - 0 = Just update the configuration struct (Default)
    *                   1 = Immedietly write the configuration to sensor
    * @Return: 1 : Success
    *          0 : Failure
*/
bool Tlv493d_EnableDisableTemp(TLV493D_dev* dev, bool flag, bool writeop)
{
    //Set temperature measurement control
    setRegBits(dev->regWriteData, W_TEMP_NEN, flag);
    dev->temp_control = flag;
    
    if(writeop == true)
        return Tlv493d_WriteOut(dev);
    
    return true;
}

/*
    * @Name: Tlv493d_Setup
    * @Description: This API configures the default configurations & write it to the sensor
    * @Param: dev - TLV493 Sensor device node
    * @Return: True: If setup successful
    *          False: If setup fails
*/
bool Tlv493d_Setup(TLV493D_dev *dev)
{
    Tlv493d_ReadOut(dev, false);
    Tlv493d_SetDefaultConfiguration(dev);
    return Tlv493d_WriteOut(dev);
    
}

/*
    * @Name: Tlv493d_UpdateData
    * @Description: This API readout the data register from the sensor
    * @Param: dev - TLV493 Sensor device node
    * @Return: True: If data received from the sensor successfully
    *          False: If data was not received from sensor
 */
bool Tlv493d_UpdateData(TLV493D_dev *dev)
{
    if (Tlv493d_ReadOut(dev, true))
    {
        if (dev->ConvMode == CONVMODE_12_BIT)
        {
            dev->dataVal.x = concate_value(getRegBits(dev->regReadData, R_BX1), getRegBits(dev->regReadData, R_BX2), false);
            dev->dataVal.y = concate_value(getRegBits(dev->regReadData, R_BY1), getRegBits(dev->regReadData, R_BY2), false);
            dev->dataVal.z = concate_value(getRegBits(dev->regReadData, R_BZ1), getRegBits(dev->regReadData, R_BZ2), false);
            if (dev->temp_control)
                dev->dataVal.t = concate_value(getRegBits(dev->regReadData, R_TEMP1), getRegBits(dev->regReadData, R_TEMP2), true);
        }
        else
        {
            dev->dataVal.x = (int8_t) getRegBits(dev->regReadData, R_BX1);
            dev->dataVal.y = (int8_t) getRegBits(dev->regReadData, R_BY1);
            dev->dataVal.z = (int8_t) getRegBits(dev->regReadData, R_BY1);
            if (dev->temp_control)
                dev->dataVal.t = (int8_t) getRegBits(dev->regReadData, R_TEMP2);
        }
        return true;
    }
    return false;
}

/*
    * @Name: Tlv493d_GetValue_8bit
    * @Description: This API is to get the latest data values received from the sensor
    * @Param: dev - TLV493 Sensor device node
 *            choice - option which value of x,y,z or t
    * @Return: individual value in 8-bit for given parameter
*/
int8_t Tlv493d_GetValue_8bit(TLV493D_dev *dev, uint8_t choice)
{
    if (choice == 0)
        return (int8_t) dev->dataVal.x;
    else if (choice == 1)
        return (int8_t) dev->dataVal.y;
    else if (choice == 2)
        return (int8_t) dev->dataVal.z;
    else if (choice == 3)
        return (int8_t) dev->dataVal.t;
    else
        return 0;
}
/*
    * @Name: Tlv493d_GetValue_16bit
    * @Description: This API is to get the latest data values received from the sensor
    * @Param: dev - TLV493 Sensor device node
    *         choice - option which value of x,y,z or t
    * @Return: individual value in 16-bit for given parameter. Actually sensor return 12 bit value
*/
int16_t Tlv493d_GetValue_16bit(TLV493D_dev *dev, uint8_t choice)
{
    if (choice == 0)
        return dev->dataVal.x;
    else if (choice == 1)
        return dev->dataVal.y;
    else if (choice == 2)
        return dev->dataVal.z;
    else if (choice == 3)
        return dev->dataVal.t;
    else
        return 0;
}