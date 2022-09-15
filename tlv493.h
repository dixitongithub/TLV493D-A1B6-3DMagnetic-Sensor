/* 
 * File:   tlv493.h
 * Author: dixit
 *
 * Created on 24 August, 2022
 */

#ifndef TLV493_H
#define	TLV493_H

#ifdef	__cplusplus
extern "C" {
#endif

    #include <avr/io.h>
    #include <string.h>
    #include "main.h"
    #include "i2c.h"
    #include "tlv493_reg.h"

    #define TLV493D_ADDR1    (0x5E << 1) //default
    #define TLV493D_ADDR2    (0x1F << 1)

    #define TLV493D_READ_SIZE          10
    #define TLV493D_READDATA_SIZE      7
    #define TLV493D_WRITE_SIZE         4
    
    #define TLV493D_B_MULT_12          0.098
    #define TLV493D_B_MULT_8           1.56

    #define TLV493D_DEFAULTMODE         LOWPOWERMODE
    #define TLV493D_DEFAULTCONVMODE     CONVMODE_12_BIT

    typedef enum CONVERSOIONMODE_e{
        CONVMODE_8_BIT = 0,
        CONVMODE_12_BIT = 1
    } ConversionMode_e;
    
    /*  Data Structure to hold the sensor data values for axis & temperature. */
    typedef struct {
        int16_t x;
        int16_t y;
        int16_t z;
        int16_t t;
    }TLV493D_data;
    
    /* Data structure to hold a sensor node specific information */
    typedef struct {
        uint8_t addr;
        AccessModes_e mode;
        ConversionMode_e ConvMode;
        bool temp_control;
        TLV493D_data dataVal;
        uint8_t regReadData[TLV493D_READ_SIZE];
        uint8_t regWriteData[TLV493D_WRITE_SIZE];
    } TLV493D_dev;
    
    /*
     * @Name: Tlv493d_Init
     * @Description: This function initializes the TLV493 Sensor data & descriptors
     * @Param: dev - TLV493 Sensor device node
     *         addr - I2C address to be configured for the provided node
     * @Return: None
     */
    void Tlv493d_Init(TLV493D_dev *dev, uint8_t addr);
    
    /*
     * @Name: Tlv493d_Reset
     * @Description: This API resets the I2C bus and initializes the sensor
     * @Param: dev - TLV493 Sensor device node
     *         addr - I2C address to be configured for the provided node
     * @Return: None
     */
    void Tlv493d_Reset(TLV493D_dev *dev, uint8_t addr);
    
    /*
     * @Name: Tlv493d_Setup
     * @Description: This API configures the default configurations & write it to the sensor
     * @Param: dev - TLV493 Sensor device node
     * @Return: True: If setup successful
     *          False: If setup fails
     */
    bool Tlv493d_Setup(TLV493D_dev *dev);
    
    /*
     * @Name: Tlv493d_UpdateData
     * @Description: This API readout the data register from the sensor
     * @Param: dev - TLV493 Sensor device node
     * @Return: True: If data received from the sensor successfully
     *          False: If data was not received from sensor
     */
    bool Tlv493d_UpdateData(TLV493D_dev *dev);
    
    /*
     * @Name: Tlv493d_GetValue_8bit
     * @Description: This API is to get the latest data values received from the sensor
     * @Param: dev - TLV493 Sensor device node
     *         choice - option 0,1,2,3 which value of x,y,z or t
     * @Return: TLV493D_data struct consisting X, Y, Z and temperature reading value
     */
    int8_t Tlv493d_GetValue_8bit(TLV493D_dev *dev, uint8_t choice);
    
    /*
    * @Name: Tlv493d_GetValue_16bit
    * @Description: This API is to get the latest data values received from the sensor
    * @Param: dev - TLV493 Sensor device node
    *         choice - option 0,1,2,3 which value of x,y,z or t
    * @Return: individual value in 16-bit for given parameter. Actually sensor return 12 bit value
*/
    int16_t Tlv493d_GetValue_16bit(TLV493D_dev *dev, uint8_t choice);
    
    /*
     * @Name: Tlv493d_SetConversionMode
     * @Description: This API is to set the conversion mode of data to 8 or 12 bit.
     * @Param: dev - TLV493 Sensor device node
     *         mode - 0 = 8 Bit conversion
     *                1 = 12 Bit conversion
     * @Return: None
     */
    void Tlv493d_SetConversionMode(TLV493D_dev* dev, ConversionMode_e mode);
    
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
    bool Tlv493d_EnableDisableTemp(TLV493D_dev* dev, bool flag, bool writeop);
    
#ifdef	__cplusplus
}
#endif

#endif	/* TLV493_H */

