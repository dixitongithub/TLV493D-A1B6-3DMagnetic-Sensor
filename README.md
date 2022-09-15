# TLV493D-A1B6-3DMagnetic-Sensor
C library for Infineon TLV493D-A1B6-3DMagnetic-Sensor

Define a TLV493d device by;
TLV493D_dev tlv493;

Sequence of library function in initialization:
Tlv493d_Reset(& tlv493, TLV493D_ADDR1);
Tlv493d_Setup(&tlv493);
Tlv493d_SetConversionMode(&tlv493, CONVMODE_12_BIT);

  
Get the data continuesly in the infinite loop:
if(Tlv493d_UpdateData(&tlv493))
{
  valueX = Tlv493d_GetValue_12bit(&tlv493, 0);
  valueY = Tlv493d_GetValue_12bit(&tlv493, 1);
  valueZ = Tlv493d_GetValue_12bit(&tlv493, 2);
}

Currently the library supports single Default (Low Power mode) of operation. will update this for more modes soon....

Enjoy!!!!
