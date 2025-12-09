/* 
 * File:   ADS1115.h
 * Author: Laurent Alloza <laurent.alloza at isae.fr>
 *
 * Created on 1 juin 2017, 07:35
 */

#ifndef ADS1115_H
#define ADS1115_H

#ifdef __cplusplus
extern "C" {
#endif

#define ADS1115_ADDRESS_ADDR_GND    0x48 // address pin low (GND)
#define ADS1115_ADDRESS_ADDR_VDD    0x49 // address pin high (VCC)
#define ADS1115_ADDRESS_ADDR_SDA    0x4A // address pin tied to SDA pin
#define ADS1115_ADDRESS_ADDR_SCL    0x4B // address pin tied to SCL pin
#define ADS1115_DEFAULT_ADDRESS     ADS1115_ADDRESS_ADDR_GND
/** Register address definitions */
#define ADS1115_RA_CONVERSION       0x00
#define ADS1115_RA_CONFIG           0x01
#define ADS1115_RA_LO_THRESH        0x02
#define ADS1115_RA_HI_THRESH        0x03
/** Config Register Bits */
#define ADS1115_CFG_OS              0x8000

#define ADS1115_MUX_P0_N1           0x0000 // default
#define ADS1115_MUX_P0_N3           0x1000
#define ADS1115_MUX_P1_N3           0x2000
#define ADS1115_MUX_P2_N3           0x3000
#define ADS1115_MUX_P0_NG           0x4000
#define ADS1115_MUX_P1_NG           0x5000
#define ADS1115_MUX_P2_NG           0x6000
#define ADS1115_MUX_P3_NG           0x7000

#define ADS1115_PGA_6P144           0x0000
#define ADS1115_PGA_4P096           0x0200
#define ADS1115_PGA_2P048           0x0400 // default
#define ADS1115_PGA_1P024           0x0600
#define ADS1115_PGA_0P512           0x0800
#define ADS1115_PGA_0P256           0x0A00
#define ADS1115_PGA_0P256B          0x0C00
#define ADS1115_PGA_0P256C          0x0E00

#define ADS1115_MODE_CONTINUOUS     0x0000
#define ADS1115_MODE_SINGLESHOT     0x0100 // default

#define ADS1115_RATE_8              0x0000
#define ADS1115_RATE_16             0x0020
#define ADS1115_RATE_32             0x0040
#define ADS1115_RATE_64             0x0060
#define ADS1115_RATE_128            0x0080 // default
#define ADS1115_RATE_250            0x00A0
#define ADS1115_RATE_475            0x00C0
#define ADS1115_RATE_860            0x00E0

#define ADS1115_COMP_MODE_HYSTERESIS    0x0000 // default
#define ADS1115_COMP_MODE_WINDOW        0x0010

#define ADS1115_COMP_POL_ACTIVE_LOW     0x0000 // default
#define ADS1115_COMP_POL_ACTIVE_HIGH    0x0008

#define ADS1115_COMP_LAT_NON_LATCHING   0x0000 // default
#define ADS1115_COMP_LAT_LATCHING       0x0004

#define ADS1115_COMP_QUE_ASSERT1    0x0000
#define ADS1115_COMP_QUE_ASSERT2    0x0001
#define ADS1115_COMP_QUE_ASSERT4    0x0002
#define ADS1115_COMP_QUE_DISABLE    0x0003 // default

#ifdef __cplusplus
}
#endif

#endif /* ADS1115_H */

