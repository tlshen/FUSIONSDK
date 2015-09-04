/*============================================================================*
 * O     O          __                   ______  __                           *
 *  \   /      /\  / /_      _    __    / /___/ / /_     _                    *
 *   [+]      /  \/ / \\    //__ / /__ / /____ / / \\   //                    *
 *  /   \    / /\  /   \\__// --/ /---/ /----// /   \\_//                     *
 * O     O  /_/  \/     \__/    \_\/ /_/     /_/ ____/_/                      *
 *                                                                            *
 *                                                                            *
 * Multi-Rotor controller firmware for Nuvoton Cortex M4 series               *
 *                                                                            *
 * Written by by T.L. Shen for Nuvoton Technology.                            *
 * tlshen@nuvoton.com/tzulan611126@gmail.com                                  *
 *                                                                            *
 *============================================================================*
 */
#ifndef CONFIG_H_
#define CONFIG_H_
#include "AHRSLib.h"
#define VERSION_CODE 142
#define BOARD_CODE   140

#define UART_BAUD_RATE 115200
#define OUTPUT_DATA_INTERVAL 20  //milliseconds
#define DEBUG_PRINT printf
#define DISPLAY_LOOP_TIME 0
#define DISPLAY_SSV_TIME 0
/************************/
/*      ACC/GYRO        */
/************************/
#define MPU6050
/************************/
/*        MAG           */
/************************/
#define HMC5883
//#define AK8975
/************************/
/*       BARO           */
/************************/
//#define BMP085 0
#define MS5611 1
#define BMP280 2
/************************/
/*       MISC           */
/************************/
#define OPTION_RC
#define BLDC
#endif //CONFIG_H_

