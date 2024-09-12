/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include "env_validate.h"

//#define HAS_OTG_USB_HOST_SUPPORT                  // USB Flash Drive support

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "QIDI_XSERIES_V46 supports up to 2 hotends / E steppers."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "QIDI_XSERIES_V46"
#endif

#ifndef DEFAULT_MACHINE_NAME
  #define DEFAULT_MACHINE_NAME BOARD_INFO_NAME
#endif

#define SRAM_EEPROM_EMULATION

//
// Servos
//
//#define SERVO0_PIN                          //PB0   // XS2-5
//#define SERVO1_PIN                          //PF7   // XS1-5
//#define SERVO2_PIN                          //PF8   // XS1-6

//
// Limit Switches
//
#define X_STOP_PIN                           PC15 //Needs to be inverted in configuration.h
#define Y_STOP_PIN                           PC14 //Needs to be inverted in configuration.h
#define Z_STOP_PIN                           PC13 //PE6 //Needs to be inverted in configuration.h

//
// Probe enable
//
#if ENABLED(PROBE_ENABLE_DISABLE) && !defined(PROBE_ENABLE_PIN)
  #define PROBE_ENABLE_PIN            SERVO0_PIN
#endif

//
// Steppers
//
#define X_STEP_PIN                          PE5
#define X_DIR_PIN                           PF1
#define X_ENABLE_PIN                        PF0     //Needs to be inverted in configuration.h

#define Y_STEP_PIN                          PF9
#define Y_DIR_PIN                           PF3
#define Y_ENABLE_PIN                        PF5     //Needs to be inverted in configuration.h

#define Z_STEP_PIN                          PA6
#define Z_DIR_PIN                           PF15
#define Z_ENABLE_PIN                        PA15    //Needs to be inverted in configuration.h

#define Z2_STEP_PIN                         PD12   
#define Z2_DIR_PIN                          PG4
#define Z2_ENABLE_PIN                       PG5     //Needs to be inverted in configuration.h

#define E0_STEP_PIN                         PB1
#define E0_DIR_PIN                          PF13    //Needs to be inverted in configuration.h
#define E0_ENABLE_PIN                       PF14    //Needs to be inverted in configuration.h

//#define E1_STEP_PIN                         //PD3
//#define E1_DIR_PIN                          //PA15
//#define E1_ENABLE_PIN                       //PD6

//
// Temperature Sensors
//
#define TEMP_0_CS_PIN                          PD13   // T1 <-> E0 MAX6675 ON SPI2
//#define TEMP_1_PIN                          //PC2   // T2 <-> E1
#define TEMP_BED_PIN                        PC2   // T0 <-> Bed EPCOS 100K B57560G104F

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PG0   // Heater0
//#define HEATER_1_PIN                        //PF2   // Heater1
#define HEATER_BED_PIN                      PB2     // Hotbed Needs to be inverted in configuration.h
#define FAN0_PIN                            PB6     // Fan0
#define FAN1_PIN                            PD7     // Fan1
#define FAN2_PIN                            PD6     // Fan2 Controler Fan
#define FAN3_PIN                            PG7     // Fan3 Chamber Fan

//
// Misc. Functions
//
#define LED_PIN                             PF10
#define SDSS                                -1    // PB12

//#define SD_DETECT_PIN                       //PF9
#define BEEPER_PIN                          PA8

#if ENABLED(WIFISUPPORT)
  //
  // WIFI
  //

  /**
   *                      -------
   *            GND | 9  |       | 8 | 3.3V
   *  (ESP-CS) PB12 | 10 |       | 7 | PB15 (ESP-MOSI)
   *           3.3V | 11 |       | 6 | PB14 (ESP-MISO)
   * (ESP-IO0)  PD7 | 12 |       | 5 | PB13 (ESP-CLK)
   * (ESP-IO4) PD10 | 13 |       | 4 | --
   *             -- | 14 |       | 3 | PE15 (ESP-EN)
   *  (ESP-RX)  PD8 | 15 |       | 2 | --
   *  (ESP-TX)  PD9 | 16 |       | 1 | PE14 (ESP-RST)
   *                      -------
   *                       WIFI
   */
  //#define ESP_WIFI_MODULE_COM                  3  // Must also set either SERIAL_PORT or SERIAL_PORT_2 to this
  //#define ESP_WIFI_MODULE_BAUDRATE      BAUDRATE  // Must use same BAUDRATE as SERIAL_PORT & SERIAL_PORT_2
  //#define ESP_WIFI_MODULE_RESET_PIN         PG7
  //#define ESP_WIFI_MODULE_ENABLE_PIN        PG8
  //#define ESP_WIFI_MODULE_GPIO0_PIN         PD7
  //#define ESP_WIFI_MODULE_GPIO4_PIN         PD10
#endif
