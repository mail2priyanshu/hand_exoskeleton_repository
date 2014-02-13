/*
 * Generated with the FPGA Interface C API Generator 12.0.0
 * for NI-RIO 12.0.0 or later.
 */

#ifndef __NiFpga_test_rig_controller_h__
#define __NiFpga_test_rig_controller_h__

#ifndef NiFpga_Version
   #define NiFpga_Version 1200
#endif

#include "NiFpga.h"

/**
 * The filename of the FPGA bitfile.
 *
 * This is a #define to allow for string literal concatenation. For example:
 *
 *    static const char* const Bitfile = "C:\\" NiFpga_test_rig_controller_Bitfile;
 */
#define NiFpga_test_rig_controller_Bitfile "NiFpga_test_rig_controller.lvbitx"

/**
 * The signature of the FPGA bitfile.
 */
static const char* const NiFpga_test_rig_controller_Signature = "AAC7E4F8C38A9D565B912A9AE799E4BA";

typedef enum
{
   NiFpga_test_rig_controller_IndicatorI32_hallsensor1 = 0x8120,
   NiFpga_test_rig_controller_IndicatorI32_hallsensor2 = 0x811C,
   NiFpga_test_rig_controller_IndicatorI32_hallsensor3 = 0x8118,
   NiFpga_test_rig_controller_IndicatorI32_hallsensor4 = 0x8114,
   NiFpga_test_rig_controller_IndicatorI32_hallsensor5 = 0x8110,
   NiFpga_test_rig_controller_IndicatorI32_hallsensor6 = 0x810C,
} NiFpga_test_rig_controller_IndicatorI32;

typedef enum
{
   NiFpga_test_rig_controller_IndicatorArrayI32_sensordata = 0x8124,
} NiFpga_test_rig_controller_IndicatorArrayI32;

typedef enum
{
   NiFpga_test_rig_controller_IndicatorArrayI32Size_sensordata = 6,
} NiFpga_test_rig_controller_IndicatorArrayI32Size;

#endif
