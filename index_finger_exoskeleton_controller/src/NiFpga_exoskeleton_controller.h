/*
 * Generated with the FPGA Interface C API Generator 12.0.0
 * for NI-RIO 12.0.0 or later.
 */

#ifndef __NiFpga_exoskeleton_controller_h__
#define __NiFpga_exoskeleton_controller_h__

#ifndef NiFpga_Version
   #define NiFpga_Version 1200
#endif

#include "NiFpga.h"

/**
 * The filename of the FPGA bitfile.
 *
 * This is a #define to allow for string literal concatenation. For example:
 *
 *    static const char* const Bitfile = "C:\\" NiFpga_exoskeleton_controller_Bitfile;
 */
#define NiFpga_exoskeleton_controller_Bitfile "NiFpga_exoskeleton_controller.lvbitx"

/**
 * The signature of the FPGA bitfile.
 */
static const char* const NiFpga_exoskeleton_controller_Signature = "25C2BD16CA64FEAF03EFFF62CBB7AF3A";

typedef enum
{
   NiFpga_exoskeleton_controller_IndicatorI32_exo_hall4 = 0x8114,
   NiFpga_exoskeleton_controller_IndicatorI32_exo_hall4_filtered = 0x8110,
   NiFpga_exoskeleton_controller_IndicatorI32_exo_mcp = 0x8140,
   NiFpga_exoskeleton_controller_IndicatorI32_exo_mcp_ddot = 0x817C,
   NiFpga_exoskeleton_controller_IndicatorI32_exo_mcp_dot = 0x8180,
   NiFpga_exoskeleton_controller_IndicatorI32_exo_mcp_filtered = 0x813C,
   NiFpga_exoskeleton_controller_IndicatorI32_exo_mcp_pip = 0x8120,
   NiFpga_exoskeleton_controller_IndicatorI32_exo_mcp_pip_filtered = 0x811C,
   NiFpga_exoskeleton_controller_IndicatorI32_exo_pip = 0x8144,
   NiFpga_exoskeleton_controller_IndicatorI32_exo_pip_ddot = 0x8154,
   NiFpga_exoskeleton_controller_IndicatorI32_exo_pip_dot = 0x8158,
   NiFpga_exoskeleton_controller_IndicatorI32_exo_pip_filtered = 0x8148,
   NiFpga_exoskeleton_controller_IndicatorI32_hallsensor1 = 0x8130,
   NiFpga_exoskeleton_controller_IndicatorI32_hallsensor2 = 0x812C,
   NiFpga_exoskeleton_controller_IndicatorI32_hallsensor3 = 0x8128,
   NiFpga_exoskeleton_controller_IndicatorI32_hallsensor4 = 0x810C,
   NiFpga_exoskeleton_controller_IndicatorI32_load1 = 0x81BC,
   NiFpga_exoskeleton_controller_IndicatorI32_load2 = 0x81B8,
   NiFpga_exoskeleton_controller_IndicatorI32_load3 = 0x81B4,
   NiFpga_exoskeleton_controller_IndicatorI32_load4 = 0x81B0,
   NiFpga_exoskeleton_controller_IndicatorI32_load5 = 0x81A0,
   NiFpga_exoskeleton_controller_IndicatorI32_load6 = 0x819C,
   NiFpga_exoskeleton_controller_IndicatorI32_motor_mcp_acceleration = 0x8194,
   NiFpga_exoskeleton_controller_IndicatorI32_motor_mcp_position = 0x8190,
   NiFpga_exoskeleton_controller_IndicatorI32_motor_mcp_velocity = 0x8198,
   NiFpga_exoskeleton_controller_IndicatorI32_motor_pip_acceleration = 0x816C,
   NiFpga_exoskeleton_controller_IndicatorI32_motor_pip_position = 0x8168,
   NiFpga_exoskeleton_controller_IndicatorI32_motor_pip_velocity = 0x8170,
} NiFpga_exoskeleton_controller_IndicatorI32;

typedef enum
{
   NiFpga_exoskeleton_controller_ControlBool_reset_motor_mcp = 0x8186,
   NiFpga_exoskeleton_controller_ControlBool_reset_motor_pip = 0x815E,
   NiFpga_exoskeleton_controller_ControlBool_stop_exo_hall4_filter = 0x811A,
   NiFpga_exoskeleton_controller_ControlBool_stop_exo_mcp = 0x817A,
   NiFpga_exoskeleton_controller_ControlBool_stop_exo_mcp_filter = 0x8136,
   NiFpga_exoskeleton_controller_ControlBool_stop_exo_mcp_pip_filter = 0x8126,
   NiFpga_exoskeleton_controller_ControlBool_stop_exo_pip = 0x8152,
   NiFpga_exoskeleton_controller_ControlBool_stop_exo_pip_filter = 0x813A,
   NiFpga_exoskeleton_controller_ControlBool_stop_motor_mcp = 0x818E,
   NiFpga_exoskeleton_controller_ControlBool_stop_motor_pip = 0x8166,
} NiFpga_exoskeleton_controller_ControlBool;

typedef enum
{
   NiFpga_exoskeleton_controller_ControlI32_motor_mcp = 0x81AC,
   NiFpga_exoskeleton_controller_ControlI32_motor_pip = 0x81A8,
} NiFpga_exoskeleton_controller_ControlI32;

typedef enum
{
   NiFpga_exoskeleton_controller_ControlU32_interval_exo_mcpuSec = 0x8174,
   NiFpga_exoskeleton_controller_ControlU32_interval_exo_pipuSec = 0x814C,
   NiFpga_exoskeleton_controller_ControlU32_interval_motor_mcpuSec = 0x8188,
   NiFpga_exoskeleton_controller_ControlU32_interval_motor_pipuSec = 0x8160,
} NiFpga_exoskeleton_controller_ControlU32;

typedef enum
{
   NiFpga_exoskeleton_controller_IndicatorArrayI32_sensordata = 0x81C0,
} NiFpga_exoskeleton_controller_IndicatorArrayI32;

typedef enum
{
   NiFpga_exoskeleton_controller_IndicatorArrayI32Size_sensordata = 24,
} NiFpga_exoskeleton_controller_IndicatorArrayI32Size;

typedef enum
{
   NiFpga_exoskeleton_controller_ControlArrayI32_motordata = 0x81A4,
} NiFpga_exoskeleton_controller_ControlArrayI32;

typedef enum
{
   NiFpga_exoskeleton_controller_ControlArrayI32Size_motordata = 6,
} NiFpga_exoskeleton_controller_ControlArrayI32Size;

#endif
