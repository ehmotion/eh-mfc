
#include "ff_lg.h"

#define FF_FT_EXT_CMD_WHL_DIGITS            0x09
#define FF_FT_EXT_CMD_RPM_LEDS              0x13

#define FF_TM_OUTPUT_REPORT_SIZE 65

#define FF_TM_CMD_MASK  0x0f
#define FF_TM_FOR03     0x03
#define FF_TM_VIB04     0x04
#define FF_TM_SPR05     0x05
#define FF_TM_FOR0E     0x0E
#define FF_TM_FOR0E00   0x00
#define FF_TM_FOR0E00   0x00
#define FF_TM_FOR0E0001 0x01
#define FF_TM_FOR0E0002 0x02
#define FF_TM_FOR0E0003 0x03

void ff_tm_decode_command (const unsigned char data[FF_TM_OUTPUT_REPORT_SIZE]);
