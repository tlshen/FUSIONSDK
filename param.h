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
#ifndef __PARAM_H__
#define __PARAM_H__
#include  "Def.h"
/* Public functions */
void paramInit(void);
bool paramTest(void);

/* Basic parameter structure */
struct param_s {
  uint8_t type;
  char * name;
  void * address;
};

#define PARAM_BYTES_MASK 0x03
#define PARAM_1BYTE  0x00
#define PARAM_2BYTES 0x01
#define PARAM_4BYTES 0x02
#define PARAM_8BYTES 0x03

#define PARAM_TYPE_INT   (0x00<<2)
#define PARAM_TYPE_FLOAT (0x01<<2)

#define PARAM_SIGNED (0x00<<3)
#define PARAM_UNSIGNED (0x01<<3)

#define PARAM_VARIABLE (0x00<<7)
#define PARAM_GROUP    (0x01<<7)

#define PARAM_RONLY (1<<6)

#define PARAM_START 1
#define PARAM_STOP  0

#define PARAM_SYNC 0x02

// User-friendly macros
#define PARAM_UINT8 (PARAM_1BYTE | PARAM_TYPE_INT | PARAM_UNSIGNED)
#define PARAM_INT8  (PARAM_1BYTE | PARAM_TYPE_INT | PARAM_SIGNED)
#define PARAM_UINT16 (PARAM_2BYTES | PARAM_TYPE_INT | PARAM_UNSIGNED)
#define PARAM_INT16  (PARAM_2BYTES | PARAM_TYPE_INT | PARAM_SIGNED)
#define PARAM_UINT32 (PARAM_4BYTES | PARAM_TYPE_INT | PARAM_UNSIGNED)
#define PARAM_INT32  (PARAM_4BYTES | PARAM_TYPE_INT | PARAM_SIGNED)

#define PARAM_FLOAT (PARAM_4BYTES | PARAM_TYPE_FLOAT | PARAM_SIGNED)

/* Macros */
#define PARAM_ADD(TYPE, NAME, ADDRESS) \
   { .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },

#define PARAM_ADD_GROUP(TYPE, NAME, ADDRESS) \
   { \
  .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },

#define PARAM_GROUP_START(NAME)  \
  static const struct param_s __params_##NAME[] __attribute__((section(".param." #NAME), used)) = { \
  PARAM_ADD_GROUP(PARAM_GROUP | PARAM_START, NAME, 0x0)

#define PARAM_GROUP_STOP(NAME) \
  PARAM_ADD_GROUP(PARAM_GROUP | PARAM_STOP, stop_##NAME, 0x0) \
  };

#endif /* __PARAM_H__ */

