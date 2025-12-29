/*! \copyright 2025 Zorxx Software. All rights reserved.
 *  \brief Linux lowlevel portability interface
 *  See LICENSE file in the root of this project source tree
 */
#ifndef _SYS_LINUX_H
#define _SYS_LINUX_H

#include <unistd.h>

typedef struct
{
   /* Note that it may be necessary to access i2c device files as root */
   const char *device;   /* e.g. "/dev/i2c-0" */
} i2c_lowlevel_config;

#endif /* _SYS_LINUX_H */