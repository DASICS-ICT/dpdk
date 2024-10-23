/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2018-2022 Advanced Micro Devices, Inc.
 */

#ifndef _IONIC_OSDEP_
#define _IONIC_OSDEP_

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

#include <rte_common.h>
#include <rte_debug.h>
#include <rte_cycles.h>
#include <rte_log.h>
#include <rte_byteorder.h>
#include <rte_io.h>
#include <rte_memory.h>
#include <rte_eal_paging.h>

#include "ionic_logs.h"

#define BIT(nr)            (1UL << (nr))
#define BIT_ULL(nr)        (1ULL << (nr))

#ifndef PAGE_SHIFT
#define PAGE_SHIFT      12
#endif

#define __iomem

#ifndef u8
#define u8 uint8_t
#endif
#ifndef u16
#define u16 uint16_t
#endif
#ifndef u32
#define u32 uint32_t
#endif
#ifndef uint64_t
#define u64 uint64_t
#endif

#ifndef __le16
#define __le16 uint16_t
#endif
#ifndef __le32
#define __le32 uint32_t
#endif
#ifndef __le64
#define __le64 uint64_t
#endif

#define ioread8(reg)		rte_read8(reg)
#define ioread32(reg)		rte_read32(rte_le_to_cpu_32(reg))
#define iowrite8(value, reg)	rte_write8(value, reg)
#define iowrite32(value, reg)	rte_write32(rte_cpu_to_le_32(value), reg)

#endif
