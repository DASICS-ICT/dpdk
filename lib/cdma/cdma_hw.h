/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2026
 */

#ifndef CDMA_HW_H
#define CDMA_HW_H

/* AXI CDMA register layout from Xilinx xaxicdma_hw.h. */
#define XAXICDMA_CR_OFFSET          0x00
#define XAXICDMA_SR_OFFSET          0x04
#define XAXICDMA_SRCADDR_OFFSET     0x18
#define XAXICDMA_SRCADDR_MSB_OFFSET 0x1C
#define XAXICDMA_DSTADDR_OFFSET     0x20
#define XAXICDMA_DSTADDR_MSB_OFFSET 0x24
#define XAXICDMA_BTT_OFFSET         0x28

#define XAXICDMA_CR_RESET_MASK   0x00000004
#define XAXICDMA_CR_SGMODE_MASK  0x00000008

#define XAXICDMA_SR_IDLE_MASK    0x00000002
#define XAXICDMA_SR_ERR_ALL_MASK 0x00000770

#define XAXICDMA_XR_IRQ_ALL_MASK 0x00007000

#endif /* CDMA_HW_H */
