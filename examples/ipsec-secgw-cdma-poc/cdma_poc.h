/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2026
 */

#ifndef CDMA_POC_H
#define CDMA_POC_H

#include <stdint.h>

struct socket_ctx;

int
cdma_poc_run_gateway_case_study(struct socket_ctx *ctx, uint32_t nb_socket_ctx);

#endif /* CDMA_POC_H */
