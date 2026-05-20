# SPDX-License-Identifier: BSD-3-Clause
# Copyright(c) 2023 Ericsson AB

if is_windows
    build = false
    reason = 'not supported on Windows'
    subdir_done()
endif

sources = files('rte_dbchecker.c')
headers = files('rte_dbchecker.h')

deps += ['platform_vfio_helper']