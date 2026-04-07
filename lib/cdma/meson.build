# SPDX-License-Identifier: BSD-3-Clause
# Copyright(c) 2026

if is_windows
    build = false
    reason = 'not supported on Windows'
    subdir_done()
endif

sources = files('rte_cdma.c', 'cdma_common.c')
headers = files('rte_cdma.h')
deps += ['platform_vfio_helper']
if get_option('enable_dbchecker')
    deps += ['dbchecker']
endif
