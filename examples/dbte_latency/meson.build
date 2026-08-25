# SPDX-License-Identifier: BSD-3-Clause

allow_experimental_apis = true

if not get_option('enable_cdma') or not get_option('enable_dbchecker')
        build = false
        reason = 'requires -Denable_cdma=true and -Denable_dbchecker=true'
        subdir_done()
endif

sources = files('main.c')
deps += ['cdma', 'dbchecker']
