allow_experimental_apis = true
sources = files(
        'main.c',
)

cflags += ['-I../../../../libs/LibDASICS/repo/include']
ldflags += ['-L../../../../libs/LibDASICS/build']
ldflags += ['-lDASICS']
ldflags += ['-T../../../../libs/LibDASICS/repo/ld.lds']