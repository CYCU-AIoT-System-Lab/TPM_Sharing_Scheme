project = libacs
common_cflag = -Wno-implicit-function-declaration -Wextra -Wall -Wfloat-equal -Wundef -Wshadow -Wpointer-arith -Wcast-align -Wstrict-prototypes -Wstrict-overflow=5 -Wwrite-strings -Waggregate-return -Wcast-qual -Wswitch-default -Wswitch-enum -Wconversion -Wunreachable-code -Wformat=2 -O2

# ====================================
# install directory of public:
# - plugins         (.so, .a, .la)
# - libtool library (.so, .a, .la)
# - header          (.h)
# - binary
# NOTE: comment out if not needed of individual install
# NOTE: created directory won't be removed by uninstall
# NOTE: header ladir need to use the library name as prefix
# ====================================
libdir              = $(prefix)/lib/${project}
libacs_ladir    = $(prefix)/include/${project}

# ====================================
# add library to install as plugin
# NOTE: can't be used with plugin_LTLIBRARIES
# ====================================
lib_LTLIBRARIES = libacs.la

# ====================================
# add source to build library
# NOTE: need to use the library name as prefix
# ====================================
libacs_la_SOURCES = lib_signal_handler.c
libacs_la_HEADERS = lib_signal_handler.h
libacs_la_CFLAGS = $(common_cflag)
libacs_la_LDFLAGS = -Wl, --no-as-needed
