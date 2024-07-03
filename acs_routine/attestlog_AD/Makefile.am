project = attestlog_AD
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
bindir              = $(prefix)/bin/${project}
attestlog_ADdir  	= $(prefix)/bin/${project}

# ====================================
# add executable to build
# ====================================
bin_PROGRAMS   = attestlog_AD

# ====================================
# add source to build executable
# NOTE: need to use the executable name as prefix
# ====================================
attestlog_AD_SOURCES = main.c
attestlog_AD_CFLAGS = $(common_cflag) @MARIADB_CFLAGS@
attestlog_AD_LDADD = -lm -L/usr/local/lib @MARIADB_LIBS@
attestlog_AD_LDFLAGS = -I/usr/local/include
