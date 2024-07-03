#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

USER_CFLAGS=$CFLAGS # save userspace CFLAGS options

AC_PREREQ([2.69])
AC_INIT([acs_routine], [0.0.1], [dachuan516@gmail.com])
#AC_SUBST([PROG_VER_INFO_LIBTOOL], [$PROG_VER_INFO_LIBTOOL])
AC_CONFIG_AUX_DIR([build-aux])
AM_INIT_AUTOMAKE([-Wall -Werror foreign subdir-objects]) # Display all errors, treat warning as error, following minimum GNU policy, enable automake to look into subdirectories
AC_CONFIG_SRCDIR([attestlog_AD])
AC_CONFIG_SRCDIR([traffic_AD])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])
AM_PROG_AR

# Checks for programs.
AC_PROG_CC
LT_INIT([disable-static])

# Checks for libraries.
#AC_CHECK_LIB([pthread], [pthread_mutex_trylock], [have_pthread=1], [have_pthread=0])
# libtrace and its dependencies can't be checked by PKG_CHECK_MODULES since they don't have .pc files
AC_CHECK_LIB([trace], [trace_create_packet], [], [echo -e "\nlibtrace not found\n"; exit 1])
AC_CHECK_LIB([trace], [trace_create], [], [echo -e "\nlibtrace not found\n"; exit 1])
AC_CHECK_LIB([trace], [trace_is_err], [], [echo -e "\nlibtrace not found\n"; exit 1])
AC_CHECK_LIB([trace], [trace_perror], [], [echo -e "\nlibtrace not found\n"; exit 1])
AC_CHECK_LIB([trace], [trace_start], [], [echo -e "\nlibtrace not found\n"; exit 1])
AC_CHECK_LIB([trace], [trace_read_packet], [], [echo -e "\nlibtrace not found\n"; exit 1])
AC_CHECK_LIB([trace], [trace_get_timespec], [], [echo -e "\nlibtrace not found\n"; exit 1])
AC_CHECK_LIB([trace], [trace_get_timeval], [], [echo -e "\nlibtrace not found\n"; exit 1])
AC_CHECK_LIB([trace], [trace_destroy], [], [echo -e "\nlibtrace not found\n"; exit 1])
AC_CHECK_LIB([trace], [trace_destroy_packet], [], [echo -e "\nlibtrace not found\n"; exit 1])
PKG_PROG_PKG_CONFIG
PKG_CHECK_MODULES([MARIADB], [mariadb >= 3.1.22])

# Checks for header files.
AC_CHECK_HEADERS([stdio.h stdlib.h stdbool.h string.h signal.h unistd.h errno.h time.h inttypes.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_TYPE_SIZE_T
AC_TYPE_INT8_T
AC_TYPE_INT16_T
AC_TYPE_INT32_T
AC_TYPE_INT64_T
AC_TYPE_UINT8_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT64_T

# Checks for library functions.
#AC_FUNC_MALLOC
#AC_FUNC_REALLOC
#AC_CHECK_FUNCS([access
#                clock_gettime
#                exit
#                printf perror
#                setvbuf strcmp strstr signal sizeof snprintf strdup strtod strtol strerror])

AC_CONFIG_FILES([Makefile
                 attestlog_AD/Makefile
                 traffic_AD/Makefile])
AC_SUBST([ADD_LIBS])
AC_SUBST([EXTRA_LIBS])
AC_SUBST([ADD_LDFLAGS])
AC_SUBST([ADD_INCLS])
AC_SUBST([LTLIBOBJS])
AC_OUTPUT
