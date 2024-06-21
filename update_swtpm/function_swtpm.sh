#!/bin/bash
#set -x
set -e

tar_flag="xf"
wget_flag="-q --show-progress"
working_dir="$HOME/Downloads"

# Note: 
#   ordering of downloading and compiling is different due to networking consideration
#   following option switches is mostly the order required to install due to dependency issue
# 1: install; others: pass
install_apt_package=0

install_pkgconfig=0
install_libtasn1=0
install_python=0
install_pip_package=0
install_libpcre2=0
install_glib=1
install_jsonglib=1
install_help2man=0
install_autoconf=0
install_gperf=0
install_m4=0
install_automake=0
install_texinfo=0
install_libtool=0
install_gmp=0
install_jsonc=0
install_tpm2tss=0
install_libtpms=0

#>1 https://github.com/stefanberger/swtpm/releases
libtpms_version="0.9.6"
libtpms_dirname="$working_dir/libtpms"
libtpms_name="$libtpms_dirname-$libtpms_version"
#>2 https://github.com/tpm2-software/tpm2-tss/releases
tpm2tss_version="4.1.3"
tpm2tss_dirname="$working_dir/tpm2tss"
tpm2tss_name="$tpm2tss_dirname-$tpm2tss_version"
#>3 https://github.com/json-c/json-c/tags
jsonc_version="0.17"
jsonc_dirname="$working_dir/jsonc"
jsonc_name="$jsonc_dirname-$jsonc_version"
#>4 https://gmplib.org/repo/
gmp_version="6.3"
gmp_dirname="$working_dir/gmp"
gmp_name="$gmp_dirname-$gmp_version"
#>5 http://ftp.twaren.net/Unix/GNU/gnu/texinfo/
texinfo_version="7.1"
texinfo_dirname="$working_dir/texinfo"
texinfo_name="$texinfo_dirname-$texinfo_version"
#>6 http://ftp.twaren.net/Unix/GNU/gnu/automake/
automake_version="1.16"
automake_dirname="$working_dir/automake"
automake_name="$automake_dirname-$automake_version"
#>7 http://ftp.twaren.net/Unix/GNU/gnu/autoconf/
autoconf_version="2.72"
autoconf_dirname="$working_dir/autoconf"
autoconf_name="$autoconf_dirname-$autoconf_version"
#>8 http://ftp.twaren.net/Unix/GNU/gnu/help2man/
help2man_version="1.49.3"
help2man_dirname="$working_dir/help2man"
help2man_name="$help2man_dirname-$help2man_version"
#>9 http://ftp.twaren.net/Unix/GNU/gnu/m4/
m4_version="1.4.19"
m4_dirname="$working_dir/m4"
m4_name="$m4_dirname-$m4_version"
#>10 http://ftp.twaren.net/Unix/GNU/gnu/gperf/
gperf_version="3.1"
gperf_dirname="$working_dir/gperf"
gperf_name="$gperf_dirname-$gperf_version"
#>11 https://pkgconfig.freedesktop.org/releases/
pkgconfig_version="0.29.2"
pkgconfig_dirname="$working_dir/pkgconfig"
pkgconfig_name="$pkgconfig_dirname-$pkgconfig_version"
#>12 http://ftp.twaren.net/Unix/GNU/gnu/libtool/
libtool_version="2.4.7"
libtool_dirname="$working_dir/libtool"
libtool_name="$libtool_dirname-$libtool_version"
#>13 http://ftp.twaren.net/Unix/GNU/gnu/libtasn1/
libtasn1_version="4.19.0"
libtasn1_dirname="$working_dir/libtasn1"
libtasn1_name="$libtasn1_dirname-$libtasn1_version"
#>14 https://www.python.org/downloads/source/
python_version="3.12.4"
python_dirname="$working_dir/python"
python_name="$python_dirname-$python_version"
#>15 pip package skipped (setuptools, packaging, ninja, meson)
#>15    newer python version installed packages will be up-to-date
#>16 https://github.com/PCRE2Project/pcre2/releases
libpcre2_version="10.44"
libpcre2_dirname="$working_dir/libpcre2"
libpcre2_name="$libpcre2_dirname-$libpcre2_version"
#>17 https://download.gnome.org/sources/glib/
glib_version="2.80.3"
glib_dirname="$working_dir/glib"
glib_name="$glib_dirname-$glib_version"
#>18 https://download.gnome.org/sources/json-glib/
jsonglib_version="1.8.0"
jsonglib_dirname="$working_dir/jsonglib"
jsonglib_name="$jsonglib_dirname-$jsonglib_version"
