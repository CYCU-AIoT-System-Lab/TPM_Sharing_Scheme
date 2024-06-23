#!/bin/bash
#set -x
set -e

tar_flag="xf"
wget_flag="-q --show-progress --no-check-certificate"
working_dir="$HOME/Downloads"

install_platform=1

# Note: 
#   ordering of downloading and compiling is different due to networking consideration
#   following option switches is mostly the order required to install due to dependency issue
# 1: install; others: pass
install_apt_package=0

install_pkgconfig=0
install_bison=0
install_openssl=0
install_python=0
install_pip_package=0
install_libpcre2=0
install_glib=0
install_jsonglib=0
install_libtasn1=0
install_help2man=0
install_autoconf=0
install_gperf=0
install_m4=0
install_automake=0
install_texinfo=0
install_libtool=0
install_gmp=0
install_jsonc=0
install_libcurl=0
install_gettext=0
install_flex=0
install_utillinux=0
install_tpm2tss=0
install_libtpms=0
install_swtpm=1

#>1 https://github.com/stefanberger/swtpm/releases
libtpms_version="0.9.6"
libtpms_ext=".tar.gz"
libtpms_dirname="$working_dir/libtpms"
libtpms_name="$libtpms_dirname-$libtpms_version"
#>2 https://github.com/tpm2-software/tpm2-tss/releases
tpm2tss_version="4.1.3"
tpm2tss_ext=".tar.gz"
tpm2tss_dirname="$working_dir/tpm2tss"
tpm2tss_name="$tpm2tss_dirname-$tpm2tss_version"
#>3 https://github.com/json-c/json-c/tags
jsonc_version="0.17"
jsonc_ext=".tar.gz"
jsonc_dirname="$working_dir/jsonc"
jsonc_name="$jsonc_dirname-$jsonc_version"
jsonc_build_path="$working_dir/jsonc-build"
#>4 https://gmplib.org/repo/
gmp_version="6.3"
gmp_dirname="$working_dir/gmp"
gmp_name="$gmp_dirname-$gmp_version"
#>5 http://ftp.twaren.net/Unix/GNU/gnu/texinfo/
texinfo_version="7.1"
texinfo_ext=".tar.gz"
texinfo_dirname="$working_dir/texinfo"
texinfo_name="$texinfo_dirname-$texinfo_version"
#>6 http://ftp.twaren.net/Unix/GNU/gnu/automake/
automake_version="1.16.3"
automake_ext=".tar.gz"
automake_dirname="$working_dir/automake"
automake_name="$automake_dirname-$automake_version"
#>7 http://ftp.twaren.net/Unix/GNU/gnu/autoconf/
autoconf_version="2.72"
autoconf_ext=".tar.gz"
autoconf_dirname="$working_dir/autoconf"
autoconf_name="$autoconf_dirname-$autoconf_version"
#>8 http://ftp.twaren.net/Unix/GNU/gnu/help2man/
help2man_version="1.49.3"
help2man_ext=".tar.xz"
help2man_dirname="$working_dir/help2man"
help2man_name="$help2man_dirname-$help2man_version"
#>9 http://ftp.twaren.net/Unix/GNU/gnu/m4/
m4_version="1.4.19"
m4_ext=".tar.gz"
m4_dirname="$working_dir/m4"
m4_name="$m4_dirname-$m4_version"
#>10 http://ftp.twaren.net/Unix/GNU/gnu/gperf/
gperf_version="3.1"
gperf_ext=".tar.gz"
gperf_dirname="$working_dir/gperf"
gperf_name="$gperf_dirname-$gperf_version"
#>11 https://pkgconfig.freedesktop.org/releases/
pkgconfig_version="0.29.2"
pkgconfig_ext=".tar.gz"
pkgconfig_dirname="$working_dir/pkgconfig"
pkgconfig_name="$pkgconfig_dirname-$pkgconfig_version"
#>12 http://ftp.twaren.net/Unix/GNU/gnu/libtool/
libtool_version="2.4.7"
libtool_ext=".tar.gz"
libtool_dirname="$working_dir/libtool"
libtool_name="$libtool_dirname-$libtool_version"
#>13 http://ftp.twaren.net/Unix/GNU/gnu/libtasn1/
libtasn1_version="4.19.0"
libtasn1_ext=".tar.gz"
libtasn1_dirname="$working_dir/libtasn1"
libtasn1_name="$libtasn1_dirname-$libtasn1_version"
#>14 https://www.python.org/downloads/source/
python_version="3.12.4"
python_ext=".tgz"
python_dirname="$working_dir/python"
python_name="$python_dirname-$python_version"
python_bin_path="$python_dirname/bin"
python_venv_path="$working_dir/python_build_venv" # venv used for library building
#>15 pip package skipped (setuptools, packaging, ninja, meson)
#>15    newer python version installed packages will be up-to-date
#>16 https://github.com/PCRE2Project/pcre2/releases
libpcre2_version="10.44"
libpcre2_ext=".tar.gz"
libpcre2_dirname="$working_dir/libpcre2"
libpcre2_name="$libpcre2_dirname-$libpcre2_version"
#>17 https://download.gnome.org/sources/glib/
glib_version="2.80.3"
glib_ext=".tar.xz"
glib_dirname="$working_dir/glib"
glib_name="$glib_dirname-$glib_version"
#>18 https://download.gnome.org/sources/json-glib/
jsonglib_version="1.8.0"
jsonglib_ext=".tar.xz"
jsonglib_dirname="$working_dir/jsonglib"
jsonglib_name="$jsonglib_dirname-$jsonglib_version"
#>19 http://ftp.twaren.net/Unix/GNU/gnu/bison/
bison_version="3.8.2"
bison_ext=".tar.gz"
bison_dirname="$working_dir/bison"
bison_name="$bison_dirname-$bison_version"
#>20 https://github.com/openssl/openssl/releases
# old: https://www.openssl.org/source/old/1.1.1/index.html
#   include libcrypto libssl openssl
#openssl_version="3.3.1"
openssl_version="1.1.1w" # newer python requires < 3.0.0, but also not too old
openssl_ext=".tar.gz"
openssl_dirname="$working_dir/openssl"
openssl_name="$openssl_dirname-$openssl_version"
#>21 https://curl.se/download.html
libcurl_version="8.8.0"
libcurl_ext=".tar.gz"
libcurl_dirname="$working_dir/libcurl"
libcurl_name="$libcurl_dirname-$libcurl_version"
#> 22 https://github.com/stefanberger/swtpm/releases
swtpm_version="0.9.0"
swtpm_ext=".tar.gz"
swtpm_dirname="$working_dir/swtpm"
swtpm_name="$swtpm_dirname-$swtpm_version"
#> 23 http://ftp.twaren.net/Unix/GNU/gnu/gettext/
gettext_version="0.22.5"
gettext_ext=".tar.gz"
gettext_dirname="$working_dir/gettext"
gettext_name="$gettext_dirname-$gettext_version"
#> 24 https://github.com/westes/flex/releases
flex_version="2.6.3" # 2.6.4 results in compiling core dump
flex_ext=".tar.gz"
flex_dirname="$working_dir/flex"
flex_name="$flex_dirname-$flex_version"
#> 25 https://github.com/util-linux/util-linux/tags
#   this is only used to build libuuid
#   installing entire project might break system
utillinux_version="2.40.1"
utillinux_ext=".tar.gz"
utillinux_dirname="$working_dir/utillinux"
utillinux_name="$utillinux_dirname-$utillinux_version"
