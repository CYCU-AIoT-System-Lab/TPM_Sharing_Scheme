#!/bin/bash
#set -x
set -e

#   Ordering of downloading and compiling is different due to
# networking consideration, following option switches is mostly
# the order required to install due to dependency issue.
#
#   All of the options below should be on by default
#
#   After the entire installation process finishes, you can
# partially re-compiling as well
#
# Known issue:
#   1. re-compiling `install_swtpm` requires `install_pkgconfig`
# enabled, or else it would complains about syntax error about
# `PKG_CHECK_MODULES` pkg-config installed macro
# -> https://stackoverflow.com/questions/36659867/pkg-check-modules-syntax-error-near-luajit
 
# 1: proceed; others: pass
enable_wget_cert_skip=1
install_apt_package=1

install_pkgconfig=1
install_libffi=1
install_bison=1
install_openssl=1
install_python=1
install_pippackage=1
install_libpcre2=1
install_glib=1
install_jsonglib=1
install_libtasn1=1
install_help2man=1
install_autoconf=1
install_gperf=1
install_m4=1
install_automake=1
install_texinfo=1
install_libtool=1
install_gmp=1
install_jsonc=1
install_libcurl=1
install_gettext=1
install_flex=1
install_utillinux=1
install_nettle=1
install_libunistring=1
install_libev=1
install_p11kit=1
install_gnutls=1
install_tcsd=1
install_tcl=1
install_expect=1
install_gawk=1
install_socat=1
install_libseccomp=1
install_tpm2tss=1
install_tpm2tools=1
install_tpm2abrmd=1
install_tpm2tssengine=1
install_libtpms=1
install_swtpm=1

# This section is package version and file extension
# You can use specific link given below to check the newest version and update it

#>1 https://github.com/stefanberger/swtpm/releases
libtpms_version="0.9.6"
libtpms_ext=".tar.gz"
#>2 https://github.com/tpm2-software/tpm2-tss/releases
tpm2tss_version="4.1.3"
tpm2tss_ext=".tar.gz"
#>3 https://github.com/json-c/json-c/tags
jsonc_version="0.17"
jsonc_ext=".tar.gz"
#>4 https://gmplib.org/repo/
gmp_version="6.3"
#>5 http://ftp.twaren.net/Unix/GNU/gnu/texinfo/
texinfo_version="7.1"
texinfo_ext=".tar.gz"
#>6 http://ftp.twaren.net/Unix/GNU/gnu/automake/
automake_version="1.16.3"
automake_ext=".tar.gz"
#>7 http://ftp.twaren.net/Unix/GNU/gnu/autoconf/
autoconf_version="2.72"
autoconf_ext=".tar.gz"
#>8 http://ftp.twaren.net/Unix/GNU/gnu/help2man/
help2man_version="1.49.3"
help2man_ext=".tar.xz"
#>9 http://ftp.twaren.net/Unix/GNU/gnu/m4/
m4_version="1.4.19"
m4_ext=".tar.gz"
#>10 http://ftp.twaren.net/Unix/GNU/gnu/gperf/
gperf_version="3.1"
gperf_ext=".tar.gz"
#>11 https://pkgconfig.freedesktop.org/releases/
pkgconfig_version="0.29.2"
pkgconfig_ext=".tar.gz"
#>12 http://ftp.twaren.net/Unix/GNU/gnu/libtool/
libtool_version="2.4.7"
libtool_ext=".tar.gz"
#>13 http://ftp.twaren.net/Unix/GNU/gnu/libtasn1/
libtasn1_version="4.19.0"
libtasn1_ext=".tar.gz"
#>14 https://www.python.org/downloads/source/
python_version="3.12.4"
python_ext=".tgz"
#>15 pip package skipped (setuptools, packaging, ninja, meson)
#>15    newer python version installed pip packages will be up-to-date
#>16 https://github.com/PCRE2Project/pcre2/releases
libpcre2_version="10.44"
libpcre2_ext=".tar.gz"
#>17 https://download.gnome.org/sources/glib/
glib_version="2.80.3"
glib_ext=".tar.xz"
#>18 https://download.gnome.org/sources/json-glib/
jsonglib_version="1.8.0"
jsonglib_ext=".tar.xz"
#>19 http://ftp.twaren.net/Unix/GNU/gnu/bison/
bison_version="3.8.2"
bison_ext=".tar.gz"
#>20 https://github.com/openssl/openssl/releases
# old: https://www.openssl.org/source/old/1.1.1/index.html
#   include libcrypto libssl openssl
#openssl_version="3.3.1"
openssl_version="1.1.1w" # newer python requires < 3.0.0, but also not too old
openssl_ext=".tar.gz"
#>21 https://curl.se/download.html
libcurl_version="8.8.0"
libcurl_ext=".tar.gz"
#> 22 https://github.com/stefanberger/swtpm/releases
swtpm_version="0.9.0"
swtpm_ext=".tar.gz"
#> 23 http://ftp.twaren.net/Unix/GNU/gnu/gettext/
gettext_version="0.22.5"
gettext_ext=".tar.gz"
#> 24 https://github.com/westes/flex/releases
flex_version="2.6.3" # 2.6.4 results in compiling core dump
flex_ext=".tar.gz"
#> 25 https://github.com/util-linux/util-linux/tags
#   this is only used to build libuuid
#   installing entire project might break system
utillinux_version="2.40.1"
utillinux_ext=".tar.gz"
#> 26 http://ftp.twaren.net/Unix/GNU/gnu/nettle/
nettle_version="3.10"
nettle_ext=".tar.gz"
#> 27 http://ftp.twaren.net/Unix/GNU/gnu/libunistring/
libunistring_version="1.2"
libunistring_ext=".tar.gz"
#> 28 https://github.com/mksdev/libev-release/tags
libev_version="4.33"
libev_ext=".tar.gz"
#> 29 https://github.com/p11-glue/p11-kit/releases
p11kit_version="0.25.3"
p11kit_ext=".tar.xz"
#> 30 https://gitlab.com/gnutls/gnutls
#   release can't be built successfully, can only be cloned directly
#> 31 https://sourceforge.net/projects/trousers/files/trousers/
tcsd_version="0.3.15"
tcsd_ext=".tar.gz"
#> 32 https://sourceforge.net/projects/tcl/files/Tcl/
tcl_version="8.6.14"
tcl_ext=".tar.gz"
#> 33 https://sourceforge.net/projects/expect/files/Expect/
expect_version="5.45.4"
expect_ext=".tar.gz"
#> 34 http://ftp.twaren.net/Unix/GNU/gnu/gawk/
gawk_version="5.3.0"
gawk_ext=".tar.gz"
#> 35 http://www.dest-unreach.org/socat/
socat_version="1.8.0.0"
socat_ext=".tar.gz"
#> 36 https://github.com/seccomp/libseccomp/releases
libseccomp_version="2.5.5"
libseccomp_ext=".tar.gz"
#> 37 https://github.com/tpm2-software/tpm2-tools/releases
tpm2tools_version="5.7"
tpm2tools_ext=".tar.gz"
#> 38 https://github.com/tpm2-software/tpm2-abrmd/releases
tpm2abrmd_version="3.0.0"
tpm2abrmd_ext=".tar.gz"
#> 39 https://github.com/tpm2-software/tpm2-tss-engine/releases
tpm2tssengine_version="1.2.0"
tpm2tssengine_ext=".tar.gz"
#> 40 https://github.com/libffi/libffi/releases
libffi_version="3.4.6"
libffi_ext=".tar.gz"

# This section is configurations critical to this script
# DO NOT MODIFY unless you understand how it work

install_platform=1

tar_flag="xf"
working_dir="$HOME/Downloads"
wget_flag="-q --show-progress" # --no-check-certificate
wget_rc_path="$HOME/.wgetrc"

github="https://github.com"
gnu_mirror="ftp://ftp.twaren.net/Unix/GNU/gnu"
gnome="https://download.gnome.org/sources"
sourceforge="https://sourceforge.net/projects"

libtpms_origin_name="libtpms"
libtpms_custom_name="libtpms"
libtpms_dirname="$working_dir/$libtpms_custom_name"
libtpms_name="$libtpms_dirname-$libtpms_version"

tpm2tss_origin_name="tpm2-tss"
tpm2tss_custom_name="tpm2tss"
tpm2tss_dirname="$working_dir/$tpm2tss_custom_name"
tpm2tss_name="$tpm2tss_dirname-$tpm2tss_version"

jsonc_origin_name="json-c"
jsonc_custom_name="jsonc"
jsonc_dirname="$working_dir/$jsonc_custom_name"
jsonc_name="$jsonc_dirname-$jsonc_version"
jsonc_build_path="$working_dir/jsonc-build"

gmp_origin_name="gmp"
gmp_custom_name="$gmp_origin_name"
gmp_dirname="$working_dir/$gmp_custom_name"
gmp_name="$gmp_dirname-$gmp_version"

texinfo_origin_name="texinfo"
texinfo_custom_name="texinfo"
texinfo_dirname="$working_dir/$texinfo_custom_name"
texinfo_name="$texinfo_dirname-$texinfo_version"

automake_origin_name="automake"
automake_custom_name="automake"
automake_dirname="$working_dir/$automake_custom_name"
automake_name="$automake_dirname-$automake_version"

autoconf_origin_name="autoconf"
autoconf_custom_name="autoconf"
autoconf_dirname="$working_dir/$autoconf_custom_name"
autoconf_name="$autoconf_dirname-$autoconf_version"

help2man_origin_name="help2man"
help2man_custom_name="help2man"
help2man_dirname="$working_dir/$help2man_custom_name"
help2man_name="$help2man_dirname-$help2man_version"

m4_origin_name="m4"
m4_custom_name="m4"
m4_dirname="$working_dir/$m4_custom_name"
m4_name="$m4_dirname-$m4_version"

gperf_origin_name="gperf"
gperf_custom_name="gperf"
gperf_dirname="$working_dir/$gperf_custom_name"
gperf_name="$gperf_dirname-$gperf_version"

pkgconfig_origin_name="pkg-config"
pkgconfig_custom_name="pkgconfig"
pkgconfig_dirname="$working_dir/$pkgconfig_custom_name"
pkgconfig_name="$pkgconfig_dirname-$pkgconfig_version"

libtool_origin_name="libtool"
libtool_custom_name="libtool"
libtool_dirname="$working_dir/$libtool_custom_name"
libtool_name="$libtool_dirname-$libtool_version"

libtasn1_origin_name="libtasn1"
libtasn1_custom_name="libtasn1"
libtasn1_dirname="$working_dir/$libtasn1_custom_name"
libtasn1_name="$libtasn1_dirname-$libtasn1_version"

python_origin_name="python"
python_custom_name="python"
python_dirname="$working_dir/$python_custom_name"
python_name="$python_dirname-$python_version"
python_bin_path="$python_dirname/bin"
python_venv_path="$working_dir/$python_custom_name-$python_version-build-venv" # venv used for library building

pippackage_origin_name="pip-package"

libpcre2_origin_name="libpcre2"
libpcre2_custom_name="libpcre2"
libpcre2_dirname="$working_dir/$libpcre2_custom_name"
libpcre2_name="$libpcre2_dirname-$libpcre2_version"

glib_origin_name="glib"
glib_custom_name="glib"
glib_dirname="$working_dir/$glib_custom_name"
glib_name="$glib_dirname-$glib_version"

jsonglib_origin_name="json-glib"
jsonglib_custom_name="jsonglib"
jsonglib_dirname="$working_dir/$jsonglib_custom_name"
jsonglib_name="$jsonglib_dirname-$jsonglib_version"

bison_origin_name="bison"
bison_custom_name="bison"
bison_dirname="$working_dir/$bison_custom_name"
bison_name="$bison_dirname-$bison_version"

openssl_origin_name="openssl"
openssl_custom_name="openssl"
openssl_dirname="$working_dir/$openssl_custom_name"
openssl_name="$openssl_dirname-$openssl_version"

libcurl_origin_name="libcurl"
libcurl_custom_name="libcurl"
libcurl_dirname="$working_dir/$libcurl_custom_name"
libcurl_name="$libcurl_dirname-$libcurl_version"

swtpm_origin_name="swtpm"
swtpm_custom_name="swtpm"
swtpm_dirname="$working_dir/$swtpm_custom_name"
swtpm_name="$swtpm_dirname-$swtpm_version"

gettext_origin_name="gettext"
gettext_custom_name="gettext"
gettext_dirname="$working_dir/$gettext_custom_name"
gettext_name="$gettext_dirname-$gettext_version"

flex_origin_name="flex"
flex_custom_name="flex"
flex_dirname="$working_dir/$flex_custom_name"
flex_name="$flex_dirname-$flex_version"

utillinux_origin_name="utillinux"
utillinux_custom_name="util-linux"
utillinux_dirname="$working_dir/$utillinux_custom_name"
utillinux_name="$utillinux_dirname-$utillinux_version"

nettle_origin_name="nettle"
nettle_custom_name="nettle"
nettle_dirname="$working_dir/$nettle_custom_name"
nettle_name="$nettle_dirname-$nettle_version"

libunistring_origin_name="libunistring"
libunistring_custom_name="libunistring"
libunistring_dirname="$working_dir/$libunistring_custom_name"
libunistring_name="$libunistring_dirname-$libunistring_version"

libev_origin_name="libev"
libev_custom_name="libev"
libev_dirname="$working_dir/$libev_custom_name"
libev_name="$libev_dirname-$libev_version"

p11kit_origin_name="p11-kit"
p11kit_custom_name="p11kit"
p11kit_dirname="$working_dir/$p11kit_custom_name"
p11kit_name="$p11kit_dirname-$p11kit_version"

gnutls_origin_name="gnutls"
gnutls_custom_name="gnutls"
gnutls_dirname="$working_dir/$gnutls_custom_name"
gnutls_name="$gnutls_dirname"

tcsd_origin_name="tcsd"
tcsd_custom_name="tcsd"
tcsd_dirname="$working_dir/$tcsd_custom_name"
tcsd_name="$tcsd_dirname-$tcsd_version"

tcl_origin_name="tcl"
tcl_custom_name="tcl"
tcl_dirname="$working_dir/$tcl_custom_name"
tcl_name="$tcl_dirname-$tcl_version"

expect_origin_name="expect"
expect_custom_name="expect"
expect_dirname="$working_dir/$expect_custom_name"
expect_name="$expect_dirname-$expect_version"

gawk_origin_name="gawk"
gawk_custom_name="gawk"
gawk_dirname="$working_dir/$gawk_custom_name"
gawk_name="$gawk_dirname-$gawk_version"

socat_origin_name="socat"
socat_custom_name="socat"
socat_dirname="$working_dir/$socat_custom_name"
socat_name="$socat_dirname-$socat_version"

libseccomp_origin_name="libseccomp"
libseccomp_custom_name="libseccomp"
libseccomp_dirname="$working_dir/$libseccomp_custom_name"
libseccomp_name="$libseccomp_dirname-$libseccomp_version"

tpm2tools_origin_name="tpm2-tools"
tpm2tools_custom_name="tpm2tools"
tpm2tools_dirname="$working_dir/$tpm2tools_custom_name"
tpm2tools_name="$tpm2tools_dirname-$tpm2tools_version"

tpm2abrmd_origin_name="tpm2-abrmd"
tpm2abrmd_custom_name="tpm2abrmd"
tpm2abrmd_dirname="$working_dir/$tpm2abrmd_custom_name"
tpm2abrmd_name="$tpm2abrmd_dirname-$tpm2abrmd_version"

tpm2tssengine_origin_name="tpm2-tss-engine"
tpm2tssengine_custom_name="tpm2tssengine"
tpm2tssengine_dirname="$working_dir/$tpm2tssengine_custom_name"
tpm2tssengine_name="$tpm2tssengine_dirname-$tpm2tssengine_version"

libffi_origin_name="libffi"
libffi_custom_name="libffi"
libffi_dirname="$working_dir/$libffi_custom_name"
libffi_name="$libffi_dirname-$libffi_version"
