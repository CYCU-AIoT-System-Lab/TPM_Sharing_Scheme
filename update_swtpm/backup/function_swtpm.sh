#!/bin/bash
#set -x
set -e

tar_flag="xf"
wget_flag="-q --show-progress"
working_dir="$HOME/Downloads"

# Note: 
#   ordering of downloading and compiling is different due to networking consideration
#   following option switch is mostly the order required to install due to dependency issue
# 1: install; others: pass
install_apt_package=0

install_help2man=1
install_autoconf=1
install_gperf=1
install_m4=1
install_automake=1
install_texinfo=1
install_libtool=1
install_gmp=1
install_jsonc=1
install_tpm2tss=1
install_pkgconfig=1
install_libtpms=1

#>1
libtpms_version="0.9.6"
libtpms_dirname="$working_dir/libtpms"
libtpms_name="$libtpms_dirname-$libtpms_version"
#>2
tpm2tss_version="4.1.3"
tpm2tss_dirname="$working_dir/tpm2tss"
tpm2tss_name="$tpm2tss_dirname-$tpm2tss_version"
#>3
jsonc_version="0.17"
jsonc_dirname="$working_dir/jsonc"
jsonc_name="$jsonc_dirname-$jsonc_version"
#>4
gmp_version="6.3"
gmp_dirname="$working_dir/gmp"
gmp_name="$gmp_dirname-$gmp_version"
#>5
texinfo_version="7.1"
texinfo_dirname="$working_dir/texinfo"
texinfo_name="$texinfo_dirname-$texinfo_version"
#>6
automake_version="1.16"
automake_dirname="$working_dir/automake"
automake_name="$automake_dirname-$automake_version"
#>7
autoconf_version="2.72"
autoconf_dirname="$working_dir/autoconf"
autoconf_name="$autoconf_dirname-$autoconf_version"
#>8
help2man_version="1.49.3"
help2man_dirname="$working_dir/help2man"
help2man_name="$help2man_dirname-$help2man_version"
#>9
m4_version="1.4.19"
m4_dirname="$working_dir/m4"
m4_name="$m4_dirname-$m4_version"
#>10
gperf_version="3.1"
gperf_dirname="$working_dir/gperf"
gperf_name="$gperf_dirname-$gperf_version"
#>11
pkgconfig_version="0.29.2"
pkgconfig_dirname="$working_dir/pkgconfig"
pkgconfig_name="$pkgconfig_dirname-$pkgconfig_version"
#>12
libtool_version="2.4.7"
libtool_dirname="$working_dir/libtool"
libtool_name="$libtool_dirname-$libtool_version"
