#!/bin/bash

#   Core utilities finished in 20240625 by belongtothenight / Da-chuan Chen
#
#   This script is used to install all of the dependency of swtpm and
# optional packages, based on oldest oldest project supported platform
# Ubuntu 18.04 and jetson-nano-jp461
#
#   You can update installing software versions by changing settings
# in `function_swtpm.sh` file
#
# !!!! It is recommended to re-run installation script for circular
# dependency issue !!!!
#
#   If still you have dependency issue, you can check the following
# links and search for the package:
# 1. GitHub: https://github.com
# 2. GitLab: https://gitlab.com
# 3. Mirror of GNU: http://ftp.twaren.net/Unix/GNU/gnu
# 4. GNOME: https://download.gnome.org
# 5. SourceForge: https://sourceforge.net
#
#   If you have encountered compiling issue, you can try the following
# steps/approaches:
# 1. Check the `README` and `INSTALL` file in the source code
# 2. If `./autogen.sh` exists, run it. Do check its content whether it
#   runs `./configure` or not.
# 3. If `./configure` exists, run it.
# 4. If `meson.build` exists, it requires meson to build. Usually, it
#   follows the following steps:
#   1. meson setup _build
#   2. meson compile -C _build
#   3. meson install -C _build
# 5. If can't find meson or ninja (version incorrect), try to use python
#   virtual environment.
# 6. If after installation, and the package is still not found by pkgconfig,
#   you can use `find . -name '*.pc'` to find package files and copy them
#   to one of the following directories:
#   1. /usr/lib/pkgconfig
#   2. /usr/local/lib/pkgconfig
# 7. If encountered libxxx.so not found after install, you can use the
#   following command:
#   1. `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path_to_lib`
#   2. `ldconfig /path_to_lib`

script=$(realpath "$0")
script_path=$(dirname "$script")
source function_swtpm.sh

#> ------------------------------------------------
if [ $enable_wget_cert_skip -eq 1 ]; then
    echo "disabling wget certification check"
    wget_config_str="check_certificate = off"
    echo "$wget_config_str" >> $wget_rc_path
fi
#> ------------------------------------------------

total_cnt=2
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg installing package"
    msg2="$msg installing failed!"
    package="$1"
}
apt_install () {
    echo "$msg1"
    sudo apt install -y $package | ts "$msg" || { echo "$msg2"; exit 1; }
}
if [ $install_apt_package -eq 1 ]; then
    echo "installing apt packages"
    update_var "mercurial" && apt_install "$package"
    update_var "gtk-doc-tools" && apt_install "$package"
fi

# Note
#   following ordering is to proceed with most easy to fail operation first
echo "downloading from sources"
total_cnt=39
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg downloading source"
    msg2="$msg skipped!"
    msg3="$msg download failed, removed partial files!"
    msg4="$msg already existed, skipped!"
}
# git clone
update_var "$gnutls_origin_name"
if [ $install_gnutls -eq 1 ]; then
    if [ -d $gnutls_name ]; then echo "$msg4"; else echo "$msg1"
        git clone https://gitlab.com/gnutls/gnutls.git $gnutls_name || { rm -rf $gnutls_name; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
# hg clone
update_var "$gmp_origin_name"
if [ $install_gmp -eq 1 ]; then
    if [ -d $gmp_name ]; then echo "$msg4"; else echo "$msg1"
        hg clone https://gmplib.org/repo/gmp-$gmp_version/ $gmp_name || { rm -rf $gmp_name; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
# wget
update_var "$libtpms_origin_name"
if [ $install_libtpms -eq 1 ]; then
    file="$libtpms_name$libtpms_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$github/stefanberger/libtpms/archive/refs/tags/v$libtpms_version$libtpms_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$tpm2tss_origin_name"
if [ $install_tpm2tss -eq 1 ]; then
    file="$tpm2tss_name$tpm2tss_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$github/tpm2-software/tpm2-tss/releases/download/$tpm2tss_version/tpm2-tss-$tpm2tss_version$tpm2tss_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$jsonc_origin_name"
if [ $install_jsonc -eq 1 ]; then
    file="$jsonc_name$jsonc_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$github/json-c/json-c/archive/refs/tags/json-c-$jsonc_version-20230812$jsonc_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$texinfo_origin_name"
if [ $install_texinfo -eq 1 ]; then
    file="$texinfo_name$texinfo_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$gnu_mirror/texinfo/texinfo-$texinfo_version$texinfo_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$automake_origin_name"
if [ $install_automake -eq 1 ]; then
    file="$automake_name$automake_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$gnu_mirror/automake/automake-$automake_version$automake_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$autoconf_origin_name"
if [ $install_autoconf -eq 1 ]; then
    file="$autoconf_name$autoconf_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$gnu_mirror/autoconf/autoconf-$autoconf_version$autoconf_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$help2man_origin_name"
if [ $install_help2man -eq 1 ]; then
    file="$help2man_name$help2man_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$gnu_mirror/help2man/help2man-$help2man_version$help2man_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$m4_origin_name"
if [ $install_m4 -eq 1 ]; then
    file="$m4_name$m4_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$gnu_mirror/m4/m4-$m4_version$m4_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$gperf_origin_name"
if [ $install_gperf -eq 1 ]; then
    file="$gperf_name$gperf_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$gnu_mirror/gperf/gperf-$gperf_version$gperf_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$pkgconfig_origin_name"
if [ $install_pkgconfig -eq 1 ]; then
    file="$pkgconfig_name$pkgconfig_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "https://pkgconfig.freedesktop.org/releases/pkg-config-$pkgconfig_version$pkgconfig_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$libtool_origin_name"
if [ $install_libtool -eq 1 ]; then
    file="$libtool_name$libtool_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$gnu_mirror/libtool/libtool-$libtool_version$libtool_ext" -O $file || { rm -f $file; echo "$msg4"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$libtasn1_origin_name"
if [ $install_libtasn1 -eq 1 ]; then
    file="$libtasn1_name$libtasn1_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$gnu_mirror/libtasn1/libtasn1-$libtasn1_version$libtasn1_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$python_origin_name"
if [ $install_python -eq 1 ]; then
    file="$python_name$python_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "https://www.python.org/ftp/python/$python_version/Python-$python_version$python_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$libpcre2_origin_name"
if [ $install_libpcre2 -eq 1 ]; then
    file="$libpcre2_name$libpcre2_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$github/PCRE2Project/pcre2/releases/download/pcre2-$libpcre2_version/pcre2-$libpcre2_version$libpcre2_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$glib_origin_name"
if [ $install_glib -eq 1 ]; then
    file="$glib_name$glib_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$gnome/glib/${glib_version%.*}/glib-$glib_version$glib_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$jsonglib_origin_name"
if [ $install_jsonglib -eq 1 ]; then
    file="$jsonglib_name$jsonglib_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$gnome/json-glib/${jsonglib_version%.*}/json-glib-$jsonglib_version$jsonglib_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$bison_origin_name"
if [ $install_bison -eq 1 ]; then
    file="$bison_name$bison_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$gnu_mirror/bison/bison-$bison_version$bison_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$openssl_origin_name"
if [ $install_openssl -eq 1 ]; then
    file="$openssl_name$openssl_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        openssl_ver_arr=(${openssl_version//./ } )
        if [[ ${openssl_ver_arr[0]} -ge 3 ]]; then
            wget $wget_flag "$github/openssl/openssl/releases/download/openssl-$openssl_version/openssl-$openssl_version$openssl_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
        else
            wget $wget_flag "https://www.openssl.org/source/old/${openssl_version::-1}/openssl-$openssl_version$openssl_ext" -O $openssl_name$openssl_ext || { rm -f $file; echo "$msg3"; exit 1; }
        fi
    fi
else echo "$msg2"; fi
update_var "$libcurl_origin_name"
if [ $install_libcurl -eq 1 ]; then
    file="$libcurl_name$libcurl_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "https://curl.se/download/curl-$libcurl_version$libcurl_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$swtpm_origin_name"
if [ $install_swtpm -eq 1 ]; then
    file="$swtpm_name$swtpm_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$github/stefanberger/swtpm/archive/refs/tags/v$swtpm_version$swtpm_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$gettext_origin_name"
if [ $install_gettext -eq 1 ]; then
    file="$gettext_name$gettext_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$gnu_mirror/gettext/gettext-$gettext_version$gettext_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$flex_origin_name"
if [ $install_flex -eq 1 ]; then
    file="$flex_name$flex_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$github/westes/flex/releases/download/v$flex_version/flex-$flex_version$flex_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$utillinux_origin_name"
if [ $install_utillinux -eq 1 ]; then
    file="$utillinux_name$utillinux_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$github/util-linux/util-linux/archive/refs/tags/v$utillinux_version$utillinux_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$nettle_origin_name"
if [ $install_nettle -eq 1 ]; then
    file="$nettle_name$nettle_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$gnu_mirror/nettle/nettle-$nettle_version$nettle_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$libunistring_origin_name"
if [ $install_libunistring -eq 1 ]; then
    file="$libunistring_name$libunistring_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$gnu_mirror/libunistring/libunistring-$libunistring_version$libunistring_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$libev_origin_name"
if [ $install_libev -eq 1 ]; then
    file="$libev_name$libev_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$github/mksdev/libev-release/archive/refs/tags/v$libev_version$libev_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$p11kit_origin_name"
if [ $install_p11kit -eq 1 ]; then
    file="$p11kit_name$p11kit_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$github/p11-glue/p11-kit/releases/download/$p11kit_version/p11-kit-$p11kit_version$p11kit_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$tcsd_origin_name"
if [ $install_tcsd -eq 1 ]; then
    file="$tcsd_name$tcsd_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$sourceforge/trousers/files/trousers/$tcsd_version/trousers-$tcsd_version$tcsd_ext/download" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$tcl_origin_name"
if [ $install_tcl -eq 1 ]; then
    file="$tcl_name$tcl_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$sourceforge/tcl/files/Tcl/$tcl_version/tcl$tcl_version-src$tcl_ext/download" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$expect_origin_name"
if [ $install_expect -eq 1 ]; then
    file="$expect_name$expect_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$sourceforge/expect/files/Expect/$expect_version/expect$expect_version$expect_ext/download" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$gawk_origin_name"
if [ $install_gawk -eq 1 ]; then
    file="$gawk_name$gawk_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$gnu_mirror/gawk/gawk-$gawk_version$gawk_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$socat_origin_name"
if [ $install_socat -eq 1 ]; then
    file="$socat_name$socat_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "www.dest-unreach.org/socat/download/socat-$socat_version$socat_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$libseccomp_origin_name"
if [ $install_libseccomp -eq 1 ]; then
    file="$libseccomp_name$libseccomp_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$github/seccomp/libseccomp/releases/download/v$libseccomp_version/libseccomp-$libseccomp_version$libseccomp_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$tpm2tools_origin_name"
if [ $install_tpm2tools -eq 1 ]; then
    file="$tpm2tools_name$tpm2tools_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$github/tpm2-software/tpm2-tools/releases/download/$tpm2tools_version/tpm2-tools-$tpm2tools_version$tpm2tools_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$tpm2abrmd_origin_name"
if [ $install_tpm2abrmd -eq 1 ]; then
    file="$tpm2abrmd_name$tpm2abrmd_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$github/tpm2-software/tpm2-abrmd/releases/download/$tpm2abrmd_version/tpm2-abrmd-$tpm2abrmd_version$tpm2abrmd_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$tpm2tssengine_origin_name"
if [ $install_tpm2tssengine -eq 1 ]; then
    file="$tpm2tssengine_name$tpm2tssengine_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$github/tpm2-software/tpm2-tss-engine/releases/download/$tpm2tssengine_version/tpm2-tss-engine-$tpm2tssengine_version$tpm2tssengine_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "$libffi_origin_name"
if [ $install_libffi -eq 1 ]; then
    file="$libffi_name$libffi_ext"
    if [ -f $file ]; then echo "$msg4"; else echo "$msg1"
        wget $wget_flag "$github/libffi/libffi/releases/download/v$libffi_version/libffi-$libffi_version$libffi_ext" -O $file || { rm -f $file; echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi

echo "creating directories to hold sources"
total_cnt=38
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg creating directory"
    msg2="$msg skipped!"
    msg3="$msg directory existed, skipped!"
}
update_var "$libtpms_origin_name"
if [ $install_libtpms -eq 1 ]; then
    if [ -d $libtpms_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $libtpms_name
    fi
else echo "$msg2"; fi
update_var "$tpm2tss_origin_name"
if [ $install_tpm2tss -eq 1 ]; then
    if [ -d $tpm2tss_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $tpm2tss_name
    fi
else echo "$msg2"; fi
update_var "$jsonc_origin_name"
if [ $install_jsonc -eq 1 ]; then
    if [ -d $jsonc_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $jsonc_name
    fi
else echo "$msg2"; fi
update_var "$texinfo_origin_name"
if [ $install_texinfo -eq 1 ]; then
    if [ -d $texinfo_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $texinfo_name
    fi
else echo "$msg2"; fi
update_var "$automake_origin_name"
if [ $install_automake -eq 1 ]; then
    if [ -d $automake_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $automake_name
    fi
else echo "$msg2"; fi
update_var "$autoconf_origin_name"
if [ $install_autoconf -eq 1 ]; then
    if [ -d $autoconf_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $autoconf_name
    fi
else echo "$msg2"; fi
update_var "$help2man_origin_name"
if [ $install_help2man -eq 1 ]; then
    if [ -d $help2man_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $help2man_name
    fi
else echo "$msg2"; fi
update_var "$m4_origin_name"
if [ $install_m4 -eq 1 ]; then
    if [ -d $m4_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $m4_name
    fi
else echo "$msg2"; fi
update_var "$gperf_origin_name"
if [ $install_gperf -eq 1 ]; then
    if [ -d $gperf_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $gperf_name
    fi
else echo "$msg2"; fi
update_var "$libtool_origin_name"
if [ $install_libtool -eq 1 ]; then
    if [ -d $libtool_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $libtool_name
    fi
else echo "$msg2"; fi
update_var "$pkgconfig_origin_name"
if [ $install_pkgconfig -eq 1 ]; then
    if [ -d $pkgconfig_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $pkgconfig_name
    fi
else echo "$msg2"; fi
update_var "$libtasn1_origin_name"
if [ $install_libtasn1 -eq 1 ]; then
    if [ -d $libtasn1_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $libtasn1_name
    fi
else echo "$msg2"; fi
update_var "$python_origin_name"
if [ $install_python -eq 1 ]; then
    if [ -d $python_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $python_name
    fi
else echo "$msg2"; fi
update_var "$libpcre2_origin_name"
if [ $install_libpcre2 -eq 1 ]; then
    if [ -d $libpcre2_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $libpcre2_name
    fi
else echo "$msg2"; fi
update_var "$glib_origin_name"
if [ $install_glib -eq 1 ]; then
    if [ -d $glib_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $glib_name
    fi
else echo "$msg2"; fi
update_var "$jsonglib_origin_name"
if [ $install_jsonglib -eq 1 ]; then
    if [ -d $jsonglib_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $jsonglib_name
    fi
else echo "$msg2"; fi
update_var "$bison_origin_name"
if [ $install_bison -eq 1 ]; then
    if [ -d $bison_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $bison_name
    fi
else echo "$msg2"; fi
update_var "$openssl_origin_name"
if [ $install_openssl -eq 1 ]; then
    if [ -d $openssl_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $openssl_name
    fi
else echo "$msg2"; fi
update_var "$libcurl_origin_name"
if [ $install_libcurl -eq 1 ]; then
    if [ -d $libcurl_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $libcurl_name
    fi
else echo "$msg2"; fi
update_var "$swtpm_origin_name"
if [ $install_swtpm -eq 1 ]; then
    if [ -d $swtpm_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $swtpm_name
    fi
else echo "$msg2"; fi
update_var "$gettext_origin_name"
if [ $install_gettext -eq 1 ]; then
    if [ -d $gettext_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $gettext_name
    fi
else echo "$msg2"; fi
update_var "$flex_origin_name"
if [ $install_flex -eq 1 ]; then
    if [ -d $flex_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $flex_name
    fi
else echo "$msg2"; fi
update_var "$utillinux_origin_name"
if [ $install_utillinux -eq 1 ]; then
    if [ -d $utillinux_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $utillinux_name
    fi
else echo "$msg2"; fi
update_var "$nettle_origin_name"
if [ $install_nettle -eq 1 ]; then
    if [ -d $nettle_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $nettle_name
    fi
else echo "$msg2"; fi
update_var "$libunistring_origin_name"
if [ $install_libunistring -eq 1 ]; then
    if [ -d $libunistring_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $libunistring_name
    fi
else echo "$msg2"; fi
update_var "$libev_origin_name"
if [ $install_libev -eq 1 ]; then
    if [ -d $libev_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $libev_name
    fi
else echo "$msg2"; fi
update_var "$p11kit_origin_name"
if [ $install_p11kit -eq 1 ]; then
    if [ -d $p11kit_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $p11kit_name
    fi
else echo "$msg2"; fi
update_var "$tcsd_origin_name"
if [ $install_tcsd -eq 1 ]; then
    if [ -d $tcsd_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $tcsd_name
    fi
else echo "$msg2"; fi
update_var "$tcl_origin_name"
if [ $install_tcl -eq 1 ]; then
    if [ -d $tcl_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $tcl_name
    fi
else echo "$msg2"; fi
update_var "$expect_origin_name"
if [ $install_expect -eq 1 ]; then
    if [ -d $expect_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $expect_name
    fi
else echo "$msg2"; fi
update_var "$gawk_origin_name"
if [ $install_gawk -eq 1 ]; then
    if [ -d $gawk_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $gawk_name
    fi
else echo "$msg2"; fi
update_var "$socat_origin_name"
if [ $install_socat -eq 1 ]; then
    if [ -d $socat_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $socat_name
    fi
else echo "$msg2"; fi
update_var "$libseccomp_origin_name"
if [ $install_libseccomp -eq 1 ]; then
    if [ -d $libseccomp_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $libseccomp_name
    fi
else echo "$msg2"; fi
update_var "$tpm2tools_origin_name"
if [ $install_tpm2tools -eq 1 ]; then
    if [ -d $tpm2tools_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $tpm2tools_name
    fi
else echo "$msg2"; fi
update_var "$tpm2abrmd_origin_name"
if [ $install_tpm2abrmd -eq 1 ]; then
    if [ -d $tpm2abrmd_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $tpm2abrmd_name
    fi
else echo "$msg2"; fi
update_var "$tpm2tssengine_origin_name"
if [ $install_tpm2tssengine -eq 1 ]; then
    if [ -d $tpm2tssengine_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $tpm2tssengine_name
    fi
else echo "$msg2"; fi
update_var "$libffi_origin_name"
if [ $install_libffi -eq 1 ]; then
    if [ -d $libffi_name ]; then echo "$msg3"; else echo "$msg1"
        mkdir $libffi_name
    fi
else echo "$msg2"; fi

echo "unzipping sources"
tar_add_flag="--strip-components=1"
total_cnt=37
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg unzipping"
    msg2="$msg skipped!"
}
update_var "$libtpms_origin_name"
if [ $install_libtpms -eq 1 ]; then echo "$msg1"
    tar $tar_flag $libtpms_name$libtpms_ext -C $libtpms_name $tar_add_flag
else echo "$msg2"; fi
update_var "$tpm2tss_origin_name"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg1"
    tar $tar_flag $tpm2tss_name$tpm2tss_ext -C $tpm2tss_name $tar_add_flag
else echo "$msg2"; fi
update_var "$jsonc_origin_name"
if [ $install_jsonc -eq 1 ]; then echo "$msg1"
    tar $tar_flag $jsonc_name$jsonc_ext -C $jsonc_name $tar_add_flag
else echo "$msg2"; fi
update_var "$texinfo_origin_name"
if [ $install_texinfo -eq 1 ]; then echo "$msg1"
    tar $tar_flag $texinfo_name$texinfo_ext -C $texinfo_name $tar_add_flag
else echo "$msg2"; fi
update_var "$automake_origin_name"
if [ $install_automake -eq 1 ]; then echo "$msg1"
    tar $tar_flag $automake_name$automake_ext -C $automake_name $tar_add_flag
else echo "$msg2"; fi
update_var "$autoconf_origin_name"
if [ $install_autoconf -eq 1 ]; then echo "$msg1"
    tar $tar_flag $autoconf_name$autoconf_ext -C $autoconf_name $tar_add_flag
else echo "$msg2"; fi
update_var "$help2man_origin_name"
if [ $install_help2man -eq 1 ]; then echo "$msg1"
    tar $tar_flag $help2man_name$help2man_ext -C $help2man_name $tar_add_flag
else echo "$msg2"; fi
update_var "$m4_origin_name"
if [ $install_m4 -eq 1 ]; then echo "$msg1"
    tar $tar_flag $m4_name$m4_ext -C $m4_name $tar_add_flag
else echo "$msg2"; fi
update_var "$gperf_origin_name"
if [ $install_gperf -eq 1 ]; then echo "$msg1"
    tar $tar_flag $gperf_name$gperf_ext -C $gperf_name $tar_add_flag
else echo "$msg2"; fi
update_var "$libtool_origin_name"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    tar $tar_flag $libtool_name$libtool_ext -C $libtool_name $tar_add_flag
else echo "$msg2"; fi
update_var "$pkgconfig_origin_name"
if [ $install_pkgconfig -eq 1 ]; then echo "$msg1"
    tar $tar_flag $pkgconfig_name$pkgconfig_ext -C $pkgconfig_name $tar_add_flag
else echo "$msg2"; fi
update_var "$libtasn1_origin_name"
if [ $install_libtasn1 -eq 1 ]; then echo "$msg1"
    tar $tar_flag $libtasn1_name$libtasn1_ext -C $libtasn1_name $tar_add_flag
else echo "$msg2"; fi
update_var "$python_origin_name"
if [ $install_python -eq 1 ]; then echo "$msg1"
    tar $tar_flag $python_name$python_ext -C $python_name $tar_add_flag
else echo "$msg2"; fi
update_var "$libpcre2_origin_name"
if [ $install_libpcre2 -eq 1 ]; then echo "$msg1"
    tar $tar_flag $libpcre2_name$libpcre2_ext -C $libpcre2_name $tar_add_flag
else echo "$msg2"; fi
update_var "$glib_origin_name"
if [ $install_glib -eq 1 ]; then echo "$msg1"
    tar $tar_flag $glib_name$glib_ext -C $glib_name $tar_add_flag
else echo "$msg2"; fi
update_var "$jsonglib_origin_name"
if [ $install_jsonglib -eq 1 ]; then echo "$msg1"
    tar $tar_flag $jsonglib_name$jsonglib_ext -C $jsonglib_name $tar_add_flag
else echo "$msg2"; fi
update_var "$bison_origin_name"
if [ $install_bison -eq 1 ]; then echo "$msg1"
    tar $tar_flag $bison_name$bison_ext -C $bison_name $tar_add_flag
else echo "$msg2"; fi
update_var "$openssl_origin_name"
if [ $install_openssl -eq 1 ]; then echo "$msg1"
    tar $tar_flag $openssl_name$openssl_ext -C $openssl_name $tar_add_flag
else echo "$msg2"; fi
update_var "$libcurl_origin_name"
if [ $install_libcurl -eq 1 ]; then echo "$msg1"
    tar $tar_flag $libcurl_name$libcurl_ext -C $libcurl_name $tar_add_flag
else echo "$msg2"; fi
update_var "$swtpm_origin_name"
if [ $install_swtpm -eq 1 ]; then echo "$msg1"
    tar $tar_flag $swtpm_name$swtpm_ext -C $swtpm_name $tar_add_flag
else echo "$msg2"; fi
update_var "$gettext_origin_name"
if [ $install_gettext -eq 1 ]; then echo "$msg1"
    tar $tar_flag $gettext_name$gettext_ext -C $gettext_name $tar_add_flag
else echo "$msg2"; fi
update_var "$flex_origin_name"
if [ $install_flex -eq 1 ]; then echo "$msg1"
    tar $tar_flag $flex_name$flex_ext -C $flex_name $tar_add_flag
else echo "$msg2"; fi
update_var "$utillinux_origin_name"
if [ $install_utillinux -eq 1 ]; then echo "$msg1"
    tar $tar_flag $utillinux_name$utillinux_ext -C $utillinux_name $tar_add_flag
else echo "$msg2"; fi
update_var "$nettle_origin_name"
if [ $install_nettle -eq 1 ]; then echo "$msg1"
    tar $tar_flag $nettle_name$nettle_ext -C $nettle_name $tar_add_flag
else echo "$msg2"; fi
update_var "$libunistring_origin_name"
if [ $install_libunistring -eq 1 ]; then echo "$msg1"
    tar $tar_flag $libunistring_name$libunistring_ext -C $libunistring_name $tar_add_flag
else echo "$msg2"; fi
update_var "$libev_origin_name"
if [ $install_libev -eq 1 ]; then echo "$msg1"
    tar $tar_flag $libev_name$libev_ext -C $libev_name $tar_add_flag
else echo "$msg2"; fi
update_var "$p11kit_origin_name"
if [ $install_p11kit -eq 1 ]; then echo "$msg1"
    tar $tar_flag $p11kit_name$p11kit_ext -C $p11kit_name $tar_add_flag
else echo "$msg2"; fi
update_var "$tcsd_origin_name"
if [ $install_tcsd -eq 1 ]; then echo "$msg1"
    tar $tar_flag $tcsd_name$tcsd_ext -C $tcsd_name $tar_add_flag
else echo "$msg2"; fi
update_var "$tcl_origin_name"
if [ $install_tcl -eq 1 ]; then echo "$msg1"
    tar $tar_flag $tcl_name$tcl_ext -C $tcl_name $tar_add_flag
else echo "$msg2"; fi
update_var "$expect_origin_name"
if [ $install_expect -eq 1 ]; then echo "$msg1"
    tar $tar_flag $expect_name$expect_ext -C $expect_name $tar_add_flag
else echo "$msg2"; fi
update_var "$gawk_origin_name"
if [ $install_gawk -eq 1 ]; then echo "$msg1"
    tar $tar_flag $gawk_name$gawk_ext -C $gawk_name $tar_add_flag
else echo "$msg2"; fi
update_var "$socat_origin_name"
if [ $install_socat -eq 1 ]; then echo "$msg1"
    tar $tar_flag $socat_name$socat_ext -C $socat_name $tar_add_flag
else echo "$msg2"; fi
update_var "$libseccomp_origin_name"
if [ $install_libseccomp -eq 1 ]; then echo "$msg1"
    tar $tar_flag $libseccomp_name$libseccomp_ext -C $libseccomp_name $tar_add_flag
else echo "$msg2"; fi
update_var "$tpm2tools_origin_name"
if [ $install_tpm2tools -eq 1 ]; then echo "$msg1"
    tar $tar_flag $tpm2tools_name$tpm2tools_ext -C $tpm2tools_name $tar_add_flag
else echo "$msg2"; fi
update_var "$tpm2abrmd_origin_name"
if [ $install_tpm2abrmd -eq 1 ]; then echo "$msg1"
    tar $tar_flag $tpm2abrmd_name$tpm2abrmd_ext -C $tpm2abrmd_name $tar_add_flag
else echo "$msg2"; fi
update_var "$tpm2tssengine_origin_name"
if [ $install_tpm2tssengine -eq 1 ]; then echo "$msg1"
    tar $tar_flag $tpm2tssengine_name$tpm2tssengine_ext -C $tpm2tssengine_name $tar_add_flag
else echo "$msg2"; fi
update_var "$libffi_origin_name"
if [ $install_libffi -eq 1 ]; then echo "$msg1"
    tar $tar_flag $libffi_name$libffi_ext -C $libffi_name $tar_add_flag
else echo "$msg2"; fi

echo "creating symlink to directories"
total_cnt=38
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg creating symlink to directory"
    msg2="$msg skipped!"
}
update_var "$gmp_origin_name"
if [ $install_gmp -eq 1 ]; then echo "$msg1"
    ln -sf $gmp_name $gmp_dirname
else echo "$msg2"; fi
update_var "$libtpms_origin_name"
if [ $install_libtpms -eq 1 ]; then echo "$msg1"
    ln -sf $libtpms_name $libtpms_dirname
else echo "$msg2"; fi
update_var "$tpm2tss_origin_name"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg1"
    ln -sf $tpm2tss_name $tpm2tss_dirname
else echo "$msg2"; fi
update_var "$jsonc_origin_name"
if [ $install_jsonc -eq 1 ]; then echo "$msg1"
    ln -sf $jsonc_name $jsonc_dirname
else echo "$msg2"; fi
update_var "$texinfo_origin_name"
if [ $install_texinfo -eq 1 ]; then echo "$msg1"
    ln -sf $texinfo_name $texinfo_dirname
else echo "$msg2"; fi
update_var "$automake_origin_name"
if [ $install_automake -eq 1 ]; then echo "$msg1"
    ln -sf $automake_name $automake_dirname
else echo "$msg2"; fi
update_var "$autoconf_origin_name"
if [ $install_autoconf -eq 1 ]; then echo "$msg1"
    ln -sf $autoconf_name $autoconf_dirname
else echo "$msg2"; fi
update_var "$help2man_origin_name"
if [ $install_help2man -eq 1 ]; then echo "$msg1"
    ln -sf $help2man_name $help2man_dirname
else echo "$msg2"; fi
update_var "$m4_origin_name"
if [ $install_m4 -eq 1 ]; then echo "$msg1"
    ln -sf $m4_name $m4_dirname
else echo "$msg2"; fi
update_var "$gperf_origin_name"
if [ $install_gperf -eq 1 ]; then echo "$msg1"
    ln -sf $gperf_name $gperf_dirname
else echo "$msg2"; fi
update_var "$libtool_origin_name"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    ln -sf $libtool_name $libtool_dirname
else echo "$msg2"; fi
update_var "$pkgconfig_origin_name"
if [ $install_pkgconfig -eq 1 ]; then echo "$msg1"
    ln -sf $pkgconfig_name $pkgconfig_dirname
else echo "$msg2"; fi
update_var "$libtasn1_origin_name"
if [ $install_libtasn1 -eq 1 ]; then echo "$msg1"
    ln -sf $libtasn1_name $libtasn1_dirname
else echo "$msg2"; fi
update_var "$python_origin_name"
if [ $install_python -eq 1 ]; then echo "$msg1"
    ln -sf $python_name $python_dirname
else echo "$msg2"; fi
update_var "$libpcre2_origin_name"
if [ $install_libpcre2 -eq 1 ]; then echo "$msg1"
    ln -sf $libpcre2_name $libpcre2_dirname
else echo "$msg2"; fi
update_var "$glib_origin_name"
if [ $install_glib -eq 1 ]; then echo "$msg1"
    ln -sf $glib_name $glib_dirname
else echo "$msg2"; fi
update_var "$jsonglib_origin_name"
if [ $install_jsonglib -eq 1 ]; then echo "$msg1"
    ln -sf $jsonglib_name $jsonglib_dirname
else echo "$msg2"; fi
update_var "$bison_origin_name"
if [ $install_bison -eq 1 ]; then echo "$msg1"
    ln -sf $bison_name $bison_dirname
else echo "$msg2"; fi
update_var "$openssl_origin_name"
if [ $install_openssl -eq 1 ]; then echo "$msg1"
    ln -sf $openssl_name $openssl_dirname
else echo "$msg2"; fi
update_var "$libcurl_origin_name"
if [ $install_libcurl -eq 1 ]; then echo "$msg1"
    ln -sf $libcurl_name $libcurl_dirname
else echo "$msg2"; fi
update_var "$swtpm_origin_name"
if [ $install_swtpm -eq 1 ]; then echo "$msg1"
    ln -sf $swtpm_name $swtpm_dirname
else echo "$msg2"; fi
update_var "$gettext_origin_name"
if [ $install_gettext -eq 1 ]; then echo "$msg1"
    ln -sf $gettext_name $gettext_dirname
else echo "$msg2"; fi
update_var "$flex_origin_name"
if [ $install_flex -eq 1 ]; then echo "$msg1"
    ln -sf $flex_name $flex_dirname
else echo "$msg2"; fi
update_var "$utillinux_origin_name"
if [ $install_utillinux -eq 1 ]; then echo "$msg1"
    ln -sf $utillinux_name $utillinux_dirname
else echo "$msg2"; fi
update_var "$nettle_origin_name"
if [ $install_nettle -eq 1 ]; then echo "$msg1"
    ln -sf $nettle_name $nettle_dirname
else echo "$msg2"; fi
update_var "$libunistring_origin_name"
if [ $install_libunistring -eq 1 ]; then echo "$msg1"
    ln -sf $libunistring_name $libunistring_dirname
else echo "$msg2"; fi
update_var "$libev_origin_name"
if [ $install_libev -eq 1 ]; then echo "$msg1"
    ln -sf $libev_name $libev_dirname
else echo "$msg2"; fi
update_var "$p11kit_origin_name"
if [ $install_p11kit -eq 1 ]; then echo "$msg1"
    ln -sf $p11kit_name $p11kit_dirname
else echo "$msg2"; fi
update_var "$tcsd_origin_name"
if [ $install_tcsd -eq 1 ]; then echo "$msg1"
    ln -sf $tcsd_name $tcsd_dirname
else echo "$msg2"; fi
update_var "$tcl_origin_name"
if [ $install_tcl -eq 1 ]; then echo "$msg1"
    ln -sf $tcl_name $tcl_dirname
else echo "$msg2"; fi
update_var "$expect_origin_name"
if [ $install_expect -eq 1 ]; then echo "$msg1"
    ln -sf $expect_name $expect_dirname
else echo "$msg2"; fi
update_var "$gawk_origin_name"
if [ $install_gawk -eq 1 ]; then echo "$msg1"
    ln -sf $gawk_name $gawk_dirname
else echo "$msg2"; fi
update_var "$socat_origin_name"
if [ $install_socat -eq 1 ]; then echo "$msg1"
    ln -sf $socat_name $socat_dirname
else echo "$msg2"; fi
update_var "$libseccomp_origin_name"
if [ $install_libseccomp -eq 1 ]; then echo "$msg1"
    ln -sf $libseccomp_name $libseccomp_dirname
else echo "$msg2"; fi
update_var "$tpm2tools_origin_name"
if [ $install_tpm2tools -eq 1 ]; then echo "$msg1"
    ln -sf $tpm2tools_name $tpm2tools_dirname
else echo "$msg2"; fi
update_var "$tpm2abrmd_origin_name"
if [ $install_tpm2abrmd -eq 1 ]; then echo "$msg1"
    ln -sf $tpm2abrmd_name $tpm2abrmd_dirname
else echo "$msg2"; fi
update_var "$tpm2tssengine_origin_name"
if [ $install_tpm2tssengine -eq 1 ]; then echo "$msg1"
    ln -sf $tpm2tssengine_name $tpm2tssengine_dirname
else echo "$msg2"; fi
update_var "$libffi_origin_name"
if [ $install_libffi -eq 1 ]; then echo "$msg1"
    ln -sf $libffi_name $libffi_dirname
else echo "$msg2"; fi

echo "compiling sources"
make_flag="-j$(nproc)"
gnu_compile () {
    ./configure $1
    make $make_flag
    sudo make $make_flag install
}
tpm_compile () {
    ./autogen.sh $1
    make $make_flag
    sudo make $make_flag install
}
meson_compile () {
    venv_bin_path="$python_venv_path/bin/activate"
    meson_path="$python_venv_path/bin/meson"
    source $venv_bin_path
    $meson_path setup _build
    $meson_path compile -C _build
    sudo bash -c "source $venv_bin_path && \
        $meson_path install -C _build"
    sudo cp _build/meson-private/*.pc /usr/local/lib/pkgconfig
}
total_cnt=40
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg compiling and installing"
    msg2="$msg skipped!"
}
update_var "$pkgconfig_origin_name"
if [ $install_pkgconfig -eq 1 ]; then echo "$msg1"
    cd $pkgconfig_dirname
    gnu_compile "--with-internal-glib" | ts "$msg"
else echo "$msg2"; fi
update_var "$libffi_origin_name"
if [ $install_libffi -eq 1 ]; then echo "$msg1"
    cd $libffi_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$bison_origin_name"
if [ $install_bison -eq 1 ]; then echo "$msg1"
    cd $bison_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$openssl_origin_name"
if [ $install_openssl -eq 1 ]; then echo "$msg1"
    cd $openssl_dirname
    openssl_ver_arr=(${openssl_version//./ } )
    if [[ ${openssl_ver_arr[0]} -ge 3 ]]; then
        ./Configure | ts "$msg"
    else
        ./config | ts "$msg"
    fi
    make $make_flag | ts "$msg"
    sudo make $make_flag install | ts "$msg"
    sudo cp libcrypto.pc /usr/local/lib/pkgconfig | ts "$msg"
    sudo cp libssl.pc /usr/local/lib/pkgconfig | ts "$msg"
    sudo cp openssl.pc /usr/local/lib/pkgconfig | ts "$msg"
else echo "$msg2"; fi
update_var "$python_origin_name"
if [ $install_python -eq 1 ]; then echo "$msg1"
    cd $python_dirname
    sudo ./configure --prefix=$python_dirname --exec-prefix=$python_dirname --enable-shared | ts "$msg"
    sudo make $make_flag | ts "$msg"
    sudo make $make_flag altinstall | ts "$msg"
    sudo ldconfig
else echo "$msg2"; fi
update_var "$pippackage_origin_name"
if [ $install_pippackage -eq 1 ]; then echo "$msg1"
    echo "$msg creating virtual environment for library building"
    "$python_bin_path/python${python_version%.*}" -m venv $python_venv_path
    #echo "$msg installing for newly built python globally"
    #pip_path="$python_dirname/bin/pip${python_version%.*}"
    #sudo -H $pip_path install -U pip setuptools | ts "$msg"
    #sudo -H $pip_path install packaging ninja meson | ts "$msg"
    echo "$msg installing for newly created virtual environment"
    source $python_venv_path/bin/activate
    pip install -U pip setuptools | ts "$msg"
    pip install packaging ninja meson | ts "$msg"
    deactivate
else echo "$msg2"; fi
update_var "$libpcre2_origin_name"
if [ $install_libpcre2 -eq 1 ]; then echo "$msg1"
    cd $libpcre2_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$glib_origin_name"
if [ $install_glib -eq 1 ]; then echo "$msg1"
    cd $glib_dirname
    meson_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$jsonglib_origin_name"
if [ $install_jsonglib -eq 1 ]; then echo "$msg1"
    cd $jsonglib_dirname
    # `meson setup` process contains `git clone`, considering fail-redo mechanism
    meson_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$libtasn1_origin_name"
if [ $install_libtasn1 -eq 1 ]; then echo "$msg1"
    cd $libtasn1_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$help2man_origin_name"
if [ $install_help2man -eq 1 ]; then echo "$msg1"
    cd $help2man_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$autoconf_origin_name"
if [ $install_autoconf -eq 1 ]; then echo "$msg1"
    cd $autoconf_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$gperf_origin_name"
if [ $install_gperf -eq 1 ]; then echo "$msg1"
    cd $gperf_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$m4_origin_name"
if [ $install_m4 -eq 1 ]; then echo "$msg1"
    cd $m4_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$automake_origin_name"
if [ $install_automake -eq 1 ]; then echo "$msg1"
    cd $automake_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$texinfo_origin_name"
if [ $install_texinfo -eq 1 ]; then echo "$msg1"
    cd $texinfo_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$libtool_origin_name"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    cd $libtool_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$gmp_origin_name"
if [ $install_gmp -eq 1 ]; then echo "$msg1"
    cd $gmp_dirname
    ./.bootstrap | ts "$msg"
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$jsonc_origin_name"
if [ $install_jsonc -eq 1 ]; then echo "$msg1"
    cd $working_dir
    mkdir $jsonc_build_path
    cd $jsonc_build_path
    cmake $jsonc_dirname | ts "$msg"
    make $make_flag | ts "$msg"
    sudo make $make_flag install | ts "$msg" 
else echo "$msg2"; fi
update_var "$libcurl_origin_name"
if [ $install_libcurl -eq 1 ]; then echo "$msg1"
    cd $libcurl_dirname
    gnu_compile "--with-openssl" | ts "$msg"
else echo "$msg2"; fi
update_var "$gettext_origin_name"
if [ $install_gettext -eq 1 ]; then echo "$msg1"
    cd $gettext_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$flex_origin_name"
if [ $install_flex -eq 1 ]; then echo "$msg1"
    cd $flex_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$utillinux_origin_name"
if [ $install_utillinux -eq 1 ]; then echo "$msg1"
    cd $utillinux_dirname
    ./autogen.sh | ts "$msg"
    gnu_compile "--disable-all-programs --enable-libuuid" | ts "$msg"
else echo "$msg2"; fi
update_var "$nettle_origin_name"
if [ $install_nettle -eq 1 ]; then echo "$msg1"
    cd $nettle_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$libunistring_origin_name"
if [ $install_libunistring -eq 1 ]; then echo "$msg1"
    cd $libunistring_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$libev_origin_name"
if [ $install_libev -eq 1 ]; then echo "$msg1"
    cd $libev_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$p11kit_origin_name"
if [ $install_p11kit -eq 1 ]; then echo "$msg1"
    cd $p11kit_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$gnutls_origin_name"
if [ $install_gnutls -eq 1 ]; then echo "$msg1"
    cd $gnutls_dirname
    # `bootstap` process contains `git clone`, considering fail-redo mechanism
    ./bootstrap | ts "$msg"
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$tcsd_origin_name"
if [ $install_tcsd -eq 1 ]; then echo "$msg1"
    cd $tcsd_dirname
    ./bootstrap.sh | ts "$msg"
    make $make_flag | ts "$msg"
    sudo make $make_flag install | ts "$msg"
else echo "$msg2"; fi
update_var "$tcl_origin_name"
if [ $install_tcl -eq 1 ]; then echo "$msg1"
    cd $tcl_dirname/unix
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$expect_origin_name"
if [ $install_expect -eq 1 ]; then echo "$msg1"
    cd $expect_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$gawk_origin_name"
if [ $install_gawk -eq 1 ]; then echo "$msg1"
    cd $gawk_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$socat_origin_name"
if [ $install_socat -eq 1 ]; then echo "$msg1"
    cd $socat_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$libseccomp_origin_name"
if [ $install_libseccomp -eq 1 ]; then echo "$msg1"
    cd $libseccomp_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$tpm2tss_origin_name"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg1"
    cd $tpm2tss_dirname
    gnu_compile "--with-udevrulesdir=/etc/udev/rules.d --with-udevrulesprefix" | ts "$msg"
    sudo udevadm control --reload-rules | ts "$msg"
    sudo udevadm trigger | ts "$msg"
    sudo ldconfig | ts "$msg"
else echo "$msg2"; fi
update_var "$tpm2tools_origin_name"
if [ $install_libseccomp -eq 1 ]; then echo "$msg1"
    cd $libseccomp_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "$tpm2abrmd_origin_name"
if [ $install_tpm2abrmd -eq 1 ]; then echo "$msg1"
    cd $tpm2abrmd_dirname
    sudo useradd --system --user-group tss | ts "$msg" || :
    gnu_compile "--with-dbuspolicydir=/etc/dbus-1/system.d --with-systemdsystemunitdir=/lib/systemd/system --datarootdir=/usr/share" | ts "$msg" # can test integration with swtpm with --enable-integration
    sudo ldconfig | ts "$msg"
    sudo pkill -HUP dbus-daemon | ts "$msg"
    sudo systemctl daemon-reload | ts "$msg"
else echo "$msg2"; fi
update_var "$tpm2tssengine_origin_name"
if [ $install_tpm2tssengine -eq 1 ]; then echo "$msg1"
    cd $tpm2tssengine_dirname
    gnu_compile | ts "$msg"
    sudo ldconfig | ts "$msg"
else echo "$msg2"; fi
update_var "$libtpms_origin_name"
if [ $install_libtpms -eq 1 ]; then echo "$msg1"
    cd $libtpms_dirname
    tpm_compile "--with-tpm2 --with-openssl --prefix=/usr" | ts "$msg"
else echo "$msg2"; fi
update_var "$swtpm_origin_name"
if [ $install_swtpm -eq 1 ]; then echo "$msg1"
    cd $swtpm_dirname
    tpm_compile "--with-openssl --prefix=/usr" | ts "$msg"
else echo "$msg2"; fi

#> ------------------------------------------------
if [ $enable_wget_cert_skip -eq 1 ]; then
    echo "enabling wget certification check"
    sed -i "/$wget_config_str/d" $wget_rc_path
fi
#> ------------------------------------------------

echo "$script finished"
