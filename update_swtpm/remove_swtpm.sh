#!/bin/bash
script=$(realpath "$0")
script_path=$(dirname "$script")
source "../common/functions.sh"
source "./function_swtpm.sh"
load_preset "./config.ini"

total_cnt=2
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg removing package"
    msg2="$msg ${BOLD}${RED}removing failed!${END}"
    package="$1"
}
apt_remove () {
    echo "$msg1"
    sudo apt remove -y $package | ts "$msg"; exit "${PIPESTATUS[0]}";
}
if [ $install_apt_package -eq 1 ]; then
    echo_notice "$script_path" "$script" "uninstalling apt packages..."
    update_var "mercurial" && apt_remove "$package"
    update_var "gtk_doc-tools" && apt_remove "$package"
fi

# Note: this section is reversed-compile ordered
echo_notice "$script_path" "$script" "uninstalling compiled sources..."
total_cnt=40
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg ${BOLD}${RED}can't be uninstalled${END}"
    msg2="$msg skipped!"
    msg3="$msg uninstalling compiled binaries"
}
update_var "$swtpm_origin_name"
if [ $install_swtpm -eq 1 ]; then echo "$msg3"
    cd $swtpm_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$libtpms_origin_name"
if [ $install_libtpms -eq 1 ]; then echo "$msg3"
    cd $libtpms_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$tpm2tssengine_origin_name"
if [ $install_tpm2tssengine -eq 1 ]; then echo "$msg3"
    cd $tpm2tssengine_dirname
    sudo make uninstall | ts "$msg"; || :
    sudo ldconfig | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$tpm2abrmd_origin_name"
if [ $install_tpm2abrmd -eq 1 ]; then echo "$msg3"
    cd $tpm2abrmd_dirname
    sudo make uninstall | ts "$msg"; || :
    sudo deluser --system tss | ts "$msg"; || :
    # sudo groupdel tss || : # auto-deleted in previous line
    sudo pkill -HUP dbus-daemon | ts "$msg"; || :
    sudo systemctl daemon-reload | ts "$msg"; || :
    sudo ldconfig | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$tpm2tools_origin_name"
if [ $install_tpm2tools -eq 1 ]; then echo "$msg3"
    cd $tpm2tools_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$tpm2tss_origin_name"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg3"
    cd $tpm2tss_dirname
    sudo make uninstall | ts "$msg"; || :
    sudo udevadm control --reload-rules | ts "$msg"; || :
    sudo udevadm trigger | ts "$msg"; || :
    sudo ldconfig | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$libseccomp_origin_name"
if [ $install_libseccomp -eq 1 ]; then echo "$msg3"
    cd $libseccomp_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$socat_origin_name"
if [ $install_socat -eq 1 ]; then echo "$msg3"
    cd $socat_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$gawk_origin_name"
if [ $install_gawk -eq 1 ]; then echo "$msg3"
    cd $gawk_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$expect_origin_name"
if [ $install_expect -eq 1 ]; then echo "$msg3"
    cd $expect_dirname
    sudo make uninstall-binaries | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$tcl_origin_name"
if [ $install_tcl -eq 1 ]; then echo "$msg3"
    echo "$msg no removing solution provided"
else echo "$msg2"; fi
update_var "$tcsd_origin_name"
if [ $install_tcsd -eq 1 ]; then echo "$msg3"
    cd $tcsd_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$gnutls_origin_name"
if [ $install_gnutls -eq 1 ]; then echo "$msg3"
    cd $gnutls_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$p11kit_origin_name"
if [ $install_p11kit -eq 1 ]; then echo "$msg3"
    cd $p11kit_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$libev_origin_name"
if [ $install_libev -eq 1 ]; then echo "$msg3"
    cd $libev_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$libunistring_origin_name"
if [ $install_libunistring -eq 1 ]; then echo "$msg3"
    cd $libunistring_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$nettle_origin_name"
if [ $install_nettle -eq 1 ]; then echo "$msg3"
    cd $nettle_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$utillinux_origin_name"
if [ $install_utillinux -eq 1 ]; then echo "$msg3"
    cd $utillinux_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$flex_origin_name"
if [ $install_flex -eq 1 ]; then echo "$msg3"
    cd $flex_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$gettext_origin_name"
if [ $install_gettext -eq 1 ]; then echo "$msg3"
    cd $gettext_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$libcurl_origin_name"
if [ $install_libcurl -eq 1 ]; then echo "$msg3"
    cd $libcurl_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$jsonc_origin_name"
if [ $install_jsonc -eq 1 ]; then echo "$msg3"
    cd "$jsonc_build_path"
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$gmp_origin_name"
if [ $install_gmp -eq 1 ]; then echo "$msg3"
    cd $gmp_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$libtool_origin_name"
if [ $install_libtool -eq 1 ]; then echo "$msg3"
    cd $libtool_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$texinfo_origin_name"
if [ $install_texinfo -eq 1 ]; then echo "$msg3"
    cd $texinfo_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$automake_origin_name"
if [ $install_automake -eq 1 ]; then echo "$msg3"
    cd $automake_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$m4_origin_name"
if [ $install_m4 -eq 1 ]; then echo "$msg3"
    cd $m4_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$gperf_origin_name"
if [ $install_gperf -eq 1 ]; then echo "$msg3"
    cd $gperf_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$autoconf_origin_name"
if [ $install_autoconf -eq 1 ]; then echo "$msg3"
    cd $autoconf_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$help2man_origin_name"
if [ $install_help2man -eq 1 ]; then echo "$msg3"
    cd $help2man_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$libtasn1_origin_name"
if [ $install_libtasn1 -eq 1 ]; then echo "$msg3"
    cd $libtasn1_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$jsonglib_origin_name"
if [ $install_jsonglib -eq 1 ]; then echo "$msg3"
    echo "$msg uninstall method unknown"
else echo "$msg2"; fi
update_var "$glib_origin_name"
if [ $install_glib -eq 1 ]; then echo "$msg3"
    echo "$msg uninstall method unknown"
else echo "$msg2"; fi
update_var "$libpcre2_origin_name"
if [ $install_libpcre2 -eq 1 ]; then echo "$msg3"
    cd $libpcre2_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$pippackage_origin_name"
if [ $install_pippackage -eq 1 ]; then echo "$msg3"
    sudo rm -rf $python_venv_path
else echo "$msg2"; fi
update_var "$python_origin_name"
if [ $install_python -eq 1 ]; then echo "$msg3"
    cd $python_dirname
    sudo make uninstall | ts "$msg"; || :
    sudo ldconfig | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$openssl_origin_name"
if [ $install_openssl -eq 1 ]; then echo "$msg3"
    cd $openssl_dirname
    sudo make uninstall | ts "$msg"; || :
    sudo rm /usr/local/lib/pkgconfig/libcrypto.pc | ts "$msg"; || :
    sudo rm /usr/local/lib/pkgconfig/libssl.pc | ts "$msg"; || :
    sudo rm /usr/local/lib/pkgconfig/openssl.pc | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$bison_origin_name"
if [ $install_bison -eq 1 ]; then echo "$msg3"
    cd $bison_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$libffi_origin_name"
if [ $install_libffi -eq 1 ]; then echo "$msg3"
    cd $libffi_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi
update_var "$pkgconfig_origin_name"
if [ $install_libtpms -eq 1 ]; then echo "$msg3"
    cd $pkgconfig_dirname
    sudo make uninstall | ts "$msg"; || :
else echo "$msg2"; fi

echo_notice "$script_path" "$script" "removing created directories"
total_cnt=39
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg removing directory"
    msg2="$msg skipped!"
}
update_var "$gmp_origin_name"
if [ $install_gmp -eq 1 ]; then echo "$msg1"
    sudo rm -rf $gmp_name
else echo "$msg2"; fi
update_var "$libtpms_origin_name"
if [ $install_libtpms -eq 1 ]; then echo "$msg1"
    sudo rm -rf $libtpms_name
else echo "$msg2"; fi
update_var "$tpm2tss_origin_name"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg1"
    sudo rm -rf $tpm2tss_name
else echo "$msg2"; fi
update_var "$jsonc_origin_name"
if [ $install_jsonc -eq 1 ]; then echo "$msg1"
    sudo rm -rf $jsonc_name
    sudo rm -rf "$jsonc_build_path"
else echo "$msg2"; fi
update_var "$texinfo_origin_name"
if [ $install_texinfo -eq 1 ]; then echo "$msg1"
    sudo rm -rf $texinfo_name
else echo "$msg2"; fi
update_var "$automake_origin_name"
if [ $install_automake -eq 1 ]; then echo "$msg1"
    sudo rm -rf $automake_name
else echo "$msg2"; fi
update_var "$autoconf_origin_name"
if [ $install_autoconf -eq 1 ]; then echo "$msg1"
    sudo rm -rf $autoconf_name
else echo "$msg2"; fi
update_var "$help2man_origin_name"
if [ $install_help2man -eq 1 ]; then echo "$msg1"
    sudo rm -rf $help2man_name
else echo "$msg2"; fi
update_var "$m4_origin_name"
if [ $install_m4 -eq 1 ]; then echo "$msg1"
    sudo rm -rf $m4_name
else echo "$msg2"; fi
update_var "$gperf_origin_name"
if [ $install_gperf -eq 1 ]; then echo "$msg1"
    sudo rm -rf $gperf_name
else echo "$msg2"; fi
update_var "$libtool_origin_name"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    sudo rm -rf $libtool_name
else echo "$msg2"; fi
update_var "$pkgconfig_origin_name"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    sudo rm -rf $pkgconfig_name
else echo "$msg2"; fi
update_var "$libtasn1_origin_name"
if [ $install_libtasn1 -eq 1 ]; then echo "$msg1"
    sudo rm -rf $libtasn1_name
else echo "$msg2"; fi
update_var "$python_origin_name"
if [ $install_python -eq 1 ]; then echo "$msg1"
    sudo rm -rf $python_name
else echo "$msg2"; fi
update_var "$libpcre2_origin_name"
if [ $install_libpcre2 -eq 1 ]; then echo "$msg1"
    sudo rm -rf $libpcre2_name
else echo "$msg2"; fi
update_var "$glib_origin_name"
if [ $install_glib -eq 1 ]; then echo "$msg1"
    sudo rm -rf $glib_name
else echo "$msg2"; fi
update_var "$jsonglib_origin_name"
if [ $install_jsonglib -eq 1 ]; then echo "$msg1"
    sudo rm -rf $jsonglib_name
else echo "$msg2"; fi
update_var "$bison_origin_name"
if [ $install_bison -eq 1 ]; then echo "$msg1"
    sudo rm -rf $bison_name
else echo "$msg2"; fi
update_var "$openssl_origin_name"
if [ $install_openssl -eq 1 ]; then echo "$msg1"
    sudo rm -rf $openssl_name
else echo "$msg2"; fi
update_var "$libcurl_origin_name"
if [ $install_libcurl -eq 1 ]; then echo "$msg1"
    sudo rm -rf $libcurl_name
else echo "$msg2"; fi
update_var "$swtpm_origin_name"
if [ $install_swtpm -eq 1 ]; then echo "$msg1"
    sudo rm -rf $swtpm_name
else echo "$msg2"; fi
update_var "$gettext_origin_name"
if [ $install_gettext -eq 1 ]; then echo "$msg1"
    sudo rm -rf $gettext_name
else echo "$msg2"; fi
update_var "$flex_origin_name"
if [ $install_flex -eq 1 ]; then echo "$msg1"
    sudo rm -rf $flex_name
else echo "$msg2"; fi
update_var "$utillinux_origin_name"
if [ $install_utillinux -eq 1 ]; then echo "$msg1"
    sudo rm -rf $utillinux_name
else echo "$msg2"; fi
update_var "$nettle_origin_name"
if [ $install_nettle -eq 1 ]; then echo "$msg1"
    sudo rm -rf $nettle_name
else echo "$msg2"; fi
update_var "$libunistring_origin_name"
if [ $install_libunistring -eq 1 ]; then echo "$msg1"
    sudo rm -rf $libunistring_name
else echo "$msg2"; fi
update_var "$libev_origin_name"
if [ $install_libev -eq 1 ]; then echo "$msg1"
    sudo rm -rf $libev_name
else echo "$msg2"; fi
update_var "$p11kit_origin_name"
if [ $install_p11kit -eq 1 ]; then echo "$msg1"
    sudo rm -rf $p11kit_name
else echo "$msg2"; fi
update_var "$gnutls_origin_name"
if [ $install_gnutls -eq 1 ]; then echo "$msg1"
    sudo rm -rf $gnutls_name
else echo "$msg2"; fi
update_var "$tcsd_origin_name"
if [ $install_tcsd -eq 1 ]; then echo "$msg1"
    sudo rm -rf $tcsd_name
else echo "$msg2"; fi
update_var "$tcl_origin_name"
if [ $install_tcl -eq 1 ]; then echo "$msg1"
    sudo rm -rf $tcl_name
else echo "$msg2"; fi
update_var "$expect_origin_name"
if [ $install_expect -eq 1 ]; then echo "$msg1"
    sudo rm -rf $expect_name
else echo "$msg2"; fi
update_var "$gawk_origin_name"
if [ $install_gawk -eq 1 ]; then echo "$msg1"
    sudo rm -rf $gawk_name
else echo "$msg2"; fi
update_var "$socat_origin_name"
if [ $install_socat -eq 1 ]; then echo "$msg1"
    sudo rm -rf $socat_name
else echo "$msg2"; fi
update_var "$libseccomp_origin_name"
if [ $install_libseccomp -eq 1 ]; then echo "$msg1"
    sudo rm -rf $libseccomp_name
else echo "$msg2"; fi
update_var "$tpm2tools_origin_name"
if [ $install_tpm2tools -eq 1 ]; then echo "$msg1"
    sudo rm -rf $tpm2tools_name
else echo "$msg2"; fi
update_var "$tpm2abrmd_origin_name"
if [ $install_tpm2abrmd -eq 1 ]; then echo "$msg1"
    sudo rm -rf $tpm2abrmd_name
else echo "$msg2"; fi
update_var "$tpm2tssengine_origin_name"
if [ $install_tpm2tssengine -eq 1 ]; then echo "$msg1"
    sudo rm -rf $tpm2tssengine_name
else echo "$msg2"; fi
update_var "$libffi_origin_name"
if [ $install_libffi -eq 1 ]; then echo "$msg1"
    sudo rm -rf $libffi_name
else echo "$msg2"; fi

echo_notice "$script_path" "$script" "removing downloaded source"
total_cnt=37
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg removing downloaded source"
    msg2="$msg skipped!"
}
update_var "$libtpms_origin_name"
if [ $install_libtpms -eq 1 ]; then echo "$msg1"
    sudo rm -f $libtpms_name$libtpms_ext
else echo "$msg2"; fi
update_var "$tpm2tss_origin_name"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg1"
    sudo rm -f $tpm2tss_name$tpm2tss_ext
else echo "$msg2"; fi
update_var "$jsonc_origin_name"
if [ $install_jsonc -eq 1 ]; then echo "$msg1"
    sudo rm -f $jsonc_name$jsonc_ext
else echo "$msg2"; fi
update_var "$texinfo_origin_name"
if [ $install_texinfo -eq 1 ]; then echo "$msg1"
    sudo rm -f $texinfo_name$texinfo_ext
else echo "$msg2"; fi
update_var "$automake_origin_name"
if [ $install_automake -eq 1 ]; then echo "$msg1"
    sudo rm -f $automake_name$automake_ext
else echo "$msg2"; fi
update_var "$autoconf_origin_name"
if [ $install_autoconf -eq 1 ]; then echo "$msg1"
    sudo rm -f $autoconf_name$autoconf_ext
else echo "$msg2"; fi
update_var "$help2man_origin_name"
if [ $install_help2man -eq 1 ]; then echo "$msg1"
    sudo rm -f $help2man_name$help2man_ext
else echo "$msg2"; fi
update_var "$m4_origin_name"
if [ $install_m4 -eq 1 ]; then echo "$msg1"
    sudo rm -f $m4_name$m4_ext
else echo "$msg2"; fi
update_var "$gperf_origin_name"
if [ $install_gperf -eq 1 ]; then echo "$msg1"
    sudo rm -f $gperf_name$m4_ext
else echo "$msg2"; fi
update_var "$libtool_origin_name"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    sudo rm -f $libtool_name$libtool_ext
else echo "$msg2"; fi
update_var "$pkgconfig_origin_name"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    sudo rm -f $pkgconfig_name$pkgconfig_ext
else echo "$msg2"; fi
update_var "$libtasn1_origin_name"
if [ $install_libtasn1 -eq 1 ]; then echo "$msg1"
    sudo rm -f $libtasn1_name$libtasn1_ext
else echo "$msg2"; fi
update_var "$python_origin_name"
if [ $install_python -eq 1 ]; then echo "$msg1"
    sudo rm -f $python_name$python_ext
else echo "$msg2"; fi
update_var "$libpcre2_origin_name"
if [ $install_libpcre2 -eq 1 ]; then echo "$msg1"
    sudo rm -f $libpcre2_name$libpcre2_ext
else echo "$msg2"; fi
update_var "$glib_origin_name"
if [ $install_glib -eq 1 ]; then echo "$msg1"
    sudo rm -f $glib_name$glib_ext
else echo "$msg2"; fi
update_var "$jsonglib_origin_name"
if [ $install_jsonglib -eq 1 ]; then echo "$msg1"
    sudo rm -f $jsonglib_name$jsonglib_ext
else echo "$msg2"; fi
update_var "$bison_origin_name"
if [ $install_bison -eq 1 ]; then echo "$msg1"
    sudo rm -f $bison_name$bison_ext
else echo "$msg2"; fi
update_var "$openssl_origin_name"
if [ $install_openssl -eq 1 ]; then echo "$msg1"
    sudo rm -f $openssl_name$openssl_ext
else echo "$msg2"; fi
update_var "$libcurl_origin_name"
if [ $install_libcurl -eq 1 ]; then echo "$msg1"
    sudo rm -f $libcurl_name$libcurl_ext
else echo "$msg2"; fi
update_var "$swtpm_origin_name"
if [ $install_swtpm -eq 1 ]; then echo "$msg1"
    sudo rm -f $swtpm_name$swtpm_ext
else echo "$msg2"; fi
update_var "$gettext_origin_name"
if [ $install_gettext -eq 1 ]; then echo "$msg1"
    sudo rm -f $gettext_name$gettext_ext
else echo "$msg2"; fi
update_var "$flex_origin_name"
if [ $install_flex -eq 1 ]; then echo "$msg1"
    sudo rm -f $flex_name$flex_ext
else echo "$msg2"; fi
update_var "$utillinux_origin_name"
if [ $install_utillinux -eq 1 ]; then echo "$msg1"
    sudo rm -f $utillinux_name$utillinux_ext
else echo "$msg2"; fi
update_var "$nettle_origin_name"
if [ $install_nettle -eq 1 ]; then echo "$msg1"
    sudo rm -f $nettle_name$nettle_ext
else echo "$msg2"; fi
update_var "$libunistring_origin_name"
if [ $install_libunistring -eq 1 ]; then echo "$msg1"
    sudo rm -f $libunistring_name$libunistring_ext
else echo "$msg2"; fi
update_var "$libev_origin_name"
if [ $install_libev -eq 1 ]; then echo "$msg1"
    sudo rm -f $libev_name$libev_ext
else echo "$msg2"; fi
update_var "$p11kit_origin_name"
if [ $install_p11kit -eq 1 ]; then echo "$msg1"
    sudo rm -f $p11kit_name$p11kit_ext
else echo "$msg2"; fi
update_var "$tcsd_origin_name"
if [ $install_tcsd -eq 1 ]; then echo "$msg1"
    sudo rm -f $tcsd_name$tcsd_ext
else echo "$msg2"; fi
update_var "$tcl_origin_name"
if [ $install_tcl -eq 1 ]; then echo "$msg1"
    sudo rm -f $tcl_name$tcl_ext
else echo "$msg2"; fi
update_var "$expect_origin_name"
if [ $install_expect -eq 1 ]; then echo "$msg1"
    sudo rm -f $expect_name$expect_ext
else echo "$msg2"; fi
update_var "$gawk_origin_name"
if [ $install_gawk -eq 1 ]; then echo "$msg1"
    sudo rm -f $gawk_name$gawk_ext
else echo "$msg2"; fi
update_var "$socat_origin_name"
if [ $install_socat -eq 1 ]; then echo "$msg1"
    sudo rm -f $socat_name$socat_ext
else echo "$msg2"; fi
update_var "$libseccomp_origin_name"
if [ $install_libseccomp -eq 1 ]; then echo "$msg1"
    sudo rm -f $libseccomp_name$libseccomp_ext
else echo "$msg2"; fi
update_var "$tpm2tools_origin_name"
if [ $install_tpm2tools -eq 1 ]; then echo "$msg1"
    sudo rm -f $tpm2tools_name$tpm2tools_ext
else echo "$msg2"; fi
update_var "$tpm2abrmd_origin_name"
if [ $install_tpm2abrmd -eq 1 ]; then echo "$msg1"
    sudo rm -f $tpm2abrmd_name$tpm2abrmd_ext
else echo "$msg2"; fi
update_var "$tpm2tssengine_origin_name"
if [ $install_tpm2tssengine -eq 1 ]; then echo "$msg1"
    sudo rm -f $tpm2tssengine_name$tpm2tssengine_ext
else echo "$msg2"; fi
update_var "$libffi_origin_name"
if [ $install_libffi -eq 1 ]; then echo "$msg1"
    sudo rm -f $libffi_name$libffi_ext
else echo "$msg2"; fi

echo_notice "$script_path" "$script" "removing symlinks"
total_cnt=38
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg removing symlink"
    msg2="$msg skipped!"
}
update_var "$gmp_origin_name"
if [ $install_gmp -eq 1 ]; then echo "$msg1"
    sudo rm -f $gmp_dirname
else echo "$msg2"; fi
update_var "$libtpms_origin_name"
if [ $install_libtpms -eq 1 ]; then echo "$msg1"
    sudo rm -f $libtpms_dirname
else echo "$msg2"; fi
update_var "$tpm2tss_origin_name"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg1"
    sudo rm -f $tpm2tss_dirname
else echo "$msg2"; fi
update_var "$jsonc_origin_name"
if [ $install_jsonc -eq 1 ]; then echo "$msg1"
    sudo rm -f $jsonc_dirname
else echo "$msg2"; fi
update_var "$texinfo_origin_name"
if [ $install_texinfo -eq 1 ]; then echo "$msg1"
    sudo rm -r $texinfo_dirname
else echo "$msg2"; fi
update_var "$automake_origin_name"
if [ $install_automake -eq 1 ]; then echo "$msg1"
    sudo rm -r $automake_dirname
else echo "$msg2"; fi
update_var "$autoconf_origin_name"
if [ $install_autoconf -eq 1 ]; then echo "$msg1"
    sudo rm -r $autoconf_dirname
else echo "$msg2"; fi
update_var "$help2man_origin_name"
if [ $install_help2man -eq 1 ]; then echo "$msg1"
    sudo rm -r $help2man_dirname
else echo "$msg2"; fi
update_var "$m4_origin_name"
if [ $install_m4 -eq 1 ]; then echo "$msg1"
    sudo rm -r $m4_dirname
else echo "$msg2"; fi
update_var "$gperf_origin_name"
if [ $install_gperf -eq 1 ]; then echo "$msg1"
    sudo rm -r $gperf_dirname
else echo "$msg2"; fi
update_var "$pkgconfig_origin_name"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    sudo rm -r $pkgconfig_dirname
else echo "$msg2"; fi
update_var "$libtool_origin_name"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    sudo rm -r $libtool_dirname
else echo "$msg2"; fi
update_var "$libtasn1_origin_name"
if [ $install_libtasn1 -eq 1 ]; then echo "$msg1"
    sudo rm -r $libtasn1_dirname
else echo "$msg2"; fi
update_var "$python_origin_name"
if [ $install_python -eq 1 ]; then echo "$msg1"
    sudo rm -r $python_dirname
else echo "$msg2"; fi
update_var "$libpcre2_origin_name"
if [ $install_libpcre2 -eq 1 ]; then echo "$msg1"
    sudo rm -r $libpcre2_dirname
else echo "$msg2"; fi
update_var "$glib_origin_name"
if [ $install_glib -eq 1 ]; then echo "$msg1"
    sudo rm -r $glib_dirname
else echo "$msg2"; fi
update_var "$jsonglib_origin_name"
if [ $install_jsonglib -eq 1 ]; then echo "$msg1"
    sudo rm -r $jsonglib_dirname
else echo "$msg2"; fi
update_var "$bison_origin_name"
if [ $install_bison -eq 1 ]; then echo "$msg1"
    sudo rm -r $bison_dirname
else echo "$msg2"; fi
update_var "$openssl_origin_name"
if [ $install_openssl -eq 1 ]; then echo "$msg1"
    sudo rm -r $openssl_dirname
else echo "$msg2"; fi
update_var "$libcurl_origin_name"
if [ $install_libcurl -eq 1 ]; then echo "$msg1"
    sudo rm -r $libcurl_dirname
else echo "$msg2"; fi
update_var "$swtpm_origin_name"
if [ $install_swtpm -eq 1 ]; then echo "$msg1"
    sudo rm -r $swtpm_dirname
else echo "$msg2"; fi
update_var "$gettext_origin_name"
if [ $install_gettext -eq 1 ]; then echo "$msg1"
    sudo rm -r $gettext_dirname
else echo "$msg2"; fi
update_var "$flex_origin_name"
if [ $install_flex -eq 1 ]; then echo "$msg1"
    sudo rm -r $flex_dirname
else echo "$msg2"; fi
update_var "$utillinux_origin_name"
if [ $install_utillinux -eq 1 ]; then echo "$msg1"
    sudo rm -r $utillinux_dirname
else echo "$msg2"; fi
update_var "$nettle_origin_name"
if [ $install_nettle -eq 1 ]; then echo "$msg1"
    sudo rm -r $nettle_dirname
else echo "$msg2"; fi
update_var "$libunistring_origin_name"
if [ $install_libunistring -eq 1 ]; then echo "$msg1"
    sudo rm -r $libunistring_dirname
else echo "$msg2"; fi
update_var "$libev_origin_name"
if [ $install_libev -eq 1 ]; then echo "$msg1"
    sudo rm -r $libev_dirname
else echo "$msg2"; fi
update_var "$p11kit_origin_name"
if [ $install_p11kit -eq 1 ]; then echo "$msg1"
    sudo rm -r $p11kit_dirname
else echo "$msg2"; fi
update_var "$tcsd_origin_name"
if [ $install_tcsd -eq 1 ]; then echo "$msg1"
    sudo rm -r $tcsd_dirname
else echo "$msg2"; fi
update_var "$tcl_origin_name"
if [ $install_tcl -eq 1 ]; then echo "$msg1"
    sudo rm -r $tcl_dirname
else echo "$msg2"; fi
update_var "$expect_origin_name"
if [ $install_expect -eq 1 ]; then echo "$msg1"
    sudo rm -r $expect_dirname
else echo "$msg2"; fi
update_var "$gawk_origin_name"
if [ $install_gawk -eq 1 ]; then echo "$msg1"
    sudo rm -r $gawk_dirname
else echo "$msg2"; fi
update_var "$socat_origin_name"
if [ $install_socat -eq 1 ]; then echo "$msg1"
    sudo rm -r $socat_dirname
else echo "$msg2"; fi
update_var "$libseccomp_origin_name"
if [ $install_libseccomp -eq 1 ]; then echo "$msg1"
    sudo rm -r $libseccomp_dirname
else echo "$msg2"; fi
update_var "$tpm2tools_origin_name"
if [ $install_tpm2tools -eq 1 ]; then echo "$msg1"
    sudo rm -r $tpm2tools_dirname
else echo "$msg2"; fi
update_var "$tpm2abrmd_origin_name"
if [ $install_tpm2abrmd -eq 1 ]; then echo "$msg1"
    sudo rm -r $tpm2abrmd_dirname
else echo "$msg2"; fi
update_var "$tpm2tssengine_origin_name"
if [ $install_tpm2tssengine -eq 1 ]; then echo "$msg1"
    sudo rm -r $tpm2tssengine_dirname
else echo "$msg2"; fi
update_var "$libffi_origin_name"
if [ $install_libffi -eq 1 ]; then echo "$msg1"
    sudo rm -r $libffi_dirname
else echo "$msg2"; fi

echo_notice "$script_path" "$script" "$script finished"
