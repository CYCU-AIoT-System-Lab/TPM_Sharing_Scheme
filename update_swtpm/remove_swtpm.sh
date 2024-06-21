#!/bin/bash
script=$(realpath "$0")
script_path=$(dirname "$script")
source function_swtpm.sh

if [ $install_apt_package -eq 1 ]; then
    echo "uninstalling apt packages"
    sudo apt remove -y mercurial
    sudo apt remove -y ftp
fi

# Note: this section is reversed-compile ordered
echo "uninstalling compiled sources"
total_cnt=18
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg can't be uninstalled"
    msg2="$msg skipped!"
    msg3="$msg uninstalling compiled binaries"
}
update_var "libtpms"
if [ $install_libtpms -eq 1 ]; then echo "$msg3"
    cd $libtpms_dirname
    sudo make uninstall | ts "$msg" || { echo "$msg1"; }
else echo "$msg2"; fi
update_var "tpm2-tss"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg3"
    cd $tpm2tss_dirname
    sudo make uninstall | ts "$msg"
else echo "$msg2"; fi
update_var "json-c"
if [ $install_jsonc -eq 1 ]; then echo "$msg3"
    cd "$working_dir/json-c-build"
    sudo make uninstall | ts "$msg" || { echo "$msg1"; }
else echo "$msg2"; fi
update_var "gmp"
if [ $install_gmp -eq 1 ]; then echo "$msg3"
    cd $gmp_dirname
    sudo make uninstall | ts "$msg" || { echo "$msg1"; }
else echo "$msg2"; fi
update_var "libtool"
if [ $install_libtool -eq 1 ]; then echo "$msg3"
    cd $libtool_dirname
    sudo make uninstall | ts "$msg"
else echo "$msg2"; fi
update_var "texinfo"
if [ $install_texinfo -eq 1 ]; then echo "$msg3"
    cd $texinfo_dirname
    sudo make uninstall | ts "$msg"
else echo "$msg2"; fi
update_var "automake"
if [ $install_automake -eq 1 ]; then echo "$msg3"
    cd $automake_dirname
    sudo make uninstall | ts "$msg"
else echo "$msg2"; fi
update_var "m4"
if [ $install_m4 -eq 1 ]; then echo "$msg3"
    cd $m4_dirname
    sudo make uninstall | ts "$msg"
else echo "$msg2"; fi
update_var "gperf"
if [ $install_gperf -eq 1 ]; then echo "$msg3"
    cd $gperf_dirname
    sudo make uninstall | ts "$msg"
else echo "$msg2"; fi
update_var "autoconf"
if [ $install_autoconf -eq 1 ]; then echo "$msg3"
    cd $autoconf_dirname
    sudo make uninstall | ts "$msg"
else echo "$msg2"; fi
update_var "help2man"
if [ $install_help2man -eq 1 ]; then echo "$msg3"
    cd $help2man_dirname
    sudo make uninstall | ts "$msg" || { echo "$msg1"; }
else echo "$msg2"; fi
update_var "json-glib"
if [ $install_jsonglib -eq 1 ]; then echo "$msg3"
    echo "$msg uninstall method unknown"
else echo "$msg2"; fi
update_var "glib"
if [ $install_glib -eq 1 ]; then echo "$msg3"
    echo "$msg uninstall method unknown"
else echo "$msg2"; fi
update_var "libpcre2"
if [ $install_libpcre2 -eq 1 ]; then echo "$msg3"
    cd $libpcre2_dirname
    sudo make uninstall | ts "$msg" || { echo "$msg1"; }
else echo "$msg2"; fi
update_var "pip-package"
if [ $install_pip_package -eq 1 ]; then echo "$msg3"
    echo "$msg2"
else echo "$msg2"; fi
update_var "python"
if [ $install_python -eq 1 ]; then echo "$msg3"
    cd $python_dirname
    sudo make uninstall | ts "$msg" || { echo "$msg1"; }
else echo "$msg2"; fi
update_var "libtasn1"
if [ $install_libtasn1 -eq 1 ]; then echo "$msg3"
    cd $libtasn1_dirname
    sudo make uninstall | ts "$msg" || { echo "$msg1"; }
else echo "$msg2"; fi
update_var "pkg-config"
if [ $install_libtpms -eq 1 ]; then echo "$msg3"
    cd $pkgconfig_dirname
    sudo make uninstall | ts "$msg" || { echo "$msg1"; }
else echo "$msg2"; fi

echo "removing created directories"
total_cnt=17
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg removing directory"
    msg2="$msg skipped!"
}
update_var "gmp"
if [ $install_gmp -eq 1 ]; then echo "$msg1"
    rm -rf $gmp_name
else echo "$msg2"; fi
update_var "libtpms"
if [ $install_libtpms -eq 1 ]; then echo "$msg1"
    rm -rf $libtpms_name
else echo "$msg2"; fi
update_var "tpm2-tss"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg1"
    rm -rf $tpm2tss_name
else echo "$msg2"; fi
update_var "json-c"
if [ $install_jsonc -eq 1 ]; then echo "$msg1"
    rm -rf $jsonc_name
    rm -rf "$working_dir/json-c-build"
else echo "$msg2"; fi
update_var "texinfo"
if [ $install_texinfo -eq 1 ]; then echo "$msg1"
    rm -rf $texinfo_name
else echo "$msg2"; fi
update_var "automake"
if [ $install_automake -eq 1 ]; then echo "$msg1"
    rm -rf $automake_name
else echo "$msg2"; fi
update_var "autoconf"
if [ $install_autoconf -eq 1 ]; then echo "$msg1"
    rm -rf $autoconf_name
else echo "$msg2"; fi
update_var "help2man"
if [ $install_help2man -eq 1 ]; then echo "$msg1"
    rm -rf $help2man_name
else echo "$msg2"; fi
update_var "m4"
if [ $install_m4 -eq 1 ]; then echo "$msg1"
    rm -rf $m4_name
else echo "$msg2"; fi
update_var "gperf"
if [ $install_gperf -eq 1 ]; then echo "$msg1"
    rm -rf $gperf_name
else echo "$msg2"; fi
update_var "libtool"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    rm -rf $libtool_name
else echo "$msg2"; fi
update_var "pkg-config"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    rm -rf $pkgconfig_name
else echo "$msg2"; fi
update_var "libtasn1"
if [ $install_libtasn1 -eq 1 ]; then echo "$msg1"
    rm -rf $libtasn1_name
else echo "$msg2"; fi
update_var "python"
if [ $install_python -eq 1 ]; then echo "$msg1"
    sudo rm -rf $python_name
else echo "$msg2"; fi
update_var "libpcre2"
if [ $install_libpcre2 -eq 1 ]; then echo "$msg1"
    rm -rf $libpcre2_name
else echo "$msg2"; fi
update_var "glib"
if [ $install_glib -eq 1 ]; then echo "$msg1"
    rm -rf $glib_name
else echo "$msg2"; fi
update_var "json-glib"
if [ $install_jsonglib -eq 1 ]; then echo "$msg1"
    rm -rf $jsonglib_name
else echo "$msg2"; fi

echo "removing downloaded source"
total_cnt=16
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg removing downloaded source"
    msg2="$msg skipped!"
}
update_var "libtpms"
if [ $install_libtpms -eq 1 ]; then echo "$msg1"
    rm -f $libtpms_name.tar.gz
else echo "$msg2"; fi
update_var "tpm2-tss"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg1"
    rm -f $tpm2tss_name.tar.gz
else echo "$msg2"; fi
update_var "json-c"
if [ $install_jsonc -eq 1 ]; then echo "$msg1"
    rm -f $jsonc_name.tar.gz
else echo "$msg2"; fi
update_var "texinfo"
if [ $install_texinfo -eq 1 ]; then echo "$msg1"
    rm -f $texinfo_name.tar.gz
else echo "$msg2"; fi
update_var "automake"
if [ $install_automake -eq 1 ]; then echo "$msg1"
    rm -f $automake_name.tar.gz
else echo "$msg2"; fi
update_var "autoconf"
if [ $install_autoconf -eq 1 ]; then echo "$msg1"
    rm -f $autoconf_name.tar.gz
else echo "$msg2"; fi
update_var "help2man"
if [ $install_help2man -eq 1 ]; then echo "$msg1"
    rm -f $help2man_name.tar.xz
else echo "$msg2"; fi
update_var "m4"
if [ $install_m4 -eq 1 ]; then echo "$msg1"
    rm -f $m4_name.tar.gz
else echo "$msg2"; fi
update_var "gperf"
if [ $install_gperf -eq 1 ]; then echo "$msg1"
    rm -f $gperf_name.tar.gz
else echo "$msg2"; fi
update_var "libtool"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    rm -f $libtool_name.tar.gz
else echo "$msg2"; fi
update_var "pkg-config"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    rm -f $pkgconfig_name.tar.gz
else echo "$msg2"; fi
update_var "libtasn1"
if [ $install_libtasn1 -eq 1 ]; then echo "$msg1"
    rm -f $libtasn1_name.tar.gz
else echo "$msg2"; fi
update_var "python"
if [ $install_python -eq 1 ]; then echo "$msg1"
    rm -f $python_name.tgz
else echo "$msg2"; fi
update_var "libpcre2"
if [ $install_libpcre2 -eq 1 ]; then echo "$msg1"
    rm -f $libpcre2_name.tar.gz
else echo "$msg2"; fi
update_var "glib"
if [ $install_glib -eq 1 ]; then echo "$msg1"
    rm -f $glib_name.tar.xz
else echo "$msg2"; fi
update_var "json-glib"
if [ $install_jsonglib -eq 1 ]; then echo "$msg1"
    rm -f $jsonglib_name.tar.xz
else echo "$msg2"; fi

echo "removing symlinks"
total_cnt=17
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg removing symlink"
    msg2="$msg skipped!"
}
update_var "gmp"
if [ $install_gmp -eq 1 ]; then echo "$msg1"
    rm -f $gmp_dirname
else echo "$msg2"; fi
update_var "libtpms"
if [ $install_libtpms -eq 1 ]; then echo "$msg1"
    rm -f $libtpms_dirname
else echo "$msg2"; fi
update_var "tpm2-tss"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg1"
    rm -f $tpm2tss_dirname
else echo "$msg2"; fi
update_var "json-c"
if [ $install_jsonc -eq 1 ]; then echo "$msg1"
    rm -f $jsonc_dirname
else echo "$msg2"; fi
update_var "texinfo"
if [ $install_texinfo -eq 1 ]; then echo "$msg1"
    rm -r $texinfo_dirname
else echo "$msg2"; fi
update_var "automake"
if [ $install_automake -eq 1 ]; then echo "$msg1"
    rm -r $automake_dirname
else echo "$msg2"; fi
update_var "autoconf"
if [ $install_autoconf -eq 1 ]; then echo "$msg1"
    rm -r $autoconf_dirname
else echo "$msg2"; fi
update_var "help2man"
if [ $install_help2man -eq 1 ]; then echo "$msg1"
    rm -r $help2man_dirname
else echo "$msg2"; fi
update_var "m4"
if [ $install_m4 -eq 1 ]; then echo "$msg1"
    rm -r $m4_dirname
else echo "$msg2"; fi
update_var "gperf"
if [ $install_gperf -eq 1 ]; then echo "$msg1"
    rm -r $gperf_dirname
else echo "$msg2"; fi
update_var "libtool"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    rm -r $libtool_dirname
else echo "$msg2"; fi
update_var "pkg-config"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    rm -r $pkgconfig_dirname
else echo "$msg2"; fi
update_var "libtasn1"
if [ $install_libtasn1 -eq 1 ]; then echo "$msg1"
    rm -r $libtasn1_dirname
else echo "$msg2"; fi
update_var "python"
if [ $install_python -eq 1 ]; then echo "$msg1"
    rm -r $python_dirname
else echo "$msg2"; fi
update_var "libpcre2"
if [ $install_libpcre2 -eq 1 ]; then echo "$msg1"
    rm -r $libpcre2_dirname
else echo "$msg2"; fi
update_var "glib"
if [ $install_glib -eq 1 ]; then echo "$msg1"
    rm -r $glib_dirname
else echo "$msg2"; fi
update_var "json-glib"
if [ $install_jsonglib -eq 1 ]; then echo "$msg1"
    rm -r $jsonglib_dirname
else echo "$msg2"; fi

echo "$script finished"
