#!/bin/bash
script=$(realpath "$0")
script_path=$(dirname "$script")
source function_swtpm.sh

if [ $install_apt_package -eq 1 ]; then
    echo "installing apt packages"
    sudo apt install -y libtasn1-6-dev
    sudo apt install -y libjson-glib-dev
    sudo apt install -y mercurial
    sudo apt install -y ftp
fi

echo "downloading from sources"
gnu_mirror="ftp://ftp.twaren.net/Unix/GNU/gnu"
total_cnt=12
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg downloading source"
    msg2="$msg skipped!"
    msg3="$msg download failed!"
}
update_var "gmp"
if [ $install_gmp -eq 1 ]; then echo "$msg1"
    hg clone https://gmplib.org/repo/gmp-$gmp_version/ $gmp_name || { echo "$msg3"; }
else echo "$msg2"; fi
update_var "libtpms"
if [ $install_libtpms -eq 1 ]; then echo "$msg1"
    wget $wget_flag https://github.com/stefanberger/libtpms/archive/refs/tags/v$libtpms_version.tar.gz -O $libtpms_name.tar.gz || { echo "$msg3"; }
else echo "$msg2"; fi
update_var "tpm2-tss"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg1"
    wget $wget_flag https://github.com/tpm2-software/tpm2-tss/releases/download/$tpm2tss_version/tpm2-tss-$tpm2tss_version.tar.gz -O $tpm2tss_name.tar.gz || { echo "$msg3"; }
else echo "$msg2"; fi
update_var "json-c"
if [ $install_jsonc -eq 1 ]; then echo "$msg1"
    wget $wget_flag https://github.com/json-c/json-c/archive/refs/tags/json-c-$jsonc_version-20230812.tar.gz -O $jsonc_name.tar.gz || { echo "$msg3"; }
else echo "$msg2"; fi
update_var "texinfo"
if [ $install_texinfo -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$gnu_mirror/texinfo/texinfo-$texinfo_version.tar.gz" -O $texinfo_name.tar.gz || { echo "$msg3"; }
else echo "$msg2"; fi
update_var "automake"
if [ $install_automake -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$gnu_mirror/automake/automake-$automake_version.tar.gz" -O $automake_name.tar.gz || { echo "$msg3"; }
else echo "$msg2"; fi
update_var "autoconf"
if [ $install_autoconf -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$gnu_mirror/autoconf/autoconf-$autoconf_version.tar.gz" -O $autoconf_name.tar.gz || { echo "$msg3"; }
else echo "$msg2"; fi
update_var "help2man"
if [ $install_help2man -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$gnu_mirror/help2man/help2man-$help2man_version.tar.xz" -O $help2man_name.tar.xz || { echo "$msg3"; }
else echo "$msg2"; fi
update_var "m4"
if [ $install_m4 -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$gnu_mirror/m4/m4-$m4_version.tar.gz" -O $m4_name.tar.gz || { echo "$msg3"; }
else echo "$msg2"; fi
update_var "gperf"
if [ $install_gperf -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$gnu_mirror/gperf/gperf-$gperf_version.tar.gz" -O $gperf_name.tar.gz || { echo "$msg3"; }
else echo "$msg2"; fi
update_var "libtool"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$gnu_mirror/libtool/libtool-$libtool_version.tar.gz" -O $libtool_name.tar.gz || { echo "$msg3"; }
else echo "$msg2"; fi
update_var "pkgconfig"
if [ $install_pkgconfig -eq 1 ]; then echo "$msg1"
    wget $wget_flag "https://pkgconfig.freedesktop.org/releases/pkg-config-$pkgconfig_version.tar.gz" -O $pkgconfig_name.tar.gz || { echo "$msg3"; }
else echo "$msg2"; fi

echo "creating directories to hold sources"
total_cnt=11
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg creating directory"
    msg2="$msg skipped!"
}
update_var "libtpms"
if [ $install_libtpms -eq 1 ]; then echo "$msg1"
    mkdir $libtpms_name
else echo "$msg2"; fi
update_var "tpm2tss"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg1"
    mkdir $tpm2tss_name
else echo "$msg2"; fi
update_var "json-c"
if [ $install_jsonc -eq 1 ]; then echo "$msg1"
    mkdir $jsonc_name
else echo "$msg2"; fi
update_var "texinfo"
if [ $install_texinfo -eq 1 ]; then echo "$msg1"
    mkdir $texinfo_name
else echo "$msg2"; fi
update_var "automake"
if [ $install_automake -eq 1 ]; then echo "$msg1"
    mkdir $automake_name
else echo "$msg2"; fi
update_var "autoconf"
if [ $install_autoconf -eq 1 ]; then echo "$msg1"
    mkdir $autoconf_name
else echo "$msg2"; fi
update_var "help2man"
if [ $install_help2man -eq 1 ]; then echo "$msg1"
    mkdir $help2man_name
else echo "$msg2"; fi
update_var "m4"
if [ $install_m4 -eq 1 ]; then echo "$msg1"
    mkdir $m4_name
else echo "$msg2"; fi
update_var "gperf"
if [ $install_gperf -eq 1 ]; then echo "$msg1"
    mkdir $gperf_name
else echo "$msg2"; fi
update_var "libtool"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    mkdir $libtool_name
else echo "$msg2"; fi
update_var "pkgconfig"
if [ $install_pkgconfig -eq 1 ]; then echo "$msg1"
    mkdir $pkgconfig_name
else echo "$msg2"; fi

echo "unzipping sources"
tar_add_flag="--strip-components=1"
total_cnt=11
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg creating directory"
    msg2="$msg skipped!"
}
update_var "libtpms"
if [ $install_libtpms -eq 1 ]; then echo "$msg1"
    tar $tar_flag $libtpms_name.tar.gz  -C $libtpms_name    $tar_add_flag
else echo "$msg2"; fi
update_var "tpm2-tss"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg1"
    tar $tar_flag $tpm2tss_name.tar.gz  -C $tpm2tss_name    $tar_add_flag
else echo "$msg2"; fi
update_var "json-c"
if [ $install_jsonc -eq 1 ]; then echo "$msg1"
    tar $tar_flag $jsonc_name.tar.gz    -C $jsonc_name      $tar_add_flag
else echo "$msg2"; fi
update_var "texinfo"
if [ $install_texinfo -eq 1 ]; then echo "$msg1"
    tar $tar_flag $texinfo_name.tar.gz  -C $texinfo_name    $tar_add_flag
else echo "$msg2"; fi
update_var "automake"
if [ $install_automake -eq 1 ]; then echo "$msg1"
    tar $tar_flag $automake_name.tar.gz -C $automake_name   $tar_add_flag
else echo "$msg2"; fi
update_var "autoconf"
if [ $install_autoconf -eq 1 ]; then echo "$msg1"
    tar $tar_flag $autoconf_name.tar.gz -C $autoconf_name   $tar_add_flag
else echo "$msg2"; fi
update_var "help2man"
if [ $install_help2man -eq 1 ]; then echo "$msg1"
    tar $tar_flag $help2man_name.tar.xz -C $help2man_name   $tar_add_flag
else echo "$msg2"; fi
update_var "m4"
if [ $install_m4 -eq 1 ]; then echo "$msg1"
    tar $tar_flag $m4_name.tar.gz       -C $m4_name         $tar_add_flag
else echo "$msg2"; fi
update_var "gperf"
if [ $install_gperf -eq 1 ]; then echo "$msg1"
    tar $tar_flag $gperf_name.tar.gz    -C $gperf_name      $tar_add_flag
else echo "$msg2"; fi
update_var "libtool"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    tar $tar_flag $libtool_name.tar.gz  -C $libtool_name    $tar_add_flag
else echo "$msg2"; fi
update_var "pkgconfig"
if [ $install_pkgconfig -eq 1 ]; then echo "$msg1"
    tar $tar_flag $pkgconfig_name.tar.gz -C $pkgconfig_name $tar_add_flag
else echo "$msg2"; fi

echo "creating symlink to directories"
total_cnt=11
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg creating symlink to directory"
    msg2="$msg skipped!"
}
update_var "gmp"
if [ $install_gmp -eq 1 ]; then echo "$msg1"
    ln -sf $gmp_name        $gmp_dirname
else echo "$msg2"; fi
update_var "libtpms"
if [ $install_libtpms -eq 1 ]; then echo "$msg1"
    ln -sf $libtpms_name    $libtpms_dirname
else echo "$msg2"; fi
update_var "tpm2tss"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg1"
    ln -sf $tpm2tss_name    $tpm2tss_dirname
else echo "$msg2"; fi
update_var "json-c"
if [ $install_jsonc -eq 1 ]; then echo "$msg1"
    ln -sf $jsonc_name      $jsonc_dirname
else echo "$msg2"; fi
update_var "texinfo"
if [ $install_texinfo -eq 1 ]; then echo "$msg1"
    ln -sf $texinfo_name    $texinfo_dirname
else echo "$msg2"; fi
update_var "automake"
if [ $install_automake -eq 1 ]; then echo "$msg1"
    ln -sf $automake_name   $automake_dirname
else echo "$msg2"; fi
update_var "autoconf"
if [ $install_autoconf -eq 1 ]; then echo "$msg1"
    ln -sf $autoconf_name   $autoconf_dirname
else echo "$msg2"; fi
update_var "help2man"
if [ $install_help2man -eq 1 ]; then echo "$msg1"
    ln -sf $help2man_name   $help2man_dirname
else echo "$msg2"; fi
update_var "m4"
if [ $install_m4 -eq 1 ]; then echo "$msg1"
    ln -sf $m4_name         $m4_dirname
else echo "$msg2"; fi
update_var "gperf"
if [ $install_gperf -eq 1 ]; then echo "$msg1"
    ln -sf $gperf_name      $gperf_dirname
else echo "$msg2"; fi
update_var "libtool"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    ln -sf $libtool_name    $libtool_dirname
else echo "$msg2"; fi
update_var "pkgconfig"
if [ $install_pkgconfig -eq 1 ]; then echo "$msg1"
    ln -sf $pkgconfig_name  $pkgconfig_dirname
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
total_cnt=12
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg compiling"
    msg2="$msg skipped!"
}
update_var "help2man"
if [ $install_help2man -eq 1 ]; then echo "$msg1"
    cd $help2man_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "autoconf"
if [ $install_autoconf -eq 1 ]; then echo "$msg1"
    cd $autoconf_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "gperf"
if [ $install_gperf -eq 1 ]; then echo "$msg1"
    cd $gperf_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "m4"
if [ $install_m4 -eq 1 ]; then echo "$msg1"
    cd $m4_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "automake"
if [ $install_automake -eq 1 ]; then echo "$msg1"
    cd $automake_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "texinfo"
if [ $install_texinfo -eq 1 ]; then echo "$msg1"
    cd $texinfo_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "libtool"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    cd $libtool_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "gmp"
if [ $install_gmp -eq 1 ]; then echo "$msg1"
    cd $gmp_dirname
    ./.bootstrap | ts "$msg"
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "json-c"
if [ $install_jsonc -eq 1 ]; then echo "$msg1"
    cd $working_dir
    mkdir json-c-build
    cd json-c-build
    cmake $jsonc_dirname | ts "$msg"
    make $make_flag | ts "$msg"
    sudo make $make_flag install | ts "$msg" 
else echo "$msg2"; fi
update_var "tpm2tss"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg1"
    cd $tpm2tss_dirname
    gnu_compile "--with-udevrulesdir=/etc/udev/rules.d --with-udevrulesprefix" | ts "$msg"
    sudo udevadm control --reload-rules && sudo udevadm trigger
    sudo ldconfig
else echo "$msg2"; fi
update_var "pkgconfig"
if [ $install_pkgconfig -eq 1 ]; then echo "$msg1"
    cd $pkgconfig_dirname
    gnu_compile "--with-internal-glib" | ts "$msg"
else echo "$msg2"; fi
update_var "libtpms"
if [ $install_libtpms -eq 1 ]; then echo "$msg1"
    cd $libtpms_dirname
    tpm_compile "--with-tpm2 --with-openssl --prefix=/usr" | ts "$msg"
else echo "$msg2"; fi

echo "$script finished"
