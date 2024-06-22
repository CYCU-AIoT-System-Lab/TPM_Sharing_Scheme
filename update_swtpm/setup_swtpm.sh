#!/bin/bash
script=$(realpath "$0")
script_path=$(dirname "$script")
source function_swtpm.sh

if [ $install_apt_package -eq 1 ]; then
    echo "installing apt packages"
    sudo apt install -y mercurial
fi

echo "downloading from sources"
github="https://github.com"
gnu_mirror="ftp://ftp.twaren.net/Unix/GNU/gnu"
gnome="https://download.gnome.org/sources"
total_cnt=22
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
    hg clone https://gmplib.org/repo/gmp-$gmp_version/ $gmp_name || { echo "$msg4"; exit 1; }
else echo "$msg2"; fi
update_var "libtpms"
if [ $install_libtpms -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$github/stefanberger/libtpms/archive/refs/tags/v$libtpms_version$libtpms_ext" -O $libtpms_name$libtpms_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi
update_var "tpm2-tss"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$github/tpm2-software/tpm2-tss/releases/download/$tpm2tss_version/tpm2-tss-$tpm2tss_version$tpm2tss_ext" -O $tpm2tss_name$tpm2tss_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi
update_var "json-c"
if [ $install_jsonc -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$github/json-c/json-c/archive/refs/tags/json-c-$jsonc_version-20230812$jsonc_ext" -O $jsonc_name$jsonc_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi
update_var "texinfo"
if [ $install_texinfo -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$gnu_mirror/texinfo/texinfo-$texinfo_version$texinfo_ext" -O $texinfo_name$texinfo_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi
update_var "automake"
if [ $install_automake -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$gnu_mirror/automake/automake-$automake_version$automake_ext" -O $automake_name$automake_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi
update_var "autoconf"
if [ $install_autoconf -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$gnu_mirror/autoconf/autoconf-$autoconf_version$autoconf_ext" -O $autoconf_name$autoconf_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi
update_var "help2man"
if [ $install_help2man -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$gnu_mirror/help2man/help2man-$help2man_version$help2man_ext" -O $help2man_name$help2man_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi
update_var "m4"
if [ $install_m4 -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$gnu_mirror/m4/m4-$m4_version$m4_ext" -O $m4_name$m4_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi
update_var "gperf"
if [ $install_gperf -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$gnu_mirror/gperf/gperf-$gperf_version$gperf_ext" -O $gperf_name$gperf_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi
update_var "pkg-config"
if [ $install_pkgconfig -eq 1 ]; then echo "$msg1"
    wget $wget_flag "https://pkgconfig.freedesktop.org/releases/pkg-config-$pkgconfig_version$pkgconfig_ext" -O $pkgconfig_name$pkgconfig_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi
update_var "libtool"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$gnu_mirror/libtool/libtool-$libtool_version$libtool_ext" -O $libtool_name$libtool_ext || { echo "$msg4"; exit 1; }
else echo "$msg2"; fi
update_var "libtasn1"
if [ $install_libtasn1 -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$gnu_mirror/libtasn1/libtasn1-$libtasn1_version$libtasn1_ext" -O $libtasn1_name$libtasn1_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi
update_var "python"
if [ $install_python -eq 1 ]; then echo "$msg1"
    wget $wget_flag "https://www.python.org/ftp/python/$python_version/Python-$python_version$python_ext" -O $python_name$python_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi
update_var "libpcre2"
if [ $install_libpcre2 -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$github/PCRE2Project/pcre2/releases/download/pcre2-$libpcre2_version/pcre2-$libpcre2_version$libpcre2_ext" -O $libpcre2_name$libpcre2_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi
update_var "glib"
if [ $install_glib -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$gnome/glib/${glib_version%.*}/glib-$glib_version$glib_ext" -O $glib_name$glib_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi
update_var "json-glib"
if [ $install_jsonglib -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$gnome/json-glib/${jsonglib_version%.*}/json-glib-$jsonglib_version$jsonglib_ext" -O $jsonglib_name$jsonglib_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi
update_var "bison"
if [ $install_bison -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$gnu_mirror/bison/bison-$bison_version$bison_ext" -O $bison_name$bison_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi
update_var "openssl"
if [ $install_openssl -eq 1 ]; then echo "$msg1"
    openssl_ver_arr=(${openssl_version//./ } )
    if [[ ${openssl_ver_arr[0]} -ge 3 ]]; then
        wget $wget_flag "$github/openssl/openssl/releases/download/openssl-$openssl_version/openssl-$openssl_version$openssl_ext" -O $openssl_name$openssl_ext || { echo "$msg3"; exit 1; }
    else
        wget $wget_flag "https://www.openssl.org/source/old/${openssl_version::-1}/openssl-$openssl_version$openssl_ext" -O $openssl_name$openssl_ext || { echo "$msg3"; exit 1; }
    fi
else echo "$msg2"; fi
update_var "libcurl"
if [ $install_libcurl -eq 1 ]; then echo "$msg1"
    wget $wget_flag "https://curl.se/download/curl-$libcurl_version$libcurl_ext" -O $libcurl_name$libcurl_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi
update_var "libuuid"
if [ $install_libuuid -eq 1 ]; then echo "$msg1"
    wget $wget_flag "https://sourceforge.net/projects/libuuid/files/libuuid-$libuuid_version$libuuid_ext/download" -O $libuuid_name$libuuid_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi
update_var "swtpm"
if [ $install_swtpm -eq 1 ]; then echo "$msg1"
    wget $wget_flag "$github/stefanberger/swtpm/archive/refs/tags/v$swtpm_version$swtpm_ext" -O $swtpm_name$swtpm_ext || { echo "$msg3"; exit 1; }
else echo "$msg2"; fi

echo "creating directories to hold sources"
total_cnt=21
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
update_var "tpm2-tss"
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
update_var "pkg-config"
if [ $install_pkgconfig -eq 1 ]; then echo "$msg1"
    mkdir $pkgconfig_name
else echo "$msg2"; fi
update_var "libtasn1"
if [ $install_libtasn1 -eq 1 ]; then echo "$msg1"
    mkdir $libtasn1_name
else echo "$msg2"; fi
update_var "python"
if [ $install_python -eq 1 ]; then echo "$msg1"
    mkdir $python_name
else echo "$msg2"; fi
update_var "libpcre2"
if [ $install_libpcre2 -eq 1 ]; then echo "$msg1"
    mkdir $libpcre2_name
else echo "$msg2"; fi
update_var "glib"
if [ $install_glib -eq 1 ]; then echo "$msg1"
    mkdir $glib_name
else echo "$msg2"; fi
update_var "json-glib"
if [ $install_jsonglib -eq 1 ]; then echo "$msg1"
    mkdir $jsonglib_name
else echo "$msg2"; fi
update_var "bison"
if [ $install_bison -eq 1 ]; then echo "$msg1"
    mkdir $bison_name
else echo "$msg2"; fi
update_var "openssl"
if [ $install_openssl -eq 1 ]; then echo "$msg1"
    mkdir $openssl_name
else echo "$msg2"; fi
update_var "libcurl"
if [ $install_libcurl -eq 1 ]; then echo "$msg1"
    mkdir $libcurl_name
else echo "$msg2"; fi
update_var "libuuid"
if [ $install_libuuid -eq 1 ]; then echo "$msg1"
    mkdir $libuuid_name
else echo "$msg2"; fi
update_var "swtpm"
if [ $install_swtpm -eq 1 ]; then echo "$msg1"
    mkdir $swtpm_name
else echo "$msg2"; fi

echo "unzipping sources"
tar_add_flag="--strip-components=1"
total_cnt=21
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg creating directory"
    msg2="$msg skipped!"
}
update_var "libtpms"
if [ $install_libtpms -eq 1 ]; then echo "$msg1"
    tar $tar_flag $libtpms_name$libtpms_ext -C $libtpms_name    $tar_add_flag
else echo "$msg2"; fi
update_var "tpm2-tss"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg1"
    tar $tar_flag $tpm2tss_name$tpm2tss_ext -C $tpm2tss_name    $tar_add_flag
else echo "$msg2"; fi
update_var "json-c"
if [ $install_jsonc -eq 1 ]; then echo "$msg1"
    tar $tar_flag $jsonc_name$jsonc_ext    -C $jsonc_name      $tar_add_flag
else echo "$msg2"; fi
update_var "texinfo"
if [ $install_texinfo -eq 1 ]; then echo "$msg1"
    tar $tar_flag $texinfo_name$texinfo_ext  -C $texinfo_name    $tar_add_flag
else echo "$msg2"; fi
update_var "automake"
if [ $install_automake -eq 1 ]; then echo "$msg1"
    tar $tar_flag $automake_name$automake_ext -C $automake_name   $tar_add_flag
else echo "$msg2"; fi
update_var "autoconf"
if [ $install_autoconf -eq 1 ]; then echo "$msg1"
    tar $tar_flag $autoconf_name$autoconf_ext -C $autoconf_name   $tar_add_flag
else echo "$msg2"; fi
update_var "help2man"
if [ $install_help2man -eq 1 ]; then echo "$msg1"
    tar $tar_flag $help2man_name$help2man_ext -C $help2man_name   $tar_add_flag
else echo "$msg2"; fi
update_var "m4"
if [ $install_m4 -eq 1 ]; then echo "$msg1"
    tar $tar_flag $m4_name$m4_ext       -C $m4_name         $tar_add_flag
else echo "$msg2"; fi
update_var "gperf"
if [ $install_gperf -eq 1 ]; then echo "$msg1"
    tar $tar_flag $gperf_name$gperf_ext    -C $gperf_name      $tar_add_flag
else echo "$msg2"; fi
update_var "libtool"
if [ $install_libtool -eq 1 ]; then echo "$msg1"
    tar $tar_flag $libtool_name$libtool_ext  -C $libtool_name    $tar_add_flag
else echo "$msg2"; fi
update_var "pkg-config"
if [ $install_pkgconfig -eq 1 ]; then echo "$msg1"
    tar $tar_flag $pkgconfig_name$pkgconfig_ext -C $pkgconfig_name $tar_add_flag
else echo "$msg2"; fi
update_var "libtasn1"
if [ $install_libtasn1 -eq 1 ]; then echo "$msg1"
    tar $tar_flag $libtasn1_name$libtasn1_ext -C $libtasn1_name   $tar_add_flag
else echo "$msg2"; fi
update_var "python"
if [ $install_python -eq 1 ]; then echo "$msg1"
    tar $tar_flag $python_name$python_ext -C $python_name          $tar_add_flag
else echo "$msg2"; fi
update_var "libpcre2"
if [ $install_libpcre2 -eq 1 ]; then echo "$msg1"
    tar $tar_flag $libpcre2_name$libpcre2_ext -C $libpcre2_name   $tar_add_flag
else echo "$msg2"; fi
update_var "glib"
if [ $install_glib -eq 1 ]; then echo "$msg1"
    tar $tar_flag $glib_name$glib_ext -C $glib_name           $tar_add_flag
else echo "$msg2"; fi
update_var "json-glib"
if [ $install_jsonglib -eq 1 ]; then echo "$msg1"
    tar $tar_flag $jsonglib_name$jsonglib_ext -C $jsonglib_name   $tar_add_flag
else echo "$msg2"; fi
update_var "bison"
if [ $install_bison -eq 1 ]; then echo "$msg1"
    tar $tar_flag $bison_name$bison_ext -C $bison_name         $tar_add_flag
else echo "$msg2"; fi
update_var "openssl"
if [ $install_openssl -eq 1 ]; then echo "$msg1"
    tar $tar_flag $openssl_name$openssl_ext -C $openssl_name         $tar_add_flag
else echo "$msg2"; fi
update_var "libcurl"
if [ $install_libcurl -eq 1 ]; then echo "$msg1"
    tar $tar_flag $libcurl_name$libcurl_ext -C $libcurl_name         $tar_add_flag
else echo "$msg2"; fi
update_var "libuuid"
if [ $install_libuuid -eq 1 ]; then echo "$msg1"
    tar $tar_flag $libuuid_name$libuuid_ext -C $libuuid_name         $tar_add_flag
else echo "$msg2"; fi
update_var "swtpm"
if [ $install_swtpm -eq 1 ]; then echo "$msg1"
    tar $tar_flag $swtpm_name$swtpm_ext -C $swtpm_name         $tar_add_flag
else echo "$msg2"; fi

echo "creating symlink to directories"
total_cnt=22
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
update_var "tpm2-tss"
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
update_var "pkg-config"
if [ $install_pkgconfig -eq 1 ]; then echo "$msg1"
    ln -sf $pkgconfig_name  $pkgconfig_dirname
else echo "$msg2"; fi
update_var "libtasn1"
if [ $install_libtasn1 -eq 1 ]; then echo "$msg1"
    ln -sf $libtasn1_name $libtasn1_dirname
else echo "$msg2"; fi
update_var "python"
if [ $install_python -eq 1 ]; then echo "$msg1"
    ln -sf $python_name $python_dirname
else echo "$msg2"; fi
update_var "libpcre2"
if [ $install_libpcre2 -eq 1 ]; then echo "$msg1"
    ln -sf $libpcre2_name $libpcre2_dirname
else echo "$msg2"; fi
update_var "glib"
if [ $install_glib -eq 1 ]; then echo "$msg1"
    ln -sf $glib_name $glib_dirname
else echo "$msg2"; fi
update_var "json-glib"
if [ $install_jsonglib -eq 1 ]; then echo "$msg1"
    ln -sf $jsonglib_name $jsonglib_dirname
else echo "$msg2"; fi
update_var "bison"
if [ $install_bison -eq 1 ]; then echo "$msg1"
    ln -sf $bison_name $bison_dirname
else echo "$msg2"; fi
update_var "openssl"
if [ $install_openssl -eq 1 ]; then echo "$msg1"
    ln -sf $openssl_name $openssl_dirname
else echo "$msg2"; fi
update_var "libcurl"
if [ $install_libcurl -eq 1 ]; then echo "$msg1"
    ln -sf $libcurl_name $libcurl_dirname
else echo "$msg2"; fi
update_var "libuuid"
if [ $install_libuuid -eq 1 ]; then echo "$msg1"
    ln -sf $libuuid_name $libuuid_dirname
else echo "$msg2"; fi
update_var "swtpm"
if [ $install_swtpm -eq 1 ]; then echo "$msg1"
    ln -sf $swtpm_name $swtpm_dirname
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
    sudo bash -c "source $venv_bin_path && $meson_path install -C _build"
    sudo cp _build/meson-private/*.pc /usr/local/lib/pkgconfig
}
total_cnt=23
progress_cnt=0
update_var () {
    progress_cnt=$((progress_cnt + 1))
    msg="$progress_cnt/$total_cnt - $1 >>"
    msg1="$msg compiling"
    msg2="$msg skipped!"
}
update_var "pkg-config"
if [ $install_pkgconfig -eq 1 ]; then echo "$msg1"
    cd $pkgconfig_dirname
    gnu_compile "--with-internal-glib" | ts "$msg"
else echo "$msg2"; fi
update_var "bison"
if [ $install_bison -eq 1 ]; then echo "$msg1"
    cd $bison_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "openssl"
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
update_var "python"
if [ $install_python -eq 1 ]; then echo "$msg1"
    cd $python_dirname
    sudo ./configure --enable-optimizations --prefix=$python_dirname --exec-prefix=$python_dirname | ts "$msg"
    make $make_flag | ts "$msg"
    sudo make $make_flag altinstall | ts "$msg"
    echo "$msg creating virtual environment for library building"
    "$python_bin_path/python${python_version%.*}" -m venv $python_venv_path
else echo "$msg2"; fi
update_var "pip-package"
if [ $install_pip_package -eq 1 ]; then echo "$msg1"
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
update_var "libpcre2"
if [ $install_libpcre2 -eq 1 ]; then echo "$msg1"
    cd $libpcre2_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "glib"
if [ $install_glib -eq 1 ]; then echo "$msg1"
    cd $glib_dirname
    meson_compile | ts "$msg"
else echo "$msg2"; fi
update_var "json-glib"
if [ $install_jsonglib -eq 1 ]; then echo "$msg1"
    cd $jsonglib_dirname
    meson_compile | ts "$msg"
else echo "$msg2"; fi
update_var "libtasn1"
if [ $install_libtasn1 -eq 1 ]; then echo "$msg1"
    cd $libtasn1_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
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
update_var "libcurl"
if [ $install_libcurl -eq 1 ]; then echo "$msg1"
    cd $libcurl_dirname
    gnu_compile "--with-openssl" | ts "$msg"
else echo "$msg2"; fi
update_var "libuuid"
if [ $install_libuuid -eq 1 ]; then echo "$msg1"
    cd $libuuid_dirname
    gnu_compile | ts "$msg"
else echo "$msg2"; fi
update_var "tpm2-tss"
if [ $install_tpm2tss -eq 1 ]; then echo "$msg1"
    cd $tpm2tss_dirname
    gnu_compile "--with-udevrulesdir=/etc/udev/rules.d --with-udevrulesprefix" | ts "$msg"
    sudo udevadm control --reload-rules && sudo udevadm trigger
    sudo ldconfig
else echo "$msg2"; fi
update_var "libtpms"
if [ $install_libtpms -eq 1 ]; then echo "$msg1"
    cd $libtpms_dirname
    tpm_compile "--with-tpm2 --with-openssl --prefix=/usr" | ts "$msg"
else echo "$msg2"; fi
update_var "swtpm"
if [ $install_swtpm -eq 1 ]; then echo "$msg1"
    cd $swtpm_dirname
    tpm_compile "--with-openssl --prefix=/usr" | ts "$msg"
else echo "$msg2"; fi

echo "$script finished"
