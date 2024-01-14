source "./function.sh"
sed -i 's@${HOME}@'"$HOME"'@' config.ini
parse "./config.ini" "display"

check_var apport_dir 1
check_var cmake_dir 1
check_var valgrind_dir 1
clear_dir $apport_dir "rmdir"
clear_dir $cmake_dir "rmdir"
clear_dir $valgrind_dir "rmdir"
