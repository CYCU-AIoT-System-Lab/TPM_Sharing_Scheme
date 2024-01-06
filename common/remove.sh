source "./function.sh"
sed -i 's@${HOME}@'"$HOME"'@' config.ini
parse "./config.ini" "display"

clear_dir $apport_dir "rmdir"
clear_dir $cmake_dir "rmdir"
clear_dir $valgrind_dir "rmdir"
