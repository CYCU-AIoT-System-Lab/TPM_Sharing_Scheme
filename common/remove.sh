source "./function.sh"
sed -i 's@${HOME}@'"$HOME"'@' config.ini
parse "./config.ini" "display"

check_var apport_dir 1
check_var cmake_dir 1
check_var valgrind_dir 1
echo_notice "common" "remove" "Var check done"

clear_dir apport_dir "rmdir"
clear_dir cmake_dir "rmdir"
clear_dir valgrind_dir "rmdir"
echo_notice "common" "remove" "Clear dir done"

unset apport_dir
unset cmake_dir
unset valgrind_dir
echo_notice "common" "remove" "Unset var done"
