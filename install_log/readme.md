# Install Log

Log recorded with command ```bash install_TPM.sh 2>&1 | tee x.log > /dev/null``` (pipe STDOUT(1) and STDERR(2) with tee to log)

## Ubuntu 18.04 VM

1. [UBT_1.log](UBT_1.log): common + setup_ibmtpm
    - PASS
2. [UBT_2.log](UBT_2.log): common + setup_ibmtpm: Implemented var set/unset check, platform dependent var accross different subdirectories.
    - PASS

## Raspbian Bullseye 2022-07-01 5.1 Kernel Debian i386 VM

1. [RPI5_1.log](RPI5_1.log): common + setup_ibmtpm + setup_optiga: System rebooted during installation.
    - L6101: Package libmysqlclient-dev is not available, but is referred to by another package
    - L6108: Package mysql-server is not available, but is referred to by another package
    - L6126: TSS_Socket_Open: Error on connect to localhost:2321
    - L6127: TSS_Socket_Open: client connect: error 111 Connection refused
    - L6128: createprimary: failed, rc 000b0008
    - L6129: TSS_RC_NO_CONNECTION - Failure connecting to lower layer
    - L6131: TSS_Socket_Open: Error on connect to localhost:2321
    - L6132: TSS_Socket_Open: client connect: error 111 Connection refused
    - L6133: createprimary: failed, rc 000b0008
    - L6134: TSS_RC_NO_CONNECTION - Failure connecting to lower layer
    - L8196: Listing './working_space/restore_to_default'... . Log stopped, system rebooted
2. [RPI5_2.log](RPI5_2.log): common + setup_ibmtpm: Replace libmysqlclient-dev and mysql-server
    - L844: Package libmariadbclient-dev is not available, but is referred to by another package.
    - L865: TSS_Socket_Open: Error on connect to localhost:2321
    - L866: TSS_Socket_Open: client connect: error 111 Connection refused
    - L867: createprimary: failed, rc 000b0008
    - L868: TSS_RC_NO_CONNECTION - Failure connecting to lower layer
    - L870: TSS_Socket_Open: Error on connect to localhost:2321
    - L871: TSS_Socket_Open: client connect: error 111 Connection refused
    - L872: createprimary: failed, rc 000b0008
    - L873: TSS_RC_NO_CONNECTION - Failure connecting to lower layer
    - Compiling: clientenroll.c:57:10: fatal error: json/json.h: No such file or directory
    - Bash: ./setup_sudo.sh: line 353 / 423: gnome-terminal: command not found
    - Bash: common/remove.sh: Failed to remove apport: Directory not empty
    - Bash: setup_ibmtpm/remove.sh: sudo: mysql: command not found
3. [RPI5_3.log](RPI5_3.log): common + setup_ibmtpm
    - L868: TSS_Socket_Open: Error on connect to localhost:2321
    - L869: TSS_Socket_Open: client connect: error 111 Connection refused
    - L870: createprimary: failed, rc 000b0008
    - L871: TSS_RC_NO_CONNECTION - Failure connecting to lower layer
    - L873: TSS_Socket_Open: Error on connect to localhost:2321
    - L874: TSS_Socket_Open: client connect: error 111 Connection refused
    - L875: createprimary: failed, rc 000b0008
    - L876: TSS_RC_NO_CONNECTION - Failure connecting to lower layer
    - Compiling: clientenroll.c:57:10: fatal error: mysql/mysql.h: No such file or directory
    - Bash: ./setup_sudo.sh: line 353 / 423: gnome-terminal: command not found
    - (not tested) Bash: common/remove.sh: Failed to remove apport: Directory not empty
    - (not tested) Bash: setup_ibmtpm/remove.sh: sudo: mysql: command not found
4. [RPI5_4.log](RPI5_4.log)/[RPI5_4_A.log](RPI5_4_A.log): common + setup_ibmtpm
    - L1004: The following packages have unmet dependencies:
        - libmariadb-dev : Conflicts: libmariadb-dev:amd64 but 1:10.5.21-0+deb11u1 is to be installed
        - libmariadb-dev:amd64 : Conflicts: libmariadb-dev but 1:10.5.21-0+deb11u1 is to be installed
    - L1024: TSS_Socket_Open: Error on connect to localhost:2321
        - L1025: TSS_Socket_Open: client connect: error 111 Connection refused
        - L1026: createprimary: failed, rc 000b0008
        - L1027: TSS_RC_NO_CONNECTION - Failure connecting to lower layer
    - L1029: TSS_Socket_Open: Error on connect to localhost:2321
        - L1030: TSS_Socket_Open: client connect: error 111 Connection refused
        - L1031: createprimary: failed, rc 000b0008
        - L1032: TSS_RC_NO_CONNECTION - Failure connecting to lower layer
    - AL37: Failed to stop whoopsie.service: Unit whoopsie.service not loaded.
    - AL45: /usr/bin/ld: ../utils/cryptoutils.o:/opt/ibmtss2.1.1/utils/cryptoutils.c:118: multiple definition of 'tssUtilsVerbose'; nvreadvalueauth.o:/opt/ibmtss2.1.1/utils12/nvreadvalueauth.c:56: first defined here
        - AL46: collect2: error: ld returned 1 exit status
        - AL47: make: *** [makefiletpmc:175: nvreadvalueauth] Error 1
    - AL48: E: Unable to correct problems, you have held broken packages.
    - AL49: ./setup_sudo.sh: line 199: mysql: command not found
        - AL50: ./setup_sudo.sh: line 200: mysql: command not found
        - AL51: ./setup_sudo.sh: line 201: mysql: command not found
        - AL52: ./setup_sudo.sh: line 202: mysql: command not found
    - AL53: clientenroll.c:57:10: fatal error: json/json.h: No such file or directory
        - AL54: 57 | #include <json/json.h>
        - AL55:    |          ^~~~~~~~~~~~~
        - AL56: compilation terminated.
        - AL57: make: *** [makefile-common:181: clientenroll.o] Error 1
    - AL58: ./setup_sudo.sh: line 281: gnome-terminal: command not found
        - AL59: ./setup_sudo.sh: line 307: gnome-terminal: command not found
        - AL60: ./setup_sudo.sh: line 320: gnome-terminal: command not found
        - AL66: ./setup_sudo.sh: line 388: gnome-terminal: command not found
        - AL67: ./setup_sudo.sh: line 458: gnome-terminal: command not found
        - AL68: ./setup_sudo.sh: line 458: gnome-terminal: command not found
        - AL69: ./setup_sudo.sh: line 458: gnome-terminal: command not found
        - AL70: ./setup_sudo.sh: line 458: gnome-terminal: command not found
        - AL71: ./setup_sudo.sh: line 458: gnome-terminal: command not found
        - AL72: ./setup_sudo.sh: line 458: gnome-terminal: command not found
        - AL73: ./setup_sudo.sh: line 458: gnome-terminal: command not found

## Raspbian Bullseye 2023-05-03 6.x Kernal Debian arm64

1. [RPI6_1.log](RPI6_1.log): common + setup_ibmtpm + setup_optiga: System rebooted during installation.
    - L6097: Package libmysqlclient-dev is not available, but is referred to by another package
    - L6104: Package mysql-server is not available, but is referred to by another package
    - L6122: TSS_Socket_Open: Error on connect to localhost:2321
    - L6123: TSS_Socket_Open: client connect: error 111 Connection refused
    - L6124: createprimary: failed, rc 000b0008
    - L6125: TSS_RC_NO_CONNECTION - Failure connecting to lower layer
    - L6127: TSS_Socket_Open: Error on connect to localhost:2321
    - L6128: TSS_Socket_Open: client connect: error 111 Connection refused
    - L6129: createprimary: failed, rc 000b0008
    - L6130: TSS_RC_NO_CONNECTION - Failure connecting to lower layer
    - Unknown terminal status. (unrecorded)
    - L8197: Listing './working_space/restore_to_default'... . Log stopped, system rebooted
2. [RPI6_2.log](RPI6_2.log): common + setup_optiga:
    - Implement common subdirectory structure: config.ini + function_<name>.sh + remove.sh + setup.sh
    - Removed auto reboot
    - PASS on Raspberry Pi 4B without TPM chip

## Jetson Nano

Empty
