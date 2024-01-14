# Install Log

Log recorded with command ```bash install_TPM.sh | tee x.log > /dev/null```

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

## Jetson Nano

Empty
