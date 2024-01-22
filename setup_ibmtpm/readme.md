# IBM TPM

Scripts here is tested on Ubuntu 18.04 VM.

Currently, ACS doesn't support TPM2.0.

## Directory Description

1. [config.ini](config.ini): Configuration file for installation.
2. [readme.md](readme.md): This file.
3. [remove.sh](remove.sh): Uninstall script.
4. [setup.sh](setup.sh): Install script approach with minimum privilege, unsuccesful.
5. [setup_sudo.sh](setup_sudo.sh): Install script approach with sudo privilege, successful.
6. [setup_sudo_old.sh](setup_sudo_old.sh): Backup of setup_sudo.sh.

## Run specific functionalities of setup.sh

1. Adjust [config.ini](config.ini) to your needs.
2. Execute ```sudo bash setup_sudo.sh```

## Installing Tools

1. Software stack: 
    1. [GitHub: kgoldman/ibmtss](https://github.com/kgoldman/ibmtss)
    2. [SourceForge: IBM's TPM 2.0 TSS Files](https://sourceforge.net/projects/ibmtpm20tss/files/)
2. Emulated TPM: 
    1. [GitHub: stefanberger/swtpm: SWTPM - Software TPM Emulator](https://github.com/stefanberger/swtpm)
    2. [SourceForge: IBM's Software TPM 2.0 Files](https://sourceforge.net/projects/ibmswtpm2/files/)
3. Demo example: 
    1. [GitHub: kgoldman/acs: IBM Attestation Client Server](https://github.com/kgoldman/acs)
    2. [SourceForge: IBM TPM Attestation Client Server Files](https://sourceforge.net/projects/ibmtpm20acs/files/)

## Related Sources

1. [libtss packages in ubuntu](https://packages.ubuntu.com/search?keywords=libtss&searchon=names)
2. [IBM Attestation Client Server测试](https://rlyown.github.io/2021/02/28/IBM-Attestation-Client-Server%E6%B5%8B%E8%AF%95/)
3. [记一次双系统配置TPM的过程](https://rlyown.github.io/2021/05/29/%E8%AE%B0%E4%B8%80%E6%AC%A1%E5%8F%8C%E7%B3%BB%E7%BB%9F%E9%85%8D%E7%BD%AETPM%E7%9A%84%E8%BF%87%E7%A8%8B/#%E7%9B%B8%E5%85%B3%E8%B5%84%E6%96%99)

## Possible Bugs

- Only tested on Ubuntu 18.04 VM, not on physical machine.

## Progress Update

1. [TPM Sharing Scheme 2023/11/06:11:22:45 Commit: 121ea01](https://youtu.be/RcyuaFtERZM)
2. [TPM Sharing Scheme 2023/11/06 20:02:42 Commit: cc7fc78](https://youtu.be/Na3WUpZXb0Q)
3. [TPM Sharing Scheme 2023/11/06 20:15:36 Commit: 803fc86](https://youtu.be/0gP2gU_3JKY)
4. [TPM Sharing Scheme 2024/01/11-15:49:39 Commit: c411b6eeb54cb63e04db3ae6017f1999f961cd10](https://youtu.be/x2bHqZr6nYA) Realize automatic installation setup script with minimum user interaction. ([File State](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/c411b6eeb54cb63e04db3ae6017f1999f961cd10))
