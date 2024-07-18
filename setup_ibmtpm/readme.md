# IBM TPM and ACS

Scripts here is tested on Ubuntu 18.04 VM.

Currently, ACS doesn't support TPM2.0.

## Usage

If no presets are set (default), the script will require the user to input the option they desire.

1. Installing platform: 1~6
    - Add preset: `touch ./ip<1~6>`, like `touch ./ip5` for preset 5.
2. config file option: the script will list out options in `./config_files/` directory.
    - Add preset: `touch ./cf<1~6>`, like `touch ./cf5` for preset 5.
    - You can duplicate the default config files in `./config_files/` and modify them to your needs. It will also be listed as an option.

## Directory Description

1. [config_files](config_files): Configuration files for installation.
    1. [config_files/config.ini.template](config_files/config.ini.template): Template for configuration file. Should not be selected.
    2. [config_files/config_dTPM_local.ini](config_files/config_dTPM_local.ini): Launch ACS setup process which local dTPM installed.
    3. [config_files/config_RA_client_launch.ini](config_files/config_RA_client_launch.ini): Launch ACS Remote Attestation client demo. Should only be selected after `config_RA_client_setup.ini` finished.
    4. [config_files/config_RA_client_setup.ini](config_files/config_RA_client_setup.ini): Launch ACS Remote Attestation client setup and demo process.
    5. [config_files/config_RA_server.ini](config_files/config_RA_server.ini): Launch ACS Remote Attestation server setup and demo process.
    6. config_files/config_vTPM_local.ini: Launch ACS setup process which only want with vTPM.
2. [function_ibmtpm.sh](function_ibmtpm.sh): Functions for both `setup.sh` and `remove.sh`.
3. [readme.md](readme.md): This file.
4. [remove.sh](remove.sh): Uninstall script.
5. [setup.sh](setup.sh): Install script approach with minimum privilege, unsuccesful.
6. [setup_sudo.sh](setup_sudo.sh): Install script approach with sudo privilege, successful.

## Run specific functionalities of setup.sh

1. Adjust specific `.ini` config file in `/setup_ibmtpm/config_files/` to your needs.
2. Execute ```sudo bash setup_sudo.sh```

## Execute with updated SWTPM

1. Adjust specific `.ini` config file in `/setup_ibmtpm/config_files/`, set setting `new_swtpm=1`.
2. Execute ```sudo bash setup_sudo.sh```

Note: New SWTPM is communicated through its socket interface, character device interface is still not supported.

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
4. https://github.com/kgoldman/ibmtss/issues/10
5. [Paper: Subverting Linux' Integrity Measurement Architecture](https://svs.informatik.uni-hamburg.de/publications/2020/2020-08-27-Bohling-IMA.pdf)
6. [Linux man page: tpm_quote_tools(8)](https://linux.die.net/man/8/tpm_quote_tools)
7. [Infineon/remote-attestation-optiga-tpm](https://github.com/Infineon/remote-attestation-optiga-tpm)
8. [LWN.net ima: directory and special files integrity protection](https://lwn.net/Articles/512364/)
9. [LWN.net LSS: Integrity for directories and special files](https://lwn.net/Articles/516334/)
10. [FiveDirections/IMA: File Integrity Monitoring for Linux](https://github.com/FiveDirections/IMA)
11. [SourceForge: Integrity Measurement Architecture (IMA) Wiki](https://sourceforge.net/p/linux-ima/wiki/Home/)
12. [YouTube: TPM (Trusted Platform Module) - Computerphile](https://youtu.be/RW2zHvVO09g?si=zcRH45MMXyjJSKKm)
13. [YouTube: An Unified TPM Event Log for Linux](https://youtu.be/FA1O3fuPQDk?si=K1z7dl8-cpbZpAis)
14. [YouTube: Using the TPM - It's Not Rocket Science (Anymore) - Johannes Holland & Peter Huewe](https://youtu.be/XwaSyHJIos8?si=Z2K7q_Jwu0uqy26w)
15. [YouTube: "TPM based attestation - how can we use it for good?" - Matthew Garrett (LCA 2020)](https://youtu.be/FobfM9S9xSI?si=5sJ616F-f-SlhL97)
16. [YouTube: Securing Embedded Linux Systems with TPM 2.0 - Philip Tricca, Intel](https://youtu.be/0qu9R7Tlw9o?si=ptPCEsUcgdMNVZz6)
17. [YouTube: Better Data Security with Commodity TPM Chips - Haris Okanovic, National Instruments](https://youtu.be/fu2RGBcb9aQ?si=_DyP3f0Lc9KXTJK_)

## Possible Bugs

- Only tested on Ubuntu 18.04 VM, not on physical machine.
- If you see `TSS_RC_NO_CONNECTION - Failure connecting to lower layer`, try press the reset button on TPM and reboot the system.
- If you see `no akecpub_<client_ipv4>.bin` not found, remove the same machine from server DB table and re-run client setup.

## Progress Update

1. [TPM Sharing Scheme 2023/11/06:11:22:45 Commit: 121ea01](https://youtu.be/RcyuaFtERZM)
2. [TPM Sharing Scheme 2023/11/06 20:02:42 Commit: cc7fc78](https://youtu.be/Na3WUpZXb0Q)
3. [TPM Sharing Scheme 2023/11/06 20:15:36 Commit: 803fc86](https://youtu.be/0gP2gU_3JKY)
4. [TPM Sharing Scheme 2024/01/11-15:49:39 Commit: c411b6eeb54cb63e04db3ae6017f1999f961cd10](https://youtu.be/x2bHqZr6nYA) Realize automatic installation setup script with minimum user interaction. ([File State](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/c411b6eeb54cb63e04db3ae6017f1999f961cd10))
