# Common Utilities

This directory holdes common utilities for this project.

## Install Steps

0. Setup OS, refer to [../doc/technology/os_setup.md](../doc/technology/os_setup.md).
1. In terminal, type ```ssh-keygen```, keep pressing ```enter``` till command is finished.
2. In terminal, type ```cat ~/.ssh/id_rsa.pub```, copy all of the output string.
3. In browser, go to <https://github.com/settings/ssh/new>, type the name of this key in title, and paste copied RSA public key in.
4. In terminal, copy the code in [./download_repo.sh](./download_repo.sh) into a local bash file and execute it without privilege.
5. After the script is finished, set [./config.ini](./config.ini) item ```install_platform``` to your platform.
6. Execute [./setup.sh](./setup.sh).
7. After the script is finished, and showed "Reboot", reboot.
8. Edit ```/common/config.ini``` to disable ```setup_optiga``` and enable other components you want to install, and execute ```/common/setup.sh``` again.

## Component install ordering

Part of the components are dependent on each other, please follow the order below to install them.

1. job_common + job_setup_optiga
2. job_setup_ibmtpm
3. (optional) job_update_swtpm + job_setup_mbc_last

## Adjust Installation Components - setup_environment

1. [./config.ini](./config.ini) set all other components to 0.
2. [./config.ini](./config.ini) set ```setup_environment = 1```.
3. Execute [./setup.sh](./setup.sh).

## Adjust Installation Components - setup_ibmtpm

1. [./config.ini](./config.ini) set all other components to 0.
2. [./config.ini](./config.ini) set ```setup_ibmtpm = 1```.
3. [./config.ini](./config.ini) set ```install_platform = <your_setting>```.
4. Execute [./setup.sh](./setup.sh).

Note: Re-installing setup_ibmtpm: ```./remove.sh; ./setup.sh```

## Adjust Installation Components - socket_com

1. [./config.ini](./config.ini) set all other components to 0.
2. [./config.ini](./config.ini) set ```setup_socket_com = 1```.
3. Execute [./setup.sh](./setup.sh).

## Directory Structure

1. [config.ini](config.ini): Configuration file for mutual dependencies installation.
2. [copy_VM.ps1](copy_VM.ps1): PowerShell script for duplicating VMware VMs.
3. [function.sh](function.sh): Common bash functions for all subprojects.
4. [readme.md](readme.md): This file.
5. [remove.sh](remove.sh): Remove installed components by [setup.sh](setup.sh) besides ones using apt.
6. [setup.sh](setup.sh): Install mutual dependencies and invoke corresponding installation scripts in other subdirectories.

## Demo Video

| No. | IDate-Time          | Commit                                                                                                                                                      | Detail                                               | Demo Video                     |
| --- | ------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------- | ------------------------------ |
| 1   | 2023/12/08-18:07:19 | [3a1293e33b725dbe6380b257f01aab1899bf61e0](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/3a1293e33b725dbe6380b257f01aab1899bf61e0/common) | Can install and execute ```../socket_com/setup.sh``` | <https://youtu.be/ZcaLBuhwKuw> |
