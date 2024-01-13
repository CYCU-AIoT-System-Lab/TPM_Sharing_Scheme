# Common Utilities

This directory holdes common utilities for this project.

## Install Steps

1. In VM terminal, type ```ssh-keygen```, keep pressing ```enter``` till command is finished.
2. In VM terminal, type ```cat ~/.ssh/id_rsa.pub```, copy all of the output string.
3. In browser, go to <https://github.com/settings/ssh/new>, type the name of this key in title, and paste copied RSA public key in.
4. In VM terminal, copy the following code into a local bash file and execute it without privilege.

```bash
#!/bin/bash
sudo apt-get install -y git
sudo apt autoremove -y
ssh-keyscan github.com >> ~/.ssh/known_hosts
git config --global user.email "dachuan516@gmail.com"
git config --global user.name  "belongtothenight"
git clone git@github.com:CYCU-AIoT-System-Lab/TPM_Sharing_Scheme.git
cd TPM_Sharing_Scheme
cd common
chmod +x setup.sh
./setup.sh
cd ..

```

## Adjust Installation Components

Adjust below settings in [config.ini](config.ini).

- ```setup_environment = 1```: Call [setup.sh](../setup_environment/setup.sh) to perform installation.
- ```setup_ibmtpm = 1```: Call [setup.sh](../ibmtpm/setup.sh) to perform installation.
- ```setup_socket_com = 1```: Call [setup.sh](../socket_com/setup.sh) to perform installation.

## Directory Structure

1. [config.ini](config.ini): Configuration file for mutual dependencies installation.
2. [copy_VM.ps1](copy_VM.ps1): PowerShell script for duplicating VMware VMs.
3. [function.sh](function.sh): Common bash functions for all subprojects.
4. [os_setup.md](os_setup.md): Guide to replicate development environment.
5. [readme.md](readme.md): This file.
6. [remove.sh](remove.sh): Remove installed components by [setup.sh](setup.sh) besides ones using apt.
7. [setup.sh](setup.sh): Install mutual dependencies and invoke corresponding installation scripts in other subdirectories.

## Demo Video

| No. | IDate-Time          | Commit                                                                                                                                                      | Detail                                               | Demo Video                     |
| --- | ------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------- | ------------------------------ |
| 1   | 2023/12/08-18:07:19 | [3a1293e33b725dbe6380b257f01aab1899bf61e0](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/3a1293e33b725dbe6380b257f01aab1899bf61e0/common) | Can install and execute ```../socket_com/setup.sh``` | <https://youtu.be/ZcaLBuhwKuw> |
