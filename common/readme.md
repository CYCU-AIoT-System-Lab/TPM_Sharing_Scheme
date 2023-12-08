# Common Utilities

This directory holdes common utilities for this project.

## Demo Video

| No. | IDate-Time          | Commit                                                                                                                                                      | Detail                                               | Demo Video                     |
| --- | ------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------- | ------------------------------ |
| 1   | 2023/12/08-18:07:19 | [3a1293e33b725dbe6380b257f01aab1899bf61e0](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/3a1293e33b725dbe6380b257f01aab1899bf61e0/common) | Can install and execute ```../socket_com/setup.sh``` | <https://youtu.be/ZcaLBuhwKuw> |

## Directory Structure

| No. | Item                                 | Description                                                 |
| --- | ------------------------------------ | ----------------------------------------------------------- |
| 1.  | [```./copy_VM.ps1```](./copy_VM.ps1) | Duplicate VM with minimum setup on Windows 11 PowerShell 7. |
| 2.  | [```./setup.sh```](./setup.sh)       | Initial environmental setup for all subprojects.            |

## Launch Setup Process in new VM

1. In VM terminal, type ```ssh-keygen```, keep pressing ```enter``` till command is finished.
2. In VM terminal, type ```cat ~/.ssh/id_rsa.pub```, copy all of the output string.
3. In browser, go to <https://github.com/settings/ssh/new>, type the name of this key in title, and paste copied RSA public key in.
4. In VM terminal, type ```sudo ls``` to acquire root privilege for this installation.
5. In VM terminal, copy the following code into a local bash file and execute it without privilege.

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
