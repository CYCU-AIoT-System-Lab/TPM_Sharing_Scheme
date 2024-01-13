# TPM_Sharing_Scheme
 
Refer to [```./common```](./common/) for overall installation script.

## Repo Structure

1. [common](common): Common utilities accross different subprojects.
2. [doc](doc): Documentation used in this project.
3. [install_log](install_log): Log file during installation for debugging purpose.
4. [setup_environment](setup_environment): Script used to install tpm2-tss, tpm2-tools, tpm2-tss-engine, tpm2-abrmd, and IBM SWTPM. SOON TO BE DEPRECATED.
5. [setup_ibmtpm](setup_ibmtpm): Script used to install IBM TSS, SWTPM, ACS, and demonstration.
6. [setup_optiga](setup_optiga): Script used to install Infineon OPTIGA TPM 2.0 Software Stack (tpm2-tss, tpm2-tools, tpm2-abrmd, tpm2-tss-engine).
7. [socket_com](socket_com): Implement socket communication between server and client.
8. [.gitignore](.gitignore): Files / Directories to ignore for this repo.
9. [README.md](README.md): This file.

## Pi Software Version

1. gcc: gcc (Raspbian 10.2.1-6+rpi1) 10.2.1 20210110
2. make: GNU Make 4.3
3. cmake (```sudo apt-get install cmake```): cmake version 3.18.4

