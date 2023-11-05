# IBM TPM

Scripts here is tested on Ubuntu 18.04 VM.

Currently, TPM2.0 only mode can't support ACS.

## Installation Steps

1. In VM terminal, type ```ssh-keygen```, keep pressing ```enter``` till command is finished.
2. In VM terminal, type ```cat /home/user/.ssh/id_rsa.pub```, copy all of the output strings.
3. In browser, go to <https://github.com/settings/ssh/new>, type the name of this key in title, and paste copied RSA public key in.
4. In VM terminal, install git with ```sudo apt-get install git```
5. In VM terminal, nevigate to your desire directory, and type ```git clone git@github.com:CYCU-AIoT-System-Lab/TPM_Sharing_Scheme.git```.
6. In VM terminal, nevigate inside cloned folder with ```cd ./TPM_Sharing_Scheme```.
7. In VM terminal, switch development branch with ```git checkout setup_ibmtpm```.
8. In VM terminal, nevigate to installation scripts with ```cd ./src/setup_ibmtpm```.
9. In VM, check the start of each scripts to see whether settings are correct.
10. In VM terminal, login as root with ```sudo -s```.
11. In VM terminal, install with ```source setup.sh```.

## Installing Tools

1. Software stack: [IBMTSS](https://github.com/kgoldman/ibmtss)
2. Emulated TPM: [SWTPM](https://github.com/stefanberger/swtpm)
3. Demo example: [IBMACS](https://github.com/kgoldman/acs)

## Related Sources

1. [libtss packages in ubuntu](https://packages.ubuntu.com/search?keywords=libtss&searchon=names)
