# IBM TPM

Scripts here is tested on Ubuntu 18.04 VM.

Currently, TPM2.0 only mode can't support ACS.

## Installation Steps

1. In VM terminal, type ```ssh-keygen```, keep pressing ```enter``` till command is finished.
2. In VM terminal, type ```cat /home/user/.ssh/id_rsa.pub```, copy all of the output strings.
3. In browser, go to <https://github.com/settings/ssh/new>, type the name of this key in title, and paste copied RSA public key in.
4. In VM terminal, copy the following commands and paste it in your terminal.

### Installation Steps - Normal Install

```shell
sudo apt-get install git
cd ~
git clone git@github.com:CYCU-AIoT-System-Lab/TPM_Sharing_Scheme.git
cd ~/TPM_Sharing_Scheme
git pull
git checkout setup_ibmtpm
cd ./setup_ibmtpm
sudo bash +x setup.sh
# exit
```

### Installation Steps - with Environment Variable kept in current terminal

```shell
sudo apt-get install git
cd ~
git clone git@github.com:CYCU-AIoT-System-Lab/TPM_Sharing_Scheme.git
cd ~/TPM_Sharing_Scheme
git pull
git checkout setup_ibmtpm
cd ./setup_ibmtpm
sudo -s # use source to run script
source setup.sh # keep environmental variables
# exit
```

## Run some of the functionality of setup.sh

In VM terminal, adjust job settings, type ```sudo bash setup.sh```

## Installing Tools

1. Software stack: [IBMTSS](https://github.com/kgoldman/ibmtss)
2. Emulated TPM: [SWTPM](https://github.com/stefanberger/swtpm)
3. Demo example: [IBMACS](https://github.com/kgoldman/acs)

## Related Sources

1. [libtss packages in ubuntu](https://packages.ubuntu.com/search?keywords=libtss&searchon=names)

## Possible Bugs

1. Database initialize setting variable error.
2. ACS demo setting variable error.
3. ACS demo verification setting variable error.
4. force_acs_sql_setting might need to be 1.
