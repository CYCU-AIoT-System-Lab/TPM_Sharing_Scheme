# Setup Environment

These scripts aim for setting up TPM development environment on VM without physical TPM attached.

Testing environment: Fresh installed ubuntu-20.04.6-desktop-amd64.iso in VM.

## Execute Steps

1. Download entire repo as zip. (~/Download/ is used for testing)
2. Go to your directory. (~/Download/)
3. ```unzip TPM_Sharing_Scheme-setup_environment.zip```
4. ```cp ./TPM_Sharing_Scheme-setup_environment/setup_environment/installation/*.sh ./```
5. ```sudo chmod +x setup_environment.sh```
6. ```sudo ./setup_environment.sh```
7. Reboot system with ```sudo reboot```

## Usage

1. Use SW TPM (For TPM simulator development)
   1. Activate
      1. Open terminal, ```./setup_environment/activate/activate_swtpm_simulator.sh```.
      2. Open new terminal, ```./setup_environment/operation/tpm2tss_swtpm_start.sh``` to start SW TPM.
   2. Reset
      1. ```./setup_environment/operation/tpm2tss_swtpm_reset.sh```
2. General Operation
   1. Kill tpm daemen
      1. ```./setup_environment/operation/tpm_daemen_killer.sh```
   2. Start tpm daemen
      1. ```./setup_environment/operation/tpm_daemen_starter.sh```

## Debug List

1. "tpm2tss.so" location.

## Feature List

1. TPM simulator & pysical chip daemen switcher.
2. Docker implementation.
3. New method to download code.
   1. Use ```ssh-genkey```, and copy the generated key content.
   2. Go to [GitHub Key Settings](https://github.com/settings/keys) to add new SSH key.
   3. Use ```git clone git@github.com:CYCU-AIoT-System-Lab/TPM_Sharing_Scheme.git```
4. Consider deploying [Gerrit](https://www.gerritcodereview.com/).

## Useful Commands

1. ```journalctl -xe```
