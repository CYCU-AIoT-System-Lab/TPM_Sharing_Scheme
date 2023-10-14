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

## Debug List

1. "tpm2tss.so" location.

## Feature List

1. TPM daemen starter.
2. TPM simulator & pysical chip daemen switcher.

## Useful Commands

1. ```journalctl -xe```
