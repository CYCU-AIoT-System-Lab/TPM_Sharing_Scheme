# Error Fixing

## 1. TSS 000b0008: getTpmVendor: TPM2_GetCapability failed

1. Configure ```/common/config.ini``` to only execute ```setup_optiga``` (disable update src)
2. Execute ```/common/remove.sh```
3. Reboot
4. Execute ```/common/setup.sh```
5. Execute ```tpm2_clear -c p```
6. Execute ```/common/remove.sh```
7. Reboot
8. Execute ```/common/setup.sh```
