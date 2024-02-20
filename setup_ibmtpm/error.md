# Error Fixing

## 1. TSS 000b0008: getTpmVendor: TPM2_GetCapability failed

1. Configure ```/common/config.ini``` to only execute ```setup_ibmtpm``` (disable update src)
2. Reboot, and run ```setup_ibmtpm``` again
