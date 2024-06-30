# Boot Phase Securiy

Purpose: Add SWTPM PCR hashing to the boot chain.

## Limitation

Any path containing spaces will not be hashed correctly.

## Usage

1. `bash hash.sh -h` to find CLI arguments to have a file.
2. `bash test_hash.sh` to test the hashing process.
3. `bash setup_mbc_last.sh` to setup Measured Boot Chain (MBC) last stage.
4. `bash mbc_last.sh` to run MBC last stage.
5. `bash remove.sh` to remove MBC last stage from system.

## Structure

1. [./.gitignore](./.gitignore): Files to ignore for git.
2. [./activate_swtpm.sh](./activate_swtpm.sh): SWTPM activation script used by daemon at startup.
3. [./dir_list.txt](./dir_list.txt): List of directories and files to hash. Can be modified to your need.
4. [./function_boot.sh](./function_boot.sh): Common functions in this directory used by multiple scripts.
5. [./hash.sh](./hash.sh): Script to hash files in `dir_list.txt` with full CLI support to use as standalone script.
6. [./launch_swtpm.sh](./launch_swtpm.sh): Script to launch SWTPM used by daemon at startup.
7. [./mbc_last.sh](./mbc_last.sh): Script to run MBC last stage file integrity check and interact with HW/SW TPM PCR.
8. [./potential_cmd.md](./potential_cmd.md): Potential tpm2-tools commands to use during development.
9. [./readme.md](./readme.md): This file.
10. [./remove_mbc_last.sh](./remove_mbc_last.sh): Master script to perform submodule removal, including SWTPM daemon removal.
11. [./remove_swtpm_daemon.sh](./remove_swtpm_daemon.sh): Script to remove SWTPM daemon from system.
12. [./setup_mbc_last.sh](./setup_mbc_last.sh): Master script to perform submodule setup, including SWTPM daemon setup.
13. [./setup_swtpm_daemon.sh](./setup_swtpm_daemon.sh): Script to setup SWTPM daemon in system to launch at boot.
14. [./test_hash.sh](./test_hash.sh): Script to test hashing process consistency.

## Methodology

1. Add to boot chain: https://belongtothenight.github.io/HOW_TO/how_to_LINUX-auto_launch_program_on_boot/
2. Extend PCR: Add log entry when extending, to PCR x.

## Notes

1. Due to no tpm2-tools found to be able to perform TPM hashing operation without overwriting the PCR, the generated hash is stored in NVM for now. This is the reason requiring additional TPM configuration and NVM R/W operations.
2. SWTPM in current setting cannot keep PCR value persistent after reboot. Therefore each startup need to perform `setup_mbc_last.md` first so that `mbc_last.md` can work with initialized PCR table.
3. If you have used remove script in this uptime, you will need to restart the system to perform setup again.

## Reference

1. https://uapi-group.org/specifications/specs/linux_tpm_pcr_registry/
2. https://security.stackexchange.com/questions/258649/usage-of-tpm-pcrs-in-linux
3. https://trustedcomputinggroup.org/wp-content/uploads/TCG_PCClient_PFP_r1p05_v23_pub.pdf Section 3.3.4, Section 8.2
4. https://stackoverflow.com/questions/75916117/go-lang-tpm2-library-pcr-extend-pcr-read-inconsistency
5. https://stackoverflow.com/questions/36718019/journalctl-remove-logs-of-a-specific-unit
