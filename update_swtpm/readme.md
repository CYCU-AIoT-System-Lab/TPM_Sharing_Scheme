# Update SWTPM

This is a system-wide error-exit installer of up-to-date SWTPM and its dependencies based on oldest project supported by this project. (Ubuntu 18.04 and jetson-nano-jp461) In total, its going to compile around 40 packages from source and install them system-wide. It is recommended to install on new OS or VM so that you can test its impact to your OS.

## Dependencies

This submodule requires the following submodules to be first executed:
- `common`
- `setup_optiga`
- `setup_ibmtpm`

The dependency investigation record is kept in `./dependency.md`.

To find newer release or information, check `./function_swtpm.sh` starting at line 80 for links to their official distribution site.

## Install

To install this submodule: 
1. Go to `/common`
2. Change `/common/config.ini` option `job_update_swtpm=1`
3. Run `./setup.sh` in `/common`

During installation, if anything caused installation to fail, you can disable those successful packages in `./config.ini` and re-run `./setup.sh` in `/common` to perform partial execution.

## Structure

- [./config.ini](./config.ini): Configuration file for this submodule. Use this file to enable or disable certain packages to be installed or removed.
- [./dependency.md](./dependency.md): Dependency investigation record.
- [./function_swtpm.sh](./function_swtpm.sh): Common functions for this submodule.
- [./readme.md](./readme.md): This file.
- [./remove_swtpm_isolated.sh](./remove_swtpm_isolated.sh): Based on selected configuration in `./config.ini` to remove certain packages in ISOLATED environment.
- [./remove_swtpm_systemwide.sh](./remove_swtpm_systemwide.sh): Based on selected configuration in `./config.ini` to remove certain packages in SYSTEM-WIDE environment, not recommended due to its system-wide nature.
- [./setup_swtpm_isolated.sh](./setup_swtpm_isolated.sh): Based on selected configuration in `./config.ini` to install certain packages in ISOLATED environment.
- [./setup_swtpm_systemwide.sh](./setup_swtpm_systemwide.sh): Based on selected configuration in `./config.ini` to install certain packages in SYSTEM-WIDE environment, not recommended due to its system-wide nature.

Note: 
1. Isolated install: Install packages in custom directory without affecting package manager or core system.
2. System-wide install: Install packages in system-wide directory `/usr`, some widely used packages (`openssl`, `glib`) may cause system-wide impact.
3. Isolated setup script is paired with isolated remove script, vice versa.
4. Isolated scripts are used in this project to prevent system-wide impact.
5. System-wide setup scripts work fine, but break other parts of this project due to shared resources (.so) overwriting.

## Usage Links

1. [https://github.com/stefanberger/swtpm/wiki](https://github.com/stefanberger/swtpm/wiki)
2. [https://github.com/stefanberger/swtpm/wiki/Using-the-IBM-TSS-with-swtpm](https://github.com/stefanberger/swtpm/wiki/Using-the-IBM-TSS-with-swtpm)
3. [https://github.com/stefanberger/swtpm/wiki/Using-the-Intel-TSS-with-swtpm](https://github.com/stefanberger/swtpm/wiki/Using-the-Intel-TSS-with-swtpm)

## Demo

| No. | Date       | Commit                                                                                                                                                 | Detail                                | Demo Video URL                                               |
| -   | -          | -                                                                                                                                                      | -                                     | -                                                            |
| 1   | 2024/06/26 | [dedda5cb0fcbb2a74c7aa4ad35bbd87a4f0b5d00](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/dedda5cb0fcbb2a74c7aa4ad35bbd87a4f0b5d00)   | Full installation on fresh new VM OS. | [https://youtu.be/fnxFj-HgCnU](https://youtu.be/fnxFj-HgCnU) |
