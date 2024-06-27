# Update SWTPM

This is a system-wide error-exit installer of up-to-date SWTPM and its dependencies based on oldest project supported by this project. (Ubuntu 18.04 and jetson-nano-jp461) In total, its going to compile around 40 packages from source and install them system-wide. It is recommended to install on new OS or VM so that you can test its impact to your OS.

## Dependencies

This submodule requires the following submodules to be first executed:
- `common`
- `setup_optiga`
- `setup_ibmtpm`

The dependency investigation record is kept in `./dependency.md`.

To find newer release or information, check `./function_swtpm.sh` starting at line 75 for links to their official distribution site.

## Install

To install this submodule: 
1. Go to `/common`
2. Change `/common/config.ini` option `job_update_swtpm=1`
3. Run `./setup.sh` in `/common`

During installation, if anything caused installation to fail, you can disable those successful packages in `./config.ini` and re-run `./setup.sh` in `/common` to perform partial execution.

## Structure

- `./config.ini`: Configuration file for this submodule. Use this file to enable or disable certain packages to be installed or removed.
- `./dependency.md`: Dependency investigation record.
- `./function_swtpm.sh`: Common functions for this submodule.
- `./readme.md`: This file.
- `./remove_swtpm`: Based on selected configuration in `./config.ini` to remove certain packages.
- `./setup_swtpm`: Based on selected configuration in `./config.ini` to install certain packages.

## Demo

| No. | Date       | Commit                                                                                                                                                 | Detail                                | Demo Video URL                                               |
| -   | -          | -                                                                                                                                                      | -                                     | -                                                            |
| 1   | 2024/06/26 | [dedda5cb0fcbb2a74c7aa4ad35bbd87a4f0b5d00](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/dedda5cb0fcbb2a74c7aa4ad35bbd87a4f0b5d00)   | Full installation on fresh new VM OS. | [https://youtu.be/fnxFj-HgCnU](https://youtu.be/fnxFj-HgCnU) |
