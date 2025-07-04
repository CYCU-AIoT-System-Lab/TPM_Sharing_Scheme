# ACS Routine

This submodule include both server and client-side setup and execution script for Routine Remote Attestation.

Server-side:
1. Routine ACS DB parsing to find state change.
2. Detect anomaly client traffic.
3. If anomaly client traffic detected, block.

Client-side:
1. Routine client use IBMACS to send BIOS log + IMA log + PCR table to server DB for state recording.
2. Detect anomaly server traffic.
3. If anomaly server traffic detected, block.

Note: Client-side is forced to use updated SWTPM with socket interface

## Dependency

Server-side:
1. job_common + job_setup_optiga
2. job_setup_ibmtpm

Client-side:
1. job_common + job_setup_optiga
2. job_setup_ibmtpm + job_update_swtpm + job_boot + job_setup_IMA (un-developed)

## Structure

1. [./attestlog_AD/](./attestlog_AD/): Server-side DB parsing binary. Output attestation state and partial log.
2. [./m4/](./m4/): Directory to hold m4 macro files after installation start.
3. [./traffic_AD/](./traffic_AD/): Anomaly traffic detection binary. First interval will measure normal traffic, then compare with following interval.
4. [./bootstrap.sh](./bootstrap.sh): Script to compile binaries in this submodule.
5. [./config.ini](./config.ini): Configuration file for this submodule.
6. [./configure.ac](./configure.ac): Binary pre-compilation setting file.
7. [./function_acsroutine.sh](./function_acsroutine.sh): Function script for this submodule.
8. [./Makefile.am](./Makefile.am): Binary compilation setting file.
9. [./readme.md](./readme.md): This file.
10. [./remove_acsroutine.sh](./remove_acsroutine.sh): Script to remove system changes made by this submodule.
11. [./setup_acsroutine.sh](./setup_acsroutine.sh): Script to setup execution environment and execute this submodule. Performing tasks specified in `config.ini`.

## Debug Links

1. ERROR: JS_ObjectGetString: getting key: imaevent1083 / processImaEntries20Pass1: done, no event 1083
    - [https://rlyown.github.io/2021/02/28/IBM-Attestation-Client-Server%E6%B5%8B%E8%AF%95/#%E5%90%AF%E5%8A%A8%E8%AF%81%E5%AE%9E](https://rlyown.github.io/2021/02/28/IBM-Attestation-Client-Server%E6%B5%8B%E8%AF%95/#%E5%90%AF%E5%8A%A8%E8%AF%81%E5%AE%9E)
    - [https://sourceforge.net/p/ibmtpm20tss/discussion/general/thread/aa4a6044/](https://sourceforge.net/p/ibmtpm20tss/discussion/general/thread/aa4a6044/)
2. TPM_RC_INTEGRITY - integrity check failed Patameter number 1
    - [https://stackoverflow.com/questions/58029219/get-a-persistant-string-in-and-out-of-the-tpm2-module](https://stackoverflow.com/questions/58029219/get-a-persistant-string-in-and-out-of-the-tpm2-module)
    - [https://github.com/tpm2-software/tpm2-tools/issues/2024](https://github.com/tpm2-software/tpm2-tools/issues/2024)
    - [https://github.com/tpm2-software/tpm2-tools/issues/3222](https://github.com/tpm2-software/tpm2-tools/issues/3222)
    - [https://tpm2-tools.readthedocs.io/en/latest/man/tpm2_create.1/](https://tpm2-tools.readthedocs.io/en/latest/man/tpm2_create.1/)
    - [https://tpm2-tools.readthedocs.io/en/latest/man/tpm2_encryptdecrypt.1/](https://tpm2-tools.readthedocs.io/en/latest/man/tpm2_encryptdecrypt.1/)
    - [https://joholl.github.io/tpm2-tools/tutorial/2019/10/09/Tools-Tutorial.html](https://joholl.github.io/tpm2-tools/tutorial/2019/10/09/Tools-Tutorial.html)
    - [https://tpm2-tools.readthedocs.io/en/latest/man/tpm2_load.1/](https://tpm2-tools.readthedocs.io/en/latest/man/tpm2_load.1/)
    - Cause: Initial boot of swtpm used diffent temporary directory than the one currently used
3. Esys_ContextLoad(0x902) - tpm:warn(2.0): out of memory for object contexts
    - [https://github.com/tpm2-software/tpm2-tools/issues/2960](https://github.com/tpm2-software/tpm2-tools/issues/2960)
    - Use command `tpm2_flushcontext -t` to solve

## Debug Reproduce Step

Require `job_update_swtpm` to be finished.

1. remove server DB records
2. delete and send new version of following dirs:
    - common
    - setup_ibmtpm
    - boot
    - acs_routine
3. re-install setup_ibmtpm with following settings: (`config_RA_client_setup.ini`)
    - server_ip
    - client_ip
    - new_swtpm=1
4. install boot
5. run acs_routine (client)
    - this seems to always trigger 0x1df error
    - restart swtpm doesn't help
    - before `job_boot`, client launch `config_RA_client_launch.ini` setting works (with new swtpm)
    - once `job_boot` is done, launching `job_acsroutine` for client side will trigger 0x1df error, using `config_RA_client_launch.ini` doesn't work anymore
    - use `systemctl stop swtpm.service` to stop new swtpm daemon, using `config_RA_client_launch.ini` works again
    - if temporary directory between `job_setup_ibmtpm` and `job_acsroutine` of swtpm is the same, then it works
    - setting `swtpm.service` user as root doesn't help
    - removing temporary files in `/tmp` doesn't help
    - tpm2_createprimary -c primary.ctx
    - tpm2_flushcontext -t # this is to prevent 0x902 outofmemory error
    - tpm2_create -C primary.ctx -u obj.pub -r obj.priv
    - tpm2_flushcontext -t # this is to prevent 0x902 outofmemory error
    - tpm2_load -C primary.ctx -u obj.pub -r obj.priv -c obj.ctx
    - tpm2_flushcontext -t # this is to prevent 0x902 outofmemory error

## Demo

| No. | Date       | Commit                                                                                                                                               | Detail                                                                             | Demo Video URL                                               |
| -   | -          | -                                                                                                                                                    | -                                                                                  | -                                                            |
| 1   | 2024/07/02 | [6335baf45e8d67e2001c72ebcc00eea2d6928cc7](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/6335baf45e8d67e2001c72ebcc00eea2d6928cc7) | IBMACS client enroll and consistent remote attestation.                            | [https://youtu.be/UyiN50dHs7s](https://youtu.be/UyiN50dHs7s) |
| 2   | 2024/07/03 | [16923cbd5143850c7375b3acbfdb114c42fd837f](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/16923cbd5143850c7375b3acbfdb114c42fd837f) | DB AD agent captured successful attestation result.                                | [https://youtu.be/TxwI3Lx93Tc](https://youtu.be/TxwI3Lx93Tc) |
| 3   | 2024/07/03 | [16923cbd5143850c7375b3acbfdb114c42fd837f](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/16923cbd5143850c7375b3acbfdb114c42fd837f) | DB AD agent captured failed attestation result.                                    | [https://youtu.be/A1Qx6gCD8ew](https://youtu.be/A1Qx6gCD8ew) |
| 4   | 2024/07/06 | [022ce7ec66beea7a6a8da80c63fcc04c56a64491](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/022ce7ec66beea7a6a8da80c63fcc04c56a64491) | Traffic AD agent detect anomaly traffic.                                           | [https://youtu.be/Q55ym9uF30A](https://youtu.be/Q55ym9uF30A) |
| 5   | 2024/07/06 | [022ce7ec66beea7a6a8da80c63fcc04c56a64491](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/022ce7ec66beea7a6a8da80c63fcc04c56a64491) | Traffic AD agent received simulated signal.                                        | [https://youtu.be/-pSbQRL8ijk](https://youtu.be/-pSbQRL8ijk) |
| 6   | 2024/07/06 | [022ce7ec66beea7a6a8da80c63fcc04c56a64491](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/022ce7ec66beea7a6a8da80c63fcc04c56a64491) | ACS Routine detects attestation anomaly and isolates the client node.              | [https://youtu.be/--U3S5v_xgw](https://youtu.be/--U3S5v_xgw) |
| 7   | 2024/07/06 | [022ce7ec66beea7a6a8da80c63fcc04c56a64491](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/022ce7ec66beea7a6a8da80c63fcc04c56a64491) | ACS Routine detects traffic anomaly and isolates the client node.                  | [https://youtu.be/bCJfk8Z3E88](https://youtu.be/bCJfk8Z3E88) |

## Reference Links

1. [https://mariadb.com/kb/en/operating-system-error-codes/](https://mariadb.com/kb/en/operating-system-error-codes/)
2. [https://mariadb.com/kb/en/mariadb-connectorc-api-functions/](https://mariadb.com/kb/en/mariadb-connectorc-api-functions/)
3. [Database Connection in C++ - MySQL or MariaDB - Steve's teacher](https://youtu.be/cSZvq7Kv6_0?si=PYCB_hs1MV6GNnMt)
4. [How to Create/Drop User in MariaDB - MariaDB Admin Tutorial - TechBrothersIT](https://youtu.be/MI4590v1QoU?si=SHSjoOksfvoxqAwQ)
5. [https://github.com/kgoldman/acs/blob/master/acs/reports.php](https://github.com/kgoldman/acs/blob/master/acs/reports.php)
6. [https://github.com/kgoldman/acs/blob/master/acs/serversql.c](https://github.com/kgoldman/acs/blob/master/acs/serversql.c)
7. [https://stackoverflow.com/questions/1746501/can-someone-give-an-example-of-cosine-similarity-in-a-very-simple-graphical-wa](https://stackoverflow.com/questions/1746501/can-someone-give-an-example-of-cosine-similarity-in-a-very-simple-graphical-wa)
8. [https://cs.stackexchange.com/questions/72193/are-there-any-efficient-algorithems-to-find-packets-similarities-in-a-list](https://cs.stackexchange.com/questions/72193/are-there-any-efficient-algorithems-to-find-packets-similarities-in-a-list)
