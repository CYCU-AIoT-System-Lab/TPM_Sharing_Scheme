# TPM_Sharing_Scheme
 
Refer to [```./common```](./common/) for overall installation script.

## Repo Structure

| No. | Item                                         | Description                                                                                                         |
| --- | ---                                          | :--                                                                                                                 |
| 1   | [./acs_routine/](./acs_routine/)             | Multiple ACS Remote Attestation system demo scripts and binaries.                                                   |
| 1   | [./boot/](./boot/)                           | Boot phase of the project.                                                                                          |
| 2   | [./deploy_repo/](./deploy_repo/)             | Common utilities accross different subprojects.                                                                     |
| 3   | [./deploy_repo/](./deploy_repo/)             | Deploy demo purpose repository.                                                                                     |
| 4   | [./device_util/](./device_util/)             | Scripts related to hardware and OS configuration.                                                                   |
| 5   | [./doc/](./doc/)                             | Documentation used in this project.                                                                                 |
| 6   | [./install_log/](./install_log/)             | Log file during installation for debugging purpose.                                                                 |
| 7   | [./post_boot/](./post_boot/)                 | Post-boot phase of the project.                                                                                     |
| 8   | [./setup_environment/](./setup_environment/) | Scripts used to install tpm2-tss, tpm2-tools, tpm2-tss-engine, tpm2-abrmd, and IBM SWTPM. DEPRECATED.               |
| 9   | [./setup_ibmtpm/](./setup_ibmtpm/)           | Scripts used to install IBM TSS, SWTPM, ACS, and demonstration.                                                     |
| 10  | [./setup_optiga/](./setup_optiga/)           | Scripts used to install Infineon OPTIGA TPM 2.0 Software Stack (tpm2-tss, tpm2-tools, tpm2-abrmd, tpm2-tss-engine). |
| 11  | [./setup_ros2/](./setup_ros2/)               | Scripts to setup/build/remove ROS2 environment and setup/build/execute/remove demo purpose package.                 |
| 12  | [./socket_com/](./socket_com/)               | Implement socket communication between server and client.                                                           |
| 13  | [./update_swtpm/](./update_swtpm/)           | System-wide installation of new SWTPM.                                                                              |
| 13  | [./gitignore](./gitignore)                   | Files / Directories to ignore for this repo.                                                                        |
| 14  | [./README.md](./README.md)                   | This file.                                                                                                          |
