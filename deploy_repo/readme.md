# Deploy Repository

This sub-module is responsible for deploying [mmWAVE_Radar](https://github.com/CYCU-AIoT-System-Lab/mmWAVE_Radar) repository from GitHub. Currently only platform 5 (Jetson Nano) is tested and supported.

## Structure

| File                                               | Description                                                            |
| ----                                               | -----------                                                            |
| [.gitignore](.gitignore)                           | Git commit ignore/filter rules.                                        |
| [config.ini](config.ini)                           | Configurations for necessary parameters.                               |
| [function_deploy_repo.sh](function_deploy_repo.sh) | Loading public functions, check existance of parameters.               |
| [readme.md](readme.md)                             | This file.                                                             |
| [remove.sh](remove.sh)                             | Execute remove.sh came with the repository.                            |
| [setup.sh](setup.sh)                               | Clone the repository and execute setup.sh came with the repository.    |
