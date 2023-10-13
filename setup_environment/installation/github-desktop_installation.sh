# =====================================================================================================
# The following code is modified from the following source:
# 1. https://github.com/kontr0x/github-desktop-install
# =====================================================================================================
# Modified by: Dachuan Chen
# Date: 2023/10/12
# Removed all "sudo" commands.
# Nevigate to your desired directory and run this script with "sudo ./github-desktop_installation.sh"
# =====================================================================================================
# Issue:
# =====================================================================================================
#!/bin/bash

# Acquire the script
wget https://raw.githubusercontent.com/kontr0x/github-desktop-install/main/installGitHubDesktop.sh
# --------------------
bash ./installGitHubDesktop.sh

# Launch Github Desktop (execute this command in the terminal)
github .
