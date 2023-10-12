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
status=$?
status=$(bash ./installGitHubDesktop.sh)
echo $status
if [ $status -eq 0 ]; then
    echo "GitHub Desktop installed successfully!"
else
    echo "GitHub Desktop installation failed!"
fi

# Launch Github Desktop
github .
