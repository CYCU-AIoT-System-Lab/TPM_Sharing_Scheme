#!/bin/bash

# Don't reboot when asked
sudo do-release-upgrade -f DistUpgradeViewNonInteractive || { echo -e "\ndo-release-upgrade failed, likely due to time out"; exit 1; }

# Should automatically reboot
