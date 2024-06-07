# SSH Setup

Following is a guide to setup SSH server service on various platforms.

## Ubuntu 18.04 VM

Refer to following link for original sources:

1. [How to install ssh on Ubuntu Linux using apt-get](https://www.cyberciti.biz/faq/how-to-install-ssh-on-ubuntu-linux-using-apt-get/)

Steps:

1. Type ```sudo apt-get install openssh-server``` in terminal to install ssh service.
2. Type ```sudo systemctl enable ssh``` in terminal to enable ssh service.
3. Type ```sudo ufw allow ssh``` in terminal to allow SSH traffic through firewall.
4. Type ```sudo reboot``` in terminal to reboot the system.

## Raspbian Bullseye 2022-07-01 5.1 Kernel Debian i386 VM

Steps:

1. Type ```sudo raspi-config``` in terminal.
2. Navigate to ```Interfacing Options``` and press ```Enter```.
3. Navigate to ```SSH``` and press ```Enter```.
4. Select ```Yes``` and press ```Enter```.
5. Navigate to ```Finish``` and press ```Enter```.
6. Type ```sudo reboot``` in terminal and press ```Enter``` to reboot the system.

## Raspbian Bullseye 2023-05-03 6.x Kernal Debian arm64

Steps:

1. Type ```sudo raspi-config``` in terminal.
2. Navigate to ```Interfacing Options``` and press ```Enter```.
3. Navigate to ```SSH``` and press ```Enter```.
4. Select ```Yes``` and press ```Enter```.
5. Navigate to ```Finish``` and press ```Enter```.
6. Type ```sudo reboot``` in terminal and press ```Enter``` to reboot the system.

## Jetson Nano

Unfinished
