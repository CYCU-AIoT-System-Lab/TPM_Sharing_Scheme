# OS Setup

Following is a guide to setup experiment environment on different OS.

## Ubuntu 18.04 VM

Ubuntu VMs are powered with VMware Workstation 17 Player and the following hardware configuration:

1. Memory: 16 GB / 16384 MB
2. Processors: 6
3. Hard Disk: 200 GB
4. CD/DVD: Default
5. Network Adapter: Default
6. USB Controller: Default
7. Sound Card: Default
8. Display: Default

Steps:

1. Download the Ubuntu 18.04.06 LTS (Bionic Beaver) Desktop image from [ubuntu releases](https://releases.ubuntu.com/18.04/).
2. Open VMware, click ```Home``` tab, then click ```Create a New Virtual Machine```.
3. Select ```Installer disc image file (iso)```, browser to the downloaded ISO file in step 1, then click ```Next```.
4. Type in ```Full name```, ```User name```, and ```Password```, then click ```Next```.
5. Give the VM a name and specify a location to store the VM, then click ```Next```.
6. In ```Maximum disk size (GB)```, specify a size for it. Minimum 20 GB is recommended. Select ```Split virtual disk into multiple files```, then click ```Next```.
7. Click ```Customize Hardware...```, change the settings for ```Memory``` and ```Processors```, click ```Close```, then click ```Finish```.
8. Go through the ubuntu installation process.
9. Open ```Terminal```, type ```sudo apt-get update; sudo apt-get upgrade -y; sudo reboot```, then press ```Enter```.

## Raspbian Bullseye 2022-07-01 5.1 Kernel Debian i386 VM

Ubuntu VMs are powered with VMware Workstation 17 Player and the following hardware configuration: (larger number makes the VM lag a lot)

1. Memory: 8 GB / 8192 MB
2. Processors: 4
3. Hard Disk: 64 GB
4. CD/DVD: Default
5. Network Adapter: Default
6. USB Controller: Default
7. Sound Card: Default
8. Display: Default

Opening new tab in terminal will cause the VM to freeze.

Refer to following link for original sources:

1. [How to Install Raspberry Pi OS on VMware Workstation](https://youtu.be/85ZJj00FKe0?si=dvpXE4e0d_rYubMP)
2. [Install Raspberry Pi Desktop on your PC or Mac](https://projects.raspberrypi.org/en/projects/install-raspberry-pi-desktop/1)

Steps:

1. Download the only available ISO for Raspberry Pi OS from [Raspberry Pi Desktop for PC and Mac](https://www.raspberrypi.com/software/raspberry-pi-desktop/).
2. Open VMware, click ```Home``` tab, then click ```Create a New Virtual Machine```.
3. Select ```Installer disc image file (iso)```, browser to the downloaded ISO file in step 1, then click ```Next```.
4. In ```Guest operating system```, select ```Linux```; in ```Version```, select ```Other Linux 5.x kernel```, then click ```Next```.
5. Give the VM a name and specify a location to store the VM, then click ```Next```.
6. In ```Maximum disk size (GB)```, specify a size for it. Minimum 20 GB is recommended. Select ```Split virtual disk into multiple files```, then click ```Next```.
7. Click ```Customize Hardware...```, change the settings for ```Memory``` and ```Processors```, click ```Close```, then click ```Finish```.
8. Wait till a blue background with a white box in the middle appears, titled ```Debian GNU/Linux menu (BIOS mode)```, select ```Graphical install```, then press ```Enter```.
9. In ```Configure the keyboard``` tab, select ```American English```, then click ```Continue```.
10. In ```Partition disks``` tab, select ```Guided - use entire disk```, then click ```Continue```.
11. In ```Partition disks``` tab, select ```SCSI3 (0,0,0) (sda) - XXX.X GB VMware, VMware Virtual S``` or the only available option, then click ```Continue```.
12. In ```Partition disks``` tab, select ```All files in one partition (recommended for new users)```, then click ```Continue```.
13. In ```Partition disks``` tab, select ```Finish partitioning and write changes to disk```, then click ```Continue```.
14. In ```Partition disks``` tab, select ```Yes```, then click ```Continue```.
15. In ```Install the GRUB boot loader on a hard disk```, select ```Yes```, then click ```Continue```.
16. In ```Install the GRUB boot loader on a hard disk```, select ```/dev/sda```, then click ```Continue```.
17. In ```Finish the installation```, select ```Continue```, then wait till the installation is finished.
18. After the VM reboots, select ```Debian GNU/Linux``` in ```GNU GRUB version 2.02``` tab, then press ```Enter```.
19. In ```Welcome to Raspberry Pi``` tab, select ```Next```.
20. (varies) In ```Select your country```, select ```Taiwan``` for Country, random for Language, default for Timezone, tick ```Use English language``` and tick ```Use US keyboard```, then click ```Next```.
21. (varies) Set username and password, then click ```Next```.
22. In ```Update Software``` tab, select ```Skip```.
23. In ```Setup Complete``` tab, select ```Done```.
24. In VMware top menu, click ```Player```, click ```Manage```, and click ```Install VMware Tools...```.
25. In VM, open ```File Manager``` on top left corner, find ```VMware Tools```, copy everything in it to disk, ```${HOME}/Documents/VMwareTools```.
26. In VM, open ```File Manager``` and find ```VMwareTools```, right click on it, select ```Extract To```, and click ```Extract```.
27. In VM, open ```Terminal``` on top left corner, type ```cd /tmp/vmware-tools-distrib```, then press ```Enter```.
28. In VM, type ```sudo ./vmware-install.pl```, then press ```Enter```; on the first prompt, type ```yes```, then press ```Enter``` for all the following prompts.
29. In VM, type ```sudo raspi-config```, then press ```Enter```; select ```3 Interface Options```, then press ```Enter```; select ```I2 SSH Enable/Disable remote command line access to your Pi using SSH```, then press ```Enter```; select ```Yes```, then press ```Enter```; select ```Finish```, then press ```Enter```.
30. In VM, type ```sudo apt-get update; sudo apt-get upgrade -y```, then press ```Enter```.
31. In VM, type ```sudo reboot```, then press ```Enter```.

## Raspbian Bullseye 2023-05-03 6.x Kernal Debian arm64 on Raspbery Pi 4 B

Steps:

1. Download the ```2023-05-03-raspios-bullseye-arm64.img.xz``` file from Index of [/raspios_arm64/images/raspios_arm64-2023-05-03](http://downloads.raspberrypi.org/raspios_arm64/images/raspios_arm64-2023-05-03/).
2. Download the Raspberry Pi Imager from [Install Raspberry Pi OS using Raspberry Pi Imager](https://www.raspberrypi.com/software/), then install it.
3. Connect a microSD card reader with the microSD card inside to your PC.
4. Open Raspberry Pi Imager, 
    1. click ```CHOOSE DEVICE```, select ```Raspberry Pi 5```;
    2. click ```CHOOSE OS```, select ```Use custom```, browse to the downloaded IMG.XZ file in step 1, then click ```Open```;
    3. click ```CHOOSE STORAGE```, choose your microSD card; click ```NEXT```;
    4. click ```EDIT SETTINGS```,
        1. (optional) tick ```Set hostname```, type a hostname for your Raspberry Pi;
        2. Modify ```Username``` and ```Password```;
        3. Modify ```SSID``` and ```Password``` for ```Configure wireless LAN```;
        4. Go to ```SERVICES``` tab, tick ```Enable SSH```, and ```User password authentication```;
        5. Click ```SAVE```.
    5. click ```YES``` and wait till the writing and verification is finished.
5. Eject the microSD card from your PC, then insert it into your Raspberry Pi.
6. Connect your Raspberry Pi to a monitor (mini HDMI), keyboard, mouse, and lastly power supply (USB-C).
    1. If you have no monitor, but android phone, install [IP Tools: WiFi Analyzer](https://play.google.com/store/apps/details?id=com.ddm.iptools&hl=en&gl=US), and scane the IP address of your Raspberry Pi with address setting ```127.0.0.1```.
    2. If you have no monitor, but don't want to scan IP address,
        1. Connect your Raspberry Pi to a router you can access and can check the IP address of your Raspberry Pi.
        2. Connect your PC to the router, enable hotspot on your PC, and connect your Raspberry Pi to the hotspot. (modify ```SSID``` and ```Password``` in step 4.4.3)
    3. After getting the IP address, use tools like [VNC Viewer](https://www.realvnc.com/en/connect/download/viewer/), [TigerVNC](https://github.com/TigerVNC/tigervnc), [PuTTY](https://www.putty.org/), or SSH available on all machines to connect and operate the machine; [WinSCP](https://winscp.net/eng/download.php), or SCP available on all machines to copy files/directories from/to remote.
7. After desktop is shown on your screen, open ```Terminal``` on your Raspberry Pi, type ```sudo raspi-config```, and enable ```SSH``` and ```SPI``` in ```Interfacing Options```.
8. In ```Terminal```, type ```sudo apt-get update; sudo apt-get upgrade -y```, then press ```Enter```.
9. In ```Terminal```, type ```sudo reboot```, then press ```Enter```.

## Raspbian Bullseye 2023-05-03 6.x Kernal Debian arm64 on Raspbery Pi 4 B

Unfinished

## Jetson Nano

Unfinished

## Common Issue

1. Computer get super laggy, clunky, buggy suddenly after long usage even after restarting (Windows 11)
    1. Go to ```Control Panel```, navigate to ```Power Options/Choose what the power buttons do/Change settings that are currently unavailable```, untick ```Turn on fast startup (recommended)```, then click ```Save changes```.
2. VMware VMs get laggy or even freeze
    1. [How to Speed Up Your Virtual Machine](https://youtu.be/cEJyqI1R36A?si=wfoM1tjbjaLoLJZ_)
