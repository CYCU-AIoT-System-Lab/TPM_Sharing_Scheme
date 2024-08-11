# SD Card Fix (Windows)

Be really careful when using Diskpart; it is extremely easy to lose an entire disk of data.  
Always backup your data on not-connected drive/machine.  
Disk Manager is also a good place to check whether SD card is detected by Windows.  

:bangbang:Proceed with caution:bangbang: Make sure selected drive is correct!

## General Approach

Use image writing tools like [Raspberry Pi Imager](https://www.raspberrypi.com/software/) or [balenaEtcher](https://etcher.balena.io/) to write a image to your SD card, they can sometimes recovers.

## SD card can't be recognized after loaded with Jetson Nano Image

1. Use [SD Card Formatter](https://www.sdcard.org/downloads/formatter/) to format with default settings.
2. Right-click on the disk (Windows File Explorer) to format to NTFS format with default settings.
3. Use [SD Card Formatter](https://www.sdcard.org/downloads/formatter/) to format with default settings.

Edge cases:

1. https://www.minitool.com/news/fix-disk-signature-collision-problem.html
2. https://learn.microsoft.com/en-us/windows-server/administration/windows-commands/uniqueid

## Out of Method Method

Sometimes, when SD card flashing is interrupted, Windows File Explorer will show that it requires formatting but can't be formatted after you click it. The following steps will fix this issue: (If the optimization needed notification appears, click ```cancel```)

1. Open Windows Terminal (Command Prompt, PowerShell, etc.).
2. Type ```diskpart``` and press Enter.
3. In the new window, type ```list disk``` and press Enter.
4. Type ```select disk #``` and press Enter. Replace ```#``` with the number of your SD card.
5. Type ```clean``` and press Enter.
6. Type ```create partition primary``` and press Enter.
7. Type ```select partition 1``` and press Enter.
8. Type ```active``` and press Enter.
9. Type ```format fs=fat32``` or ```format fs=fat32 quick``` and press Enter. (wait for it to finish, and you should see it appear in File Explorer)

Original sources:

1. [Format SD Card (Microsoft Community)](https://answers.microsoft.com/en-us/windows/forum/all/format-sd-card/8177d725-b12d-4d1c-a799-efcb3df3c53f)
2. [How to Install Ubuntu on MicroSD (ask Ubuntu)](https://askubuntu.com/questions/1126409/how-to-install-ubuntu-on-microsd)

## Nothing Works

Sometimes, following any steps online just creates more errors. The solution to this: RESTART YOUR COMPUTER (without fast startup)
