# SD Card Fix

Sometimes when SD cards flashing is interrupted, Windows File Explorer will shows that it requires formatting, but can't be formatted after you click it. The following steps will fix this issue: (If the optimization needed notification appeared, click ```cancel```)

1. Open Windows Terminal (Command Prompt, PowerShell, etc.).
2. Type ```diskpart``` and press Enter.
3. In the new window, type ```list disk``` and press Enter.
4. Type ```select disk #``` and press Enter. Replace ```#``` with the number of your SD card.
5. Type ```clean``` and press Enter.
6. Type ```create partition primary``` and press Enter.
7. Type ```select partition 1``` and press Enter.
8. Type ```active``` and press Enter.
9. Type ```format fs=fat32``` and press Enter. (wait for it to finish, and you should see it appear in File Explorer)

Original sources:

1. [Format SD Card (Microsoft Community)](https://answers.microsoft.com/en-us/windows/forum/all/format-sd-card/8177d725-b12d-4d1c-a799-efcb3df3c53f)
2. [How to Install Ubuntu on MicroSD (ask Ubuntu)](https://askubuntu.com/questions/1126409/how-to-install-ubuntu-on-microsd)
