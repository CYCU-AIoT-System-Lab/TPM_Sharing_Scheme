# Server Communication

## Demo

1. TPM Sharing Scheme 2023/12/07-19:52:02 Commit: 33d8ed85d8d6204850d2a6763a7d6ebddee37fc3 <https://youtu.be/8Zay1Opzdgk>
2. TPM Sharing Scheme 2023/12/07-21:00:59 Commit: 0ee2072329c0306d6dfa493f87cd4deb5bc3e8c2 <https://youtu.be/zTvb3QRjuMI>

## Dev Deploy Command

```
git stash && git stash clear && git pull && chmod +x setup.sh && ./setup.sh
```

## Directory Info

| No. | Item                                       | Description                                                        |
| --- | ------------------------------------------ | ------------------------------------------------------------------ |
| 1.  | ```./bin```                                | Contain generated executable, after ```./setup.sh``` is executed.  |
| 2.  | ```./build```                              | Contain generated cmake files, after ```./setup.sh``` is executed. |
| 3.  | [```./client```](./client/)                | Contain client side code.                                          |
| 4.  | [```./server```](./server/)                | Contain server side code.                                          |
| 5.  | [```./CMakeLists.txt```](./CMakeLists.txt) | Project overall cmake file.                                        |
| 6.  | [```./config.ini```](./config.ini)         | Project settings.                                                  |
| 7.  | [```./readme.md```](./readme.md)           | This file.                                                         |
| 8.  | [```./setup.sh```](./setup.sh)             | Installation and setup script.                                     |

## References

1. <https://stackoverflow.com/questions/6318809/how-do-i-grab-an-ini-value-within-a-shell-script>
2. <https://devicetests.com/change-gnome-terminal-title-command-line>

## ToDo

1. Make newly created terminal active.
2. Reverse the order of client and server installation.
