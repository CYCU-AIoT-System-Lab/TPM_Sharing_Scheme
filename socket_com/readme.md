# Server Communication

## Functionality

| No. | Item                                         | Functionalities                                                                                                                  |
| --- | -------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| 1.  | [```./setup.sh```](./setup.sh)               | Install requrements, clean, build, install, run, Address Sanitizer, Valgrind, GDB, generate documentation, host doc, browse doc. |
| 2.  | [```./server/server.c```](./server/server.c) | Print out message.                                                                                                               |
| 3.  | [```./client/client.c```](./client/client.c) | Print out message.                                                                                                               |

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
3. <https://stackoverflow.com/questions/14219092/bash-script-bin-bashm-bad-interpreter-no-such-file-or-directory>
4. <https://stackoverflow.com/questions/11783932/how-do-i-add-a-linker-or-compile-flag-in-a-cmake-file>

## Demo

| No. |      Date-Time      |                                                                             Commit                                                                              | Detail                                                              |           Demo Video           |
| :-: | :-----------------: | :-------------------------------------------------------------------------------------------------------------------------------------------------------------: | ------------------------------------------------------------------- | :----------------------------: |
|  1  | 2023/12/07-19:52:02 | [33d8ed85d8d6204850d2a6763a7d6ebddee37fc3](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/33d8ed85d8d6204850d2a6763a7d6ebddee37fc3/socket_com) | Script add func: install req, clean, build, install, run.           | <https://youtu.be/8Zay1Opzdgk> |
|  2  | 2023/12/07-21:00:59 | [0ee2072329c0306d6dfa493f87cd4deb5bc3e8c2](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/0ee2072329c0306d6dfa493f87cd4deb5bc3e8c2/socket_com) | Script adjust launch order.                                         | <https://youtu.be/zTvb3QRjuMI> |
|  3  | 2023/12/07-23:12:45 | [c02cc3e3f7f36cd3964232ff4268dca5f7b374ed](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/c02cc3e3f7f36cd3964232ff4268dca5f7b374ed/socket_com) | Script add func: doc, host, browse.                                 | <https://youtu.be/hjNEifu7EuQ> |
|  4  | 2023/12/09-18:48:29 | [d456f29fc1effecaad1a8032d08fbac8cff3d46d](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/d456f29fc1effecaad1a8032d08fbac8cff3d46d/socket_com) | Script add func: mem leak check.                                    | <https://youtu.be/mfSQhBRE4bI> |
|  5  | 2023/12/12-16:14:44 | [11e996d702a2497bac8cdee19f467dfe3e3b5180](https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/11e996d702a2497bac8cdee19f467dfe3e3b5180/socket_com) | Script add func: options for program checking, ASAN, Valgrind, GDB. | <https://youtu.be/Vd62zc_kahs> |

## ToDo

1. Finish socket implementation.
