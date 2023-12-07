# Server Communication

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
