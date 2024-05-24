# OS TTY Interface

Why is this useful? It allows the OS to not render any GUI, and instead just render text. This can be extremely helpful for computation power limited embedded systems.

## TTY Screenshot Program

1. Install:
    ```bash
    git clone https://github.com/jwilk/fbcat.git
    cd fbcat
    make
    sudo make install
    ```
2. Screenshot to PNG format:
    ```bash
    # Take a screenshot of TTY 3 and save it to screenshot.png
    fbgrab -i -c 3 screenshot.png
    ```
3. Screenshot to PPM format:
    ```bash
    # Take a screenshot of current TTY and save it to screenshot.ppm
    fbcat > screenshot.ppm
    ```
    ```bash
    # Take a screenshot of TTY 3 and save it to screenshot.ppm
    fbgrab -c 3 screenshot.ppm
    ```
4. Perform psuedo-screen-capture:
    ```bash
    count=1 \
    while true; do \
        fbgrab -c 3 $HOME/Pictures/cap1/$count.ppm; \
        count=$((count+1)); \
    done
    ```
    It is recommended to use `PPM` format for this instead of `PNG` format, as `PPM` format is faster to write to disk, and experiment shwon that `PNG` format can cause USB drive unable to unmount and kernel crash when CPU underload.

## References

1. [https://github.com/jwilk/fbcat](https://github.com/jwilk/fbcat)
