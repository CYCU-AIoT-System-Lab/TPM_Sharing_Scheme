# OS TTY Interface

Why is this useful? It allows the OS to not render any GUI, and instead just render text. This can be extremely helpful for computation power limited embedded systems.

To use the full CPU power, remember to logout the TTY for GUI and switch to text only ones.

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
        fbgrab -c 3 -i $HOME/Pictures/cap1/$count.png; \
        count=$((count+1)); \
        sleep 0.5; \
    done
    ```
    Experiment shown that `PNG` format can cause USB drive unable to unmount and kernel crash when CPU underload. However, `fbgrab` doesn't have the option to save in `PPM` format. Therefore, the above script uses `PNG` format.

## References

1. [https://github.com/jwilk/fbcat](https://github.com/jwilk/fbcat)
