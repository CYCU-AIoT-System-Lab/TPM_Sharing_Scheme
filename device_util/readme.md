# Device Utilities

## Use Linux perf ([./flame_graph.sh](./flame_graph.sh))

1. Download `perf` from Linux kernel source code and install with instruction [here](https://askubuntu.com/a/753796).
2. Download [./stackcollapse-perf.pl](https://raw.githubusercontent.com/brendangregg/FlameGraph/master/stackcollapse-perf.pl) and [./flamegraph.pl](https://github.com/brendangregg/FlameGraph/blob/master/flamegraph.pl) in the same directory as this script.
3. Change `monitoring_PID` to the PID of the process you want to monitor in [./flame_graph.sh](./flame_graph.sh).
4. Change `monitoring_TIME` to the seconds you want to monitor in [./flame_graph.sh](./flame_graph.sh).
5. Run `bash flame_graph.sh` to generate SVG file.
6. Open the SVG file with a browser to see the flame graph.

## Structure

- [./upgrade_jetson_18](./upgrade_jetson_18): Upgrade Jetpack Ubuntu 18.04 to 20.04.
- [./pwm_fan_jetson.sh](./pwm_fan_jetson.sh): Control Jetson Nano PWM fan.
- [./ups_setup_jetson.sh](./ups_setup_jetson.sh): Setup waveshare Jetson Nano UPS display monitor, for JetPack Ubuntu 18.04.
