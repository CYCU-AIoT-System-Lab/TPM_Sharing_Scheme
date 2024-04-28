# Jetson Nano Hardware

The jetson nano we use also equipped with a PWM fan and a UPS power module, this is the guide on how to use them.

## PWM Fan

After connecting the fan header to the board, you can control the fan with following command:

```bash
sudo sh -c 'echo 255 > /sys/devices/pwm-fan/target_pwm'
```
The above command will set the fan speed to 100% ($255\div255$). Changing `255` to other values will change the fan speed accordingly.

## UPS Power Module

After connecting the UPS power module to the board, and make sure the jumper close to round power connector is connected, use the following command to install UPS information displaying software.

```bash
sudo apt-get install python3-smbus -y
sudo apt-get install python-smbus -y
git clone https://github.com/waveshare/UPS-Power-Module
cd UPS-Power-Module
sudo ./install.sh
```

## References

1. [https://www.waveshare.com/wiki/UPS_Power_Module](https://www.waveshare.com/wiki/UPS_Power_Module)
2. [https://blog.cavedu.com/2019/10/04/nvidia-jetson-nano-fan/](https://blog.cavedu.com/2019/10/04/nvidia-jetson-nano-fan/)
