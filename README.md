# Espism

This repo contins schematics and code to build a low cost ISM radio to wifi gateway.

Look at [hardware/v2](https://github.com/kanflo/espism/tree/master/hardware/v2) for the hardware and [verification](https://github.com/kanflo/espism/tree/master/verification) for the software used to do hardware verification. The directory [mqtt_sniffer](https://github.com/kanflo/espism/tree/master/mqtt_sniffer) contains an ISM to MQTT sniffer. Blog post [is here](http://johan.kanflo.com/bridging-ism-radio-and-wifi-for-lunch-money/).

#### Building

```
git clone --recursive https://github.com/SuperHouse/esp-open-rtos.git
vi esp-open-rtos/include/ssid_config.h # Fix wifi credentials
git clone https://github.com/kanflo/eor-spi.git esp-open-rtos/extras/spi
git clone https://github.com/kanflo/eor-rfm69 esp-open-rtos/extras/rfm69
export EOR_ROOT=$PWD/esp-open-rtos
cd mqtt_sniffer
make && make flash
```
