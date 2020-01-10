# Espism

This repo contins schematics and code to build a low cost ISM radio to wifi gateway.

Look at [hardware/v2](https://github.com/kanflo/espism/tree/master/hardware/v2) for the hardware and [verification](https://github.com/kanflo/espism/tree/master/verification) for the software used to do hardware verification. The directory [mqtt_sniffer](https://github.com/kanflo/espism/tree/master/mqtt_sniffer) contains an ISM to MQTT sniffer. Blog post [is here](http://johan.kanflo.com/bridging-ism-radio-and-wifi-for-lunch-money/).

#### Building

```
git clone https://github.com/kanflo/espism.git
cd espism
git submodule init && git submodule update
cd mqtt_sniffer
vi esp-open-rtos/include/private_ssid_config.h # Fix wifi credentials
make && make flash
```

Wifi credentials, in case you did not know, are written to `esp-open-rtos/include/private_ssid_config.h` as 

```
#define WIFI_SSID "your ssid"
#define WIFI_PASS "your key"
```
