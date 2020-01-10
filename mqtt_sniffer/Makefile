PROGRAM=mqtt_sniffer
EXTRA_COMPONENTS = extras/rboot-ota extras/paho_mqtt_c
OTA=1

# MQTT host config. Override with one, more or all of:
#  make MQTT_HOST=172.16.3.100 MQTT_PORT=123345 MQTT_USER=mrrobot MQTT_PASS=secret
MQTT_HOST ?= iot.eclipse.org
MQTT_PORT ?= 1883
MQTT_USER ?= NULL
MQTT_PASS ?= NULL
HW_REV    ?= 4

# You should change this ;)
RFM_KEY ?= sampleEncryptKey

# Things will go south without this, flags are not inherited by EOR
PROGRAM_CFLAGS = -Wall -Wl,-EL -nostdlib  -ffunction-sections -fdata-sections -g -O2 -std=gnu99

# Add our config
PROGRAM_CFLAGS += -DCONFIG_HW_REV=$(HW_REV) -DCONFIG_RFM_KEY=\"$(RFM_KEY)\"
PROGRAM_CFLAGS += -DCONFIG_MQTT_HOST=\"$(MQTT_HOST)\" -DCONFIG_MQTT_PORT=$(MQTT_PORT) 
ifneq ($(MQTT_USER), NULL)
  PROGRAM_CFLAGS += -DCONFIG_MQTT_USER=\"$(MQTT_USER)\"
endif
ifneq ($(MQTT_PASS), NULL)
  PROGRAM_CFLAGS += -DCONFIG_MQTT_PASS=\"$(MQTT_PASS)\"
endif

PROGRAM_INC_DIR = . ./rfm69
PROGRAM_SRC_DIR = . ./rfm69

include esp-open-rtos/common.mk
