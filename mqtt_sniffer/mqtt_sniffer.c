/* 
 * The MIT License (MIT)
 * 
 * Copyright (c) 2016 Johan Kanflo (github.com/kanflo)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <esp8266.h>
#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <esp/spi.h>
#include <rfm69.h>
#include <ssid_config.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <timers.h>
#include <queue.h>
#include <ota-tftp.h>
#include <rboot-integration.h>
#include <rboot.h>
#include <rboot-api.h>
#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>

typedef enum {
    led_off = 0,
    led_flash_once,
    led_flash_double,
    led_flash_slow,
    led_flash_fast
} led_state_t;


#define HW_REV CONFIG_HW_REV

/** The various espism boards I have build */
#if HW_REV==1
 #define LED_GPIO      (15)
 #define RFM69_INT      (5)
 #define RFM69_CS       (4)
#elif HW_REV==2
 #define LED_GPIO       (0)
 #define RFM69_INT      (5)
 #define RFM69_CS       (4)
#elif HW_REV==3
 #define LED_GPIO      (15)
 #define RFM69_INT      (5)
 #define RFM69_CS       (4)
 #define RFM69_RESET   (16) // Only on v3
#elif HW_REV==4
 #define LED_GPIO      (15)
 #define RFM69_INT      (5)
 #define RFM69_CS       (4)
 #define RFM69_RESET   (16) // Only on v4
#else
 #error "Unsupported HW revision"
#endif


#define MAX_ISM_PKT_SIZE  (64)
#define PUB_QUEUE_DEPTH   (10)

#define MQTT_HOST CONFIG_MQTT_HOST
#define MQTT_PORT CONFIG_MQTT_PORT

#ifdef CONFIG_MQTT_USER
 #define MQTT_USER CONFIG_MQTT_USER
#else // CONFIG_MQTT_USER
 #define MQTT_USER NULL
#endif // CONFIG_MQTT_USER

#ifdef CONFIG_MQTT_PASS
 #define MQTT_PASS CONFIG_MQTT_PASS
#else // CONFIG_MQTT_PASS
 #define MQTT_PASS NULL
#endif // CONFIG_MQTT_PASS

#define RFM_KEY CONFIG_RFM_KEY

SemaphoreHandle_t wifi_alive;
SemaphoreHandle_t sniffer_alive;
QueueHandle_t publish_queue;

volatile led_state_t g_led_state = led_off;

#define delay_ms(ms) vTaskDelay(ms / portTICK_PERIOD_MS)

static const char* get_station_mac(void);
static const char* get_station_ip(void);

static void set_led(bool on)
{
    gpio_write(LED_GPIO, on?0:1);
}

static void set_led_state(led_state_t state)
{
    g_led_state = state;
}

static void heartbeat_task(void *param)
{
    (void) param;
    char msg[MAX_ISM_PKT_SIZE];
    while(1) {
        delay_ms(60000);
        printf("Publishing heartbeat\n");
#if 0
        /** Currently cannot do this as the entries in the queue are RFM
         *  packets and not arbitrary strings
         */
        snprintf(msg, sizeof(msg)-1, "heartbeat:%s", get_station_ip());
#else
        memset(msg, 0, sizeof(msg));
#endif
        if (xQueueSend(publish_queue, (void*) msg, 0) == pdFALSE) {
            printf("Publish queue overflow.\n");
        }
    }
}

static void led_task(void *pvParameters)
{
    while(1) {
        switch(g_led_state) {
            case led_off:
                delay_ms(100); // Do not while(1). Todo: Use semaphore
                break;
            case led_flash_once:
                set_led(1);  delay_ms(50);
                set_led(0);
                g_led_state = led_off;
                break;
            case led_flash_double:
                set_led(1); delay_ms(50);
                set_led(0); delay_ms(250);
                set_led(1); delay_ms(50);
                set_led(0); delay_ms(950);
                break;
            case led_flash_fast:
                set_led(1); delay_ms(10);
                set_led(0); delay_ms(450);
                break;
            case led_flash_slow:
                set_led(1); delay_ms(500);
                set_led(0); delay_ms(500);
                break;
        }
    }
}

static bool ism_init()
{
    static bool init = false;
    if (init) {
        return true;
    }
    init = true;
#ifdef RFM69_RESET
    rfm69_setResetPin(RFM69_RESET);
    rfm69_reset();
#endif // RFM69_RESET

    spi_init(1, SPI_MODE0, SPI_FREQ_DIV_1M, 1, SPI_LITTLE_ENDIAN, true);
    /** true = RFM69HW, false = RFM69W */
    bool ok = rfm69_init(RFM69_CS, false);
    if (!ok) {
        printf("RFM69 init failed!\n");
        set_led_state(led_flash_slow);
        return false;
    } else {
        printf("RFM69 init ok\n");
        gpio_enable(RFM69_INT, GPIO_INPUT);
        rfm69_sleep();
        /** set output power, +10 dBm */
        rfm69_setPowerDBm(10);
        /** enable CSMA/CA algorithm */
        rfm69_setCSMA(true);
        rfm69_setAutoReadRSSI(true);
        (void) rfm69_setAESEncryption((void*) RFM_KEY, 16);
        rfm69_setPowerLevel(0); // max 31
    }
    return true;
}

static void ism_task(void *pvParameters)
{
    if (!ism_init()) {
        return;
    }
    while(1) {
        xSemaphoreTake(sniffer_alive, portMAX_DELAY);
        uint8_t data[MAX_ISM_PKT_SIZE];
        memset(data, 0, MAX_ISM_PKT_SIZE);
        int len = rfm69_receive((char*) data, MAX_ISM_PKT_SIZE);
        if (len > 0) {
            set_led_state(led_flash_once);
            int rssi = rfm69_getRSSI();
            data[0] = len+1; // [len data[0] data[1] ... data[len-1] rssi]
            data[len] = (int8_t) rssi;
//            rfm69_sleep();
            if (xQueueSend(publish_queue, (void*) data, 0) == pdFALSE) {
                printf("Publish queue overflow.\n");
            }
        }
        xSemaphoreGive(sniffer_alive);
    }
}

// Return our IP address as a string "a.b.c.d" or "0.0.0.0" if we cannot determine it
static const char* get_station_ip(void)
{
    static char ip[16] = "0.0.0.0";
    ip[0] = 0;
    struct ip_info ipinfo;
    (void) sdk_wifi_get_ip_info(STATION_IF, &ipinfo);
    snprintf(ip, sizeof(ip), IPSTR, IP2STR(&ipinfo.ip));
    return (char*) ip;
}

// Use MAC address for Station as unique ID
static const char* get_station_mac(void)
{
    static char my_id[32];
    static bool my_id_done = false;
    uint8_t hwaddr[6];
    if (my_id_done)
        return my_id;
    if (!sdk_wifi_get_macaddr(STATION_IF, (uint8_t*)hwaddr))
        return NULL;
    snprintf(my_id, sizeof(my_id), "%02x%02x%02x%02x%02x%02x", MAC2STR(hwaddr));
    my_id_done = true;
    return my_id;
}

// MQTT publish helper
static bool mqtt_publish_message(mqtt_client_t *client, char *topic, char *message)
{
//    printf("pub %s : %s\n", topic, message);
    mqtt_message_t pub;
    pub.payload = message;
    pub.payloadlen = strlen(message);
    pub.dup = 0;
    pub.qos = MQTT_QOS1;
    pub.retained = 0;
    if (MQTT_SUCCESS != mqtt_publish(client, topic, &pub)) {
        printf("Error while publishing message\n");
        return false;
    }
    return true;
}

static void mqtt_task(void *pvParameters)
{
    int ret = 0;
    mqtt_network_t network;
    mqtt_client_t client = mqtt_client_default;
    char mqtt_client_id[20];
    uint8_t mqtt_buf[100];
    uint8_t mqtt_readbuf[100];
    mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;

    mqtt_network_new(&network);
    snprintf(mqtt_client_id, sizeof(mqtt_client_id)-1, "espism-%s", get_station_mac());

    while(1) {
        char msg[3*MAX_ISM_PKT_SIZE];
        xSemaphoreTake(wifi_alive, portMAX_DELAY);
        printf("Connecting to MQTT server %s ... ", MQTT_HOST);
        ret = mqtt_network_connect(&network, MQTT_HOST, MQTT_PORT);
        if(ret) {
            printf("error: %d\n", ret);
            mqtt_network_disconnect(&network);
            taskYIELD();
            continue;
        }
        printf("done\n");
        mqtt_client_new(&client, &network, 5000, mqtt_buf, sizeof(mqtt_buf), mqtt_readbuf, sizeof(mqtt_readbuf)); // Todo: Remove magic numbers

        data.willFlag = 0;
        data.MQTTVersion = 3;
        data.clientID.cstring = mqtt_client_id;
        data.username.cstring = MQTT_USER;
        data.password.cstring = MQTT_PASS;
        data.keepAliveInterval = 10;
        data.cleansession = 0;
        printf("Send MQTT connect ...\n");
        ret = mqtt_connect(&client, &data);
        if(ret) {
            printf("error: %d\n", ret);
            mqtt_network_disconnect(&network);
            taskYIELD();
            continue;
        }
        printf("MQTT connected\n");
        xSemaphoreGive(sniffer_alive);
        set_led_state(led_off);
        xQueueReset(publish_queue);

        snprintf(msg, sizeof(msg)-1, "boot:%s", get_station_ip());
        printf("Publishing %s/%s'\n", mqtt_client_id, msg);
        (void) mqtt_publish_message(&client, (char*) mqtt_client_id, (char*) msg);

        while(1) {
            uint8_t pkt[MAX_ISM_PKT_SIZE];
            while(xQueueReceive(publish_queue, (void*) pkt, 0) == pdTRUE) {
                uint8_t msg_len = sizeof(msg);
                uint8_t len = pkt[0];
                int8_t rssi = (int8_t) pkt[len-1];
                msg[0] = 0;
                for (int i=1; i<len-1; i++) {
                    char byte[4];
                    snprintf(byte, sizeof(byte)-1, "%02x", pkt[i]);
                    strncat(msg, byte, msg_len);
                    msg_len -= 2;
                }
                snprintf(msg, sizeof(msg)-1, "%s[%d]", msg, rssi);
                printf("Publishing %s/%s'\n", mqtt_client_id, msg);
                (void) mqtt_publish_message(&client, (char*) mqtt_client_id, (char*) msg);
            }

            ret = mqtt_yield(&client, 1000);
            if (ret == MQTT_DISCONNECTED) {
                xSemaphoreTake(sniffer_alive, portMAX_DELAY);
                break;
            }
        }
        printf("Connection dropped, request restart\n");
        mqtt_network_disconnect(&network);
        delay_ms(1000);
        taskYIELD();
    }
}

static void wifi_task(void *pvParameters)
{
    uint8_t status  = 0;
    uint8_t retries = 30;
    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };

    printf("WiFi: connecting to WiFi\n");
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);

    while(1) {
        while ((status != STATION_GOT_IP) && (retries)) {
            status = sdk_wifi_station_get_connect_status();
            printf("Wifi status : %d\n", status);
            if(status == STATION_WRONG_PASSWORD) {
                printf("WiFi: wrong password\n");
                break;
            } else if(status == STATION_NO_AP_FOUND) {
                printf("WiFi: AP not found\n");
                break;
            } else if(status == STATION_CONNECT_FAIL) {
                printf("WiFi: connection failed\n");
                break;
            }
            delay_ms(1000);
            --retries;
        }
        if (status == STATION_GOT_IP) {
            printf("WiFi: Connected\n");
            xSemaphoreGive(wifi_alive);
            taskYIELD();
        }

        while ((status = sdk_wifi_station_get_connect_status()) == STATION_GOT_IP) {
            xSemaphoreGive(wifi_alive);
            taskYIELD();
        }
        printf("WiFi: disconnected\n");
        sdk_wifi_station_disconnect();
        delay_ms(1000);
    }
}

void user_init(void)
{
    gpio_enable(LED_GPIO, GPIO_OUTPUT);
    set_led(false);

    uart_set_baud(0, 115200);
    rboot_config conf = rboot_get_config();
    printf("\n\nEspism Sniffer\n");
    printf("SDK version:%s\n", sdk_system_get_sdk_version());
    printf("HW rev %d\n", HW_REV);

    printf("Running on flash slot %d / %d\n", conf.current_rom, conf.count);
    printf("Image addresses in flash:\n");
    for(int i = 0; i <conf.count; i++) {
        printf("%c%d: offset 0x%08x\n", i == conf.current_rom ? '*':' ', i, conf.roms[i]);
    }
    ota_tftp_init_server(TFTP_PORT);
    set_led_state(led_flash_double);
    xTaskCreate(&led_task,  "led_task",   256, NULL, 2, NULL);
    delay_ms(5000); /** not sure why I added this ;) */

    vSemaphoreCreateBinary(wifi_alive);
    vSemaphoreCreateBinary(sniffer_alive);
    xSemaphoreTake(sniffer_alive, portMAX_DELAY);
    publish_queue = xQueueCreate(PUB_QUEUE_DEPTH, MAX_ISM_PKT_SIZE);
    xTaskCreate(&wifi_task, "wifi_task",  256, NULL, 2, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 1024, NULL, 4, NULL);
    xTaskCreate(&ism_task,  "ism_task",   256, NULL, 2, NULL);
    xTaskCreate(&heartbeat_task,  "heartbeat_task",   256, NULL, 2, NULL);
}
