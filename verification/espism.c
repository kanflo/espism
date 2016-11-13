/* 
 *
 * This application is used for hardware verification of the Espisn board. Build, flash, boot and type 'help'.
 * Oh, the UART runs at 115200 baud.
 */

#include "espressif/esp_common.h"
#include "esp/uart.h"

#include <stdint.h>
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <esp8266.h>
#include <esp/uart.h>
#include <stdio.h>
#include <spi.h>
#include <rfm69.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "ota-tftp.h"
#include "rboot-integration.h"
#include "rboot.h"
#include "rboot-api.h"


#define HW_REV   (2) // 1..3

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
#else
 #error "Unsupported HW revision"
#endif

#define LOOP_DELAY_MS 2000

#define MAX_ARGC (10)

#define delay_ms(ms) vTaskDelay(ms / portTICK_RATE_MS)

static bool adc_display = false;
static bool btn_display = false;
static bool rx_display = false;
static bool tx_display = false;

static void set_led(bool on)
{
    gpio_write(LED_GPIO, on?0:1);
}

static bool button_pressed()
{
    uint16_t val = sdk_system_adc_read();
    return val > 100;
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

    spi_init(iHSPI);
    bool ok = rfm69_init(RFM69_CS, false); // false = RFM69W, true = RFM69HW

    if (!ok) {
        printf("RFM69 init failed!\n");
        return false;
    } else {
        printf("RFM69 init ok\n");
        gpio_enable(RFM69_INT, GPIO_INPUT);
        rfm69_sleep(); // init RF module and put it to sleep
        rfm69_setPowerDBm(10); // // set output power, +10 dBm
        rfm69_setCSMA(true); // enable CSMA/CA algorithm
        rfm69_setAutoReadRSSI(true);
        (void) rfm69_setAESEncryption((void*) "sampleEncryptKey", 16);
        rfm69_setPowerLevel(31); // max 31
    }
    return true;
}

static bool cmd_adc(uint32_t argc, char *argv[])
{
    if (argc == 1) {
        uint16_t val = sdk_system_adc_read();
        printf("adc %d\n", val);
    } else if (argc == 2) {
        adc_display = strcmp(argv[1], "on") == 0;
    } else {
        return false;
    }
    return true;
}

static bool cmd_led(uint32_t argc, char *argv[])
{
    if (argc == 2) {
        set_led(strcmp(argv[1], "on") == 0);
    } else {
        return false;
    }
    return true;
}

static bool cmd_button(uint32_t argc, char *argv[])
{
    if (argc == 1) {
        printf("Button is %spressed\n", button_pressed() ? "" : "not ");
    } else if (argc == 2) {
        btn_display = strcmp(argv[1], "on") == 0;
    } else {
        return false;
    }
    return true;
}

static bool cmd_ism_init(uint32_t argc, char *argv[])
{
    return ism_init();
}

static bool cmd_rx(uint32_t argc, char *argv[])
{
    if (argc == 1) {
        // send a packet and let RF module sleep
        if (ism_init()) {
            char data[32];
            memset(data, 0, sizeof(data));
            int len = rfm69_receive(data, sizeof(data));
            if (len > 0) {
                int rssi = rfm69_getRSSI();
                set_led(true);
                rfm69_sleep();
                printf("Got %d bytes :", len);
                for (int i=0; i<len; i++) {
                    printf(" %02x", data[i]);
                }
                printf(" [%-3d]\n", rssi);
                set_led(false);
            }
        }
    } else if (argc == 2) {
        rx_display = strcmp(argv[1], "on") == 0;
        printf("yes:%d\n", rx_display);
    } else {
        return false;
    }
    return true;
}

static bool cmd_tx(uint32_t argc, char *argv[])
{
    if (argc == 1) {
        // send a packet and let RF module sleep
        char testdata[] = {1, 42, 0x00, 'H', 'e', 'l', 'l', 'o', ' ', 'W', 'o', 'r', 'l', 'd', '!'};
        if (ism_init()) {
            set_led(true);
            rfm69_send(testdata, sizeof(testdata));
            set_led(false);
            rfm69_sleep();
            printf("Message sent\n");
        }
    } else if (argc == 2) {
        tx_display = strcmp(argv[1], "on") == 0;
    } else {
        return false;
    }
    return true;
}

static bool cmd_write_register(uint32_t argc, char *argv[])
{
    if (argc == 3) {
        uint32_t reg = atoi(argv[1]);
        uint32_t val = atoi(argv[2]);
        printf("%x = %d\n", reg, val);
        rfm69_writeRegister(reg, val);
        return true;
    } else {
        return false;
    }
}

static bool cmd_read_register(uint32_t argc, char *argv[])
{
    if (argc == 2) {
        uint32_t reg = atoi(argv[1]);
        printf("[%d] == 0x%x\n", reg, rfm69_readRegister(reg));
        return true;
    } else {
        return false;
    }
}

static bool cmd_dump_registers(uint32_t argc, char *argv[])
{
    rfm69_dumpRegisters();        
    return true;
}

static bool cmd_help(uint32_t argc, char *argv[])
{
    printf(
        "adc [<on|off>]      Print adc value or start/stop polling\n"
        "led <on|off>        Control LED\n"
        "button [<on|off>]   Print button value or start/stop polling\n"
        "init                Initialize RFM69CW\n"
        "tx                  TX one packet\n"
        "rx                  RX one packet\n"
        "registers           Dump RFM69CW registers\n"
        "w <reg> <value>     Write RFM69CW register\n"
        "r <reg>             Read RFM69CW register\n"
    );
    return true;
}

static void handle_command(char *cmd)
{
    char *argv[MAX_ARGC];
    int argc = 1;
    char *temp, *rover;
    bool ok = true;
    memset((void*) argv, 0, sizeof(argv));
    argv[0] = cmd;
    rover = cmd;
    // Split string "<command> <argument 1> <argument 2>  ...  <argument N>"
    // into argv, argc style
    while(argc < MAX_ARGC && (temp = strstr(rover, " "))) {
        rover = &(temp[1]);
        argv[argc++] = rover;
        *temp = 0;
    }

    if (strlen(argv[0]) > 0) {
        if (strcmp(argv[0], "help") == 0) ok = cmd_help(argc, argv);
        else if (strcmp(argv[0], "adc") == 0) ok = cmd_adc(argc, argv);
        else if (strcmp(argv[0], "led") == 0) ok = cmd_led(argc, argv);
        else if (strcmp(argv[0], "button") == 0) ok = cmd_button(argc, argv);
        else if (strcmp(argv[0], "init") == 0) ok = cmd_ism_init(argc, argv);
        else if (strcmp(argv[0], "tx") == 0) ok = cmd_tx(argc, argv);
        else if (strcmp(argv[0], "rx") == 0) ok = cmd_rx(argc, argv);
        else if (strcmp(argv[0], "registers") == 0) ok = cmd_dump_registers(argc, argv);
        else if (strcmp(argv[0], "w") == 0) ok = cmd_write_register(argc, argv);
        else if (strcmp(argv[0], "r") == 0) ok = cmd_read_register(argc, argv);
        else printf("Unknown command %s, try 'help'\n", argv[0]);
        if (ok) printf("ok\n");
    }
}

static void adc_task(void *pvParameters)
{
    while (1) {
        if (adc_display) {
            printf("adc %d\n", sdk_system_adc_read());
        }
        delay_ms(LOOP_DELAY_MS);
    }
}

static void btn_task(void *pvParameters)
{
    while (1) {
        if (btn_display) {
            printf("Button is %spressed\n", button_pressed() ? "" : "not ");
        }
        delay_ms(LOOP_DELAY_MS);
    }
}

static void rx_task(void *pvParameters)
{
    while(1) {
        if (rx_display) {
            (void) ism_init();
            cmd_rx(1, 0);
        } else {
            delay_ms(LOOP_DELAY_MS);
        }
    }
}

static void tx_task(void *pvParameters)
{
    while(1) {
        if (tx_display) {
            (void) ism_init();
            cmd_tx(1, 0);
        }
        delay_ms(LOOP_DELAY_MS);
    }
}

static void cli_task(void *pvParameters)
{
    char ch;
    char cmd[81];
    int i = 0;
    printf("\n\n\nEspism CLI. Type 'help<enter>' for, well, help\n");
    printf("%% ");
    while(1) {
        if (read(0, (void*)&ch, 1)) { // 0 is stdin
            printf("%c", ch);
            if (ch == '\n' || ch == '\r') {
                cmd[i] = 0;
                i = 0;
                printf("\n");
                handle_command((char*) cmd);
                printf("%% ");
            } else {
                if (i < sizeof(cmd)) cmd[i++] = ch;
            }
        }
    }
}

static void led_flash()
{
    delay_ms(250);
    for (int i = 0; i < 3; i++) {
        set_led(true);
        delay_ms(80);
        set_led(false);
        delay_ms(150);
    }

}

void user_init(void)
{
    gpio_enable(LED_GPIO, GPIO_OUTPUT);
    set_led(false);
    led_flash();

    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());
    rboot_config conf = rboot_get_config();
    printf("\r\n\r\nEsproom.\r\nCurrently running on flash slot %d / %d.\r\n\r\n", conf.current_rom, conf.count);

    printf("Image addresses in flash:\r\n");
    for(int i = 0; i <conf.count; i++) {
        printf("%c%d: offset 0x%08x\r\n", i == conf.current_rom ? '*':' ', i, conf.roms[i]);
    }
    ota_tftp_init_server(TFTP_PORT);

    xTaskCreate(&adc_task, (signed char *)"adc", 256, NULL, 2, NULL);
    xTaskCreate(&btn_task, (signed char *)"btn", 256, NULL, 2, NULL);
    xTaskCreate(&rx_task,  (signed char *)"rx" , 256, NULL, 2, NULL);
    xTaskCreate(&tx_task,  (signed char *)"tx" , 256, NULL, 2, NULL);
    xTaskCreate(&cli_task, (signed char *)"cli", 256, NULL, 2, NULL);
}
