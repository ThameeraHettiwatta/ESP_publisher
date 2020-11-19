/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "tcpip_adapter.h"
#include "soc/rtc_periph.h"
#include "esp32/rom/cache.h"
#include "driver/spi_slave.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/igmp.h"

#include "mqtt_client.h"


/*
Pins in use. The SPI Master can use the GPIO mux, so feel free to change these if needed.
*/
#define GPIO_HANDSHAKE 16
#define GPIO_MOSI 12
#define GPIO_MISO 13
#define GPIO_SCLK 15
#define GPIO_CS 14

static const char *TAG = "MQTT_EXAMPLE";

typedef struct {
    int16_t sensor_id;
    uint8_t seq_no;
    int16_t sensor_rssi;
    int16_t sensor_acc[3];
    int16_t sensor_gyro[3];
    int16_t sensor_mag[3];
    float acc[3];
    float gyro[3];
    float mag[3];
} sensor_data;

sensor_data data_t;

esp_mqtt_client_handle_t client;

uint8_t mac[6];

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        // case MQTT_EVENT_SUBSCRIBED:
        //     ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        //     msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        //     ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        //     break;
        // case MQTT_EVENT_UNSUBSCRIBED:
        //     ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        //     break;
        case MQTT_EVENT_PUBLISHED:
            // ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        // .uri = "mqtt://mqtt_server:1234@192.168.43.187:1883", //lahiru
        // .uri = "mqtt://mqtt_server:1234@192.168.8.103:1883", //thameera
        // .uri = "mqtt://twhkvnkt:0qLBb25EOa3T@m11.cloudmqtt.com:17595", //cloud mqtt

        ////.uri = "mqtt://192.168.43.187:1883", //lahiru
        //.uri = "mqtt://mqtt_server:1234@192.168.8.102:1883", //thameera
        .uri = "mqtt://twhkvnkt:0qLBb25EOa3T@192.168.9.230:1883", //cloud mqtt
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}


//Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void my_post_setup_cb(spi_slave_transaction_t *trans) {
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1<<GPIO_HANDSHAKE));
}

//Called after transaction is sent/received. We use this to set the handshake line low.
void my_post_trans_cb(spi_slave_transaction_t *trans) {
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1<<GPIO_HANDSHAKE));
}

void app_main()
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    
    esp_wifi_get_mac(ESP_MAC_WIFI_STA, mac);

    mqtt_app_start();

    esp_err_t ret;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg={
        .mode=0,
        .spics_io_num=GPIO_CS,
        .queue_size=3,
        .flags=0,
        .post_setup_cb=my_post_setup_cb,
        .post_trans_cb=my_post_trans_cb
    };

    //Configuration for the handshake line
    gpio_config_t io_conf={
        .intr_type=GPIO_INTR_DISABLE,
        .mode=GPIO_MODE_OUTPUT,
        .pin_bit_mask=(1<<GPIO_HANDSHAKE)
    };

    //Configure handshake line as output
    gpio_config(&io_conf);
    //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    //Initialize SPI slave interface
    ret=spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, 1);
    assert(ret==ESP_OK);

    WORD_ALIGNED_ATTR uint8_t sendbuf[128]= {0};
    // WORD_ALIGNED_ATTR char recvbuf[129]="";
    WORD_ALIGNED_ATTR uint8_t recvbuf[128] = {0};
    memset(recvbuf, 0, 128);
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    while(1) {
        //Clear receive buffer, set send buffer to something sane
        memset(recvbuf, 0xA5, 128);
        // sprintf(sendbuf, "This is the receiver, sending data for transmission number %04d.", n);
        //Set up a transaction of 128 bytes to send/receive
        t.length=128*8;
        t.tx_buffer=sendbuf;
        t.rx_buffer=recvbuf;
        /* This call enables the SPI slave interface to send/receive to the sendbuf and recvbuf. The transaction is
        initialized by the SPI master, however, so it will not actually happen until the master starts a hardware transaction
        by pulling CS low and pulsing the clock etc. In this specific example, we use the handshake line, pulled up by the
        .post_setup_cb callback that is called as soon as a transaction is ready, to let the master know it is free to transfer
        data.
        */
        ret=spi_slave_transmit(HSPI_HOST, &t, portMAX_DELAY);
        // memcpy(sendbuf ,  recvbuf, 128);
        //spi_slave_transmit does not return until the master has done a transmission, so by here we have sent our data and
        //received data from the master. Print it.
        // printf("Received: %s\n", recvbuf);
        // esp_log_buffer_hex("Beacon_DEMO: data:", recvbuf, (int) sizeof(recvbuf));

        sendbuf[1] = 0;

        // printf("Received: %d\n", recvbuf[1]);
        if(recvbuf[1] == 0x16)
        {
            sendbuf[1] = 0x16;

            data_t.sensor_rssi = (int8_t)recvbuf[0];
            data_t.seq_no = recvbuf[3];
            printf("Sequence number: %d \n", (int) data_t.seq_no);
            data_t.sensor_id = (uint16_t)recvbuf[5] | ((uint16_t)(recvbuf[4])) << 8;
            int i;
            for( i=0; i<3; i++)
            {
                data_t.sensor_acc[i] = (uint16_t)(recvbuf[i*2+7]) <<8 | (uint16_t)recvbuf[i*2+6];
                //-- calculate acceleration, unit G, range -16, +16    
                data_t.acc[i] = (data_t.sensor_acc[i] * 1.0) / (32768.0/16.0);

                data_t.sensor_gyro[i] = (uint16_t)(recvbuf[i*2+13]) <<8 | (uint16_t)recvbuf[i*2+12];
                //-- calculate rotation, unit deg/s, range -250, +250
                data_t.gyro[i] = (data_t.sensor_gyro[i] * 1.0) / (65536.0 / 500.0);

                data_t.sensor_mag[i] = (uint16_t)(recvbuf[i*2+19]) <<8 | (uint16_t)recvbuf[i*2+18];
                //-- calculate magnetic field, unit microtesla, range -250, +250
                //-- calculate magnetism, unit uT, range +-4900
                // return 1.0 * data;
                data_t.mag[i] = (data_t.sensor_mag[i] * 1.0);
            }

                                
            char cPayload[250];
            sprintf(cPayload,
                "{\"seq_no\": %d, \"mac\": \"%02X:%02X:%02X:%02X:%02X:%02X\", \"snsr_id\": %d, \"rssi\": %d, \"acc_x\": %f, \"acc_y\": %f, \"acc_z\": %f, \"gyr_x\": %f, \"gyr_y\": %f, \"gyr_z\": %f, \"mag_x\": %f, \"mag_y\": %f, \"mag_z\": %f}",
                    data_t.seq_no, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], data_t.sensor_id, data_t.sensor_rssi, 
                    data_t.acc[0], data_t.acc[1], data_t.acc[2], 
                    data_t.gyro[0], data_t.gyro[1], data_t.gyro[2],
                    data_t.mag[0], data_t.mag[1], data_t.mag[2]);
            // client: mqtt client handle
            // topic: topic string
            // data: payload string (set to NULL, sending empty payload message)
            // len: data length, if set to 0, length is calculated from payload string
            // qos: qos of publish message
            // retain: retain flag
            esp_mqtt_client_publish(client, "/topic/esp2", cPayload, 0, 0, 0);
            // esp_log_buffer_hex("JSON data",cPayload,500);
        }
        
}
}