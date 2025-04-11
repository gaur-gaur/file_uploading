#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/param.h>
#include <sys/unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_netif.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "host/ble_gap.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "driver/i2c_master.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "esp_http_server.h"
#include "protocol_examples_common.h"
#include "file_serving_example_common.h"
#include "cJSON.h"
#include <time.h>
#include <sys/time.h>
#include "freertos/semphr.h"
#include <math.h>
#include "esp_mac.h"
#include "nvs.h"
#include <mqtt_client.h>
#include "esp_sntp.h"
#include "driver/gpio.h"
#include <errno.h>
#include "tinyusb.h"
#include "tusb_msc_storage.h"
#include "wear_levelling.h"
#include "esp_partition.h"
#include "tusb.h"
#include "ff.h"
#include "tusb_cdc_acm.h"

// UART Configuration
#define UART_NUM UART_NUM_1
#define UART_NUM_UART2 UART_NUM_2
#define UART_TXD1_PIN 17
#define UART_RXD1_PIN 18
#define UART_TXD2_PIN 6
#define UART_RXD2_PIN 7
#define BUF_SIZE 1024
#define RD_BUF_SIZE BUF_SIZE
#define I2C_BUS_PORT  0
#define OLED_PIXEL_CLOCK_HZ    (400 * 1000)
#define OLED_PIN_NUM_SDA       8
#define OLED_PIN_NUM_SCL       9
#define OLED_PIN_NUM_RST       -1
#define OLED_I2C_HW_ADDR       0x3C
#define OLED_H_RES             128
#define OLED_V_RES             64
#define OLED_CMD_BITS          8
#define OLED_PARAM_BITS        8
#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_FATFS_MAX_LFN)
#define MAX_FILE_SIZE   (200*1024) // 200 KB      
#define MAX_FILE_SIZE_STR "200KB"
#define SCRATCH_BUFSIZE  2048
#define NUM_TARGETS 5
#define NUM_COMMANDS 20
#define BLE_NOTIFY_MAX_LEN 20  // Define your BLE notify size limit
#define CONNECT_TIMEOUT_MS 20000
#define CONNECT_TIMEOUT_MS_wifi 60000
#define TIMEOUT_MS 1000
#define CHECK_INTERVAL 500
#define TIMEOUT_SECONDS 20
#define MQTT_BROKER_URI "mqtts://otplai.com:8883"
#define MQTT_USER "oyt"
#define MQTT_PASSWORD "123456789"
#define UART_NUM_UART0 UART_NUM_0  // Change to UART0
#define START_STRING "<START>"
#define STOP_STRING "<STOP>"
#define FILE_PATH1  "/fatfs/config.json"
#define BUTTON_GPIO  GPIO_NUM_15  // Change this to your actual button GPIO
#define COUNTDOWN_THRESHOLD 4   // Maximum countdown value
#define STORAGE_NAMESPACE "storage"
#define LAST_MODE_KEY "last_mode"
#define MAX_FILES 10  // Maximum number of files to keep

static bool usb_msc_initialized = false;
static wl_handle_t wl_handle = WL_INVALID_HANDLE;
static bool fatfs_initialized = false;
void usb_msc_init(void);
void example_lvgl_demo_ui(lv_disp_t *disp, double Result[5]);
uint8_t cmd_rcv=0;
int status=0;
double unpack_double(uint8_t *bytes_received);
void ble_app_advertise(void);
void combine_data_and_calculate();
void upload_every_minute();
static uint16_t tx_handle;
void convertDoubleArrayToString(const double *arr, int size, char *result);
char buff_rec_data[1024];
char combine_array[1024];
char combine_array1[1024];
char combine_array_ble[1024];
double responseArray[NUM_COMMANDS] = {0};
double SensorResponse[NUM_COMMANDS] = {0};
double Result[NUM_TARGETS] = {0};
void combine_data_string();
bool flag_uploading = false;
bool flag_uploading_wifi = false;
bool flag_uploading_uart2 = false;
bool ble_start_detected = false;
void wifi_initate_with_Server();
bool flag_serverstart = false;
bool flag_uploading_internal_data_save = false;
void update_config_from_file(const char *file_path);
void wifi_init_mqtt();
const char *base_path = "/fatfs";
esp_err_t example_restart_file_server(const char *base_path);
SemaphoreHandle_t xMutex;
int reboot_count=0,restart_gsm=0;
int publish_to_mqtt(const char *topic_tx, const char *buff_rec_data);
static void wifi_init_sta(void);
char WIFI_SSID[32] = "aa";
char WIFI_PASS[32] = "00000000";
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static bool wifi_connected = false;
static bool mqtt_connected = false;
static esp_mqtt_client_handle_t mqtt_client = NULL;
int INTERVAL;
char SEN_ID[50];
float Latitude;
float Longitude;
uint8_t mac_id[6];
char mac_str[18];
uint8_t sensor_id[16] = {0};
char topic_rx[50];
char topic_tx[50];
static char wifi_ip_address[16] = {0};
void update_system_time_from_gsm();
static const char *TAG = "BLE_UART";
void ble_init();
void oled_init(void);
bool strt_flg=0;
volatile bool flag_gsm = false;
volatile bool flag_wifi = false;
volatile bool flag_ble = false;
volatile bool flag_uart2 = false;
volatile bool flag_uart0 = false;
TaskHandle_t mqttReceiveHandle = NULL;
TaskHandle_t mqttPublishHandle = NULL;
TaskHandle_t bleNotifyHandle = NULL;
TaskHandle_t uart2Handle = NULL;
TaskHandle_t uartEventHandle = NULL;
bool flag_uploading_ble = false;
char USER[20];
#define MQTT_WILL_TOPIC "hydrosensor/%s/status"  // Will topic using MAC address
#define MQTT_WILL_MESSAGE "disconnected"         // Will message payload
#define MQTT_WILL_QOS 2                          // Will QoS level
#define MQTT_WILL_RETAIN 1                    // Whether to retain will message
char will_topic[50];
 //=================================================================================================================================================================================
/* Service UUID */
static const ble_uuid128_t service_uuid = BLE_UUID128_INIT(
    0x6E, 0x40, 0x00, 0x01, 0xB5, 0xA3, 0xF3, 0x93,
    0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E);
/* RX Characteristic UUID */
static const ble_uuid128_t char_uuid_rx = BLE_UUID128_INIT(
    0x6E, 0x40, 0x00, 0x02, 0xB5, 0xA3, 0xF3, 0x93,
    0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E);
/* TX Characteristic UUID */
static const ble_uuid128_t char_uuid_tx = BLE_UUID128_INIT(
    0x6E, 0x40, 0x00, 0x03, 0xB5, 0xA3, 0xF3, 0x93,
    0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E);
struct file_server_data {
    char base_path[ESP_VFS_PATH_MAX + 1];                                                //Base path of file storage
    char scratch[SCRATCH_BUFSIZE];                                                      //Scratch buffer for temporary storage during file transfer
};
//==================================================================================================================================================================================
uint8_t commands[NUM_COMMANDS][15] = {
   {0x90, 0x09, 0x01, 0x01, 0x07, 0xD0, 0x00, 0x05, 0x52, 0x31, 0x31, 0x3B}, // Command R11 12 bytes
    {0x90, 0x09, 0x01, 0x01, 0x07, 0xD0, 0x00, 0x05, 0x52, 0x31, 0x32, 0x3B}, // Command R12 12 bytes
    {0x90, 0x09, 0x01, 0x01, 0x07, 0xD0, 0x00, 0x05, 0x52, 0x31, 0x33, 0x3B}, // Command R13 12 bytes
    {0x90, 0x09, 0x01, 0x01, 0x07, 0xD0, 0x00, 0x05, 0x52, 0x31, 0x34, 0x3B}, // Command R14 12 bytes
    {0x90, 0x09, 0x01, 0x01, 0x07, 0xD0, 0x00, 0x05, 0x52, 0x32, 0x31, 0x3B}, // Command R21 12 bytes
    {0x90, 0x09, 0x01, 0x01, 0x07, 0xD0, 0x00, 0x05, 0x52, 0x32, 0x32, 0x3B}, // Command R22 12 bytes
    {0x90, 0x09, 0x01, 0x01, 0x07, 0xD0, 0x00, 0x05, 0x52, 0x32, 0x33, 0x3B}, // Command R23 12 bytes
    {0x90, 0x09, 0x01, 0x01, 0x07, 0xD0, 0x00, 0x05, 0x52, 0x32, 0x34, 0x3B}, // Command R24 12 bytes
    {0x90, 0x09, 0x01, 0x01, 0x07, 0xD0, 0x00, 0x05, 0x52, 0x33, 0x31, 0x3B}, // Command R31 12 bytes
    {0x90, 0x09, 0x01, 0x01, 0x07, 0xD0, 0x00, 0x05, 0x52, 0x33, 0x32, 0x3B}, // Command R32 12 bytes
    {0x90, 0x09, 0x01, 0x01, 0x07, 0xD0, 0x00, 0x05, 0x52, 0x33, 0x33, 0x3B}, // Command R33 12 bytes
    {0x90, 0x09, 0x01, 0x01, 0x07, 0xD0, 0x00, 0x05, 0x52, 0x33, 0x34, 0x3B}, // Command R34 12 bytes
    {0x90, 0x08, 0x01, 0x01, 0x03, 0xE8, 0x03, 0xE8, 0x54, 0x31, 0x3B},        // Command T1 11 bytes
    {0x90, 0x08, 0x01, 0x01, 0x03, 0xE8, 0x03, 0xE8, 0x54, 0x32, 0x3B},        // Command T2 11 bytes
    {0x90, 0x08, 0x01, 0x00, 0x13, 0x88, 0x00, 0x04, 0x43, 0x31, 0x3B},        // Command C1 11 bytes
    {0x90, 0x08, 0x01, 0x00, 0x13, 0x88, 0x00, 0x04, 0x43, 0x32, 0x3B},         // Command C2 11 bytes
    {0x90, 0x0C, 0x01, 0x01, 0x07, 0xD0, 0x00, 0x05, 0x52, 0x31, 0x31, 0x5F, 0x32, 0x57, 0x3B},
    {0x90, 0x0C, 0x01, 0x01, 0x07, 0xD0, 0x00, 0x05, 0x52, 0x31, 0x32, 0x5F, 0x32, 0x57, 0x3B},
    {0x90, 0x0C, 0x01, 0x01, 0x07, 0xD0, 0x00, 0x05, 0x52, 0x33, 0x33, 0x5F, 0x32, 0x57, 0x3B},
    {0x90, 0x0C, 0x01, 0x01, 0x07, 0xD0, 0x00, 0x05, 0x52, 0x33, 0x34, 0x5F, 0x32, 0x57, 0x3B},
};
uint8_t command_sizes[NUM_COMMANDS] = {12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11,15,15,15,15};
//==================================================================================================================================================================================
double Coeff[16][5] = {
        {0.1 , 0.2 , 0.3 , 0.05, 0.1},
        {0.1 , 0.2 , 0.3 , 0.05, 0.1},
        {0.1 , 0.2 , 0.3 , 0.05, 0.1},
        {0.1 , 0.2 , 0.3 , 0.05, 0.1},
        {0.1 , 0.2 , 0.3 , 0.05, 0.1},
        {0.1 , 0.2 , 0.3 , 0.05, 0.1},
        {0.1 , 0.2 , 0.3 , 0.05, 0.1},
        {0.1 , 0.2 , 0.3 , 0.05, 0.1},
        {0.1 , 0.2 , 0.3 , 0.05, 0.1},
        {0.1 , 0.2 , 0.3 , 0.05, 0.1},
        {0.1 , 0.2 , 0.3 , 0.05, 0.1},
        {0.1 , 0.2 , 0.3 , 0.05, 0.1},
        {0.1 , 0.2 , 0.3 , 0.05, 0.1},
        {0.1 , 0.2 , 0.3 , 0.05, 0.1},
        {0.1 , 0.2 , 0.3 , 0.05, 0.1},
        {0.1 , 0.2 , 0.3 , 0.05, 0.1},
 };
double Temp_Coeff[5] = {0.5, 0.038798, 0.5, 0.038798, 25.678};
double Cond_Coeff[6] = {0.01177, 0.00744, 1.136, -0.432, 76.897,42.705};
double TDS_Coeff = 0.5;
int MAX_ITERS = 5;
int MAX_ITERS_LS = 5;
double MAX_TOL = 0.001;
//================================================================================================================================================================================
static void monitor_file_task(void *arg) {
    while (1) {
        DIR *dir = opendir("/fatfs");
        if (dir == NULL) {
            ESP_LOGE(TAG, "Failed to open directory");
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }
        struct dirent *entry;
        struct stat file_stat;
        char file_path[FILE_PATH_MAX];
        time_t oldest_time = 0;
        char oldest_file[FILE_PATH_MAX] = {0};
        int file_count = 0;
        // Iterate through all files in the directory
        while ((entry = readdir(dir)) != NULL) {
            if (entry->d_type != DT_REG) {
                continue;                                                                                     // Skip non-regular files
            }
            // Check if the file name starts with "combined_data"
            if (strncmp(entry->d_name, "combined_data", 13) == 0) {
                snprintf(file_path, sizeof(file_path), "/fatfs/%.38s", entry->d_name);
                if (stat(file_path, &file_stat) == 0) {
                    file_count++;
                    // Track the oldest file
                    if (oldest_time == 0 || file_stat.st_mtime < oldest_time) {
                        oldest_time = file_stat.st_mtime;
                        strncpy(oldest_file, file_path, sizeof(oldest_file));
                    }
                }
            }
        }
        closedir(dir);
        // If the number of files exceeds the limit, delete the oldest file
        if (file_count > MAX_FILES && oldest_file[0] != '\0') {
            ESP_LOGI(TAG, "Deleting oldest file: %s", oldest_file);
            if (remove(oldest_file) == 0) {
                ESP_LOGI(TAG, "Successfully deleted file: %s", oldest_file);
            } else {
                ESP_LOGE(TAG, "Failed to delete file: %s", oldest_file);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
//=============================================================================================================================================================================
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        esp_wifi_connect();
        ESP_LOGI(TAG, "Retrying Wi-Fi connection...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        snprintf(wifi_ip_address, sizeof(wifi_ip_address), IPSTR, IP2STR(&event->ip_info.ip));       // Store IP address
        wifi_connected = true;
        ESP_LOGI(TAG, "Got IP: %s", wifi_ip_address);
    }
}
//===========================================================================================================================================================================
static void wifi_init_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    // Register Wi-Fi event handlers
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
wifi_config_t wifi_config = {
        .sta = {
            .ssid = {0},
            .password = {0},
        },
    };
// Copy the updated Wi-Fi credentials into the struct
    strncpy((char *)wifi_config.sta.ssid, (char *)WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, (char *)WIFI_PASS, sizeof(wifi_config.sta.password) - 1);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
    ESP_LOGI(TAG, "Connecting to Wi-Fi...");
    // Display countdown on the OLED
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    int elapsed_time = 0;
    int countdown_time = CONNECT_TIMEOUT_MS_wifi / 1000;                                                                     // Countdown in seconds
    while (!wifi_connected && elapsed_time < CONNECT_TIMEOUT_MS_wifi) {
        // Update countdown display
        char countdown_text[32];
        snprintf(countdown_text, sizeof(countdown_text), "Connecting... %d", countdown_time);
        lv_label_set_text(label, countdown_text);
        lv_task_handler();
        // Wait for the next check
        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL));
        elapsed_time += CHECK_INTERVAL;
        // Update countdown time
        if (elapsed_time % 1000 == 0) {
            countdown_time--;
        }
    }
    if (wifi_connected) {
        ESP_LOGI(TAG, "Wi-Fi connected successfully.");
        // Update OLED display to show IP address
        char success_text[64];
        snprintf(success_text, sizeof(success_text), "Connected!\nIP: %s", wifi_ip_address);
        lv_label_set_text(label, success_text);
        lv_task_handler();
        example_restart_file_server(base_path);                                                             // Start the file server
    } else {
      ESP_LOGW(TAG, "Wi-Fi connection timed out.");
        lv_label_set_text(label, "No Hotspot Found.");
        lv_task_handler();
        esp_wifi_stop();                                                                                    // Stop Wi-Fi to save power
    }
}
//=============================================================================================================================================
void get_esp32_mac_id(uint8_t *mac_id) {
    if (mac_id == NULL) {
        ESP_LOGE(TAG, "Invalid MAC ID buffer");
        return;
    }
    // Get the MAC address for the default Wi-Fi interface (station mode)
    esp_err_t err = esp_efuse_mac_get_default(mac_id);
    if (err == ESP_OK) {
		snprintf(mac_str, sizeof(mac_str), "%02X%02X%02X%02X%02X%02X",
                 mac_id[0], mac_id[1], mac_id[2], mac_id[3], mac_id[4], mac_id[5]);
        ESP_LOGI("ESP32_MAC", "MAC ID retrieved successfully");
         ESP_LOGI(TAG, "ESP32 MAC ID: %s", mac_str);                                                         // Print MAC ID
    } else {
        ESP_LOGE("ESP32_MAC", "Failed to retrieve MAC ID, error: %d", err);
        memset(mac_id, 0, 6);                                                                               // Assign default zeros in case of failure
    }
}
//==================================================================================================================================
void generate_mqtt_topics(char *topic_rx, char *topic_tx, size_t buffer_size) {
    get_esp32_mac_id(mac_id);
    snprintf(topic_rx, buffer_size, "hydrosensor/%s/TX", mac_str);
    snprintf(topic_tx, buffer_size, "hydrosensor/%s/RX", mac_str);
    ESP_LOGI(TAG, "Subscribe topic: %s", topic_rx);
    ESP_LOGI(TAG, "Publish topic: %s", topic_tx);
}
//============================================================================================================================================
void get_sensor_id(uint8_t *sensor_id) {
    uint8_t command[] = {0x63, 0x00, 0x3B};                                                              // Command to request sensor ID
    uint8_t response[16] = {0};                                                                           // Buffer for response
    const uint8_t default_id[16] = {0};                                                                  // Default ID when no response is received
    // Send command
    int bytes_written = uart_write_bytes(UART_NUM, (const char *)command, sizeof(command));
    if (bytes_written != sizeof(command)) {
        ESP_LOGE(TAG, "Failed to send command");
        memcpy(sensor_id, default_id, sizeof(default_id));
        return;
    }
    // Wait for response
    int bytes_read = uart_read_bytes(UART_NUM, response, sizeof(response), pdMS_TO_TICKS(TIMEOUT_MS));
    if (bytes_read == sizeof(response)) {
        // Successfully read the sensor ID
        memcpy(sensor_id, response, sizeof(response));
        ESP_LOGI(TAG, "Sensor ID read successfully");
         printf("Sensor ID: ");
    for (int i = 0; i < sizeof(sensor_id); i++) {
        printf("%02X ", sensor_id[i]);
    }
    } else {
        // No valid response, set default ID
        memcpy(sensor_id, default_id, sizeof(default_id));
        ESP_LOGW(TAG, "No response received, setting default ID");
         printf("Sensor ID1: ");
        for (int i = 0; i < sizeof(default_id); i++) {
            printf("%02X ", sensor_id[i]);
        }
    }
}
//===================================================================================================================================================================================
// Task to monitor Wi-Fi and MQTT status
void wifi_monitor_task(void *pvParameters) {
	wifi_init_mqtt();
    while (1) {
        if (!wifi_connected) {
           ESP_LOGI(TAG, "Reconnecting to Wi-Fi...");
            esp_wifi_connect();
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}
//==================================================================================================================================================================================
// Task to publish messages to the MQTT topic
void mqtt_publish_task(void *pvParameters) {
    while (1) {
        if (mqtt_connected) {
			if(flag_uploading_wifi==1){
            combine_data_string();                                                                                     // Prepare JSON data
            if (strlen(combine_array) > 0) {
               // ESP_LOGI(TAG, "Publishing message to MQTT...");
                esp_err_t err = esp_mqtt_client_publish(mqtt_client, topic_tx, combine_array, strlen(combine_array), 2, 0);
                flag_uploading_wifi=0;
                if (err == ESP_OK) {
                  //  ESP_LOGI(TAG, "Published message successfully: %s", combine_array);

                } else {
               // ESP_LOGE(TAG, "Failed to publish message: %s", esp_err_to_name(err));
                }
            }
             else {
              // ESP_LOGW(TAG, "Combine array is empty, skipping MQTT publish.");
            }
            }
        } else {
          // ESP_LOGW(TAG, "MQTT not connected. Waiting...");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
//==================================================================================================================================================================================
void stop_mqtt_and_wifi() {
    if (mqtt_client) {
    	char disconnecting_msg[50];
    	snprintf(disconnecting_msg, sizeof(disconnecting_msg), "disconnecting (manual)");
        esp_mqtt_client_publish(mqtt_client, will_topic, disconnecting_msg, strlen(disconnecting_msg), 2, 1);
        ESP_LOGI("MQTT", "Stopping MQTT...");
        esp_mqtt_client_stop(mqtt_client);
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        mqtt_connected = false;
        ESP_LOGI("MQTT", "MQTT fully stopped.");
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // Ensure MQTT stops first
}
//==================================================================================================================================================================================
void time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI(TAG, "Time synchronization event received.");
}
//==================================================================================================================================================================================
// Initialize SNTP
void initialize_sntp() {
    if (esp_sntp_enabled()) {
        ESP_LOGW(TAG, "SNTP already initialized. Skipping initialization.");
        return;
    }
   // ESP_LOGI(TAG, "Initializing SNTP...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int max_retries = 10;
    while (timeinfo.tm_year < (2020 - 1900) && ++retry < max_retries) {
        ESP_LOGI(TAG, "Waiting for time sync... (%d/%d)", retry, max_retries);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        setenv("TZ", "IST-5:30", 1);                                                       // "IST-5:30" means UTC+5:30
        tzset();
        time(&now);
        localtime_r(&now, &timeinfo);
    }
    if (timeinfo.tm_year < (2020 - 1900)) {
        ESP_LOGW(TAG, "Time sync failed!");
    } else {
        // Set the timezone for India (IST = UTC +5:30)
        setenv("TZ", "IST-5:30", 1);                                                       // "IST-5:30" means UTC+5:30
        tzset();                                                                          // Apply the time zone settings
        // Get updated local time
        time(&now);
        localtime_r(&now, &timeinfo);
        ESP_LOGI(TAG, "Time synchronized: %s", asctime(&timeinfo));
    }
}
//==================================================================================================================================================================================
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t mqtt_client = event->client;
    int msg_id;
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            mqtt_connected = true;
            // Publish connected status
           // snprintf(will_topic, sizeof(will_topic), MQTT_WILL_TOPIC, mac_str);
            char connected_msg[50];
            snprintf(connected_msg, sizeof(connected_msg), "connected (IP: %s)", wifi_ip_address);
            esp_mqtt_client_publish(mqtt_client, will_topic, connected_msg, strlen(connected_msg), 2, 1);
            msg_id = esp_mqtt_client_subscribe(mqtt_client, topic_rx, 2);
            // Initialize SNTP only if not already running
            if (!esp_sntp_enabled()) {
                initialize_sntp();
            }
            //ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            mqtt_connected = false;
            break;
        case MQTT_EVENT_SUBSCRIBED:
           // ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
           // ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
          //  ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            break;
        case MQTT_EVENT_ERROR:
           // ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
           // ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}
//==================================================================================================================================================================================
static void print_coefficients() {
    ESP_LOGI(TAG, "Printing Updated Coefficients:");
    for (int t = 0; t < 16; t++) {
        printf("Coefficients for Target %d: {", t + 1);
        for (int i = 0; i < 5; i++) {
           // printf("{%.2f, %.2f}", Coeff[t][i], Coeff[t][i + 1]);
            printf("%.2f", Coeff[t][i]);
            if (i < 5 - 1) {
                printf(", ");
            }
        }
        printf("}\n");
    }
}
//==================================================================================================================================================================================
static void update_coefficients(const char *file_path) {
    update_config_from_file(file_path);
    print_coefficients();                                                                                          // Print updated coefficients
}
//==================================================================================================================================================================================
void update_config_from_file(const char *file_path) {
	ESP_LOGI(TAG, "File path_config: %s\n", file_path);
    FILE *file = fopen(file_path, "r");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open file: %s", file_path);
        return;
    }
    // Get the file size
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    rewind(file);
    if (file_size <= 0) {
        ESP_LOGE(TAG, "Invalid file size: %ld", file_size);
        fclose(file);
        return;
    }
    // Allocate memory to read the file content
    char *json_data = (char *)malloc(file_size + 1);
    if (!json_data) {
        ESP_LOGE(TAG, "Failed to allocate memory for file content");
        fclose(file);
        return;
    }
    // Read the file content
    fread(json_data, 1, file_size, file);
    json_data[file_size] = '\0'; // Null-terminate the string
    fclose(file);
    ESP_LOGI(TAG, "File content read successfully");
    // Parse the JSON data
    cJSON *root = cJSON_Parse(json_data);
    free(json_data); // Free memory after parsing
    if (!root) {
        ESP_LOGE(TAG, "Failed to parse JSON");
        return;
    }
    // Update INTERVAL
    cJSON *interval = cJSON_GetObjectItem(root, "INTERVAL");
    if (cJSON_IsNumber(interval)) {
        INTERVAL = interval->valueint;
        ESP_LOGI(TAG, "Updated INTERVAL: %d", INTERVAL);
    }
    // Update SEN_ID
    cJSON *sen_id = cJSON_GetObjectItem(root, "sen_id");
    if (cJSON_IsString(sen_id)) {
        strncpy(SEN_ID, sen_id->valuestring, sizeof(SEN_ID));
        ESP_LOGI(TAG, "Updated SEN_ID: %s", SEN_ID);
    }
    // Update USER
    cJSON *user = cJSON_GetObjectItem(root, "USER");
    if (cJSON_IsString(user)) {
        strncpy(USER, user->valuestring, sizeof(USER));
        ESP_LOGI(TAG, "Updated USER: %s", USER);
    }
    // Update Wi-Fi credentials (SSID and password)
    cJSON *wifi_ssid_json = cJSON_GetObjectItem(root, "wifi_ssid");
    cJSON *wifi_password_json = cJSON_GetObjectItem(root, "wifi_password");
    if (cJSON_IsString(wifi_ssid_json) && cJSON_IsString(wifi_password_json)) {
        strncpy(WIFI_SSID, wifi_ssid_json->valuestring, sizeof(WIFI_SSID) - 1);
        strncpy(WIFI_PASS, wifi_password_json->valuestring, sizeof(WIFI_PASS) - 1);
       ESP_LOGI(TAG, "Updated Wi-Fi SSID: %s", WIFI_SSID);
       ESP_LOGI(TAG, "Updated Wi-Fi Password: %s", WIFI_PASS);
    }
    // Update Temp_Coeff
        cJSON *temp_coeff = cJSON_GetObjectItem(root, "Temp_Coeff");
        if (cJSON_IsArray(temp_coeff) && cJSON_GetArraySize(temp_coeff) == 5) {
            for (int i = 0; i < 5; i++) {
                Temp_Coeff[i] = cJSON_GetArrayItem(temp_coeff, i)->valuedouble;
            }
            ESP_LOGI(TAG, "Updated Temp_Coeff: {%.2f, %.6f, %.2f, %.6f, %.3f}", Temp_Coeff[0], Temp_Coeff[1], Temp_Coeff[2], Temp_Coeff[3], Temp_Coeff[4]);
        }
        // Update Cond_Coeff
        cJSON *cond_coeff = cJSON_GetObjectItem(root, "Cond_Coeff");
        if (cJSON_IsArray(cond_coeff) && cJSON_GetArraySize(cond_coeff) == 6) {
            for (int i = 0; i < 6; i++) {
                Cond_Coeff[i] = cJSON_GetArrayItem(cond_coeff, i)->valuedouble;
            }
            ESP_LOGI(TAG, "Updated Cond_Coeff: {%.5f, %.5f, %.3f, %.3f, %.3f, %.3f}", Cond_Coeff[0], Cond_Coeff[1], Cond_Coeff[2], Cond_Coeff[3], Cond_Coeff[4], Cond_Coeff[5]);
        }
    // Update TDS_Coeff
    cJSON *tds_coeff = cJSON_GetObjectItem(root, "TDS_Coeff");
    if (cJSON_IsNumber(tds_coeff)) {
        TDS_Coeff = tds_coeff->valuedouble;
        ESP_LOGI(TAG, "Updated TDS_Coeff: %.2f", TDS_Coeff);
    }
    // Update MAX_ITERS
    cJSON *max_iters = cJSON_GetObjectItem(root, "MAX_ITERS");
    if (cJSON_IsNumber(max_iters)) {
        MAX_ITERS = max_iters->valueint;
        ESP_LOGI(TAG, "Updated MAX_ITERS: %d", MAX_ITERS);
    }
    // Update MAX_ITERS_LS
    cJSON *max_iters_ls = cJSON_GetObjectItem(root, "MAX_ITERS_LS");
    if (cJSON_IsNumber(max_iters_ls)) {
        MAX_ITERS_LS = max_iters_ls->valueint;
        ESP_LOGI(TAG, "Updated MAX_ITERS_LS: %d", MAX_ITERS_LS);
    }
    // Update MAX_TOL
    cJSON *max_tol = cJSON_GetObjectItem(root, "MAX_TOL");
    if (cJSON_IsNumber(max_tol)) {
        MAX_TOL = max_tol->valuedouble;
        ESP_LOGI(TAG, "Updated MAX_TOL: %.3f", MAX_TOL);
    }
    // Update Latitude
    cJSON *latitude = cJSON_GetObjectItem(root, "Latitude");
    if (cJSON_IsNumber(latitude)) {
       Latitude = latitude->valuedouble;
        ESP_LOGI(TAG, "Updated Latitude: %.6f", Latitude);
    }
    // Update Longitude
    cJSON *longitude = cJSON_GetObjectItem(root, "Longitude");
    if (cJSON_IsNumber(longitude)) {
       Longitude = longitude->valuedouble;
     ESP_LOGI(TAG, "Updated Longitude: %.6f", Longitude);
     }
    // Update Coeff (16x5 array)
    cJSON *coefficients = cJSON_GetObjectItem(root, "Coeff");
    if (coefficients && cJSON_IsArray(coefficients) && cJSON_GetArraySize(coefficients) == 16) {
        for (int t = 0; t < 16; t++) {
            cJSON *target = cJSON_GetArrayItem(coefficients, t);
            if (cJSON_IsArray(target) && cJSON_GetArraySize(target) == 5) {
                for (int i = 0; i < 5; i++) {
                    Coeff[t][i] = cJSON_GetArrayItem(target, i)->valuedouble;
                }
            }
        }
    }
    cJSON_Delete(root);
}
//================================================================================================================================================================================
void interval_uploading(){
        static unsigned long lastMsg = 0;
        unsigned long now = esp_log_timestamp();
        if(INTERVAL!=0){
        if (now - lastMsg > INTERVAL*1000*60) {
            lastMsg = now;
            flag_uploading=1;
			flag_uploading_wifi=1;
			flag_uploading_ble=1;
			flag_uploading_uart2=1;
            flag_uploading_internal_data_save=1;
        }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
}
//=================================================================================================================================================================================
static esp_err_t index_html_get_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "index_html_get_handler called for URI: %s", req->uri);
    httpd_resp_set_status(req, "307 Temporary Redirect");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);                                                                              //Response body can be empty
    return ESP_OK;
}
//===================================================================================================================================================================================
static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    extern const unsigned char favicon_ico_start[] asm("_binary_favicon_ico_start");
    extern const unsigned char favicon_ico_end[]   asm("_binary_favicon_ico_end");
    const size_t favicon_ico_size = (favicon_ico_end - favicon_ico_start);
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_send(req, (const char *)favicon_ico_start, favicon_ico_size);
    return ESP_OK;
}
//==================================================================================================================================================================================
static esp_err_t http_resp_dir_html(httpd_req_t *req, const char *dirpath)
{
    char entrypath[FILE_PATH_MAX];
    char entrysize[16];
    const char *entrytype;
    struct dirent *entry;
    struct stat entry_stat;
    DIR *dir = opendir(dirpath);
    const size_t dirpath_len = strlen(dirpath);
    strlcpy(entrypath, dirpath, sizeof(entrypath));                                                        //Retrieve the base path of file storage to construct the full path
    if (!dir) {
        ESP_LOGE(TAG, "Failed to stat dir : %s", dirpath);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Directory does not exist");                        //Respond with 404 Not Found
        return ESP_FAIL;
    }
    httpd_resp_sendstr_chunk(req, "<!DOCTYPE html><html><body>");                                          //Send HTML file header
    /* Get handle to embedded file upload script */
    extern const unsigned char upload_script_start[] asm("_binary_upload_script_html_start");
    extern const unsigned char upload_script_end[]   asm("_binary_upload_script_html_end");
    const size_t upload_script_size = (upload_script_end - upload_script_start);
    httpd_resp_send_chunk(req, (const char *)upload_script_start, upload_script_size);                    //Add file upload form and script which on execution sends a POST request to /upload
    /*Send file-list table definition and column labels */
    httpd_resp_sendstr_chunk(req,
        "<table class=\"fixed\" border=\"1\">"
        "<col width=\"800px\" /><col width=\"300px\" /><col width=\"300px\" /><col width=\"100px\" />"
        "<thead><tr><th>Name</th><th>Type</th><th>Size (Bytes)</th><th>Delete</th></tr></thead>"
        "<tbody>");
    /* Iterate over all files / folders and fetch their names and sizes */
    while ((entry = readdir(dir)) != NULL) {
        entrytype = (entry->d_type == DT_DIR ? "directory" : "file");
        strlcpy(entrypath + dirpath_len, entry->d_name, sizeof(entrypath) - dirpath_len);
        if (stat(entrypath, &entry_stat) == -1) {
            ESP_LOGE(TAG, "Failed to stat %s : %s", entrytype, entry->d_name);
            continue;
        }
        sprintf(entrysize, "%ld", entry_stat.st_size);
        ESP_LOGI(TAG, "Found %s : %s (%s bytes)", entrytype, entry->d_name, entrysize);
        /* Send chunk of HTML file containing table entries with file name and size */
        httpd_resp_sendstr_chunk(req, "<tr><td><a href=\"");
        httpd_resp_sendstr_chunk(req, req->uri);
        httpd_resp_sendstr_chunk(req, entry->d_name);
        if (entry->d_type == DT_DIR) {
            httpd_resp_sendstr_chunk(req, "/");
        }
        httpd_resp_sendstr_chunk(req, "\">");
        httpd_resp_sendstr_chunk(req, entry->d_name);
        httpd_resp_sendstr_chunk(req, "</a></td><td>");
        httpd_resp_sendstr_chunk(req, entrytype);
        httpd_resp_sendstr_chunk(req, "</td><td>");
        httpd_resp_sendstr_chunk(req, entrysize);
        httpd_resp_sendstr_chunk(req, "</td><td>");
        httpd_resp_sendstr_chunk(req, "<form method=\"post\" action=\"/delete");
        httpd_resp_sendstr_chunk(req, req->uri);
        httpd_resp_sendstr_chunk(req, entry->d_name);
        httpd_resp_sendstr_chunk(req, "\"><button type=\"submit\">Delete</button></form>");
        httpd_resp_sendstr_chunk(req, "</td></tr>\n");
    }
    closedir(dir);
    httpd_resp_sendstr_chunk(req, "</tbody></table>");                                                        //Finish the file list table
    httpd_resp_sendstr_chunk(req, "</body></html>");                                                         //Send remaining chunk of HTML file to complete it
    httpd_resp_sendstr_chunk(req, NULL);                                                                     //Send empty chunk to signal HTTP response completion
    return ESP_OK;
}
#define IS_FILE_EXT(filename, ext) \
    (strcasecmp(&filename[strlen(filename) - sizeof(ext) + 1], ext) == 0)
//==================================================================================================================================================================================
/* Set HTTP response content type according to file extension */
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename)
{
    if (IS_FILE_EXT(filename, ".pdf")) {
        return httpd_resp_set_type(req, "application/pdf");
    } else if (IS_FILE_EXT(filename, ".html")) {
        return httpd_resp_set_type(req, "text/html");
    } else if (IS_FILE_EXT(filename, ".jpeg")) {
        return httpd_resp_set_type(req, "image/jpeg");
    } else if (IS_FILE_EXT(filename, ".ico")) {
        return httpd_resp_set_type(req, "image/x-icon");
    } 
    return httpd_resp_set_type(req, "text/plain");                                                              //For any other type always set as plain text
}
//==================================================================================================================================================================================
/*Copies the full path into destination buffer and returns pointer to path (skipping the preceding base path)*/
static const char* get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize)
{
    const size_t base_pathlen = strlen(base_path);
    size_t pathlen = strlen(uri);
    const char *quest = strchr(uri, '?');
    if (quest) {
        pathlen = MIN(pathlen, quest - uri);
    }
    const char *hash = strchr(uri, '#');
    if (hash) {
        pathlen = MIN(pathlen, hash - uri);
    }
    if (base_pathlen + pathlen + 1 > destsize) {  
        return NULL;                                                                                          //Full path string won't fit into destination buffer
    }
    strcpy(dest, base_path);                                                                                 //Construct full path (base + path)
    strlcpy(dest + base_pathlen, uri, pathlen + 1);    
    return dest + base_pathlen;                                                                              //Return pointer to path, skipping the base
}
//===================================================================================================================================================================================
/* Handler to download a file kept on the server */
static esp_err_t download_get_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    FILE *fd = NULL;
    struct stat file_stat;
    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,req->uri, sizeof(filepath));
    if (!filename) {
        ESP_LOGE(TAG, "Filename is too long");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");                         //Respond with 500 Internal Server Error
        return ESP_FAIL;
    }
    if (filename[strlen(filename) - 1] == '/') {                                                               //If name has trailing '/', respond with directory contents
        return http_resp_dir_html(req, filepath);
    }
    if (stat(filepath, &file_stat) == -1) {   
        if (strcmp(filename, "/index.html") == 0) {                                                           //If file not present on SPIFFS check if URI corresponds to one of the hardcoded paths
            return index_html_get_handler(req);
        } else if (strcmp(filename, "/favicon.ico") == 0) {
            return favicon_get_handler(req);
        }
      ESP_LOGE(TAG, "Failed to stat file : %s", filepath);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");                                 //Respond with 404 Not Found
        return ESP_FAIL;
    }
    fd = fopen(filepath, "r");
    if (!fd) {
      ESP_LOGE(TAG, "Failed to read existing file : %s", filepath);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");            //Respond with 500 Internal Server Error
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Sending file : %s (%ld bytes)...", filename, file_stat.st_size);
    set_content_type_from_file(req, filename);
    /* Set response headers to force file download */
    httpd_resp_set_type(req, "application/octet-stream");  // Binary file type
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"combined_data.txt\"");
    /* Retrieve the pointer to scratch buffer for temporary storage */
    char *chunk = ((struct file_server_data *)req->user_ctx)->scratch;
    size_t chunksize;
    do {      
        chunksize = fread(chunk, 1, SCRATCH_BUFSIZE, fd);                                                    //Read file in chunks into the scratch buffer
        if (chunksize > 0) {       
            if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK) {                                    //Send the buffer contents as HTTP response chunk
                fclose(fd);
                ESP_LOGE(TAG, "File sending failed!");
                httpd_resp_sendstr_chunk(req, NULL);                                                         //Abort sending file               
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");            //Respond with 500 Internal Server Error
               return ESP_FAIL;
           }
        }     
    } while (chunksize != 0);                                                                                //Keep looping till the whole file is sent
    fclose(fd);                                                                                              //Close file after sending complete
   ESP_LOGI(TAG, "File sending complete");
    /* Respond with an empty chunk to signal HTTP response completion */
#ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
    httpd_resp_set_hdr(req, "Connection", "close");
#endif
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}
//==================================================================================================================================================================================
/* Handler to upload a file onto the server */
static esp_err_t upload_post_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    FILE *fd = NULL;
    struct stat file_stat;
    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,req->uri + sizeof("/upload") - 1, sizeof(filepath));                                             
    if (!filename) {      
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");                                 //Respond with 500 Internal Server Error
        return ESP_FAIL;
    }
   /* Filename cannot have a trailing '/' */
    if (filename[strlen(filename) - 1] == '/') {
      ESP_LOGE(TAG, "Invalid filename : %s", filename);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid filename");
        return ESP_FAIL;
    }
    if (stat(filepath, &file_stat) == 0) {
       ESP_LOGE(TAG, "File already exists : %s", filepath);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File already exists");                                          //Respond with 400 Bad Request
        return ESP_FAIL;
    }
    /* File cannot be larger than a limit */
    if (req->content_len > MAX_FILE_SIZE) {
     ESP_LOGE(TAG, "File too large : %d bytes", req->content_len);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File size must be less than " MAX_FILE_SIZE_STR "!");            //Respond with 400 Bad Request
        return ESP_FAIL;
    }
    fd = fopen(filepath, "w");
    if (!fd) {
       ESP_LOGE(TAG, "Failed to create file : %s", filepath);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to create file");                                //Respond with 500 Internal Server Error
        return ESP_FAIL;
    }
   ESP_LOGI(TAG, "Receiving file : %s...", filename);
    char *buf = ((struct file_server_data *)req->user_ctx)->scratch;                                                      //Retrieve the pointer to scratch buffer for temporary storage
    int received;
    /*Content length of the request gives the size of the file being uploaded */    
    int remaining = req->content_len;
    while (remaining > 0) {
        ESP_LOGI(TAG, "Remaining size : %d", remaining);
        if ((received = httpd_req_recv(req, buf, MIN(remaining, SCRATCH_BUFSIZE))) <= 0) {                                //Receive the file part by part into a buffer
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {           
                continue;                                                                                                 //Retry if timeout occurred
            }                   
            fclose(fd);                                                                                                  //In case of unrecoverable error, close and delete the unfinished file
            unlink(filepath);
           ESP_LOGE(TAG, "File reception failed!");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive file");                        //Respond with 500 Internal Server Error
            return ESP_FAIL;
        }     
        if (received && (received != fwrite(buf, 1, received, fd))) {                                                   //Write buffer content to file on storage          
            fclose(fd);                                                                                                 //Couldn't write everything to file!Storage may be full?
            unlink(filepath);
           ESP_LOGE(TAG, "File write failed!");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to write file to storage");               //Respond with 500 Internal Server Error
            return ESP_FAIL;
        }        
        remaining -= received;                                                                                          //Keep track of remaining size of  the file left to be uploaded
    }    
    fclose(fd);                                                                                                         //Close file upon upload completion
   ESP_LOGI(TAG, "File reception complete");
    /* Redirect onto root to see the updated file list */
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
#ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
    httpd_resp_set_hdr(req, "Connection", "close");
#endif
    httpd_resp_sendstr(req, "File uploaded successfully");  
    update_coefficients(filepath);
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_sendstr(req, "File uploaded and coefficients updated successfully.");
    return ESP_OK;
}
//===================================================================================================================================================================================
/* Handler to delete a file from the server */
static esp_err_t delete_post_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    struct stat file_stat;
    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,
                                             req->uri  + sizeof("/delete") - 1, sizeof(filepath));
    if (!filename) {       
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");                            //Respond with 500 Internal Server Error
        return ESP_FAIL;
    }
    /* Filename cannot have a trailing '/' */
    if (filename[strlen(filename) - 1] == '/') {
     ESP_LOGE(TAG, "Invalid filename : %s", filename);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid filename");
        return ESP_FAIL;
    }
    if (stat(filepath, &file_stat) == -1) {
       ESP_LOGE(TAG, "File does not exist : %s", filename);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File does not exist");                                    //Respond with 400 Bad Request
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Deleting file : %s", filename);
    unlink(filepath);                                            
    /* Redirect onto root to see the updated file list */
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
#ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
    httpd_resp_set_hdr(req, "Connection", "close");
#endif
    httpd_resp_sendstr(req, "File deleted successfully");
    return ESP_OK;
}
//==================================================================================================================================================================================
/* Function to start the file server */
esp_err_t example_start_file_server(const char *base_path)
{
    static struct file_server_data *server_data = NULL;
    if (server_data) {
       ESP_LOGE(TAG, "File server already started");
        return ESP_ERR_INVALID_STATE;
    }
    /* Allocate memory for server data */
    server_data = calloc(1, sizeof(struct file_server_data));
    if (!server_data) {
       //ESP_LOGE(TAG, "Failed to allocate memory for server data");
        return ESP_ERR_NO_MEM;
    }
    strlcpy(server_data->base_path, base_path,sizeof(server_data->base_path));           
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    // Increase send wait timeout
    config.send_wait_timeout = 30;
   ESP_LOGI(TAG, "Starting HTTP Server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) != ESP_OK) {
       ESP_LOGE(TAG, "Failed to start file server!");
        return ESP_FAIL;
    }
   ESP_LOGI(TAG, "HTTP server started successfully");
    /* URI handler for getting uploaded files */
    httpd_uri_t file_download = {
        .uri       = "/*",                                                                                    //Match all URIs of type /path/to/file
        .method    = HTTP_GET,
        .handler   = download_get_handler,
        .user_ctx  = server_data                                                                             //Pass server data as context
    };
    httpd_register_uri_handler(server, &file_download);
    /* URI handler for uploading files to server */
    httpd_uri_t file_upload = {
        .uri       = "/upload/*",                                                                           //Match all URIs of type /upload/path/to/file
        .method    = HTTP_POST,
        .handler   = upload_post_handler,
        .user_ctx  = server_data                                                                            //Pass server data as context
    };
    httpd_register_uri_handler(server, &file_upload);
    /* URI handler for deleting files from server */
    httpd_uri_t file_delete = {
        .uri       = "/delete/*",                                                                           //Match all URIs of type /delete/path/to/file
        .method    = HTTP_POST,
        .handler   = delete_post_handler,
        .user_ctx  = server_data                                                                            //Pass server data as context
    };
    httpd_register_uri_handler(server, &file_delete);
    return ESP_OK;
}
//=================================================================================================================================================================================
/*void combine_data_string2(){
char combine_result[100];
convertDoubleArrayToString(responseArray,20,combine_array_ble);
if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
strcat(combine_array_ble, ",");
 xSemaphoreGive(xMutex);
    }
convertDoubleArrayToString(Result,5,combine_result);
if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
strcat(combine_array_ble, combine_result);
 xSemaphoreGive(xMutex);
    }
}*/
void combine_data_string2() {
    char combine_result[512];
    // Clear BLE buffer before combining
    combine_array_ble[0] = '\0';
    // Client mode: Add responseArray + Result
    if (strcmp(USER, "client") == 0) {
        convertDoubleArrayToString(responseArray, 20, combine_array_ble);  // Adds d1 to d20
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            strcat(combine_array_ble, ",");  // Add comma between d20 and d21
            xSemaphoreGive(xMutex);
        }
    }
    // Add d21 to d25 (Result values)
    convertDoubleArrayToString(Result, 5, combine_result);
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        if (strlen(combine_array_ble) > 0) {
            strcat(combine_array_ble, ",");  // Ensure proper separator
        }
        strcat(combine_array_ble, combine_result);  // Append result values
        xSemaphoreGive(xMutex);
    }
}
//==================================================================================================================================================================================
void combine_data_string1() {
    char formatted_data1[4096] = {0};
    strcat(formatted_data1, "{");
    // Define the keys for responseArray and Result
    const char *keys[] = {
        "R11", "R12", "R13", "R14", "R21", "R22", "R23", "R24",
        "R31", "R32", "R33", "R34", "T1", "T2", "C1", "C2",
        "R11_2W", "R12_2W", "R33_2W", "R34_2W",
        "CL", "PH", "CND", "TDS", "TEMP"
    };
    // Add the responseArray (d1 to d20) data
    for (int i = 0; i < 20; i++) {
        char entry[128];
        snprintf(entry, sizeof(entry), "\"%s\":\"%.5f\"", keys[i], responseArray[i]);
        strcat(formatted_data1, entry);
        strcat(formatted_data1, ",");
    }
    // Add the Result (d21 to d25) data
    for (int i = 0; i < 5; i++) {
        char entry[128];
        snprintf(entry, sizeof(entry), "\"%s\":\"%.5f\"", keys[i + 20], Result[i]);
        strcat(formatted_data1, entry);
        if (i < 4) {
            strcat(formatted_data1, ",");
        }
    }
    strcat(formatted_data1, "}"); // Close the JSON object
     if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    strcpy(combine_array1, formatted_data1); // Copy the result to combine_array
    xSemaphoreGive(xMutex);
    }
   //ESP_LOGI(TAG, "Formatted JSON: %s", combine_array1);
}
//==================================================================================================================================================================================
static uint16_t conn_handle = 0;  // Store the connection handle
void ble_notify_task(void *param) {
    while (1) {
    	 //ESP_LOGI(TAG, "Flag state: %d", flag_uploading_ble);
        if (flag_uploading_ble == 1) {
        	ESP_LOGI(TAG, "Flag state: %d", flag_uploading_ble);
        	combine_data_string2();
            struct os_mbuf *om = ble_hs_mbuf_from_flat(combine_array_ble, strlen(combine_array_ble));
            if (om) {
                int rc = ble_gatts_notify_custom(conn_handle, tx_handle, om);
                flag_uploading_ble = 0;  // Reset flag after sending
                if (rc == 0) {
                    ESP_LOGI(TAG, "Auto Notification Sent: %s", combine_array_ble);

                } else {
                    ESP_LOGE(TAG, "Failed to send notification: %d", rc);
                }
            }
        }
        vTaskDelay(1000/ portTICK_PERIOD_MS);  // Avoid busy looping
    }
}
//=================================================================================================================================================================================
/*GATT Service Access Callback for Receiving and Sending Data Over BLE */
static int gatt_svr_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        /*Handle write operation (data sent from BLE client to RX characteristic) */
      //ESP_LOGI(TAG, "Received data over BLE: %.*s", ctxt->om->om_len, ctxt->om->om_data);
       /*Process received data */
        char buffer[64] = {0};
        if (ctxt->om->om_len > 0 && ctxt->om->om_len < sizeof(buffer)) {
            strncpy(buffer, (char *)ctxt->om->om_data, ctxt->om->om_len);
            //ESP_LOGI(TAG, "Received command: %s", buffer);
    if (strchr(buffer, 'a')) {
            //ESP_LOGI(TAG, "Step1: Buffer contains 'a'");
              combine_data_string2();
           //Send `combine_array` as a notification to the client
            struct os_mbuf *om = ble_hs_mbuf_from_flat(combine_array_ble, strlen(combine_array_ble));
            if (om) {
                int rc = ble_gatts_notify_custom(conn_handle, tx_handle, om);
                if (rc == 0) {
                  //  ESP_LOGI(TAG, "Notification sent to BLE client: %s", combine_array_ble);
                } else {
                  // ESP_LOGE(TAG, "Failed to send notification: %d", rc);
                }
            }
        }
       if (strchr(buffer, 'b')) {
          //ESP_LOGI(TAG, "Step2: Buffer contains 'b'");
           flag_serverstart=1;
        }
        } else {
           //ESP_LOGW(TAG, "Invalid or oversized BLE write data.");
        }
    } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
       /*Handle read operation (send data from server to BLE client)*/
        os_mbuf_append(ctxt->om, combine_array_ble, strlen(combine_array_ble));
      //ESP_LOGI(TAG, "Sent read response: %s", combine_array_ble);
    }
    return 0;
}
//==================================================================================================================================================================================
/*GATT Service Definition*/
static struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (ble_uuid_t *)&service_uuid,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = (ble_uuid_t *)&char_uuid_rx,                                                         //RX Characteristic: Write
                .access_cb = gatt_svr_chr_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {             
                .uuid = (ble_uuid_t *)&char_uuid_tx,                                                         //TX Characteristic: Notify
                .access_cb = gatt_svr_chr_access_cb,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &tx_handle,                                                                    //Store the handle for notifications
            },
            {0},                                                                                             //End of characteristics
        },
    },
    {0},                                                                                                    //End of services
};
//==================================================================================================================================================================================
/*BLE GATT Service Registration*/
static void gatt_svr_register_svcs(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    int rc;
    rc = ble_gatts_count_cfg(gatt_svr_svcs);                                                                 //Count configuration
    if (rc != 0) {
       //ESP_LOGE(TAG, "Failed to count GATT services, rc=%d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(gatt_svr_svcs);                                                                  //Add services
    if (rc != 0) {
       //ESP_LOGE(TAG, "Failed to add GATT services, rc=%d", rc);
        return;
    }
   //ESP_LOGI(TAG, "GATT Services Registered");
    vTaskDelay(100/ portTICK_PERIOD_MS);                                   
}
//==================================================================================================================================================================================
/*BLE event handling*/
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
              //ESP_LOGI(TAG, "Connected to client.");
            } else {
               //ESP_LOGI(TAG, "Connection failed; restarting advertising.");
                ble_app_advertise();
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
           //ESP_LOGI(TAG, "Disconnected from client; restarting advertising.");
            ble_app_advertise();
            break;
        case BLE_GAP_EVENT_ADV_COMPLETE:
          //ESP_LOGI(TAG, "Advertising completed; restarting.");
            ble_app_advertise();
            break;
        default:
            break;
    }
    return 0;
}
//===================================================================================================================================================================================
/*BLE Host Task*/
static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();       //Run NimBLE event loop (BLE tasks)
}
//===================================================================================================================================================================================
/*BLE Advertising Configuration Callback*/
static void ble_app_on_sync(void) {
    uint8_t addr_val[6] = {0};
    ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr_val, NULL);
    ESP_LOGI(TAG, "Device address: %02x:%02x:%02x:%02x:%02x:%02x",
             addr_val[5], addr_val[4], addr_val[3],
             addr_val[2], addr_val[1], addr_val[0]);
    ble_app_advertise();
  }
//==================================================================================================================================================================================
void ble_app_advertise(void) {
    struct ble_hs_adv_fields adv_fields;
    memset(&adv_fields, 0, sizeof(adv_fields));
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;                                     //Add general discoverability and no BR/EDR support    
    adv_fields.uuids128 = (ble_uuid128_t[]){                                                                 //Add Nordic UART Service UUID (128-bit)
        BLE_UUID128_INIT(0x6E, 0x40, 0x00, 0x01, 0xB5, 0xA3, 0xF3, 0x93,
                         0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E)};
    adv_fields.num_uuids128 = 1;
    adv_fields.uuids128_is_complete = 1;
    adv_fields.name = (uint8_t *)"BLE_UART";                                                                 //Add a complete local name
    adv_fields.name_len = strlen((char *)adv_fields.name);
    adv_fields.name_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&adv_fields);                                                            //Set advertising fields
    if (rc != 0) {
       //ESP_LOGE(TAG, "Failed to set advertising fields, rc=%d", rc);
        return;
    }
   /*Advertising parameters*/
    struct ble_gap_adv_params adv_params = {0};
    adv_params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN; // 30ms
    adv_params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MAX; // 60ms
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;         // Undirected connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;         // General discoverable
    /*Start advertising*/
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
       //ESP_LOGE(TAG, "Failed to start advertising, rc=%d", rc);
    } else {
       //ESP_LOGI(TAG, "Advertising started successfully with NUS UUID.");
    }
}
//===================================================================================================================================================================================
/*BLE Initialization*/
void ble_init() {
    ble_svc_gap_init();
    ble_svc_gatt_init();  
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    gatt_svr_register_svcs(NULL, NULL);
    nimble_port_freertos_init(ble_host_task);
}
//==================================================================================================================================================================================
/*UART Initialization*/
void uart_init() {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TXD1_PIN, UART_RXD1_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
   uart_driver_install(UART_NUM, BUF_SIZE * 2,0 ,0,NULL,0);
   ESP_LOGI(TAG, "UART1 Initialized");
    uart_config_t uart_config2 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM_UART2, &uart_config2);
    uart_set_pin(UART_NUM_UART2, UART_TXD2_PIN, UART_RXD2_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_UART2, BUF_SIZE * 2, 0, 0, NULL, 0);
    ESP_LOGI(TAG, "UART2 Initialized");
}
//==================================================================================================================================================================================
void update_system_time(const char *time_str) {
    struct tm tm = {0};                                                             //Ensure all fields are initialized to zero
    char formatted_time[20];                                                        //Buffer to hold formatted time
    snprintf(formatted_time, sizeof(formatted_time), "%s %s", strtok((char *)time_str, "T"), strtok(NULL, "T"));                    // Convert "2025-02-27T12:21:05" to "2025-02-27 12:21:05"
    strptime(formatted_time, "%Y-%m-%d %H:%M:%S", &tm);
    time_t t = mktime(&tm);
    if (t == -1) {
        ESP_LOGE(TAG, "Failed to convert time string to time_t");
        return;
    }
    struct timeval now = {.tv_sec = t, .tv_usec = 0};
    settimeofday(&now, NULL);
    ESP_LOGI(TAG, "System time updated: %s", formatted_time);
}
//==================================================================================================================================================================================
void send_time_request() {
    ESP_LOGI(TAG, "Sending 'T' command to request time");
    uart_write_bytes(UART_NUM_UART2, "T", 1);
    uint8_t data[BUF_SIZE];
        int len = uart_read_bytes(UART_NUM_UART2, data, BUF_SIZE, 2000 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI(TAG, "Received Time: %s", data);
                update_system_time((char *)data);
        }else {
            ESP_LOGW(TAG, "No response received for time request");
        }
}
//==================================================================================================================================================================================
void upload_everymin_uart2() {
    if(flag_uploading_uart2 == 1) {
        combine_data_string2();
        if (strlen(combine_array_ble) > 0) {
            uart_write_bytes(UART_NUM_UART2, "S", 1);
            uart_write_bytes(UART_NUM_UART2, combine_array_ble, strlen(combine_array_ble));
             uart_write_bytes(UART_NUM_UART2, "E", 1);
            //uart_write_bytes(UART_NUM_UART2, "\r\n", 2);
            ESP_LOGI(TAG, "Sent data over UART2: %s", combine_array_ble);
            flag_uploading_uart2 = 0;
        } else {
            ESP_LOGW(TAG, "No valid data to send.");
        }
    }
}
//=================================================================================================================================================================================
// Add this function to periodically send combined data to UART2
void send_data_to_uart2_task(void *param) {
	  send_time_request();
     while (1) {
        upload_everymin_uart2();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
//=================================================================================================================================================================================
unsigned long millis(void) {
    uint64_t time_in_us = esp_timer_get_time();                                                       //Get time in microseconds
    return (unsigned long)(time_in_us / 1000);                                                        //Convert microseconds to milliseconds
}
//=================================================================================================================================================================================
void send_gsm_command1(const char *command, const char *expected_response, int timeout_ms) {
    uart_write_bytes(UART_NUM_UART2, command, strlen(command));
    uart_write_bytes(UART_NUM_UART2, "\r\n", 2);
    char response[1024] = {0};
	char full_res[1024];
	unsigned long last_time=millis();
	while((millis()-last_time)<timeout_ms){
    uart_read_bytes(UART_NUM_UART2, response, sizeof(response) - 1, pdMS_TO_TICKS(1000));
	strcat(full_res, response);
	 if (strstr(full_res, expected_response)) {
            ESP_LOGI(TAG, "Expected response received.");
			break;
        }
        else {
            //ESP_LOGW(TAG, "Unexpected response.");
        }
	}
}
//==================================================================================================================================================================================
/*Function to send a command over GSM and read the response*/
int send_gsm_command(const char *command, const char *expected_response, int timeout_ms) {
    uart_write_bytes(UART_NUM_UART2, command, strlen(command));
    uart_write_bytes(UART_NUM_UART2, "\r\n", 2);
    uint8_t response[BUF_SIZE] = {0};
    int len = uart_read_bytes(UART_NUM_UART2, response, sizeof(response) - 1, pdMS_TO_TICKS(timeout_ms));
    if (len > 0) {
        response[len] = '\0';
        ESP_LOGI(TAG, "GSM Response: %s", response);
        if (strstr((char *)response, expected_response)) {
            ESP_LOGI(TAG, "Expected response received.");
            return 1;
        } else {
            //ESP_LOGW(TAG, "Unexpected response.");
            return 0;
        }
    } else {
        ESP_LOGE(TAG, "No response or timeout.");
         return 0;
    }
}
//==================================================================================================================================================================================
/*Function to initialize GSM*/
void gsm_init() {
    const char *init_commands[] = {
		"AT+QMTCLOSE=0",
        "ATI",
        "AT+CFUN=1,1",
        "AT+CPIN?",
        "AT+CFUN?",
        "AT+CREG?",
        "AT+CGREG?",
        "AT+COPS?",
        "AT+CSQ",
        "AT+QICSGP=1,1,\"airtelgprs.com\"",
        "AT+QIACT?",
        "AT+QFLST=\"UFS:*\"",
        "AT+QMTCFG=\"SSL\",0,1,2",
        "AT+QMTCFG=\"recv/mode\",0,1,0",
        "AT+QSSLCFG=\"seclevel\",2,0",
        "AT+QSSLCFG=\"sslversion\",2,4",
        "AT+QSSLCFG=\"ciphersuite\",2,0XFFFF",
        "AT+QSSLCFG=\"ignorelocaltime\",0,1",
        "AT+QMTCFG=\"VERSION\",0,4",
    };
    for (int i = 0; i < sizeof(init_commands) / sizeof(init_commands[0]); i++) {
       ESP_LOGI("GSM", "Sending command: %s", init_commands[i]);
       send_gsm_command1(init_commands[i], "OK",3000);
    }
}
//==================================================================================================================================================================================
int connect_to_mqtt() {
    char response[100];
    int len;
    ESP_LOGI(TAG, "statusmqtt: %d", status);
    send_gsm_command1("AT+QMTCLOSE=0", "OK", 3000);                                                     //Step 1: Close any existing connection
    ESP_LOGI(TAG, "Closed existing MQTT connection");
    send_gsm_command1("AT+QMTOPEN=0,\"otplai.com\",8883", "+QMTOPEN:", 8000);                           //Step 2: Open a new connection
    ESP_LOGI(TAG, "Opened new MQTT connection");
	uart_write_bytes(UART_NUM_UART2, "AT+QMTCONN=0,\"864218065369138\",\"oyt\",\"123456789\"", strlen("AT+QMTCONN=0,\"864218065369138\",\"oyt\",\"123456789\""));                // Step 3: Connect to the MQTT broker
    uart_write_bytes(UART_NUM_UART2, "\r\n", 2);
    len = uart_read_bytes(UART_NUM_UART2, response, sizeof(response) - 1, 5000 / portTICK_PERIOD_MS);                                 // Step 4: Read response from GSM module
    //ESP_LOGE(TAG, "response_mqtt:%s--%d",response,len);
    if (len > 0) {
        response[len] = '\0';
        if (strstr(response, "+QMTCONN:")) {
            ESP_LOGI(TAG, "MQTT Connection successful");
            return 1;
        }
    }
    ESP_LOGE(TAG, "MQTT Connection failed");
    return 0;
}
//=================================================================================================================================================================================
void subscribe_to_mqtt() {
    char command[128];
    snprintf(command, sizeof(command), "AT+QMTSUB=0,1,\"%s\",2", topic_rx);
    send_gsm_command(command, "+QMTSUB: 0,1,0", 2000);
}
//=================================================================================================================================================================================
void fetch_mqtt_message(int client_id, int recv_id) {
    char command[64];
    snprintf(command, sizeof(command), "AT+QMTRECV=%d,%d", client_id, recv_id);
   // ESP_LOGI(TAG, "Sending command: %s", command);
    uart_write_bytes(UART_NUM_UART2, command, strlen(command));
    uart_write_bytes(UART_NUM_UART2, "\r\n", 2);                                                                //Append CRLF to the command
    uint8_t buffer[BUF_SIZE] = {0};
    int len = uart_read_bytes(UART_NUM_UART2, buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(5000));
 //   ESP_LOGI(TAG, "Data length received2: %d", len);
    if (len > 0) {
        buffer[len] = '\0';                                                                                    //Null-terminate the received data
     //   ESP_LOGI(TAG, "MQTT Message Response: %s", buffer);
        // Parse the response for the topic and message
        const char *start = strstr((char *)buffer, "+QMTRECV: ");
        if (start) {
            // Locate the topic
            start = strchr(start, '"');
            if (start) {
                start++;                                                                                        //Skip the first quote
                char *topic_end = strchr(start, '"');
                if (topic_end) {
                    *topic_end = '\0';                                                                         //Null-terminate the topic
               //    ESP_LOGI(TAG, "Topic: %s", start);
                    // Locate the message
                    char *message_start = strchr(topic_end + 1, '"');
                    if (message_start) {
                        message_start++;                                                                       //Skip the first quote of the message
                        char *message_end = strchr(message_start, '"');
                        if (message_end) {
                            *message_end = '\0';                                                               //Null-terminate the message
                         //   ESP_LOGI(TAG, "Message: %s", message_start);
                             memcpy(buff_rec_data, message_start, sizeof(message_start));
                        //    ESP_LOGI(TAG, "Message_new: %s", buff_rec_data);
                        } else {
                       //     ESP_LOGE(TAG, "Failed to parse the message.");
                        }
                    } else {
                   //     ESP_LOGE(TAG, "Message start not found.");
                    }
                } else {
                  //  ESP_LOGE(TAG, "Topic end not found.");
                }
            } else {
              //  ESP_LOGE(TAG, "Topic start not found.");
            }
        } else {
          //  ESP_LOGE(TAG, "Expected '+QMTRECV:' response not found.");
        }
    } else {
       // ESP_LOGE(TAG, "Failed to fetch MQTT message or timeout.");
    }
}
//==================================================================================================================================================================================
int publish_to_mqtt(const char *topic_tx, const char *buff_rec_data) {
    char command[256];
    char response[1024];
    int len = 0;
    snprintf(command, sizeof(command), "AT+QMTPUBEX=0,1,2,0,\"%s\",%d", topic_tx, (int)strlen(buff_rec_data));
    // Send the publish command and check for ">" prompt
    if (!send_gsm_command(command, ">", 5000)) {
        ESP_LOGE(TAG, "MQTT publish command failed.");
        return 0;
    }
    // Send payload data
    uart_write_bytes(UART_NUM_UART2, buff_rec_data, strlen(buff_rec_data));
    uart_write_bytes(UART_NUM_UART2, "\r\n", 2);
    // Wait and read response from UART
    len = uart_read_bytes(UART_NUM_UART2, response, sizeof(response) - 1, 5000 / portTICK_PERIOD_MS);
    ESP_LOGE(TAG, "response_data: %s", response);
    if (len > 0) {
        response[len] = '\0';
        if (strstr(response, "OK") || strstr(response, "+QMTPUBEX:")) {
            ESP_LOGI(TAG, "Published to topic: %s, payload: %s", topic_tx, buff_rec_data);
            return 1;
        }
    }
    ESP_LOGE(TAG, "Failed to publish to topic: %s", topic_tx);
    return 0;
}
//==================================================================================================================================================================================
void convertDoubleArrayToString(const double *arr, int size, char *result) {
    char temp[1024];                                                                                    //Temporary buffer for each number
    result[0] = '\0';                                                                                   //Initialize the result string as empty
    for (int i = 0; i < size; i++) {
       /*Convert each double to a string and append it to the result*/
        sprintf(temp, "%.5f", arr[i]);                                                                  //Format the double with two decimal places
        strcat(result, temp);                                                                           //Append to the result buffer
     /*Add a comma after all but the last element*/
        if (i < size - 1) {
            strcat(result, ",");
        }
    }
}
//==================================================================================================================================================================================
void upload_every_minute(){
	 //ESP_LOGI(TAG, "status: %d", status);
	if(flag_uploading==1){
                combine_data_string();
                if (strlen(combine_array) > 0) {
                   status= publish_to_mqtt(topic_tx, combine_array);
                   if(status==1){
                    ESP_LOGI(TAG, "Published testing topic: %s", topic_tx);
                    flag_uploading=0;
                    }
                    else {
						ESP_LOGI(TAG, "false uploading: %s", topic_tx);
					}
                } else {
                    ESP_LOGW(TAG, "No valid data to publish.");
                }
         }
}
//=================================================================================================================================================================================
/*void combine_data_string() {
    char formatted_data[2048] = {0};                                                                   //Allocate enough space for JSON
    strcat(formatted_data, "{"); 
    // Adding Latitude and Longitude
     strcat(formatted_data, "\"position\":{");
    char location_data[128];
    snprintf(location_data, sizeof(location_data), "\"lat\":\"%.6f\",\"lng\":\"%.6f\"", Latitude, Longitude);
    strcat(formatted_data, location_data);
     strcat(formatted_data, "}");
     strcat(formatted_data, ",");
    for (int i = 0; i < 20; i++) {                                                                     //Add data values (d1 to d20)
        char data_entry[64];
        snprintf(data_entry, sizeof(data_entry), "\"d%d\":\"%.5f\"", i + 1, responseArray[i]);
        strcat(formatted_data, data_entry);
        if (i < 19) {                                                                                  //Add a comma for all except the last element
            strcat(formatted_data, ",");
        }
    }
    for (int i = 0; i < 5; i++) {                                                                      //Add result values (d21 to d25)
        char result_entry[64];
        snprintf(result_entry, sizeof(result_entry), ",\"d%d\":\"%.5f\"", i + 21, Result[i]);
        strcat(formatted_data, result_entry);
    }
    // Add AFE_id as d26
    char sensor_id_entry[128];
    snprintf(sensor_id_entry, sizeof(sensor_id_entry), ",\"d26\":\"");
    strcat(formatted_data, sensor_id_entry);
    // Append AFE_id bytes in hexadecimal format
    for (int i = 0; i < sizeof(sensor_id); i++) {
        char hex[4];  // Space for hex representation
        snprintf(hex, sizeof(hex), "%02X", sensor_id[i]);
        strcat(formatted_data, hex);
    }
    strcat(formatted_data, "\"");
    // Add Sen_id as d27
    char sen_id_entry[128];
    snprintf(sen_id_entry, sizeof(sen_id_entry), ",\"d27\":\"%s\"", SEN_ID);
    strcat(formatted_data, sen_id_entry);
    strcat(formatted_data, ",\"header\":{");    
     const char* headers[] = {
        "R11", "R12", "R13", "R14", "R21", "R22", "R23", "R24",
        "R31", "R32", "R33", "R34", "T1", "T2", "C1", "C2",
        "R11_2W", "R12_2W", "R33_2W", "R34_2W",
        "CL", "PH", "CND", "TDS", "TEMP", "AFE_id", "Sen_id"
    };                                                       //Add header section
    for (int i = 0; i < 27; i++) {
        char header_entry[64];
        snprintf(header_entry, sizeof(header_entry), "\"h%d\":\"%s\"", i + 1, headers[i]);
        strcat(formatted_data, header_entry);
        if (i < 26) {                                                                                        //Add a comma for all except the last element
            strcat(formatted_data, ",");
        }
    }
    strcat(formatted_data, "}}"); 
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    strcpy(combine_array, formatted_data);   
     xSemaphoreGive(xMutex);
    }
   //ESP_LOGI(TAG, "Formatted JSON: %s", combine_array);
}*/
void combine_data_string() {
    char formatted_data[2048] = {0};
    strcat(formatted_data, "{");
    // Always include position
    strcat(formatted_data, "\"position\":{");
    char location_data[128];
    snprintf(location_data, sizeof(location_data), "\"lat\":\"%.6f\",\"lng\":\"%.6f\"", Latitude, Longitude);
    strcat(formatted_data, location_data);
    strcat(formatted_data, "},");
    // If USER is "client", include 20 sensor readings
    if (strcmp(USER, "client") == 0) {
        for (int i = 0; i < 20; i++) {
            char data_entry[64];
            snprintf(data_entry, sizeof(data_entry), "\"d%d\":\"%.5f\",", i + 1, responseArray[i]);
            strcat(formatted_data, data_entry);
        }
    }
    // Always include 5 result values (mapped to d21 to d25)
    for (int i = 0; i < 5; i++) {
        char result_entry[64];
        snprintf(result_entry, sizeof(result_entry), "\"d%d\":\"%.5f\"",
                 (strcmp(USER, "client") == 0) ? i + 21 : i + 21, Result[i]);
        strcat(formatted_data, result_entry);
        strcat(formatted_data, ",");
    }
    // AFE_id (d26)
    char afe_id_entry[128];
    snprintf(afe_id_entry, sizeof(afe_id_entry), "\"d26\":\"");
    strcat(formatted_data, afe_id_entry);
    for (int i = 0; i < sizeof(sensor_id); i++) {
        char hex[4];
        snprintf(hex, sizeof(hex), "%02X", sensor_id[i]);
        strcat(formatted_data, hex);
    }
    strcat(formatted_data, "\",");
    // Sen_id (d27)
    char sen_id_entry[128];
    snprintf(sen_id_entry, sizeof(sen_id_entry), "\"d27\":\"%s\",", SEN_ID);
    strcat(formatted_data, sen_id_entry);
    // Start header section
    strcat(formatted_data, "\"header\":{");
    if (strcmp(USER, "client") == 0) {
        // Full 27 headers for client mode
        const char* headers[] = {
            "R11", "R12", "R13", "R14", "R21", "R22", "R23", "R24",
            "R31", "R32", "R33", "R34", "T1", "T2", "C1", "C2",
            "R11_2W", "R12_2W", "R33_2W", "R34_2W",
            "CL", "PH", "CND", "TDS", "TEMP", "AFE_id", "Sen_id"
        };
        for (int i = 0; i < 27; i++) {
            char header_entry[64];
            snprintf(header_entry, sizeof(header_entry), "\"h%d\":\"%s\"", i + 1, headers[i]);
            strcat(formatted_data, header_entry);
            if (i < 26) strcat(formatted_data, ",");
        }
    } else {
        // User mode headers (only result + AFE_id + Sen_id, starting from h21)
        const char* result_headers[] = { "CL", "PH", "CND", "TDS", "TEMP" };
        for (int i = 0; i < 5; i++) {
            char header_entry[64];
            snprintf(header_entry, sizeof(header_entry), "\"h%d\":\"%s\"", i + 21, result_headers[i]);
            strcat(formatted_data, header_entry);
            strcat(formatted_data, ",");
        }
        // AFE_id
        strcat(formatted_data, "\"h26\":\"AFE_id\",");
        // Sen_id
        strcat(formatted_data, "\"h27\":\"Sen_id\"");
    }
    // Close header and JSON object
    strcat(formatted_data, "}}");
    // Protect combine_array with mutex
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        strcpy(combine_array, formatted_data);
        xSemaphoreGive(xMutex);
    }
}
//==============================================================================================================================================================
void mqtt_receive_task(void *param) {
	gsm_init();
	update_system_time_from_gsm();
	connect_to_mqtt();
	subscribe_to_mqtt();
    while (1) {
        char resp[200];
        int len = uart_read_bytes(UART_NUM_UART2, resp, sizeof(resp), pdMS_TO_TICKS(3000));                //Read data from UART
        if (len > 0) {
            resp[len] = '\0';
            const char *start = strstr((char *)resp, "+QMTRECV: ");                                        //Check for +QMTRECV response
            if (start) {
                start += strlen("+QMTRECV: ");
                int client_id = 0, recv_id = 0;
                if (sscanf(start, "%d,%d", &client_id, &recv_id) == 2) {                                   //Extract client ID and receive ID
                  //  ESP_LOGI(TAG, "Client ID: %d, Recv ID: %d", client_id, recv_id);
                   //Fetch the received MQTT message
                    char command[64];
                    snprintf(command, sizeof(command), "AT+QMTRECV=%d,%d", client_id, recv_id);
                    fetch_mqtt_message(client_id, recv_id);
                  //  ESP_LOGI(TAG, "Message received: %s", buff_rec_data);                                  //Assume fetch_mqtt_message stores the received message in buff_rec_data
                  // Combine the responseArray and Result into a JSON format
                    combine_data_string();
                 //  Publish the JSON-formatted data to another topic
                    if (strlen(combine_array) > 0) {
                        publish_to_mqtt(topic_tx, combine_array);
                        ESP_LOGI(TAG, "Published JSON data to topic: %s", topic_tx);
                    } else {
                        ESP_LOGW(TAG, "No valid data to publish.");
                    }
                }
            }
        } else {
            const char *start1 = strstr((char *)resp, "+QMTSTAT: ");
            if (start1) {
				memset(resp, 0, sizeof(resp));
				ESP_LOGI(TAG, "status11: %d", status);start1 = 0;
                status=connect_to_mqtt();
                ESP_LOGI(TAG, "status111111: %d", status);
                subscribe_to_mqtt();
            }
        }
        if(restart_gsm==1){gsm_init();restart_gsm=0;}
        if(status==0){ ESP_LOGI(TAG, "status22: %d", status);status=connect_to_mqtt();subscribe_to_mqtt();}
        if(status==0){reboot_count++;if(reboot_count>2){reboot_count=0;restart_gsm=1;}}
        upload_every_minute();
    }
}
//==================================================================================================================================================================================
/*Function to send a command over UART and read the response*/
void send_command_and_receive_response(int commandIndex) {
	// int64_t start_time = esp_timer_get_time();
    ESP_LOGI(TAG, "Executing command %d", commandIndex);
    int commandLength = command_sizes[commandIndex];                                                                   //Get the correct length for the command
    ESP_LOGI(TAG, "Sending command: ");
    for (int i = 0; i < commandLength; i++) {
        printf("%02X ", commands[commandIndex][i]);
    }
    printf("\n");
    int bytes_sent = uart_write_bytes(UART_NUM, (const char *)commands[commandIndex], commandLength);                 //Send the command via UART
    if (bytes_sent != commandLength) {
        //ESP_LOGE(TAG, "Error sending command. Sent %d bytes out of %d.", bytes_sent, commandLength);
        return;
    }
    ESP_LOGI(TAG, "Command sent successfully.");
  /*Wait for response with an increased timeout (3 seconds)*/
    uint8_t response[64];
    int len = 0;
             len = uart_read_bytes(UART_NUM, response, 4, pdMS_TO_TICKS(3000));
            if (len >= 4) {
                ESP_LOGI(TAG, "Response received (%d bytes):", len);
                for (int i = 0; i < len; i++) {
                    printf("%02X ", response[i]);
                }
                printf("\n");
               /*Process response as a double*/
                double doubleValue = unpack_double(response);
               ESP_LOGI(TAG, "Converted Value: %.8f", doubleValue);

                responseArray[commandIndex]=doubleValue;
                ESP_LOGI(TAG, "Converted Value responseArray: %.8f", responseArray[commandIndex]);
            } else {
                ESP_LOGW(TAG, "Incomplete response received.");
            }
    //int64_t end_time = esp_timer_get_time();  // Store end time
   // ESP_LOGI(TAG, "send_command_and_receive_response() execution time: %lld microseconds", end_time - start_time);
}
//===================================================================================================================================================================================
/*Convert 4-byte response to a double*/
double unpack_double(uint8_t *bytes_received) {
    float temp_float;
    memcpy(&temp_float, bytes_received, sizeof(float));
    return (double)temp_float;
}
//==================================================================================================================================================================================
double ObjFunc(double Coeff[16][5], double Rdata_T[16], double X_0[2]) {
    double pH_0 = X_0[0];
    double HCLO_0 = X_0[1];
    double MSE = 0;
    for (int i = 0; i < 16; i++) {
        double coef_C = Coeff[i][0];
        double coef_pH = Coeff[i][1];
        double coef_HCLO1 = Coeff[i][2];
        double coef_HCLO2 = Coeff[i][3];
        double fi = Rdata_T[i] - (coef_pH * pH_0) - (coef_HCLO1 * HCLO_0) - (coef_HCLO2 * HCLO_0 * HCLO_0) - coef_C;
        MSE += fi * fi;
    }
    return MSE;
}
//==================================================================================================================================================================================
void ObjFunc_grad(double Coeff[16][5], double Rdata_T[16], double X_0[2], double grad[2]) {
    double pH_0 = X_0[0];
    double HCLO_0 = X_0[1];
    grad[0] = 0;
    grad[1] = 0;
    for (int i = 0; i < 16; i++) {
        double coef_C = Coeff[i][0];
        double coef_pH = Coeff[i][1];
        double coef_HCLO1 = Coeff[i][2];
        double coef_HCLO2 = Coeff[i][3];
        double fi = Rdata_T[i] - (coef_pH * pH_0) - (coef_HCLO1 * HCLO_0) - (coef_HCLO2 * HCLO_0 * HCLO_0) - coef_C;
        grad[0] += -2 * fi * coef_pH;
        grad[1] += -2 * fi * coef_HCLO1 - 4 * fi * coef_HCLO2 * HCLO_0;
    }
}
//===================================================================================================================================================================================
double LineSearch_alpha_val(double Coeff[16][5], double Rdata_T[16], double X_k[2], double S_k[2], double alpha_vals[2], int MAX_ITERS) {
    const double gr = 1.618;
    double a = alpha_vals[0];
    double b = alpha_vals[1];
    for (int iter = 0; iter < MAX_ITERS; iter++) {
        double L = b - a;
        double c = b - L / gr;
        double d = a + L / gr;
        double X_c[2] = {X_k[0] + c * S_k[0], X_k[1] + c * S_k[1]};
        double X_d[2] = {X_k[0] + d * S_k[0], X_k[1] + d * S_k[1]};
        double FC = ObjFunc(Coeff, Rdata_T, X_c);
        double FD = ObjFunc(Coeff, Rdata_T, X_d);
        if (FC < FD) {
            b = d;
        } else {
            a = c;
        }
    }
    return 0.5 * (a + b);
}
//==================================================================================================================================================================================
void SD_optim(double Coeff[16][5], double Rdata_T[16], double X_0[2], int MAX_ITERS_SD, int MAX_ITERS_LS, double MAX_TOL, double result[2]) {
    double Xk[2] = {X_0[0], X_0[1]};
    double Sk[2];
    double grad[2];
    ObjFunc_grad(Coeff, Rdata_T, Xk, grad);
    Sk[0] = -grad[0];
    Sk[1] = -grad[1];
    double F_obj_K = ObjFunc(Coeff, Rdata_T, Xk);
    for (int iter = 0; iter < MAX_ITERS_SD; iter++) {
        double alpha_vals[2] = {-1, 1};
        double alpha_k = LineSearch_alpha_val(Coeff, Rdata_T, Xk, Sk, alpha_vals, MAX_ITERS_LS);
        double Xkp1[2] = {Xk[0] + alpha_k * Sk[0], Xk[1] + alpha_k * Sk[1]};
        double F_obj_Kp1 = ObjFunc(Coeff, Rdata_T, Xkp1);
        if (fabs(F_obj_K - F_obj_Kp1) < MAX_TOL) {
            result[0] = Xkp1[0];
            result[1] = Xkp1[1];
            return;
        }
        ObjFunc_grad(Coeff, Rdata_T, Xkp1, grad);
        Sk[0] = -grad[0];
        Sk[1] = -grad[1];
        Xk[0] = Xkp1[0];
        Xk[1] = Xkp1[1];
        F_obj_K = F_obj_Kp1;
    }
    result[0] = Xk[0];
    result[1] = Xk[1];
}
//=================================================================================================================================================================================
void processSensorResponse() {
    memcpy(SensorResponse, responseArray, sizeof(responseArray));
     for (int i = 0; i < 20; i++) {
       //ESP_LOGI(TAG, "SensorResponse[%d]: %f", i, SensorResponse[i]);
    }
}
//=================================================================================================================================================================================
void EstimateTarget(double SensorResponse[20], double Coeff[16][5], double Temp_Coeff[5], double Cond_Coeff[6], double TDS_Coeff, int MAX_ITERS, int MAX_ITERS_LS, double MAX_TOL, double Result[5]) {
	processSensorResponse();
    double Temp_resp[2] = {SensorResponse[12], SensorResponse[13]};
    double T_ref = Temp_Coeff[0]*(Temp_resp[0] - Temp_Coeff[1]) + Temp_Coeff[2]*(Temp_resp[1] - Temp_Coeff[3]) + Temp_Coeff[4];
    double Cond_resp[2] = {SensorResponse[14], SensorResponse[15]};
    double C_ref = Cond_Coeff[2] * pow(Cond_resp[0] - Cond_Coeff[0], -1.5) + Cond_Coeff[3] * pow(Cond_resp[1] - Cond_Coeff[1], -1.5) + Cond_Coeff[4] * T_ref + Cond_Coeff[5];
    // Check for NaN and replace with 0
       if (isnan(C_ref)) {
           C_ref = 0.0;
       }
    double TDS_ref = C_ref * TDS_Coeff;
    // Check for NaN and replace with 0
        if (isnan(TDS_ref)) {
            TDS_ref = 0.0;
        }
    double Resp[16];
     for (int i = 0; i < 12; i++) {
        Resp[i] = (SensorResponse[i] / Coeff[i][4]) - 1;
    }
    for (int i = 16; i < 20; i++) {
        Resp[i-4] = (SensorResponse[i] / Coeff[i-4][4]) - 1;
    }
    double OptimX[2];
    double X_0[2] = {7.0, 0.0};
    SD_optim(Coeff, Resp, X_0, MAX_ITERS, MAX_ITERS_LS, MAX_TOL, OptimX);
     // Store results in Result array
    Result[0] = OptimX[1];  // Chlorine_ref
    Result[1] = OptimX[0];  // pH_ref
    Result[2] = C_ref;      // CND
    Result[3] = TDS_ref;    // TDS
    Result[4] = T_ref;      // TEMP
    ESP_LOGI(TAG, "Chlorine_ref: %f", Result[0]);
    ESP_LOGI(TAG, "pH_ref: %f", Result[1]);
    ESP_LOGI(TAG, "C_ref: %f", Result[2]);
    ESP_LOGI(TAG, "TDS_ref: %f", Result[3]);
    ESP_LOGI(TAG, "T_ref: %f", Result[4]);
}
//==================================================================================================================================================================================
void combine_data_and_calculate() {
	//int64_t start_time = esp_timer_get_time();
   // ESP_LOGI(TAG, "Executing all commands due to invalid input...");
    vTaskDelay(pdMS_TO_TICKS(100));
    /*Flush UART buffer before sending commands*/
    uint8_t temp[BUF_SIZE];
    while (uart_read_bytes(UART_NUM, temp, BUF_SIZE, pdMS_TO_TICKS(100)) > 0) {
      //  ESP_LOGI(TAG, "Flushing old UART data: %.*s", BUF_SIZE, temp);
    }
    /*Step 1: Send all commands and wait for responses*/
    for (int i = 0; i < NUM_COMMANDS; i++) {
        //ESP_LOGI(TAG, "Sending command %d...", i);
        send_command_and_receive_response(i);
        vTaskDelay(pdMS_TO_TICKS(100));                                             
    }
    /*Step 2: Perform calculations after collecting all responses*/
    EstimateTarget(SensorResponse, Coeff, Temp_Coeff, Cond_Coeff, TDS_Coeff, MAX_ITERS, MAX_ITERS_LS, MAX_TOL, Result);
    lv_disp_t *disp = lv_disp_get_default();                                                                                   //Get the default LVGL display
    example_lvgl_demo_ui(disp, Result);
    //int64_t end_time = esp_timer_get_time();  // Store end time
    //ESP_LOGI(TAG, "combine_data_and_calculate() execution time: %lld microseconds", end_time - start_time);
}
//===================================================================================================================================================================================
// Declare the bitmap data for the logo
static const uint8_t hydroscope_logo_bitmap[] = {
 	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x7f, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x1f, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x87, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xc7, 0xe1, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xe3, 0xf8, 0x07, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xe1, 0xfe, 0x00, 0x07, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0xff, 0xfc, 0x00, 0xff, 0x80, 0x00, 0x3f, 0xff, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x03, 0xff, 0xe0, 0x00, 0x3f, 0xf0, 0x00, 0x07, 0xff, 0xc0, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0f, 0xff, 0x00, 0x00, 0x0f, 0xfc, 0x00, 0x00, 0xff, 0xf0, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x1f, 0xfc, 0x00, 0x00, 0x03, 0xff, 0x80, 0x00, 0x3f, 0xf8, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x7f, 0xf0, 0x00, 0x00, 0x00, 0xff, 0xe0, 0x00, 0x0f, 0xfe, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x3f, 0xfc, 0x00, 0x03, 0xff, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x01, 0xff, 0x80, 0x00, 0x18, 0x00, 0x0f, 0xff, 0x00, 0x01, 0xff, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x03, 0xfe, 0x00, 0x00, 0xf0, 0x00, 0x03, 0xff, 0xc0, 0x00, 0x7f, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0xfc, 0x00, 0x07, 0x80, 0x00, 0x00, 0xff, 0xf0, 0x00, 0x3f, 0xe0, 0x00, 0x00, 
	0x00, 0x00, 0x0f, 0xf8, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x7f, 0xfc, 0x00, 0x1f, 0xf0, 0x00, 0x00, 
	0x00, 0x00, 0x0f, 0xf0, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x1f, 0xfe, 0x00, 0x0f, 0xf0, 0x00, 0x00, 
	0x00, 0x00, 0x1f, 0xf0, 0x01, 0xf8, 0x00, 0x00, 0x00, 0x07, 0xff, 0x00, 0x0f, 0xf8, 0x00, 0x00, 
	0x00, 0x00, 0x3f, 0xe0, 0x03, 0xf0, 0x00, 0x00, 0x00, 0x03, 0xff, 0xc0, 0x07, 0xfc, 0x00, 0x00, 
	0x00, 0x00, 0x3f, 0xe0, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0xff, 0xc0, 0x07, 0xfc, 0x00, 0x00, 
	0x00, 0x00, 0x3f, 0xc0, 0x0f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xe0, 0x03, 0xfc, 0x00, 0x00, 
	0x00, 0x00, 0x3f, 0xc0, 0x0f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xf0, 0x03, 0xfc, 0x00, 0x00, 
	0x00, 0x00, 0x3f, 0xc0, 0x1f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf0, 0x03, 0xfc, 0x00, 0x00, 
	0x00, 0x00, 0x3f, 0xc0, 0x1f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf0, 0x03, 0xfc, 0x00, 0x00, 
	0x00, 0x00, 0x3f, 0xc0, 0x1f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf0, 0x03, 0xfc, 0x00, 0x00, 
	0x00, 0x00, 0x3f, 0xc0, 0x1f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf0, 0x03, 0xfc, 0x00, 0x00, 
	0x00, 0x00, 0x3f, 0xe0, 0x1f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xf0, 0x07, 0xfc, 0x00, 0x00, 
	0x00, 0x00, 0x3f, 0xe0, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xf0, 0x07, 0xfc, 0x00, 0x00, 
	0x00, 0x00, 0x1f, 0xe0, 0x0f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0xff, 0xe0, 0x0f, 0xf8, 0x00, 0x00, 
	0x00, 0x00, 0x1f, 0xf0, 0x07, 0xfc, 0x00, 0x00, 0x00, 0x01, 0xff, 0xe0, 0x0f, 0xf8, 0x00, 0x00, 
	0x00, 0x00, 0x0f, 0xf8, 0x07, 0xfe, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x1f, 0xf0, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0xfc, 0x03, 0xff, 0xc0, 0x00, 0x00, 0x1f, 0xff, 0x80, 0x3f, 0xe0, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0xfe, 0x01, 0xff, 0xf8, 0x00, 0x00, 0xff, 0xff, 0x00, 0x7f, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x01, 0xff, 0x00, 0x7f, 0xff, 0xc0, 0x1f, 0xff, 0xfe, 0x00, 0xff, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xff, 0x80, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x03, 0xff, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x7f, 0xe0, 0x07, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x07, 0xfe, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x3f, 0xf8, 0x01, 0xff, 0xff, 0xff, 0xff, 0x00, 0x1f, 0xfc, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x1f, 0xfe, 0x00, 0x3f, 0xff, 0xff, 0xf8, 0x00, 0x7f, 0xf8, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x01, 0xff, 0xff, 0x80, 0x03, 0xff, 0xe0, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x01, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x80, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0x80, 0x00, 0x00, 0x01, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xfe, 0x00, 0x00, 0x7f, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x18, 0x68, 0x6f, 0xc7, 0xc7, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x18, 0x64, 0xd8, 0x64, 0x2c, 0x19, 0xe7, 0x9e, 0x3e, 0x1c, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x1f, 0xe3, 0x98, 0x27, 0xc8, 0x0a, 0x0c, 0x31, 0xa1, 0x62, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x18, 0x23, 0x18, 0x64, 0xcc, 0x18, 0xe8, 0x21, 0xa1, 0x7c, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x18, 0x63, 0x1f, 0xc4, 0x67, 0xf3, 0xef, 0x9f, 0x3f, 0x3e, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
//===================================================================================================================================================================================
// Image descriptor for the logo
static const lv_img_dsc_t hydroscope_logo = {
    .header.cf = LV_IMG_CF_ALPHA_1BIT,                                                     // Color format: 1-bit indexed
    .header.always_zero = 0,
    .header.reserved = 0,
    .header.w = 128,                                                                       // Width of the image
    .header.h = 64,                                                                        // Height of the image
    .data_size = sizeof(hydroscope_logo_bitmap),                                           // Data size
    .data = hydroscope_logo_bitmap,                                                        // Pointer to the bitmap data
};
//===================================================================================================================================================================================
/*Function to create and display the results on the OLED*/
void example_lvgl_demo_ui(lv_disp_t *disp, double Result[5]) {
    lv_obj_t *scr = lv_disp_get_scr_act(disp);                                                              //Get the active screen
    lv_obj_clean(scr);
      const char *result_names[] = {"CL", "PH", "CND", "TDS", "TEMP"};                                                                                    //Clear the screen before displaying new results
   /*Create and display a label for each target*/
    for (int i = 0; i < 5; i++) {
        lv_obj_t *label = lv_label_create(scr);                                                                  //Create a new label
       /*Format the text to display the target and its result*/
        char buffer[256];
       snprintf(buffer, sizeof(buffer), "%s: %.5f", result_names[i], Result[i]);
      /*Set the text of the label*/
        lv_label_set_text(label, buffer);
        lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, i * 12);                                                 //Position the label, spacing 20px apart vertically
        lv_obj_set_scroll_dir(label, LV_DIR_VER);
        lv_obj_scroll_to_y(label, 0, LV_ANIM_ON);                                                         //Scroll to the top (Y = 0)
    }
}
//==================================================================================================================================================================================
void oled_init(void) {
   //ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = OLED_PIN_NUM_SDA,
        .scl_io_num = OLED_PIN_NUM_SCL,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));  
  //ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = OLED_I2C_HW_ADDR,
        .scl_speed_hz = OLED_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = OLED_CMD_BITS,
        .lcd_param_bits = OLED_PARAM_BITS,
        .dc_bit_offset = 6,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));
   //ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = OLED_PIN_NUM_RST,
        .flags = {
            .reset_active_high = false,
        },
        .color_space = ESP_LCD_COLOR_SPACE_MONOCHROME,
        .bits_per_pixel = 1,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
    if (OLED_PIN_NUM_RST >= 0) {
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    }
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    //ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = OLED_H_RES * OLED_V_RES,
        .double_buffer = true,
        .hres = OLED_H_RES,
        .vres = OLED_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
    };
    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);
    //Display the Hydroscope logo at startup
   //ESP_LOGI(TAG, "Display Hydroscope logo");
    lv_obj_t *scr = lv_disp_get_scr_act(disp);                                                       // Get the active screen
    lv_obj_clean(scr);                                                                               // Clear the screen
    lv_obj_t *img = lv_img_create(lv_scr_act());                                                     // Create the image object
    lv_img_set_src(img, &hydroscope_logo);                                                           // Set the logo source
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);                                                        // Center the image
    lv_task_handler();                                                                               // Refresh LVGL display
    vTaskDelay(pdMS_TO_TICKS(2000));
    lv_obj_clean(scr);
    wifi_init_sta();
    vTaskDelay(pdMS_TO_TICKS(500));
       if (lvgl_port_lock(0)) {
      example_lvgl_demo_ui(disp, Result);
       lvgl_port_unlock();
    }  
}
//===================================================================================================================================================================================
static esp_err_t get_latest_uploaded_file(char *file_path, size_t file_path_len) {
    ESP_LOGI(TAG, "File path_get: %s\n", file_path);
    DIR *dir = opendir("/fatfs");
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open fatfs directory.");
        return ESP_FAIL;
    }
    struct dirent *entry;
    struct stat file_stat;
    time_t latest_mtime = 0; // To store the modification time of the latest file
    while ((entry = readdir(dir)) != NULL) {
        /* Skip any file that is not 'config.json' */
        if (strcmp(entry->d_name, "config.json") != 0) {
            continue;
        }
        /* Construct full path to file */
        char full_path[FILE_PATH_MAX];
        strlcpy(full_path, "/fatfs/", sizeof(full_path));
        if (strlcat(full_path, entry->d_name, sizeof(full_path)) >= sizeof(full_path)) {
            ESP_LOGW(TAG, "File name too long, skipping: %s", entry->d_name);
            continue;
        }
        /* Check file metadata */
        if (stat(full_path, &file_stat) == 0) {
            if (S_ISREG(file_stat.st_mode)) { // Check if it's a regular file
                ESP_LOGI(TAG, "Checking file: %s, mtime: %lld, ctime: %lld", entry->d_name, file_stat.st_mtime, file_stat.st_ctime);
                if (file_stat.st_mtime > latest_mtime) {
                    latest_mtime = file_stat.st_mtime;
                    strlcpy(file_path, full_path, file_path_len); // Save the latest file path
                }
            }
        } else {
            ESP_LOGW(TAG, "Failed to stat file: %s", entry->d_name);
        }
    }
    closedir(dir);
    if (latest_mtime == 0) {
        ESP_LOGW(TAG, "No valid files found in fatfs.");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Latest file found: %s", file_path);
    return ESP_OK;
}
//==================================================================================================================================================================================
void storage_init() {
	if (fatfs_initialized) {
        return;
    }
    ESP_LOGI(TAG, "Initializing storage...");
    // Mount configuration
    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 10,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    const char* base_path = "/fatfs";
    // Mount FATFS with wear levelling
    esp_err_t ret = esp_vfs_fat_spiflash_mount_rw_wl(base_path, "storage", &mount_config, &wl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(ret));
        return;
    }
    // Get and log filesystem information
        FATFS *fs;
        DWORD fre_clust, fre_sect, tot_sect;
        FRESULT f_res = f_getfree("0:", &fre_clust, &fs);
        if (f_res == FR_OK) {
            tot_sect = (fs->n_fatent - 2) * fs->csize;
            fre_sect = fre_clust * fs->csize;
            ESP_LOGI(TAG, "Storage Info: Total=%luKB, Free=%luKB",
                    tot_sect / 2, fre_sect / 2);
        } else {
            ESP_LOGW(TAG, "Couldn't get storage info (FRESULT: %d)", f_res);
        }
    fatfs_initialized = true;
    ESP_LOGI(TAG, "Storage initialized successfully");
    /*Dynamically find and load the latest file*/
        char latest_file_path[FILE_PATH_MAX] = {0};
        if (get_latest_uploaded_file(latest_file_path, sizeof(latest_file_path)) == ESP_OK) {
           ESP_LOGI(TAG, "Loading coefficients from the latest uploaded file: %s", latest_file_path);
            update_coefficients(latest_file_path);
        } else {
           ESP_LOGW(TAG, "No uploaded files found. Using default coefficients.");
        }
}
//==================================================================================================================================================================================
static void cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    if (event->type == CDC_EVENT_RX) {
        uint8_t buf[64];
        size_t len = 0;
        esp_err_t err = tinyusb_cdcacm_read(itf, buf, sizeof(buf), &len);
        if (err == ESP_OK && len > 0) {
            buf[len] = '\0'; // Null-terminate for printing
            ESP_LOGI(TAG, "Received (%d bytes): %s", len, (char*)buf);
            // Echo back
            tinyusb_cdcacm_write_queue(itf, buf, len);
            tinyusb_cdcacm_write_flush(itf, 0);
        }
    }
}
//==================================================================================================================================================================================
void usb_msc_init(void) {
    if (usb_msc_initialized) return;
    ESP_LOGI(TAG, "Initializing USB MSC + CDC...");
    // 1. Init TinyUSB core
    tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .string_descriptor_count = 0,
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = NULL,
        .hs_configuration_descriptor = NULL,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = NULL,
#endif
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    // 2. Init CDC
    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = cdc_rx_callback, // Optional: assign your RX callback here
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    // 3. Init MSC using SPI flash (FATFS)
    const tinyusb_msc_spiflash_config_t config_spi = {
        .wl_handle = wl_handle,
    };
    ESP_ERROR_CHECK(tinyusb_msc_storage_init_spiflash(&config_spi));

    usb_msc_initialized = true;
    ESP_LOGI(TAG, "USB MSC + CDC initialized successfully");
}
//==================================================================================================================================================================================
esp_err_t example_mount_storage(const char *base_path) {
    if (fatfs_initialized) {
       ESP_LOGI(TAG, "fatfs already initialized, mounting storage.");
        return ESP_OK;
    }
    storage_init();                                                                                        //Ensure SPIFFS is initialized before mounting
    if (!fatfs_initialized) {
       ESP_LOGE(TAG, "fatfs initialization failed.");
        return ESP_FAIL;
    }
   ESP_LOGI(TAG, "Mounting fatfs storage at base path: %s", base_path);
    return ESP_OK; 
}
//===================================================================================================================================================================================
void update_system_time_from_gsm() {
    const char *command = "AT+CCLK?";
    uart_write_bytes(UART_NUM_UART2, command, strlen(command));
    uart_write_bytes(UART_NUM_UART2, "\r\n", 2);
    //Read the response
    char response[128] = {0};
    int len = uart_read_bytes(UART_NUM_UART2, (uint8_t *)response, sizeof(response) - 1, pdMS_TO_TICKS(5000));
    if (len > 0) {
        response[len] = '\0';
       // ESP_LOGI(TAG, "GSM Time Response: %s", response);
        // Parse the response
        const char *start = strstr(response, "+CCLK: \"");
        if (start) {
            start += 8; // Move past '+CCLK: "'
            int year, month, day, hour, minute, second;
            if (sscanf(start, "%2d/%2d/%2d,%2d:%2d:%2d", &year, &month, &day, &hour, &minute, &second) == 6) {
               // ESP_LOGI(TAG, "Parsed Time: 20%02d-%02d-%02d %02d:%02d:%02d", year, month, day, hour, minute, second);
                // Set the system time
                struct tm timeinfo = {
                    .tm_year = year + 100,                                                                   // Years since 1900
                    .tm_mon = month - 1,                                                                     // Months since January
                    .tm_mday = day,
                    .tm_hour = hour,
                    .tm_min = minute,
                    .tm_sec = second,
                };
                time_t t = mktime(&timeinfo);
                 t += (5 * 3600) + (30 * 60);
                localtime_r(&t, &timeinfo);
                struct timeval now = {.tv_sec = t};
                settimeofday(&now, NULL);
               ESP_LOGI(TAG, "System time updated to IST: %s", asctime(&timeinfo));
            } else {
              //ESP_LOGE(TAG, "Failed to parse time from GSM response.");
            }
        } else {
         //ESP_LOGE(TAG, "GSM time not found in response.");
        }
    } else {
      //ESP_LOGE(TAG, "No response from GSM module for time query.");
    }
}
//===================================================================================================================================================================================
void store_data_to_fatfs(const char *data) {
    // Get the current date
    time_t now;
    struct tm timeinfo;
    char file_path[64], time_buffer[64];
    time(&now);
    localtime_r(&now, &timeinfo);
    // Create a filename based on the current date
    strftime(file_path, sizeof(file_path), "/fatfs/combined_data_%Y-%m-%d", &timeinfo);
    ESP_LOGI(TAG, "Saving data to file: %s\n", file_path);
    // Open file in append mode
    FILE *file = fopen(file_path, "a");
    if (file == NULL) {
         ESP_LOGE(TAG, "Failed to open file for writing: %s", file_path);
        return;
    }
    // Get current timestamp
    strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    fprintf(file, "%s | %s\n", data, time_buffer); // Write data with timestamp
    fclose(file); // Close the file
     ESP_LOGI(TAG, "Data with timestamp written to FATFS: %s | %s", data, time_buffer);
}

//==================================================================================================================================================================================
void read_fatfs_file(const char *file_path) {
   FILE *file = fopen(file_path, "r");                                                                         //Open the file in read mode
   if (file == NULL) {
      //ESP_LOGE(TAG, "Failed to open file for reading: %s", file_path);
       return;
    }
   //ESP_LOGI(TAG, "Reading data from file: %s", file_path);
    char line[128];                                                                                           //Buffer to store each line of the file
    while (fgets(line, sizeof(line), file)) {
        line[strcspn(line, "\n")] = '\0';                                                                     //Remove the trailing newline
        //ESP_LOGI(TAG, "File Content: %s", line);
    }
    fclose(file);                                                                                             //Close the file after reading
  //ESP_LOGI(TAG, "Finished reading file: %s", file_path);
}
//===================================================================================================================================================================================
void sensor_data(){
	while(1){
	combine_data_and_calculate();
	vTaskDelay(pdMS_TO_TICKS(100));
	 if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
	 memset(combine_array, 0, sizeof(combine_array)); 
	  xSemaphoreGive(xMutex);
    }
	 if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
	  memset(combine_array1, 0, sizeof(combine_array1));
	   xSemaphoreGive(xMutex);
    }
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
	    memset(combine_array_ble, 0, sizeof(combine_array_ble));
	     xSemaphoreGive(xMutex);
    }
	combine_data_string();
	//ESP_LOGI(TAG, "combined array: %s", combine_array);
	vTaskDelay(pdMS_TO_TICKS(100));
	combine_data_string1();
	//ESP_LOGI(TAG, "combined array1: %s", combine_array1);
	vTaskDelay(pdMS_TO_TICKS(100));
	  if(flag_uploading_internal_data_save==1) {
		   //store_data_to_spiffs("/fatfs/combined_data.txt",combine_array1);
		   store_data_to_fatfs(combine_array1);
		   flag_uploading_internal_data_save=0;
	  }
     //read_fatfs_file("/fatfs/combined_data.txt");
     vTaskDelay(pdMS_TO_TICKS(100));                                                                
     combine_data_string2(); 
     //ESP_LOGI(TAG, "Non-JSON formatted combined array: %s", combine_array_ble);
	}
}
//==================================================================================================================================================================================
void wifi_initate_with_Server(){
	const char *base_path = "/fatfs";
/*    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    ESP_LOGI(TAG, "Connected to Wi-Fi");*/
    ESP_ERROR_CHECK(example_start_file_server(base_path));
   // ESP_LOGI(TAG, "File server started"); 
}
//===================================================================================================================================================================================
void wifi_init_mqtt(){
	// Generate the will topic with MAC address
    snprintf(will_topic, sizeof(will_topic), MQTT_WILL_TOPIC, mac_str);
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = MQTT_BROKER_URI,
            .verification.skip_cert_common_name_check = true,
        },
        .credentials = {
            .username = MQTT_USER,
            .authentication.password = MQTT_PASSWORD,
        },
		.session = {
		    .last_will = {
		    .topic = will_topic,
		    .msg = MQTT_WILL_MESSAGE,
		    .msg_len = strlen(MQTT_WILL_MESSAGE),
		    .qos = MQTT_WILL_QOS,
		    .retain = MQTT_WILL_RETAIN,
	   }
      }
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
    esp_mqtt_client_start(mqtt_client);
}
//==================================================================================================================================================================================
esp_err_t example_restart_file_server(const char *base_path)
{
    static httpd_handle_t server = NULL;
    //Stop the server if it's already running
    if (server) {
       //ESP_LOGI(TAG, "Stopping the HTTP server");
        if (httpd_stop(server) != ESP_OK) {
            //ESP_LOGE(TAG, "Failed to stop the HTTP server");
            return ESP_FAIL;
        }
        //ESP_LOGI(TAG, "HTTP server stopped successfully");
        server = NULL;
    }
   //ESP_LOGI(TAG, "Restarting the HTTP server");
    esp_err_t err = example_start_file_server(base_path);
    if (err != ESP_OK) {
       //ESP_LOGE(TAG, "Failed to restart the HTTP server");
        return err;
    }
    //ESP_LOGI(TAG, "HTTP server restarted successfully");
    return ESP_OK;
}
//==================================================================================================================================================================================
void stop_all_tasks() {
   ESP_LOGI("TASK_CTRL", "Stopping all tasks...");
    if (mqttReceiveHandle) {
        vTaskDelete(mqttReceiveHandle);
        mqttReceiveHandle = NULL;
       ESP_LOGI("TASK_CTRL", "MQTT Receive Task Stopped.");
    } else {
      ESP_LOGI("TASK_CTRL", "MQTT Receive Task was already stopped.");
    }
    if (mqttPublishHandle) {
    	stop_mqtt_and_wifi();
        vTaskDelete(mqttPublishHandle);
        mqttPublishHandle = NULL;

       ESP_LOGI("TASK_CTRL", "MQTT Publish Task Stopped.");
    } else {
       ESP_LOGI("TASK_CTRL", "MQTT Publish Task was already stopped.");
    }
    if (bleNotifyHandle) {
       ESP_LOGI(TAG, "Stopping BLE Task...");
          // Stop Advertising
          int rc = ble_gap_adv_stop();
          if (rc == 0) {
       ESP_LOGI(TAG, "BLE Advertising Stopped.");
          } else {
        ESP_LOGE(TAG, "BLE Advertising Stop Failed: %d", rc);
          }
          nimble_port_stop();
          vTaskDelay(pdMS_TO_TICKS(1000));
          nimble_port_deinit();
          vTaskDelay(pdMS_TO_TICKS(1000));
          vTaskDelete(bleNotifyHandle);
          bleNotifyHandle = NULL;
        ESP_LOGI(TAG, "BLE Task Stopped Properly.");
      } else {
      ESP_LOGI("TASK_CTRL", "BLE Host Task was already stopped.");
    }
    if (uart2Handle) {
        vTaskDelete(uart2Handle);
        uart2Handle = NULL;
       ESP_LOGI("TASK_CTRL", "UART2 Task Stopped.");
    } else {
       ESP_LOGI("TASK_CTRL", "UART2 Task was already stopped.");
    }
   ESP_LOGI("TASK_CTRL", "All tasks checked.");
}
//==================================================================================================================================================================================
void restart_tasks() {
   ESP_LOGI("TASK_CTRL", "Restarting tasks...");
    if (flag_gsm && mqttReceiveHandle == NULL) {
        xTaskCreate(mqtt_receive_task, "mqtt_receive_task", 16384, NULL, 5, &mqttReceiveHandle);
      ESP_LOGI("TASK_CTRL", "MQTT Receive Task started (flag_gsm=%d, handle=%p).", flag_gsm, mqttReceiveHandle);
    } else {
       ESP_LOGI("TASK_CTRL", "MQTT Receive Task not started (flag_gsm=%d, handle=%p).", flag_gsm, mqttReceiveHandle);
    }
    if (flag_wifi && mqttPublishHandle == NULL) {
        xTaskCreate(wifi_monitor_task, "wifi_monitor_task", 4096, NULL, 2, NULL);
        xTaskCreate(mqtt_publish_task, "mqtt_publish_task", 8192, NULL, 2, &mqttPublishHandle);
       ESP_LOGI("TASK_CTRL", "MQTT Publish Task started (flag_wifi=%d, handle=%p).", flag_wifi, mqttPublishHandle);
    } else {
      ESP_LOGI("TASK_CTRL", "MQTT Publish Task not started (flag_wifi=%d, handle=%p).", flag_wifi, mqttPublishHandle);
    }
    if (flag_ble && bleNotifyHandle == NULL) {
    	nimble_port_init();
    	ble_init();
        xTaskCreate(ble_notify_task, "ble_notify_task", 8192, NULL, 8, &bleNotifyHandle);
       ESP_LOGI("TASK_CTRL", "BLE Host Task started (flag_ble=%d, handle=%p).", flag_ble, bleNotifyHandle);
    } else {
       ESP_LOGI("TASK_CTRL", "BLE Host Task not started (flag_ble=%d, handle=%p).", flag_ble, bleNotifyHandle);
    }
    if (flag_uart2 && uart2Handle == NULL) {
        xTaskCreate(send_data_to_uart2_task, "send_data_to_uart2_task", 8192, NULL, 5, &uart2Handle);
      ESP_LOGI("TASK_CTRL", "UART2 Task started (flag_uart2=%d, handle=%p).", flag_uart2, uart2Handle);
    } else {
      ESP_LOGI("TASK_CTRL", "UART2 Task not started (flag_uart2=%d, handle=%p).", flag_uart2, uart2Handle);
    }
}
//==================================================================================================================================================================================
void save_last_mode(int mode) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        nvs_set_i32(my_handle, LAST_MODE_KEY, mode);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
}
//==================================================================================================================================================================================
int load_last_mode() {
    nvs_handle_t my_handle;
    int32_t last_mode = 0;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        nvs_get_i32(my_handle, LAST_MODE_KEY, &last_mode);
        nvs_close(my_handle);
    }
    return last_mode;
}
//==================================================================================================================================================================================
void activate_mode(int countdown_value) {
    stop_all_tasks();
    vTaskDelay(pdMS_TO_TICKS(500)); // Small delay
    lv_obj_clean(lv_scr_act());
    lv_obj_t *mode_label = lv_label_create(lv_scr_act());
    lv_obj_align(mode_label, LV_ALIGN_CENTER, 0, 0);
    if (countdown_value == 1) {
        flag_gsm = true;
        flag_wifi = flag_ble = flag_uart2 =false;
        ESP_LOGI("MODE", "GSM Mode Activated");
        lv_label_set_text(mode_label, "Mode: GSM");
    } else if (countdown_value == 2) {
        flag_wifi = true;
        flag_gsm = flag_ble = flag_uart2 =false;
        ESP_LOGI("MODE", "WiFi Mode Activated");
        lv_label_set_text(mode_label, "Mode: WiFi");
    } else if (countdown_value == 3) {
        flag_ble = true;
        flag_gsm = flag_wifi = flag_uart2 =false;
        ESP_LOGI("MODE", "BLE Mode Activated");
        lv_label_set_text(mode_label, "Mode: BLE");
    } else if (countdown_value == 4) {
        flag_uart2 = true;
        flag_gsm = flag_wifi = flag_ble =false;
        ESP_LOGI("MODE", "LoRa Mode Activated");
        lv_label_set_text(mode_label, "Mode: LoRa");
    }
    lv_task_handler();                                                                                   // Refresh OLED
    strt_flg = 1;
    save_last_mode(countdown_value); // Save the last mode
    ESP_LOGI("BUTTON", "Button fully released. Ready for next selection.");
}
//==================================================================================================================================================================================
void store_default_config_in_nvs() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("default_config", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle for default config: %s", esp_err_to_name(err));
        return;
    }
    // Corrected default configuration JSON
    const char* default_config =
        "{\n"
        "  \"INTERVAL\": 1,\n"
        "  \"sen_id\": \"A12345\",\n"
    	"  \"USER\": \"client\",\n"
        "  \"wifi_ssid\": \"Testing\",\n"
        "  \"wifi_password\": \"Test_1234\",\n"
        "  \"Latitude\": 26.85277,\n"
        "  \"Longitude\": 75.77954,\n"
        "  \"Coeff\": [\n"
        "    [0.1, 0.2, 0.3, 0.05, 0.1],\n"
        "    [0.1, 0.2, 0.3, 0.05, 0.1],\n"
        "    [0.1, 0.2, 0.3, 0.05, 0.1],\n"
        "    [0.1, 0.2, 0.3, 0.05, 0.1],\n"
        "    [0.1, 0.2, 0.3, 0.05, 0.1],\n"
        "    [0.1, 0.2, 0.3, 0.05, 0.1],\n"
        "    [0.1, 0.2, 0.3, 0.05, 0.1],\n"
        "    [0.1, 0.2, 0.3, 0.05, 0.1],\n"
        "    [0.1, 0.2, 0.3, 0.05, 0.1],\n"
        "    [0.1, 0.2, 0.3, 0.05, 0.1],\n"
        "    [0.1, 0.2, 0.3, 0.05, 0.1],\n"
        "    [0.1, 0.2, 0.3, 0.05, 0.1],\n"
        "    [0.1, 0.2, 0.3, 0.05, 0.1],\n"
        "    [0.1, 0.2, 0.3, 0.05, 0.1],\n"
        "    [0.1, 0.2, 0.3, 0.05, 0.1],\n"
        "    [0.1, 0.2, 0.3, 0.05, 0.1]\n"
        "  ],\n"
        "  \"Temp_Coeff\": [0.5, 0.038798, 0.5, 0.038798, 25.678],\n"
        "  \"Cond_Coeff\": [0.01177, 0.00744, 1.136, -0.432, 76.897, 42.705],\n"
        "  \"TDS_Coeff\": 0.5,\n"
        "  \"MAX_ITERS\": 5,\n"
        "  \"MAX_ITERS_LS\": 5,\n"
        "  \"MAX_TOL\": 0.001\n"
        "}";
    err = nvs_set_blob(my_handle, "config_json", default_config, strlen(default_config));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error storing default config in NVS: %s", esp_err_to_name(err));
    } else {
        nvs_commit(my_handle);
        ESP_LOGI(TAG, "Default configuration stored in NVS");
    }
    nvs_close(my_handle);
}
//==================================================================================================================================================================================
void restore_default_config() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("default_config", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle for default config: %s", esp_err_to_name(err));
        return;
    }
    // First get the size needed
    size_t required_size = 0;
    err = nvs_get_blob(my_handle, "config_json", NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Error getting default config size: %s", esp_err_to_name(err));
        nvs_close(my_handle);
        return;
    }
    if (required_size == 0) {
        ESP_LOGE(TAG, "No default config found in NVS");
        nvs_close(my_handle);
        return;
    }
    // Allocate buffer and read the config
    char* default_config = malloc(required_size + 1);
    if (default_config == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for default config");
        nvs_close(my_handle);
        return;
    }
    err = nvs_get_blob(my_handle, "config_json", default_config, &required_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading default config: %s", esp_err_to_name(err));
        free(default_config);
        nvs_close(my_handle);
        return;
    }
    default_config[required_size] = '\0'; // Null-terminate
    nvs_close(my_handle);
    // Write the default config to the FATFS file
    const char* file_path = "/fatfs/config.json";
    FILE* file = fopen(file_path, "w");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open config file for writing");
        free(default_config);
        return;
    }
    size_t written = fwrite(default_config, 1, required_size, file);
    if (written != required_size) {
        ESP_LOGE(TAG, "Failed to write complete default config to file");
    } else {
        ESP_LOGI(TAG, "Default configuration restored successfully");
        // Update the running configuration
        update_config_from_file(file_path);
        // Show success message on OLED
        lv_obj_clean(lv_scr_act());
        lv_obj_t *success_label = lv_label_create(lv_scr_act());
        lv_obj_align(success_label, LV_ALIGN_CENTER, 0, 0);
        lv_label_set_text(success_label, "Default config restored!");
        lv_task_handler();
    }
    fclose(file);
    free(default_config);
    // Keep the message visible for a while
    vTaskDelay(pdMS_TO_TICKS(2000));
}
//==================================================================================================================================================================================
#define LONG_PRESS_TIME_MS 2000  // 2 seconds for long press
void button_task(void *pvParameters) {
    gpio_reset_pin(BUTTON_GPIO);
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);
    int countdown_value = 0;
    bool button_pressed = false;
    TickType_t press_start_time = 0;
    bool long_press_detected = false;
    while (1) {
        // Check if button is pressed
        if (gpio_get_level(BUTTON_GPIO) == 0) {
            if (!button_pressed) {
                button_pressed = true;
                press_start_time = xTaskGetTickCount();
                long_press_detected = false;
                ESP_LOGI("BUTTON", "Button pressed");
                // Clear and update OLED display
                lv_obj_clean(lv_scr_act());
                lv_obj_t *press_label = lv_label_create(lv_scr_act());
                lv_obj_align(press_label, LV_ALIGN_CENTER, 0, 0);
                lv_label_set_text(press_label, "Button pressed");
                lv_task_handler();
            } else {
                // Check for long press
                if (!long_press_detected &&
                    (xTaskGetTickCount() - press_start_time) >= pdMS_TO_TICKS(LONG_PRESS_TIME_MS)) {
                    long_press_detected = true;
                    ESP_LOGI("BUTTON", "Long press");
                    // Clear and update OLED display
                    lv_obj_clean(lv_scr_act());
                    lv_obj_t *long_press_label = lv_label_create(lv_scr_act());
                    lv_obj_align(long_press_label, LV_ALIGN_CENTER, 0, 0);
                    lv_label_set_text(long_press_label, "Restoring...");
                    lv_task_handler();
                    // Restore default config
                    restore_default_config();
                }
            }
        } else {
            if (button_pressed) {
                button_pressed = false;
                if (!long_press_detected) {
                    // Normal short press handling
                    countdown_value = (countdown_value % COUNTDOWN_THRESHOLD) + 1;
                    ESP_LOGI("BUTTON", "Button released. Countdown: %d", countdown_value);
                    // Clear and update OLED display
                    lv_obj_clean(lv_scr_act());
                    lv_obj_t *count_label = lv_label_create(lv_scr_act());
                    lv_obj_align(count_label, LV_ALIGN_CENTER, 0, 0);
                    char countdown_text[32];
                    snprintf(countdown_text, sizeof(countdown_text), "Select Mode: %d", countdown_value);
                    lv_label_set_text(count_label, countdown_text);
                    lv_task_handler();
                    // Call activate_mode function when a valid countdown value is reached
                    activate_mode(countdown_value);
                }
                // Delay to debounce the button release
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Check button state every 100ms
    }
}
//==================================================================================================================================================================================
/*Main App*/
void app_main(void)
{
   xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL) {
        printf("Failed to create mutex\n");
        return;
    }
  //Initialize NVS (Non-Volatile Storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
        ESP_ERROR_CHECK(ret);
    }
    // Store default config in NVS if not already stored
    store_default_config_in_nvs();
    storage_init();
    const char *base_path = "/fatfs";
    ret = example_mount_storage(base_path);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount fatfs storage.");
        return;
    }
   usb_msc_init();
   uart_init();
   generate_mqtt_topics(topic_rx, topic_tx, sizeof(topic_rx));
   oled_init();
   get_sensor_id(sensor_id);
   // Load the last mode and activate it
      int last_mode = load_last_mode();
      if (last_mode > 0 && last_mode <= COUNTDOWN_THRESHOLD) {
          activate_mode(last_mode);
      }
   xTaskCreate(sensor_data, "sensor_data", 8192, NULL, 2, NULL);
   xTaskCreate(monitor_file_task, "monitor_file_task", 4096, NULL, 2, NULL);
   xTaskCreate(button_task, "button_task", 4096, NULL, 1, NULL);
 while (1) {
     if (tud_cdc_connected()) {
         tud_cdc_write_str("ESP32-S3: Hello from USB CDC + MSC mode!\r\n");
         tud_cdc_write_flush();
     }
	if(strt_flg==1){restart_tasks();strt_flg=0;}
	 if(flag_serverstart==1){
		 wifi_initate_with_Server();flag_serverstart=0;
	 }
	 interval_uploading();
     vTaskDelay(pdMS_TO_TICKS(1000));
    }   
 }
