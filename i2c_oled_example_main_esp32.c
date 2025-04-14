#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/param.h>
#include <sys/unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include "esp_vfs.h"
#include "esp_spiffs.h"
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
#include "esp_mac.h"
#include <math.h>


// UART Configuration
#define UART_NUM UART_NUM_1
#define UART_NUM_UART2 UART_NUM_2
#define UART_TXD1_PIN 4
#define UART_RXD1_PIN 2
#define UART_TXD2_PIN 16
#define UART_RXD2_PIN 17
#define BUF_SIZE 1024
#define RD_BUF_SIZE BUF_SIZE

#define I2C_BUS_PORT  0
#define OLED_PIXEL_CLOCK_HZ    (400 * 1000)
#define OLED_PIN_NUM_SDA       21
#define OLED_PIN_NUM_SCL       22
#define OLED_PIN_NUM_RST       -1
#define OLED_I2C_HW_ADDR       0x3C
#define OLED_H_RES             128
#define OLED_V_RES             64
#define OLED_CMD_BITS          8
#define OLED_PARAM_BITS        8

#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN)
#define MAX_FILE_SIZE   (200*1024) // 200 KB      
#define MAX_FILE_SIZE_STR "200KB"
#define SCRATCH_BUFSIZE  2048
#define NUM_TARGETS 5
#define NUM_COMMANDS 20
#define BLE_NOTIFY_MAX_LEN 20  // Define your BLE notify size limit
#define CONNECT_TIMEOUT_MS 20000
#define CONNECT_TIMEOUT_MS_wifi 60000
#define CHECK_INTERVAL 500   
#define TIMEOUT_MS 1000

#define TAG "FILE_MONITOR"
#define FILE_PATH "/spiffs/combined_data.txt" // Adjust the path as per your file system setup
#define MAX_FILE_SIZE_MB 2
#define MAX_FILE_SIZE_BYTES (MAX_FILE_SIZE_MB * 1024 * 1024)
#define LINES_TO_DELETE 100

static char wifi_ip_address[16] = {0};
static bool spiffs_initialized = false; // Flag to track SPIFFS initialization
void example_lvgl_demo_ui(lv_disp_t *disp, double Result[5]);
uint8_t cmd_rcv=0;
// Function prototype
double unpack_double(uint8_t *bytes_received);
void ble_app_advertise(void);
void combine_data_and_calculate();
static uint16_t tx_handle;
// Declare the function prototype
void convertDoubleArrayToString(const double *arr, int size, char *result);
char buff_rec_data[1024];
char combine_array[1024];
char combine_array1[1024];
char combine_array_ble[1024];
const char* topic_rx = "hydrosensor/hydro/TX"; // Subscribe topic
const char* topic_tx = "hydrosensor/hydro/RX"; // Publish topic
double responseArray[NUM_COMMANDS] = {0};
double SensorResponse[NUM_COMMANDS] = {0};
double Result[NUM_TARGETS] = {0};
void combine_data_string();
bool flag_uploading_uart2 = false;
void wifi_initate_with_Server();
bool flag_serverstart = false;
bool flag_uploading_internal_data_save = false;
void update_config_from_file(const char *file_path);
 // Mutex handle
SemaphoreHandle_t xMutex;
const char *base_path = "/spiffs";
esp_err_t example_restart_file_server(const char *base_path);
static void wifi_init_sta(void);
/*#define WIFI_SSID "aa"
#define WIFI_PASS "00000000"*/
char WIFI_SSID[32] = "aa";
char WIFI_PASS[32] = "00000000";
static bool wifi_connected = false;
int INTERVAL;
char SEN_ID[50];
 //==========================================================================================================================================================
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
    char base_path[ESP_VFS_PATH_MAX + 1];       //Base path of file storage
    char scratch[SCRATCH_BUFSIZE];              //Scratch buffer for temporary storage during file transfer
};
// Logger tag
static const char *TAG = "BLE_UART";
//===========================================================================================================================================================
//double SensorResponse[20] = {0.689, 0.609, 0.149, 0.413, 0.028, 0.108, 0.079, 0.073, 0.282, 0.107, 0.144, 0.583, 0.043, 0.043, 0.012, 0.014, 0.69, 0.609, 0.144, 0.584};
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
double Temp_Coeff[3] = {0.5, 0.5, 22};
double Cond_Coeff[3] = {0.15, 0.25, -5.5};
double TDS_Coeff = 0.5;
int MAX_ITERS = 5;
int MAX_ITERS_LS = 5;
double MAX_TOL = 0.001;
//===========================================================================================================================================================
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
//==============================================================================================================================================================
static void print_coefficients() {
    ESP_LOGI(TAG, "Printing Updated Coefficients:");
   // printf("Temp_Coeff: {%.2f, %.2f, %.2f}\n", Temp_Coeff[0], Temp_Coeff[1], Temp_Coeff[2]);
  //  printf("Cond_Coeff: {%.2f, %.2f, %.2f}\n", Cond_Coeff[0], Cond_Coeff[1], Cond_Coeff[2]);
  //  printf("TDS_Coeff: %.2f\n", TDS_Coeff);
   //  printf("MAX_ITERS: %d\n", MAX_ITERS);
    // printf("MAX_ITERS_LS: %d\n", MAX_ITERS_LS);
    //  printf("MAX_TOL: %.3f\n", MAX_TOL);
    // Print the coefficient matrix for Targets (Coeff)
    for (int t = 0; t < 16; t++) {
        printf("Coefficients for Target %d: {", t + 1);
        for (int i = 0; i < 5; i++) {
           // printf("{%.2f, %.2f}", Coeff[t][i], Coeff[t][i + 1]);
            printf("%.2f", Coeff[t][i]);
            if (i < 5 - 1) { // Only add a comma if it's not the last pair
                printf(", ");
            }
        }
        printf("}\n");
    }
}
//=============================================================================================================================================================
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false; // Reset connection flag
        esp_wifi_connect();
        ESP_LOGI(TAG, "Retrying Wi-Fi connection...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        snprintf(wifi_ip_address, sizeof(wifi_ip_address), IPSTR, IP2STR(&event->ip_info.ip)); // Store IP address
        wifi_connected = true;
        //ESP_LOGI(TAG, "Got IP: %s", wifi_ip_address);
    }
}
//=============================================================================================================================================
static void wifi_init_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    // Register Wi-Fi event handlers
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    // Wi-Fi configuration
   /* wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };*/
   wifi_config_t wifi_config = {
        .sta = {
            .ssid = {0},       // Ensure it's properly initialized
            .password = {0},   // Ensure it's properly initialized
        },
    };
    // Copy the updated Wi-Fi credentials into the struct
    strncpy((char *)wifi_config.sta.ssid, (char *)WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, (char *)WIFI_PASS, sizeof(wifi_config.sta.password) - 1);     
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
   // ESP_LOGI(TAG, "Connecting to Wi-Fi...");
    // Display countdown on the OLED
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    int elapsed_time = 0;
    int countdown_time = CONNECT_TIMEOUT_MS_wifi / 1000; // Countdown in seconds
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
       // ESP_LOGI(TAG, "Wi-Fi connected successfully.");
        // Update OLED display to show IP address
        char success_text[64];
        snprintf(success_text, sizeof(success_text), "Connected!\nIP: %s", wifi_ip_address);
        lv_label_set_text(label, success_text);
        lv_task_handler();
        // Start the file server
        example_restart_file_server(base_path);
    } else {
        //ESP_LOGW(TAG, "Wi-Fi connection timed out.");
        // Update OLED display to show timeout message
        lv_label_set_text(label, "No Hotspot Found.");
        lv_task_handler();
        // Stop Wi-Fi to save power
        esp_wifi_stop();
    }
}
//=============================================================================================================================================
char* combine_char_arrays(const char *array1, const char *array2, const uint8_t *mac_address) {
    if (array1 == NULL || array2 == NULL || mac_address == NULL) {
        return NULL; // Handle invalid inputs
    }
    // Calculate the size of the final string
    size_t len1 = strlen(array1);
    size_t len2 = strlen(array2);
    size_t mac_len = 17; // MAC address in "XX:XX:XX:XX:XX:XX" format
    size_t total_len = len1 + len2 + mac_len + 1; // Include space for null terminator
    // Allocate memory for the combined string
    char *result = (char *)malloc(total_len);
    if (result == NULL) {
        return NULL; // Handle memory allocation failure
    }
    // Format the MAC address as a string
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_address[0], mac_address[1], mac_address[2],
             mac_address[3], mac_address[4], mac_address[5]);
    // Combine the strings
    snprintf(result, total_len, "%s%s%s", array1, array2, mac_str);
    return result;
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
        ESP_LOGI("ESP32_MAC", "MAC ID retrieved successfully");
    } else {
        ESP_LOGE("ESP32_MAC", "Failed to retrieve MAC ID, error: %d", err);
        memset(mac_id, 0, 6); // Assign default zeros in case of failure
    }
}
//============================================================================================================================================
void get_sensor_id(uint8_t *sensor_id) {
    uint8_t command[] = {0x63, 0x00, 0x3B}; // Command to request sensor ID
    uint8_t response[16] = {0};           // Buffer for response
    const uint8_t default_id[16] = {0};   // Default ID when no response is received
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
    } else {
        // No valid response, set default ID
        memcpy(sensor_id, default_id, sizeof(default_id));
        ESP_LOGW(TAG, "No response received, setting default ID");
    }
}
//=============================================================================================================================================
// Custom function to read a line from a file
static ssize_t custom_getline(char **lineptr, size_t *n, FILE *stream) {
    if (!lineptr || !n || !stream) {
        return -1;
    }
    size_t size = 0;
    if (*lineptr == NULL || *n == 0) {
        *n = 128; // Initial buffer size
        *lineptr = malloc(*n);
        if (!*lineptr) {
            return -1;
        }
    }
    char *ptr = *lineptr;
    int c;
    while ((c = fgetc(stream)) != EOF) {
        if (size + 1 >= *n) {
            *n *= 2;
            char *new_ptr = realloc(*lineptr, *n);
            if (!new_ptr) {
                return -1;
            }
            ptr = new_ptr + (ptr - *lineptr);
            *lineptr = new_ptr;
        }
        *ptr++ = c;
        size++;
        if (c == '\n') {
            break;
        }
    }
    if (size == 0) {
        return -1;
    }
    *ptr = '\0';
    return size;
}
//=============================================================================================================================================
static void trim_file(const char *file_path, size_t lines_to_remove) {
    FILE *file = fopen(file_path, "r");
    if (file == NULL) {
       // ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
    FILE *temp_file = fopen("/spiffs/temp_file.txt", "w");
    if (temp_file == NULL) {
        //ESP_LOGE(TAG, "Failed to create temporary file");
        fclose(file);
        return;
    }
    // Read and skip the first `lines_to_remove` lines
    char *buffer = NULL;
    size_t buffer_size = 0;
    ssize_t line_length;
    size_t lines_skipped = 0;
    while (lines_skipped < lines_to_remove && (line_length = custom_getline(&buffer, &buffer_size, file)) != -1) {
        lines_skipped++;
    }
    // Write the remaining lines to the temporary file
    while ((line_length = custom_getline(&buffer, &buffer_size, file)) != -1) {
        fwrite(buffer, 1, line_length, temp_file);
    }
    free(buffer);
    fclose(file);
    fclose(temp_file);
    // Replace the original file with the temporary file
    if (remove(file_path) != 0) {
       // ESP_LOGE(TAG, "Failed to remove the original file");
    } else if (rename("/spiffs/temp_file.txt", file_path) != 0) {
        //ESP_LOGE(TAG, "Failed to rename the temporary file");
    }
}
//=============================================================================================================================================
static size_t get_file_size(const char *file_path) {
    struct stat st;
    if (stat(file_path, &st) != 0) {
        ESP_LOGE(TAG, "Failed to get file stats");
        return 0;
    }
    return st.st_size;
}
//=============================================================================================================================================
static void monitor_file_task(void *arg) {
    while (1) {
        size_t file_size = get_file_size(FILE_PATH);
        if (file_size > MAX_FILE_SIZE_BYTES) {
            ESP_LOGW(TAG, "File size exceeded %d MB. Current size: %zu bytes", MAX_FILE_SIZE_MB, file_size);
            trim_file(FILE_PATH, LINES_TO_DELETE);
        } else {
            ESP_LOGI(TAG, "File size is under limit. Current size: %zu bytes", file_size);
        }
        //vTaskDelay(pdMS_TO_TICKS(3600000)); // Wait for 1 hour
    vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
//=============================================================================================================================================
static void update_coefficients(const char *file_path) {
    update_config_from_file(file_path);
    print_coefficients();                                                                                  // Print updated coefficients 
}
//=============================================================================================================================================
void update_config_from_file(const char *file_path) {
    FILE *file = fopen(file_path, "r");
    if (!file) {
       // ESP_LOGE(TAG, "Failed to open file: %s", file_path);
        return;
    }
    // Get the file size
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    rewind(file);
    if (file_size <= 0) {
      //  ESP_LOGE(TAG, "Invalid file size: %ld", file_size);
        fclose(file);
        return;
    }
    // Allocate memory to read the file content
    char *json_data = (char *)malloc(file_size + 1);
    if (!json_data) {
       // ESP_LOGE(TAG, "Failed to allocate memory for file content");
        fclose(file);
        return;
    }
    // Read the file content
    fread(json_data, 1, file_size, file);
    json_data[file_size] = '\0'; // Null-terminate the string
    fclose(file);
    //ESP_LOGI(TAG, "File content read successfully");
    // Parse the JSON data
    cJSON *root = cJSON_Parse(json_data);
    free(json_data); // Free memory after parsing
    if (!root) {
        //ESP_LOGE(TAG, "Failed to parse JSON");
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
    if (cJSON_IsArray(temp_coeff) && cJSON_GetArraySize(temp_coeff) == 3) {
        for (int i = 0; i < 3; i++) {
            Temp_Coeff[i] = cJSON_GetArrayItem(temp_coeff, i)->valuedouble;
        }
        ESP_LOGI(TAG, "Updated Temp_Coeff: {%.2f, %.2f, %.2f}", Temp_Coeff[0], Temp_Coeff[1], Temp_Coeff[2]);
    }
    // Update Cond_Coeff
    cJSON *cond_coeff = cJSON_GetObjectItem(root, "Cond_Coeff");
    if (cJSON_IsArray(cond_coeff) && cJSON_GetArraySize(cond_coeff) == 3) {
        for (int i = 0; i < 3; i++) {
            Cond_Coeff[i] = cJSON_GetArrayItem(cond_coeff, i)->valuedouble;
        }
        ESP_LOGI(TAG, "Updated Cond_Coeff: {%.2f, %.2f, %.2f}", Cond_Coeff[0], Cond_Coeff[1], Cond_Coeff[2]);
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
//=============================================================================================================================================
void interval_uploading(){
        static unsigned long lastMsg = 0;
        unsigned long now = esp_log_timestamp();
        if(INTERVAL!=0){	
        if (now - lastMsg > INTERVAL*1000*60) {
            lastMsg = now;
            flag_uploading_uart2=1;
            flag_uploading_internal_data_save=1;
        }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 1 second
}
//=============================================================================================================================================
void copy_to_const_char(const char **topic_tx, const char *src) {
    if (!topic_tx || !src) {
        printf("Invalid pointers\n");
        return;
    }
    // Allocate memory for the destination string
    *topic_tx = malloc(strlen(src) + 1);
    if (*topic_tx == NULL) {
        printf("Memory allocation failed\n");
        return;
    }
    // Copy the source string into the allocated memory
    strcpy((char *)(*topic_tx), src);
}
//=============================================================================================================================================
void update_topic(){
	copy_to_const_char(&topic_tx, SEN_ID);
}
//=============================================================================================================================================================
static esp_err_t index_html_get_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "index_html_get_handler called for URI: %s", req->uri);
    httpd_resp_set_status(req, "307 Temporary Redirect");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);                                                                              //Response body can be empty
    return ESP_OK;
}
//=============================================================================================================================================================
static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    extern const unsigned char favicon_ico_start[] asm("_binary_favicon_ico_start");
    extern const unsigned char favicon_ico_end[]   asm("_binary_favicon_ico_end");
    const size_t favicon_ico_size = (favicon_ico_end - favicon_ico_start);
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_send(req, (const char *)favicon_ico_start, favicon_ico_size);
    return ESP_OK;
}
//=============================================================================================================================================================
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
//=============================================================================================================================================================
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
//=============================================================================================================================================================
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
//=============================================================================================================================================================
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
       // ESP_LOGE(TAG, "Failed to stat file : %s", filepath);      
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");                                 //Respond with 404 Not Found
        return ESP_FAIL;
    }
    fd = fopen(filepath, "r");
    if (!fd) {
       // ESP_LOGE(TAG, "Failed to read existing file : %s", filepath);    
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");            //Respond with 500 Internal Server Error
        return ESP_FAIL;
    }
    //ESP_LOGI(TAG, "Sending file : %s (%ld bytes)...", filename, file_stat.st_size);
    set_content_type_from_file(req, filename);
    /* Retrieve the pointer to scratch buffer for temporary storage */
    char *chunk = ((struct file_server_data *)req->user_ctx)->scratch;
    size_t chunksize;
    do {      
        chunksize = fread(chunk, 1, SCRATCH_BUFSIZE, fd);                                                    //Read file in chunks into the scratch buffer
        if (chunksize > 0) {       
            if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK) {                                    //Send the buffer contents as HTTP response chunk
                fclose(fd);
              //  ESP_LOGE(TAG, "File sending failed!");              
                httpd_resp_sendstr_chunk(req, NULL);                                                         //Abort sending file               
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");            //Respond with 500 Internal Server Error
               return ESP_FAIL;
           }
        }     
    } while (chunksize != 0);                                                                                //Keep looping till the whole file is sent
    fclose(fd);                                                                                              //Close file after sending complete
   // ESP_LOGI(TAG, "File sending complete");
    /* Respond with an empty chunk to signal HTTP response completion */
#ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
    httpd_resp_set_hdr(req, "Connection", "close");
#endif
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}
//=============================================================================================================================================================
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
       // ESP_LOGE(TAG, "Invalid filename : %s", filename);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid filename");
        return ESP_FAIL;
    }
    if (stat(filepath, &file_stat) == 0) {
       // ESP_LOGE(TAG, "File already exists : %s", filepath);     
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File already exists");                                          //Respond with 400 Bad Request
        return ESP_FAIL;
    }
    /* File cannot be larger than a limit */
    if (req->content_len > MAX_FILE_SIZE) {
      //  ESP_LOGE(TAG, "File too large : %d bytes", req->content_len);       
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File size must be less than " MAX_FILE_SIZE_STR "!");            //Respond with 400 Bad Request
        return ESP_FAIL;
    }
    fd = fopen(filepath, "w");
    if (!fd) {
      //  ESP_LOGE(TAG, "Failed to create file : %s", filepath);     
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to create file");                                //Respond with 500 Internal Server Error
        return ESP_FAIL;
    }
    //ESP_LOGI(TAG, "Receiving file : %s...", filename);   
    char *buf = ((struct file_server_data *)req->user_ctx)->scratch;                                                      //Retrieve the pointer to scratch buffer for temporary storage
    int received;
    /*Content length of the request gives the size of the file being uploaded */    
    int remaining = req->content_len;
    while (remaining > 0) {
        //ESP_LOGI(TAG, "Remaining size : %d", remaining);       
        if ((received = httpd_req_recv(req, buf, MIN(remaining, SCRATCH_BUFSIZE))) <= 0) {                                //Receive the file part by part into a buffer
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {           
                continue;                                                                                                 //Retry if timeout occurred
            }                   
            fclose(fd);                                                                                                  //In case of unrecoverable error, close and delete the unfinished file
            unlink(filepath);
           // ESP_LOGE(TAG, "File reception failed!");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive file");                        //Respond with 500 Internal Server Error
            return ESP_FAIL;
        }     
        if (received && (received != fwrite(buf, 1, received, fd))) {                                                   //Write buffer content to file on storage          
            fclose(fd);                                                                                                 //Couldn't write everything to file!Storage may be full?
            unlink(filepath);
           // ESP_LOGE(TAG, "File write failed!");          
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to write file to storage");               //Respond with 500 Internal Server Error
            return ESP_FAIL;
        }        
        remaining -= received;                                                                                          //Keep track of remaining size of  the file left to be uploaded
    }    
    fclose(fd);                                                                                                         //Close file upon upload completion
   // ESP_LOGI(TAG, "File reception complete");
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
//=============================================================================================================================================================
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
//=============================================================================================================================================================
/* Function to start the file server */
esp_err_t example_start_file_server(const char *base_path)
{
    static struct file_server_data *server_data = NULL;
    if (server_data) {
       // ESP_LOGE(TAG, "File server already started");
        return ESP_ERR_INVALID_STATE;
    }
    /* Allocate memory for server data */
    server_data = calloc(1, sizeof(struct file_server_data));
    if (!server_data) {
       // ESP_LOGE(TAG, "Failed to allocate memory for server data");
        return ESP_ERR_NO_MEM;
    }
    strlcpy(server_data->base_path, base_path,sizeof(server_data->base_path));           
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    // Increase send wait timeout
    config.send_wait_timeout = 30; // 30 seconds
    
    //ESP_LOGI(TAG, "Starting HTTP Server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) != ESP_OK) {
       // ESP_LOGE(TAG, "Failed to start file server!");
        return ESP_FAIL;
    }
   // ESP_LOGI(TAG, "HTTP server started successfully");
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
//==============================================================================================================================================================
void combine_data_string2(){
char combine_result[100];
convertDoubleArrayToString(responseArray,20,combine_array_ble);
if (xSemaphoreTake(xMutex, portMAX_DELAY)) {//3
strcat(combine_array_ble, ",");
 xSemaphoreGive(xMutex);
    }
convertDoubleArrayToString(Result,5,combine_result);
if (xSemaphoreTake(xMutex, portMAX_DELAY)) {//4
strcat(combine_array_ble, combine_result);
 xSemaphoreGive(xMutex);
    }
}
//=============================================================================================================================================
void combine_data_string1() {
    char formatted_data1[4096] = {0}; // Allocate enough space for JSON
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
    ESP_LOGI(TAG, "Formatted JSON: %s", combine_array1);
}
//==============================================================================================================================================================
/*GATT Service Access Callback for Receiving and Sending Data Over BLE */
static int gatt_svr_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        /*Handle write operation (data sent from BLE client to RX characteristic) */
        //ESP_LOGI(TAG, "Received data over BLE: %.*s", ctxt->om->om_len, ctxt->om->om_data);
       /*Process received data */
        char buffer[64] = {0};
        if (ctxt->om->om_len > 0 && ctxt->om->om_len < sizeof(buffer)) {
            strncpy(buffer, (char *)ctxt->om->om_data, ctxt->om->om_len);
           // ESP_LOGI(TAG, "Received command: %s", buffer);
            
           if (strchr(buffer, 'a')) {
            //ESP_LOGI(TAG, "Step1: Buffer contains 'a'");
              combine_data_string2();
           // Send `combine_array` as a notification to the client
            struct os_mbuf *om = ble_hs_mbuf_from_flat(combine_array_ble, strlen(combine_array_ble));
            if (om) {
                int rc = ble_gatts_notify_custom(conn_handle, tx_handle, om);
                if (rc == 0) {
                   // ESP_LOGI(TAG, "Notification sent to BLE client: %s", combine_array_ble);
                } else {
                   // ESP_LOGE(TAG, "Failed to send notification: %d", rc);
                }
            }
           }
             if (strchr(buffer, 'b')) {
             // ESP_LOGI(TAG, "Step2: Buffer contains 'b'");
              flag_serverstart=1;
           }
        } else {
           // ESP_LOGW(TAG, "Invalid or oversized BLE write data.");
        }
    } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
       /*Handle read operation (send data from server to BLE client)*/  
        os_mbuf_append(ctxt->om, combine_array_ble, strlen(combine_array_ble));  
       // ESP_LOGI(TAG, "Sent read response: %s", combine_array_ble);
    }
    return 0;
}
//==============================================================================================================================================================
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
//==============================================================================================================================================================
/*BLE GATT Service Registration*/
static void gatt_svr_register_svcs(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    int rc;
    rc = ble_gatts_count_cfg(gatt_svr_svcs);                                                                 //Count configuration
    if (rc != 0) {
       // ESP_LOGE(TAG, "Failed to count GATT services, rc=%d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(gatt_svr_svcs);                                                                  //Add services
    if (rc != 0) {
       // ESP_LOGE(TAG, "Failed to add GATT services, rc=%d", rc);
        return;
    }
   // ESP_LOGI(TAG, "GATT Services Registered");
    vTaskDelay(100/ portTICK_PERIOD_MS);                                   
}
//==============================================================================================================================================================
/*BLE event handling*/
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
               // ESP_LOGI(TAG, "Connected to client."); 
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
//==============================================================================================================================================================
/*BLE Host Task*/
static void ble_host_task(void *param)
{
    nimble_port_run();
    vTaskDelay(pdMS_TO_TICKS(100));                                                                                             //Run NimBLE event loop (BLE tasks)
}
//==============================================================================================================================================================
/*BLE Advertising Configuration Callback*/
static void ble_app_on_sync(void) {
    uint8_t addr_val[6] = {0};
    ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr_val, NULL);
    ESP_LOGI(TAG, "Device address: %02x:%02x:%02x:%02x:%02x:%02x",
             addr_val[5], addr_val[4], addr_val[3],
             addr_val[2], addr_val[1], addr_val[0]);
    ble_app_advertise();
  }
//==============================================================================================================================================================
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
       // ESP_LOGE(TAG, "Failed to start advertising, rc=%d", rc);
    } else {
        //ESP_LOGI(TAG, "Advertising started successfully with NUS UUID.");
    }
}

//==============================================================================================================================================================
/*BLE Initialization*/
void ble_init() {
    ble_svc_gap_init();
    ble_svc_gatt_init();  
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    gatt_svr_register_svcs(NULL, NULL);
    nimble_port_freertos_init(ble_host_task);
}
//==============================================================================================================================================================
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
     //ESP_LOGI(TAG, "UART1 Initialized");
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
   // ESP_LOGI(TAG, "UART2 Initialized");
}
//==============================================================================================================================================================
void upload_everymin_uart2() {
    if(flag_uploading_uart2 == 1) {
        combine_data_string2();  // This will update combine_array with the JSON data.
        if (strlen(combine_array_ble) > 0) {
            // Send the formatted JSON string (combine_array) over UART2
            uart_write_bytes(UART_NUM_UART2, "S", 1);
           //uart_write_bytes(UART_NUM_UART2, SEN_ID, strlen(SEN_ID));
            //uart_write_bytes(UART_NUM_UART2, " ", 1);  // Add a space after SEN_ID
            uart_write_bytes(UART_NUM_UART2, combine_array_ble, strlen(combine_array_ble));
             uart_write_bytes(UART_NUM_UART2, "E", 1);
            //uart_write_bytes(UART_NUM_UART2, "\r\n", 2);  // Add a line break after the data
            ESP_LOGI(TAG, "Sent data over UART2: %s", combine_array_ble);
             //ESP_LOGI(TAG, "Sent data over UART2: S %s %s E", SEN_ID, combine_array_ble);
            flag_uploading_uart2 = 0;
        } else {
            ESP_LOGW(TAG, "No valid data to send.");
        }
    }
}
//=============================================================================================================================================
// Add this function to periodically send combined data to UART2
void send_data_to_uart2_task(void *param) {
     while (1) {
        upload_everymin_uart2();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
//================================================================================================================================================================
/*Updated MQTT receive task with status checks*/
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
//===============================================================================================================================================================
void combine_data_string() {
    char formatted_data[2048] = {0};                                                                   //Allocate enough space for JSON
    strcat(formatted_data, "{"); 
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
    strcat(formatted_data, ",\"header\":{");    
     const char* headers[] = {
        "R11", "R12", "R13", "R14", "R21", "R22", "R23", "R24",
        "R31", "R32", "R33", "R34", "T1", "T2", "C1", "C2",
        "R11_2W", "R12_2W", "R33_2W", "R34_2W",
        "CL", "PH", "CND", "TDS", "TEMP"
    };                                                       //Add header section
    for (int i = 0; i < 25; i++) {
        char header_entry[64];
        snprintf(header_entry, sizeof(header_entry), "\"h%d\":\"%s\"", i + 1, headers[i]);
        strcat(formatted_data, header_entry);
        if (i < 24) {                                                                                  //Add a comma for all except the last element
            strcat(formatted_data, ",");
        }
    }
    strcat(formatted_data, "}}"); 
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {                                                                      //Close JSON object
    strcpy(combine_array, formatted_data);   
     xSemaphoreGive(xMutex);
    }                                                           //Copy the result to combine_array
    //ESP_LOGI(TAG, "Formatted JSON: %s", combine_array);
}
//==============================================================================================================================================================
/*Function to send a command over UART and read the response*/
void send_command_and_receive_response(int commandIndex) {
    ESP_LOGI(TAG, "Executing command %d", commandIndex); 
    int commandLength = command_sizes[commandIndex];                                                                   //Get the correct length for the command
    ESP_LOGI(TAG, "Sending command: ");
    for (int i = 0; i < commandLength; i++) {
        printf("%02X ", commands[commandIndex][i]);
    }
    printf("\n");
    int bytes_sent = uart_write_bytes(UART_NUM, (const char *)commands[commandIndex], commandLength);                 //Send the command via UART
    if (bytes_sent != commandLength) {
        ESP_LOGE(TAG, "Error sending command. Sent %d bytes out of %d.", bytes_sent, commandLength);
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
                ESP_LOGI(TAG, "Converted ValueresponseArray: %.8f", responseArray[commandIndex]);
            } else {
                ESP_LOGW(TAG, "Incomplete response received.");
            } 
}
//==============================================================================================================================================================
/*Convert 4-byte response to a double*/
double unpack_double(uint8_t *bytes_received) {
    float temp_float;
    memcpy(&temp_float, bytes_received, sizeof(float));
    return (double)temp_float;
}
//==============================================================================================================================================================
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
//===================================================================================================================================
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
//=====================================================================================================================================
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
//========================================================================================================================================
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
//=========================================================================================================================================
void processSensorResponse() {
    memcpy(SensorResponse, responseArray, sizeof(responseArray));  // Copy data directly
     for (int i = 0; i < 20; i++) {
        ESP_LOGI(TAG, "SensorResponse[%d]: %f", i, SensorResponse[i]);
    }
}
//============================================================================================================================================
void EstimateTarget(double SensorResponse[20], double Coeff[16][5], double Temp_Coeff[3], double Cond_Coeff[3], double TDS_Coeff, int MAX_ITERS, int MAX_ITERS_LS, double MAX_TOL, double Result[5]) {
	processSensorResponse();
    double Temp_resp[2] = {SensorResponse[12], SensorResponse[13]};
    double T_ref = Temp_Coeff[0] * Temp_resp[0] + Temp_Coeff[1] * Temp_resp[1] + Temp_Coeff[2];
    double Cond_resp[2] = {SensorResponse[14], SensorResponse[15]};
    double C_ref = Cond_Coeff[0] / (Cond_resp[0] - Cond_Coeff[1] * T_ref - Cond_Coeff[2]);
    double TDS_ref = C_ref * TDS_Coeff;
    double Resp[16];
   /* for (int i = 0; i < 16; i++) {
        Resp[i] = (SensorResponse[i] / Coeff[i][4]) - 1;
    }*/
     for (int i = 0; i < 12; i++) {
        Resp[i] = (SensorResponse[i] / Coeff[i][4]) - 1;
    }
    for (int i = 16; i < 20; i++) {
        Resp[i-4] = (SensorResponse[i] / Coeff[i-4][4]) - 1;
    }
    double OptimX[2];
    double X_0[2] = {7.0, 0.0};
    SD_optim(Coeff, Resp, X_0, MAX_ITERS, MAX_ITERS_LS, MAX_TOL, OptimX);
/*    double pH_ref = OptimX[0];
    double Chlorine_ref = OptimX[1];*/
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
//==============================================================================================================================================================
void combine_data_and_calculate() {
	 int64_t start_time = esp_timer_get_time(); 
   // ESP_LOGI(TAG, "Executing all commands due to invalid input...");
    vTaskDelay(pdMS_TO_TICKS(100));
    /*Flush UART buffer before sending commands*/
    uint8_t temp[BUF_SIZE];
    while (uart_read_bytes(UART_NUM, temp, BUF_SIZE, pdMS_TO_TICKS(100)) > 0) {
        ESP_LOGI(TAG, "Flushing old UART data: %.*s", BUF_SIZE, temp);
    }
    /*Step 1: Send all commands and wait for responses*/
    for (int i = 0; i < NUM_COMMANDS; i++) {
        ESP_LOGI(TAG, "Sending command %d...", i);
        send_command_and_receive_response(i);
        vTaskDelay(pdMS_TO_TICKS(100));                                             
    }
    /*Step 2: Perform calculations after collecting all responses*/
     EstimateTarget(SensorResponse, Coeff, Temp_Coeff, Cond_Coeff, TDS_Coeff, MAX_ITERS, MAX_ITERS_LS, MAX_TOL, Result);
    lv_disp_t *disp = lv_disp_get_default();                                                                  //Get the default LVGL display
    example_lvgl_demo_ui(disp, Result); 
    int64_t end_time = esp_timer_get_time();  // Store end time
    ESP_LOGI(TAG, "combine_data_and_calculate() execution time: %lld microseconds", end_time - start_time); 
}
//==============================================================================================================================================================
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
//=======================================================================
// Image descriptor for the logo
static const lv_img_dsc_t hydroscope_logo = {
    .header.cf = LV_IMG_CF_ALPHA_1BIT,  // Color format: 1-bit indexed
    .header.always_zero = 0,
    .header.reserved = 0,
    .header.w = 128,                      // Width of the image
    .header.h = 64,                       // Height of the image
    .data_size = sizeof(hydroscope_logo_bitmap), // Data size
    .data = hydroscope_logo_bitmap,         // Pointer to the bitmap data
};
//===========================================================================================================================================
/*Function to create and display the results on the OLED*/
void example_lvgl_demo_ui(lv_disp_t *disp, double Result[5]) {
    lv_obj_t *scr = lv_disp_get_scr_act(disp);                                                              //Get the active screen    
    lv_obj_clean(scr);   
      const char *result_names[] = {"CL", "PH", "CND", "TDS", "TEMP"};                                                                                    //Clear the screen before displaying new results
   /*Create and display a label for each target*/
    for (int i = 0; i < 5; i++) {
        lv_obj_t *label = lv_label_create(scr);                                                            //Create a new label
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
//==============================================================================================================================================================
void oled_init(void) {
   // ESP_LOGI(TAG, "Initialize I2C bus");
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
   // ESP_LOGI(TAG, "Install SSD1306 panel driver");
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
   // ESP_LOGI(TAG, "Initialize LVGL");
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
    // Display the Hydroscope logo at startup
   // ESP_LOGI(TAG, "Display Hydroscope logo");
    lv_obj_t *scr = lv_disp_get_scr_act(disp);  // Get the active screen
    lv_obj_clean(scr);                          // Clear the screen
    lv_obj_t *img = lv_img_create(lv_scr_act());        // Create the image object
    lv_img_set_src(img, &hydroscope_logo);      // Set the logo source
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);   // Center the image
    lv_task_handler();                          // Refresh LVGL display
    vTaskDelay(pdMS_TO_TICKS(2000));            // Display logo for 3 seconds
    lv_obj_clean(scr);
    wifi_init_sta();
    vTaskDelay(pdMS_TO_TICKS(500)); 
       if (lvgl_port_lock(0)) {
       example_lvgl_demo_ui(disp, Result);
       lvgl_port_unlock();
    }  
}
//============================================================================================================================================================
static esp_err_t get_latest_uploaded_file(char *file_path, size_t file_path_len) {
    DIR *dir = opendir("/spiffs");
    if (!dir) {
        //ESP_LOGE(TAG, "Failed to open SPIFFS directory.");
        return ESP_FAIL;
    }
    struct dirent *entry;
    struct stat file_stat;
    time_t latest_mtime = 0;                                                                             //To store the modification time of the latest file
    while ((entry = readdir(dir)) != NULL) {
        /*Skip internal file (combined_data.txt)*/
        if (strcmp(entry->d_name, "combined_data.txt") == 0) {
            continue;  // Skip internal file
        }
      /*Construct full path to file*/
        char full_path[FILE_PATH_MAX];
        /*Copy base path and append file name safely*/
        strlcpy(full_path, "/spiffs/", sizeof(full_path));
        if (strlcat(full_path, entry->d_name, sizeof(full_path)) >= sizeof(full_path)) {
            //ESP_LOGW(TAG, "File name too long, skipping: %s", entry->d_name);
            continue;  
        }       
        if (stat(full_path, &file_stat) == 0) {
            if (S_ISREG(file_stat.st_mode)) {                                                             //Check if it's a regular file
             /*Log the file's modification and creation times for debugging*/
             //  ESP_LOGI(TAG, "Checking file: %s, mtime: %lld, ctime: %lld", entry->d_name, file_stat.st_mtime, file_stat.st_ctime);          
                if (file_stat.st_mtime > latest_mtime) {
                    latest_mtime = file_stat.st_mtime;
                    strlcpy(file_path, full_path, file_path_len);                                         //Save the latest file path
                }
            }
        } else {
           // ESP_LOGW(TAG, "Failed to stat file: %s", entry->d_name);
        }
    }
    closedir(dir);
    if (latest_mtime == 0) {
        //ESP_LOGW(TAG, "No valid files found in SPIFFS.");
        return ESP_FAIL;
    }
    //ESP_LOGI(TAG, "Latest file found: %s", file_path);
    return ESP_OK;
}
//==============================================================================================================================================================
void spiffs_init() {
	if (spiffs_initialized) {
        //ESP_LOGW(TAG, "SPIFFS is already initialized. Skipping re-initialization.");
        return;
    }
    //ESP_LOGI(TAG, "Initializing SPIFFS...");
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",                                                                                // Mount point
        .partition_label = NULL,                                                                               //Use default partition label
        .max_files = 10,                                                                                        //Maximum number of files allowed
        .format_if_mount_failed = true                                                                         //Format if mount fails
    };
   /*Initialize and mount the SPIFFS file system*/
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            //ESP_LOGE(TAG, "Failed to mount or format filesystem.");
        } else if (ret == ESP_ERR_NOT_FOUND) {
           // ESP_LOGE(TAG, "Failed to find SPIFFS partition.");
        } else {
            //ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    } else {
        //ESP_LOGI(TAG, "SPIFFS initialized successfully.");
    }
#ifdef CONFIG_EXAMPLE_SPIFFS_CHECK_ON_START
    //ESP_LOGI(TAG, "Performing SPIFFS_check().");
    ret = esp_spiffs_check(conf.partition_label);
    if (ret != ESP_OK) {
        //ESP_LOGE(TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
        return;
    } else {
        //ESP_LOGI(TAG, "SPIFFS_check() successful.");
    }
#endif
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
       // ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s). Formatting...", esp_err_to_name(ret));
        esp_spiffs_format(conf.partition_label);                                                                  //Format the partition if info retrieval fails
        return;
    } else {
        //ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
   /*Check consistency of reported partition size info*/
    if (used > total) {
        //ESP_LOGW(TAG, "Number of used bytes cannot be larger than total. Performing SPIFFS_check().");
        ret = esp_spiffs_check(conf.partition_label);
        if (ret != ESP_OK) {
            //ESP_LOGE(TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
            return;
        } else {
           // ESP_LOGI(TAG, "SPIFFS_check() successful.");
        }
    }
     spiffs_initialized = true; 
   /*Dynamically find and load the latest file*/
    char latest_file_path[FILE_PATH_MAX] = {0};
    if (get_latest_uploaded_file(latest_file_path, sizeof(latest_file_path)) == ESP_OK) {
        ESP_LOGI(TAG, "Loading coefficients from the latest uploaded file: %s", latest_file_path);
        update_coefficients(latest_file_path);
    } else {
        ESP_LOGW(TAG, "No uploaded files found. Using default coefficients.");
    }
}
//=============================================================================================================================================================
esp_err_t example_mount_storage(const char *base_path) {
    if (spiffs_initialized) {
        //ESP_LOGI(TAG, "SPIFFS already initialized, mounting storage.");
        return ESP_OK;
    }
    spiffs_init();                                                                                             //Ensure SPIFFS is initialized before mounting
    if (!spiffs_initialized) {
        //ESP_LOGE(TAG, "SPIFFS initialization failed.");
        return ESP_FAIL;
    }
   // ESP_LOGI(TAG, "Mounting SPIFFS storage at base path: %s", base_path);
    return ESP_OK; 
}
//==============================================================================================================================================================
void store_data_to_spiffs(const char *file_path, const char *data) {
    FILE *file = fopen(file_path, "a");                                                                          //Open file in append mode
    if (file == NULL) {
       //ESP_LOGE(TAG, "Failed to open file for writing: %s", file_path);
        return;
    }
    // Get the current date and time
    time_t now;
    struct tm timeinfo;
    char time_buffer[64];
    time(&now);
    localtime_r(&now, &timeinfo);
    // Format the time as "YYYY-MM-DD HH:MM:SS"
    strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    // Write data and timestamp to the file
    fprintf(file, "%s | %s\n", data, time_buffer);                                                                             //Write data with a newline
    fclose(file);                                                                                              //Close the file
    ESP_LOGI(TAG, "Data with timestamp written to SPIFFS: %s | %s", data, time_buffer);
}
//==============================================================================================================================================================
void read_spiffs_file(const char *file_path) {
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
//==============================================================================================================================================================
void sensor_data(){
	while(1){
	combine_data_and_calculate();	
	 vTaskDelay(pdMS_TO_TICKS(100));
	 if (xSemaphoreTake(xMutex, portMAX_DELAY)) { //1
	 memset(combine_array, 0, sizeof(combine_array)); 
	  xSemaphoreGive(xMutex);
    }
	 if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
	  memset(combine_array1, 0, sizeof(combine_array1));
	   xSemaphoreGive(xMutex);
    }
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {//2
	    memset(combine_array_ble, 0, sizeof(combine_array_ble));
	     xSemaphoreGive(xMutex);
    } 
	 combine_data_string();  
	  //ESP_LOGI(TAG, "combined array: %s", combine_array);    
	   vTaskDelay(pdMS_TO_TICKS(100));                                         //Clear the buffer before use
	 combine_data_string1(); 
	// ESP_LOGI(TAG, "combined array1: %s", combine_array1);
	 vTaskDelay(pdMS_TO_TICKS(100));
	  if(flag_uploading_internal_data_save==1) {
		   store_data_to_spiffs("/spiffs/combined_data.txt",combine_array1);             //Store the combined data into SPIFFS every minute    
		   flag_uploading_internal_data_save=0;
	  }                                      
     //read_spiffs_file("/spiffs/combined_data.txt");                                                       //Read the file to verify the data is stored
     vTaskDelay(pdMS_TO_TICKS(100));                                                                
     combine_data_string2(); 
    // ESP_LOGI(TAG, "Non-JSON formatted combined array: %s", combine_array_ble);
	}
}
//==============================================================================================================================================
void wifi_initate_with_Server(){
	const char *base_path = "/spiffs";
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    //ESP_LOGI(TAG, "Connected to Wi-Fi");
    ESP_ERROR_CHECK(example_start_file_server(base_path));
   // ESP_LOGI(TAG, "File server started"); 
}
//============================================================================================================================================
esp_err_t example_restart_file_server(const char *base_path)
{
    static httpd_handle_t server = NULL;
    // Stop the server if it's already running
    if (server) {
        //ESP_LOGI(TAG, "Stopping the HTTP server");
        if (httpd_stop(server) != ESP_OK) {
           // ESP_LOGE(TAG, "Failed to stop the HTTP server");
            return ESP_FAIL;
        }
        //ESP_LOGI(TAG, "HTTP server stopped successfully");
        server = NULL;
    }
    // Restart the server
    //ESP_LOGI(TAG, "Restarting the HTTP server");
    esp_err_t err = example_start_file_server(base_path);
    if (err != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to restart the HTTP server");
        return err;
    }
    //ESP_LOGI(TAG, "HTTP server restarted successfully");
    return ESP_OK;
}
//==============================================================================================================================================================
/*Main App*/
void app_main(void)
{
   //ESP_LOGI(TAG, "Starting BLE UART Example");
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
   //ESP_LOGI(TAG, "NVS Initialized");
     spiffs_init();  
    const char *base_path = "/spiffs";
    ret = example_mount_storage(base_path);
    if (ret != ESP_OK) {
       // ESP_LOGE(TAG, "Failed to mount SPIFFS storage.");
        return;
    } 
    update_topic();
 // Initialize UART
   uart_init();
   //ESP_LOGI(TAG, "Starting  UART "); 
  //Initialize OLED
   oled_init();
  // ESP_LOGI(TAG, "OLED INITIALIZED");    
  //Initialize NimBLE BLE stack
   nimble_port_init();
   //ESP_LOGI(TAG, "NimBLE Start");
  //Initialize BLE
   ble_init();
   //ESP_LOGI(TAG, "Starting BLE "); 
   xTaskCreate(send_data_to_uart2_task, "send_data_to_uart2_task", 8192, NULL, 5, NULL);
   xTaskCreate(ble_host_task, "ble_host_task", 16384, NULL,8, NULL);
   xTaskCreate(sensor_data, "sensor_data", 8192, NULL, 2, NULL);   
   xTaskCreate(monitor_file_task, "monitor_file_task", 4096, NULL, 5, NULL);
   printf("Enter command number (0-19):\n");  
    
 while (1) {
	 if(flag_serverstart==1){
		 wifi_initate_with_Server();flag_serverstart=0;
	 }  
	 interval_uploading();
        vTaskDelay(pdMS_TO_TICKS(1000));  
    }   
 }
 
 
 

