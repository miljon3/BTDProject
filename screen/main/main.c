#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "driver/gpio.h"

#include "axp192.h"
#include "sgm2578.h"
#include "st7789.h"
#include "fontx.h"
#include "bmpfile.h"
#include "decode_jpeg.h"
#include "decode_png.h"
#include "pngle.h"

#include "driver/i2c.h" // driver for i2c
#include <math.h>
#include <time.h>
#include <sys/time.h>  // Required for gettimeofday
#include "driver/rmt.h"

// mqtt includes
#include <stdint.h>
#include <stddef.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_log.h"
#include "mqtt_client.h"

#define RMT_BUZZER_RESOLUTION_HZ 1000000 // 1MHz resolution
#define RMT_BUZZER_GPIO_NUM      2

#define CONFIG_BROKER_URL "mqtt://10.0.0.11:1883"

#define BUZZER_PIN 2 // Example GPIO pin for buzzer

#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define RMT_CLK_DIV 100

#define INTERVAL 400
#define WAIT vTaskDelay(INTERVAL)

// M5stickC-Plus stuff
#if CONFIG_M5STICK_C_PLUS
#define CONFIG_WIDTH 135
#define CONFIG_HEIGHT 240
#define CONFIG_MOSI_GPIO 15
#define CONFIG_SCLK_GPIO 13
#define CONFIG_CS_GPIO 5 
#define CONFIG_DC_GPIO 23
#define CONFIG_RESET_GPIO 18
#define CONFIG_BL_GPIO -1
#define CONFIG_LED_GPIO 10
#define CONFIG_OFFSETX 52
#define CONFIG_OFFSETY 40
#endif

// M5stickC-Plus2 stuff
#if CONFIG_M5STICK_C_PLUS2
#define CONFIG_WIDTH 135
#define CONFIG_HEIGHT 240
#define CONFIG_MOSI_GPIO 15
#define CONFIG_SCLK_GPIO 13
#define CONFIG_CS_GPIO 5 
#define CONFIG_DC_GPIO 14
#define CONFIG_RESET_GPIO 12
#define CONFIG_BL_GPIO -1
#define CONFIG_LED_GPIO 19
#define CONFIG_OFFSETX 52
#define CONFIG_OFFSETY 40
#endif

static const char *TAG = "MAIN";
FontxFile fx16G[2];
FontxFile fx24G[2];
FontxFile fx32G[2];

esp_mqtt_client_handle_t mqtt_client;

TFT_t dev;
int width = CONFIG_WIDTH;
int height = CONFIG_HEIGHT;

esp_err_t mountSPIFFS(char * partition_label, char * mount_point) {
	ESP_LOGI(TAG, "Initializing SPIFFS");
	esp_vfs_spiffs_conf_t conf = {
		.base_path = mount_point,
		.partition_label = partition_label,
		.max_files = 10, // maximum number of files which can be open at the same time
		.format_if_mount_failed = true
	};

	// Use settings defined above to initialize and mount SPIFFS filesystem.
	// Note: esp_vfs_spiffs_register is an all-in-one convenience function.
	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret == ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
		}
		return ret;
	}

	size_t total = 0, used = 0;
	//ret = esp_spiffs_info(NULL, &total, &used);
	ret = esp_spiffs_info(partition_label, &total, &used);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
	} else {
		ESP_LOGI(TAG, "Partition [%s] size: total: %d, used: %d", mount_point, total, used);
	}
	return ret;
}

static int getFileSize(char *fullPath) {
	struct stat st;
	if (stat(fullPath, &st) == 0)
		return st.st_size;
	return -1;
}

static void printDirectory(char * path) {
	DIR* dir = opendir(path);
	assert(dir != NULL);
	while (true) {
		struct dirent *pe = readdir(dir);
		if (!pe) break;
		if (pe->d_type == 1) {
			char fullPath[64];
			strcpy(fullPath, path);
			strcat(fullPath, "/");
			strcat(fullPath, pe->d_name);
			int fsize = getFileSize(fullPath);
			ESP_LOGI(__FUNCTION__,"%s d_name=%s d_ino=%d fsize=%d", path, pe->d_name, pe->d_ino, fsize);
		}
		if (pe->d_type == 2) {
			char subDir[127];
			sprintf(subDir,"%s%.64s", path, pe->d_name);
			ESP_LOGI(TAG, "subDir=[%s]", subDir);
			printDirectory(subDir);

		}
	}
	closedir(dir);
}

// FROM ACTIVITY.C
/* setting up I2C connection*/
# define I2C_MASTER_SCL_IO           22                         /*!< GPIO number used for I2C master clock */
# define I2C_MASTER_SDA_IO           21                         /*!< GPIO number used for I2C master data  */
# define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number */
# define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
# define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
# define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
# define I2C_MASTER_TIMEOUT_MS       1000

/* initializing the sensor*/
# define MPU6886_SENSOR_ADDR                     0x68        /*!< Slave address of the MPU6866 sensor */
# define MPU6886_WHO_AM_I_REG_ADDR               0x75        /*!< Register addresses of the "who am I" register */
# define MPU6886_SMPLRT_DIV_REG_ADDR             0x19
# define MPU6886_CONFIG_REG_ADDR                 0x1A
# define MPU6886_ACCEL_CONFIG_REG_ADDR           0x1C
# define MPU6886_ACCEL_CONFIG_2_REG_ADDR         0x1D
# define MPU6886_FIFO_EN_REG_ADDR                0x23
# define MPU6886_INT_PIN_CFG_REG_ADDR            0x37
# define MPU6886_INT_ENABLE_REG_ADDR             0x38
# define MPU6886_ACCEL_XOUT_REG_ADDR             0x3B
# define MPU6886_USER_CRTL_REG_ADDR              0x6A
# define MPU6886_PWR_MGMT_1_REG_ADDR             0x6B
# define MPU6886_PWR_MGMT_2_REG_ADDR             0x6C

#define HIGH_ACCEL_THRESHOLD 2.5  // Threshold for high acceleration (impact)
#define LOW_ACCEL_THRESHOLD 0.5   // Threshold for low acceleration (free fall)

static esp_err_t mpu6886_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6886_SENSOR_ADDR, &reg_addr, 1, data, len, 10*I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t mpu6886_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6886_SENSOR_ADDR, write_buf, sizeof(write_buf), 10*I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

static void init_mpu6886(void)
{
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_PWR_MGMT_1_REG_ADDR, 0x00));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_PWR_MGMT_1_REG_ADDR, (0x01 << 7)));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_PWR_MGMT_1_REG_ADDR, (0x01 << 0)));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_ACCEL_CONFIG_REG_ADDR, 0x18));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_CONFIG_REG_ADDR, 0x01));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_SMPLRT_DIV_REG_ADDR, 0x05));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_INT_ENABLE_REG_ADDR, 0x00));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_ACCEL_CONFIG_2_REG_ADDR, 0x00));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_USER_CRTL_REG_ADDR, 0x00));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_FIFO_EN_REG_ADDR, 0x00));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_INT_PIN_CFG_REG_ADDR, 0x22));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_INT_ENABLE_REG_ADDR, 0x01));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "MPU6866 intialized successfully");
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

static void getAccelAdc(int16_t* ax, int16_t* ay, int16_t* az)
{
    uint8_t buf[6];
    mpu6886_register_read(MPU6886_ACCEL_XOUT_REG_ADDR, buf, 6);

    *ax = ((int16_t)buf[0] << 8) | buf[1];
    *ay = ((int16_t)buf[2] << 8) | buf[3];
    *az = ((int16_t)buf[4] << 8) | buf[5];
}

static void getAccelData(float* ax, float* ay, float* az)
{
    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
    getAccelAdc(&accX, &accY, &accZ);
    float aRes = 16.0 / 32768.0;

    *ax = (float)accX * aRes;
    *ay = (float)accY * aRes;
    *az = (float)accZ * aRes;
}


float norm(float ax, float ay, float az) {
    return sqrt(ax * ax + ay * ay + az * az);
}

float applyFilter(float x, float inputCoeff[], float outputCoeff, float yn1, float xn1)
{
    // Filter the signal using the difference equation
    float yn = outputCoeff * yn1 + inputCoeff[0] * x + inputCoeff[1] * xn1;
    return yn;
}

// END FROM ACTIVITY.C

void show_message(bool walking, int steps, TFT_t * dev, int width, int height)
{
	char* text = "walking";
	if (!walking)
	{
		text = "OK";
	}
	
	uint8_t buffer[FontxGlyphBufSize];
	uint8_t fontWidth;
	uint8_t fontHeight;
	// GetFontx(fx16G, 0, buffer, &fontWidth, &fontHeight);
	GetFontx(fx24G, 0, buffer, &fontWidth, &fontHeight);
	GetFontx(fx32G, 0, buffer, &fontWidth, &fontHeight);

	uint16_t color;
	uint8_t ascii[40];
	uint16_t xpos = 0;
	uint16_t ypos = 15;

	lcdFillScreen(dev, BLACK);
	color = WHITE;
	// lcdSetFontDirection(dev, 0);
	xpos = ((width - fontHeight) / 2) - 1;
	ypos = (height - (11 * fontWidth)) / 2;
	lcdSetFontDirection(dev, DIRECTION90);

	strcpy((char *)ascii, text);
	lcdDrawString(dev, fx32G, xpos, ypos, ascii, color);

	// xpos = xpos - (24 * xd) - (margin * xd);
	xpos = xpos + 40;
	// ypos = ypos + 30;
	sprintf((char *)ascii, "Steps: %d", steps);
	// strcpy((char *)ascii, text);
	lcdDrawString(dev, fx32G, xpos, ypos, ascii, color);
}

void display_message(char* message, TFT_t * dev, int width, int height)
{
	uint8_t buffer[FontxGlyphBufSize];
	uint8_t fontWidth;
	uint8_t fontHeight;
	// GetFontx(fx16G, 0, buffer, &fontWidth, &fontHeight);
	GetFontx(fx24G, 0, buffer, &fontWidth, &fontHeight);
	GetFontx(fx32G, 0, buffer, &fontWidth, &fontHeight);

	uint16_t color;
	uint8_t ascii[40];
	uint16_t xpos = 0;
	uint16_t ypos = 15;

	lcdFillScreen(dev, BLACK);
	color = WHITE;
	// lcdSetFontDirection(dev, 0);
	xpos = ((width - fontHeight) / 2) - 1;
	ypos = (height - (13 * fontWidth)) / 2;
	lcdSetFontDirection(dev, DIRECTION90);

	strcpy((char *)ascii, message);
	lcdDrawString(dev, fx32G, xpos, ypos, ascii, color);
}

void buzzer_init() {
    rmt_config_t rmt_tx_config = RMT_DEFAULT_CONFIG_TX(BUZZER_PIN, RMT_TX_CHANNEL);
    rmt_tx_config.clk_div = RMT_CLK_DIV;
    rmt_config(&rmt_tx_config);
    rmt_driver_install(rmt_tx_config.channel, 0, 0);
}

void buzzer_play_tone(float frequency, uint32_t duration_ms) {
    int num_items = duration_ms * 1000 / (1000000 / RMT_CLK_DIV);
    rmt_item32_t *items = (rmt_item32_t *)malloc(num_items * sizeof(rmt_item32_t));
    if (items == NULL) {
        printf("Memory allocation failed!\n");
        return;
    }

    float period_us = 1000000 / frequency;
    for (int i = 0; i < num_items; i++) {
        items[i].level0 = 5;
        items[i].duration0 = (uint32_t)(period_us / 2);
        items[i].level1 = 0;
        items[i].duration1 = (uint32_t)(period_us / 2);
    }

    rmt_write_items(RMT_TX_CHANNEL, items, num_items, false);
    rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);
    free(items);
}

#define BUTTON_A_GPIO 37
#define BUTTON_B_GPIO 38

static QueueHandle_t gpio_evt_queue = NULL;
static TimerHandle_t cancel_timer = NULL;
static bool fall_detected = false;
static bool cancel_flag = false;
static bool alert_sent = false;

static void IRAM_ATTR button_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void init_buttons(void) {
    gpio_config_t io_conf;

    // Configure Button A
    io_conf.intr_type = GPIO_INTR_NEGEDGE;  // Trigger on falling edge
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_A_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Configure Button B
    io_conf.pin_bit_mask = (1ULL << BUTTON_B_GPIO);
    gpio_config(&io_conf);

    // Install GPIO ISR handler service
    gpio_install_isr_service(0);
    
    // Attach the interrupt service routine
    gpio_isr_handler_add(BUTTON_A_GPIO, button_isr_handler, (void *)BUTTON_A_GPIO);
    gpio_isr_handler_add(BUTTON_B_GPIO, button_isr_handler, (void *)BUTTON_B_GPIO);
}

void button_task(void* arg) {
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            if(io_num == BUTTON_A_GPIO) {
                ESP_LOGI(TAG, "Button A pressed");
                // Handle Button A press
                if (fall_detected) {
                    ESP_LOGI(TAG, "Fall alert cancelled");
                    cancel_flag = true;
                    fall_detected = false;
                    display_message("Fall alert cancelled!", &dev, width, height);
                }
            } else if(io_num == BUTTON_B_GPIO) {
                ESP_LOGI(TAG, "Button B pressed");
                // Handle Button B press
            }
        }
    }
}



// mqtt functionality

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static esp_mqtt5_user_property_item_t user_property_arr[] = {
        {"board", "esp32"},
        {"u", "user"},
        {"p", "password"}
    };

#define USE_PROPERTY_ARR_SIZE   sizeof(user_property_arr)/sizeof(esp_mqtt5_user_property_item_t)

static esp_mqtt5_publish_property_config_t publish_property = {
    .payload_format_indicator = 1,
    .message_expiry_interval = 1000,
    .topic_alias = 0,
    .response_topic = "/last/1",
    .correlation_data = "123456",
    .correlation_data_len = 6,
};

static esp_mqtt5_subscribe_property_config_t subscribe_property = {
    .subscribe_id = 25555,
    .no_local_flag = false,
    .retain_as_published_flag = false,
    .retain_handle = 0,
    .is_share_subscribe = true,
    .share_name = "group1",
};

static esp_mqtt5_subscribe_property_config_t subscribe1_property = {
    .subscribe_id = 25555,
    .no_local_flag = true,
    .retain_as_published_flag = false,
    .retain_handle = 0,
};

static esp_mqtt5_unsubscribe_property_config_t unsubscribe_property = {
    .is_share_subscribe = true,
    .share_name = "group1",
};

static esp_mqtt5_disconnect_property_config_t disconnect_property = {
    .session_expiry_interval = 60,
    .disconnect_reason = 0,
};

static void print_user_property(mqtt5_user_property_handle_t user_property)
{
    if (user_property) {
        uint8_t count = esp_mqtt5_client_get_user_property_count(user_property);
        if (count) {
            esp_mqtt5_user_property_item_t *item = malloc(count * sizeof(esp_mqtt5_user_property_item_t));
            if (esp_mqtt5_client_get_user_property(user_property, item, &count) == ESP_OK) {
                for (int i = 0; i < count; i ++) {
                    esp_mqtt5_user_property_item_t *t = &item[i];
                    ESP_LOGI(TAG, "key is %s, value is %s", t->key, t->value);
                    free((char *)t->key);
                    free((char *)t->value);
                }
            }
            free(item);
        }
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt5_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    ESP_LOGD(TAG, "free heap size is %" PRIu32 ", minimum %" PRIu32, esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        print_user_property(event->property->user_property);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        print_user_property(event->property->user_property);
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        esp_mqtt5_client_set_publish_property(client, &publish_property);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    // case MQTT_EVENT_UNSUBSCRIBED:
    //     ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    //     print_user_property(event->property->user_property);
    //     esp_mqtt5_client_set_user_property(&disconnect_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    //     esp_mqtt5_client_set_disconnect_property(client, &disconnect_property);
    //     esp_mqtt5_client_delete_user_property(disconnect_property.user_property);
    //     disconnect_property.user_property = NULL;
    //     esp_mqtt_client_disconnect(client);
    //     break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        print_user_property(event->property->user_property);
        ESP_LOGI(TAG, "payload_format_indicator is %d", event->property->payload_format_indicator);
        ESP_LOGI(TAG, "response_topic is %.*s", event->property->response_topic_len, event->property->response_topic);
        ESP_LOGI(TAG, "correlation_data is %.*s", event->property->correlation_data_len, event->property->correlation_data);
        ESP_LOGI(TAG, "content_type is %.*s", event->property->content_type_len, event->property->content_type);
        ESP_LOGI(TAG, "TOPIC=%.*s", event->topic_len, event->topic);
        ESP_LOGI(TAG, "DATA=%.*s", event->data_len, event->data);
        
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        print_user_property(event->property->user_property);
        ESP_LOGI(TAG, "MQTT5 return code is %d", event->error_handle->connect_return_code);
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt5_app_start(void)
{
    esp_mqtt5_connection_property_config_t connect_property = {
        .session_expiry_interval = 10,
        .maximum_packet_size = 1024,
        .receive_maximum = 65535,
        .topic_alias_maximum = 2,
        .request_resp_info = true,
        .request_problem_info = true,
        .will_delay_interval = 10,
        .payload_format_indicator = true,
        .message_expiry_interval = 10,
        .response_topic = "/test/response",
        .correlation_data = "123456",
        .correlation_data_len = 6,
    };

    esp_mqtt_client_config_t mqtt5_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
        .session.protocol_ver = MQTT_PROTOCOL_V_5,
        .network.disable_auto_reconnect = true,
        .credentials.username = "123",
        .credentials.authentication.password = "456",
        .session.last_will.topic = "/last/1",
        .session.last_will.msg = "i will leave",
        .session.last_will.msg_len = 12,
        .session.last_will.qos = 1,
        .session.last_will.retain = true,
    };

#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt5_cfg.uri, "FROM_STDIN") == 0) {
        int count = 0;
    
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt5_cfg.broker.address.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    mqtt_client = esp_mqtt_client_init(&mqtt5_cfg);

    /* Set connection properties and user properties */
    esp_mqtt5_client_set_connect_property(mqtt_client, &connect_property);

    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt5_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

// mqtt end


static void cancel_timer_callback(TimerHandle_t xTimer) {
    ESP_LOGI(TAG, "Cancel timer has finished");

    if (!cancel_flag) {
        ESP_LOGI(TAG, "Sending fall alert to server...");
        esp_mqtt_client_publish(mqtt_client, "/alarms/1", "data_3", 0, 1, 1);

        alert_sent = true;
        fall_detected = false;
        display_message("Fall alert sent!", &dev, width, height);
        buzzer_play_tone(2000, 3000);
    }
}

void app_main(void)
{
    // Mount SPIFFS File System on FLASH
    ESP_LOGI(TAG, "Initializing SPIFFS");
    ESP_ERROR_CHECK(mountSPIFFS("storage1", "/fonts"));
    printDirectory("/fonts");

    // Initialize I2C
    i2c_master_init();

    // Init accelerometer
    ESP_LOGI(TAG, "I2C initialized successfully");
    init_mpu6886();

    // Start Task
    AXP192_PowerOn();
    AXP192_ScreenBreath(11);

    spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO);
    lcdInit(&dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);

    // Set font file
    InitFontx(fx16G, "/fonts/ILGH16XB.FNT", ""); // 8x16Dot Gothic
    InitFontx(fx24G, "/fonts/ILGH24XB.FNT", ""); // 12x24Dot Gothic
    InitFontx(fx32G, "/fonts/ILGH32XB.FNT", ""); // 16x32Dot Gothic

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    init_buttons();
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);

    int width = CONFIG_WIDTH;
    int height = CONFIG_HEIGHT;

    float hpfCoeff[] = {0.99220722, -0.99220722};
    float hpfOutputCoeff = 0.98441445;

    float lpfCoeff[] = {0.03045903, 0.03045903};
    float lpfOutputCoeff = 0.93908194;

    float accelX, accelY, accelZ;
    float xn1 = 0;
    float yn1 = 0;
    float yn = 0;
    float xn = 0;

    bool fall_detected = false;
    bool cancel_flag = false;
    bool alertSent = false;

    buzzer_init();
    
    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(nvs_flash_init()); 
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect());
    // nvs_flash_init(); // this is important in wifi case to store configurations , code will not work if this is not added
    // wifi_connection();
    // wifi_init_sta();

    mqtt5_app_start();

    while (1) {
        getAccelData(&accelX, &accelY, &accelZ);
        xn = norm(accelX, accelY, accelZ) - 1.05;

        // Apply the HPF
        yn = applyFilter(xn, hpfCoeff, hpfOutputCoeff, yn1, xn1);

        // Apply the LPF
        yn = applyFilter(yn, lpfCoeff, lpfOutputCoeff, yn1, xn1);

        xn1 = xn;
        yn1 = yn;
        // ESP_LOGI(TAG, "yn: %f", yn);

        // Check for fall: High acceleration followed by low acceleration
        if (!fall_detected && yn > HIGH_ACCEL_THRESHOLD) {
            fall_detected = true;
            // Start the cancellation timer
            cancel_flag = false;
            alertSent = false;
            TimerHandle_t cancel_timer = xTimerCreate("CancelTimer", pdMS_TO_TICKS(10000), pdFALSE, (void *)0, cancel_timer_callback);
            if (cancel_timer != NULL) {
                ESP_LOGI(TAG, "Starting the cancellation timer");
                xTimerStart(cancel_timer, 0);
            }
        } else if (fall_detected && yn < LOW_ACCEL_THRESHOLD) {
            // Fall detected
            if (!cancel_flag && !alertSent) {
                display_message("Fall detected!", &dev, width, height);
                buzzer_play_tone(2000, 3000); // Alert for fall
                // Wait for the cancellation window to expire before sending the alert
            }
            fall_detected = false; // Reset flag
        }

        usleep(10000);
    }
}