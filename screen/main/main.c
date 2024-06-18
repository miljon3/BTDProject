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

#define RMT_BUZZER_RESOLUTION_HZ 1000000 // 1MHz resolution
#define RMT_BUZZER_GPIO_NUM      2

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


static void cancel_timer_callback(TimerHandle_t xTimer) {
    if (!cancel_flag && fall_detected) {
        ESP_LOGI(TAG, "Sending fall alert to server...");
        // TODO: Send msg to server
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
    ESP_ERROR_CHECK(mountSPIFFS("storage2", "/images"));
    printDirectory("/images");

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

    struct timeval time_last_step, time_current_step;
    gettimeofday(&time_last_step, NULL);
    display_message("", &dev, width, height);

    buzzer_init();
    vTaskDelay(10000 / portTICK_PERIOD_MS); // Wait for 10 seconds
    buzzer_play_tone(1000, 500); // Play a 1 kHz tone for 500 ms
    buzzer_play_tone(2000, 3000); // Play a 2 kHz tone for 3 seconds

    display_message("Carl tripped!", &dev, width, height);

    while (1) {
        getAccelData(&accelX, &accelY, &accelZ);
        xn = norm(accelX, accelY, accelZ) - 1.05;

        // Apply the HPF
        yn = applyFilter(xn, hpfCoeff, hpfOutputCoeff, yn1, xn1);

        // Apply the LPF
        yn = applyFilter(yn, lpfCoeff, lpfOutputCoeff, yn1, xn1);

        xn1 = xn;
        yn1 = yn;
        ESP_LOGI(TAG, "yn: %f", yn);

        // Check for fall: High acceleration followed by low acceleration
        if (!fall_detected && yn > HIGH_ACCEL_THRESHOLD) {
            fall_detected = true;
            gettimeofday(&time_current_step, NULL);
            // Start the cancellation timer
            cancel_flag = false;
            alertSent = false;
            TimerHandle_t cancel_timer = xTimerCreate("CancelTimer", pdMS_TO_TICKS(10000), pdFALSE, (void *)0, cancel_timer_callback);
            if (cancel_timer != NULL) {
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