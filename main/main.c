#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lsm6dsv16x_reg.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// I²C Configuration
#define I2C_MASTER_SCL_IO 2 // GPIO2 for SCL (SCLK)
#define I2C_MASTER_SDA_IO 1 // GPIO1 for SDA (PICO)
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000 // 400kHz
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000

// LSM6DSV16X Configuration
#define LSM6DSV16X_I2C_ADDR 0x6B     // SA0 pulled high
#define LSM6DSV16X_WHO_AM_I_VAL 0x70 // Expected WHO_AM_I value
#define LSM6DSV16X_INT1_GPIO 5       // GPIO5 for INT1 interrupt

static const char *TAG = "LSM6DSV16X";

// Global variables for interrupt handling
static TaskHandle_t data_task_handle = NULL;
static volatile int64_t interrupt_timestamp_us = 0;
static volatile uint32_t isr_call_count = 0;

/**
 * @brief GPIO ISR handler for LSM6DSV16X INT1 pin
 * 
 * Captures timestamp and notifies data acquisition task
 */
static void IRAM_ATTR imu_isr_handler(void *arg) {
  if (data_task_handle == NULL) {
    return;
  }
  // Capture timestamp immediately for precise timing
  interrupt_timestamp_us = esp_timer_get_time();
  isr_call_count++;
  // Notify data task to process FIFO
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(data_task_handle, &xHigherPriorityTaskWoken);
  
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

/**
 * @brief Millisecond delay function for LSM6DSV16X driver
 *
 * @param ms Delay in milliseconds
 */
static void platform_delay(uint32_t ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 * @brief I²C write function for LSM6DSV16X driver
 *
 * @param handle I²C port number
 * @param reg Register address
 * @param bufp Pointer to data buffer
 * @param len Number of bytes to write
 * @return 0 on success, -1 on error
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len) {
  i2c_port_t i2c_num = (i2c_port_t)(int)handle;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (LSM6DSV16X_I2C_ADDR << 1) | I2C_MASTER_WRITE,
                        true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write(cmd, (uint8_t *)bufp, len, true);
  i2c_master_stop(cmd);

  esp_err_t ret =
      i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);

  return (ret == ESP_OK) ? 0 : -1;
}

/**
 * @brief I²C read function for LSM6DSV16X driver
 *
 * @param handle I²C port number
 * @param reg Register address
 * @param bufp Pointer to data buffer
 * @param len Number of bytes to read
 * @return 0 on success, -1 on error
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len) {
  i2c_port_t i2c_num = (i2c_port_t)(int)handle;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (LSM6DSV16X_I2C_ADDR << 1) | I2C_MASTER_WRITE,
                        true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd); // Repeated start
  i2c_master_write_byte(cmd, (LSM6DSV16X_I2C_ADDR << 1) | I2C_MASTER_READ,
                        true);

  if (len > 1) {
    i2c_master_read(cmd, bufp, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, bufp + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);

  esp_err_t ret =
      i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "I²C read failed: reg=0x%02X, addr=0x%02X, error=%s", reg,
             LSM6DSV16X_I2C_ADDR, esp_err_to_name(ret));
  }

  return (ret == ESP_OK) ? 0 : -1;
}

/**
 * @brief Scan I²C bus for devices
 */
static void i2c_scan_bus(void) {
  ESP_LOGI(TAG, "Scanning I²C bus...");
  uint8_t devices_found = 0;

  for (uint8_t addr = 0x03; addr < 0x78; addr++) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                         pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "  Found device at address: 0x%02X", addr);
      devices_found++;
    }
  }

  if (devices_found == 0) {
    ESP_LOGW(TAG, "No I²C devices found! Check wiring and power.");
  } else {
    ESP_LOGI(TAG, "I²C scan complete. %d device(s) found.", devices_found);
  }
}

/**
 * @brief Initialize I²C master bus
 *
 * @return ESP_OK on success
 */
static esp_err_t i2c_master_init(void) {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };

  esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
  if (err != ESP_OK) {
    return err;
  }

  return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

void app_main(void) {
  esp_err_t ret;

  ESP_LOGI(
      TAG,
      "Starting LSM6DSV16X Milestone 1: I²C Communication & Device Detection");
  ESP_LOGI(TAG, "Pin Configuration: SDA=GPIO%d, SCL=GPIO%d", I2C_MASTER_SDA_IO,
           I2C_MASTER_SCL_IO);
  ESP_LOGI(TAG, "I²C Address: 0x%02X, Frequency: %d Hz", LSM6DSV16X_I2C_ADDR,
           I2C_MASTER_FREQ_HZ);

  // Initialize I²C bus
  ret = i2c_master_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I²C master initialization failed: %s", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "I²C master initialized successfully");

  // Add delay for device power-up
  ESP_LOGI(TAG, "Waiting for device power-up...");
  vTaskDelay(pdMS_TO_TICKS(200));

  // Scan I²C bus to find devices
  i2c_scan_bus();

  // Initialize LSM6DSV16X device context
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = (void *)I2C_MASTER_NUM;

  // Try reading WHO_AM_I register with multiple attempts
  uint8_t whoami = 0;
  int32_t err = -1;
  
  ESP_LOGI(TAG, "Attempting to read WHO_AM_I register (0x0F)...");
  
  for (int attempt = 0; attempt < 5; attempt++) {
    vTaskDelay(pdMS_TO_TICKS(50));
    err = lsm6dsv16x_device_id_get(&dev_ctx, &whoami);
    ESP_LOGI(TAG, "  Attempt %d: WHO_AM_I = 0x%02X (error: %ld)", 
             attempt + 1, whoami, err);
    
    if (err == 0 && whoami == LSM6DSV16X_WHO_AM_I_VAL) {
      break;
    }
  }

  if (err != 0) {
    ESP_LOGE(TAG, "Failed to read WHO_AM_I register after 5 attempts");
    ESP_LOGE(TAG, "Troubleshooting:");
    ESP_LOGE(TAG, "  1. Check SDA (GPIO%d) and SCL (GPIO%d) wiring",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    ESP_LOGE(TAG, "  2. Verify 3.3V and GND connections");
    ESP_LOGE(TAG,
             "  3. Power cycle the LSM6DSV16X (disconnect/reconnect 3.3V)");
    ESP_LOGE(TAG, "  4. Try address 0x6A if SA0 is pulled low");
    return;
  }

  if (whoami == LSM6DSV16X_WHO_AM_I_VAL) {
    ESP_LOGI(TAG, "✓ LSM6DSV16X device detected successfully!");
    ESP_LOGI(TAG, "Milestone 1 Complete: I²C communication established");
  } else {
    ESP_LOGE(TAG, "✗ Device ID mismatch! Read 0x%02X, expected 0x%02X", 
             whoami, LSM6DSV16X_WHO_AM_I_VAL);
    ESP_LOGE(TAG, "Device may need power cycle or is in an unexpected state.");
    return;
  }

  // ========== Software Reset ==========
  ESP_LOGI(TAG, "Performing software reset...");
  err = lsm6dsv16x_sw_reset(&dev_ctx);
  if (err != 0) {
    ESP_LOGE(TAG, "Software reset failed (error: %ld)", err);
    return;
  }
  vTaskDelay(pdMS_TO_TICKS(50));

  // Enable Block Data Update (BDU) to ensure data integrity
  err = lsm6dsv16x_block_data_update_set(&dev_ctx, 1);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to enable BDU (error: %ld)", err);
    return;
  }

  // ========== MILESTONE 2: Configure Sensor for 120 Hz HAODR Mode ==========
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Starting Milestone 2: Configure 120 Hz HAODR Mode");

  // Wait for device to be ready
  vTaskDelay(pdMS_TO_TICKS(10));

  // Set accelerometer to High-Accuracy ODR (HAODR) mode first, then set ODR
  err = lsm6dsv16x_xl_mode_set(&dev_ctx, LSM6DSV16X_XL_HIGH_ACCURACY_ODR_MD);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to set accelerometer HAODR mode (error: %ld)", err);
    return;
  }

  err = lsm6dsv16x_xl_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_120Hz);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to set accelerometer ODR (error: %ld)", err);
    return;
  }

  err = lsm6dsv16x_xl_full_scale_set(&dev_ctx, LSM6DSV16X_4g);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to set accelerometer full-scale (error: %ld)", err);
    return;
  }

  ESP_LOGI(TAG,
           "Accelerometer configured: 120 Hz HAODR, ±4g");

  // Set gyroscope to High-Accuracy ODR (HAODR) mode first, then set ODR
  err = lsm6dsv16x_gy_mode_set(&dev_ctx, LSM6DSV16X_GY_HIGH_ACCURACY_ODR_MD);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to set gyroscope HAODR mode (error: %ld)", err);
    return;
  }

  err = lsm6dsv16x_gy_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_120Hz);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to set gyroscope ODR (error: %ld)", err);
    return;
  }

  err = lsm6dsv16x_gy_full_scale_set(&dev_ctx, LSM6DSV16X_2000dps);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to set gyroscope full-scale (error: %ld)", err);
    return;
  }

  ESP_LOGI(TAG,
           "Gyroscope configured: 120 Hz HAODR, ±2000 dps");

  // ========== MILESTONE 3: Enable SFLP for Quaternion Generation ==========
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Starting Milestone 3: SFLP & Quaternion/Euler Data");

  // Enable SFLP game rotation (quaternion output)
  err = lsm6dsv16x_sflp_game_rotation_set(&dev_ctx, 1);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to enable SFLP game rotation (error: %ld)", err);
    return;
  }

  // Configure SFLP data rate to 120 Hz
  err = lsm6dsv16x_sflp_data_rate_set(&dev_ctx, LSM6DSV16X_SFLP_120Hz);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to set SFLP data rate (error: %ld)", err);
    return;
  }

  // Configure FIFO to batch SFLP game rotation data
  lsm6dsv16x_fifo_sflp_raw_t sflp_fifo_cfg = {
      .game_rotation = 1, .gravity = 0, .gbias = 0};
  err = lsm6dsv16x_fifo_sflp_batch_set(&dev_ctx, sflp_fifo_cfg);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to configure SFLP FIFO batching (error: %ld)", err);
    return;
  }

  // Do NOT batch accel/gyro to FIFO — read them from output registers instead
  err = lsm6dsv16x_fifo_xl_batch_set(&dev_ctx, LSM6DSV16X_XL_NOT_BATCHED);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to disable accelerometer FIFO batching (error: %ld)", err);
    return;
  }

  err = lsm6dsv16x_fifo_gy_batch_set(&dev_ctx, LSM6DSV16X_GY_NOT_BATCHED);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to disable gyroscope FIFO batching (error: %ld)", err);
    return;
  }

  // Set FIFO mode to continuous (stream) for SFLP quaternion data
  err = lsm6dsv16x_fifo_mode_set(&dev_ctx, LSM6DSV16X_STREAM_MODE);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to set FIFO mode (error: %ld)", err);
    return;
  }

  ESP_LOGI(TAG, "SFLP enabled: 120 Hz quaternion output via FIFO, accel/gyro via output registers");

  // Enable interrupt generation with latched mode
  lsm6dsv16x_interrupt_mode_t int_mode = {
      .enable = 1,
      .lir = 1,  // Latched: INT stays asserted until status is read
  };
  err = lsm6dsv16x_interrupt_enable_set(&dev_ctx, int_mode);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to enable interrupts (error: %ld)", err);
    return;
  }

  // ========== MILESTONE 4: Configure GPIO Interrupt ==========
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Starting Milestone 4: Interrupt-Driven Data Acquisition");

  // Store task handle for ISR notification
  data_task_handle = xTaskGetCurrentTaskHandle();

  // Configure GPIO for INT1 interrupt
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_POSEDGE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = (1ULL << LSM6DSV16X_INT1_GPIO),
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .pull_up_en = GPIO_PULLUP_ENABLE,
  };
  ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure INT1 GPIO: %s", esp_err_to_name(ret));
    return;
  }

  // Install GPIO ISR service and add handler
  ret = gpio_install_isr_service(0);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
    return;
  }

  ret = gpio_isr_handler_add(LSM6DSV16X_INT1_GPIO, imu_isr_handler, NULL);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add ISR handler: %s", esp_err_to_name(ret));
    return;
  }

  // Route gyroscope data-ready (DRDY) interrupt to INT1 pin
  lsm6dsv16x_pin_int_route_t int1_route = {0};
  int1_route.drdy_g = 1;  // Gyroscope data-ready on INT1
  err = lsm6dsv16x_pin_int1_route_set(&dev_ctx, &int1_route);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to route DRDY_G interrupt to INT1 (error: %ld)", err);
    return;
  }

  ESP_LOGI(TAG, "GPIO interrupt configured: INT1=GPIO%d, rising edge", LSM6DSV16X_INT1_GPIO);
  ESP_LOGI(TAG, "Gyroscope DRDY interrupt routed to INT1");
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Starting interrupt-driven data acquisition (Ctrl+] to stop)...");
  ESP_LOGI(TAG, "Format: [Sample] Timestamp(us) | Accel(mg): X, Y, Z | "
                "Gyro(mdps): X, Y, Z | Euler(deg): Roll, Pitch, Yaw");
  ESP_LOGI(TAG, "");

  // Data acquisition loop — interrupt-driven via gyroscope DRDY
  uint32_t sample_count = 0;
  int16_t accel_raw[3];
  int16_t gyro_raw[3];
  float accel_mg[3];
  float gyro_mdps[3];
  float quat_w, quat_x, quat_y, quat_z;
  float roll_deg, pitch_deg, yaw_deg;

  // Union for converting f16→f32 via lsm6dsv16x_from_f16_to_f32()
  union {
    uint32_t u;
    float f;
  } f16_conv;

  while (1) {
    // Wait for gyroscope data-ready interrupt notification
    uint32_t notification = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

    if (notification == 0) {
      // Timeout — no interrupt received
      continue;
    }

    // Capture the ISR timestamp for this exact sample
    int64_t sample_timestamp_us = interrupt_timestamp_us;

    // Clear latched interrupt by reading ALL_INT_SRC register
    lsm6dsv16x_all_sources_t all_src;
    err = lsm6dsv16x_all_sources_get(&dev_ctx, &all_src);
    if (err != 0) {
      ESP_LOGW(TAG, "Failed to read interrupt sources (error: %ld)", err);
      continue;
    }

    // ---- Read accelerometer from output registers ----
    err = lsm6dsv16x_acceleration_raw_get(&dev_ctx, accel_raw);
    if (err != 0) {
      ESP_LOGW(TAG, "Failed to read accelerometer (error: %ld)", err);
      continue;
    }
    accel_mg[0] = lsm6dsv16x_from_fs4_to_mg(accel_raw[0]);
    accel_mg[1] = lsm6dsv16x_from_fs4_to_mg(accel_raw[1]);
    accel_mg[2] = lsm6dsv16x_from_fs4_to_mg(accel_raw[2]);

    // ---- Read gyroscope from output registers ----
    err = lsm6dsv16x_angular_rate_raw_get(&dev_ctx, gyro_raw);
    if (err != 0) {
      ESP_LOGW(TAG, "Failed to read gyroscope (error: %ld)", err);
      continue;
    }
    gyro_mdps[0] = lsm6dsv16x_from_fs2000_to_mdps(gyro_raw[0]);
    gyro_mdps[1] = lsm6dsv16x_from_fs2000_to_mdps(gyro_raw[1]);
    gyro_mdps[2] = lsm6dsv16x_from_fs2000_to_mdps(gyro_raw[2]);

    // ---- Read SFLP game rotation quaternion from FIFO ----
    bool have_quat = false;
    lsm6dsv16x_fifo_status_t fifo_status;
    err = lsm6dsv16x_fifo_status_get(&dev_ctx, &fifo_status);
    if (err == 0 && fifo_status.fifo_level > 0) {
      // Drain FIFO entries, keeping the latest quaternion
      for (uint16_t i = 0; i < fifo_status.fifo_level; i++) {
        lsm6dsv16x_fifo_out_raw_t fifo_data;
        err = lsm6dsv16x_fifo_out_raw_get(&dev_ctx, &fifo_data);
        if (err != 0) {
          break;
        }

        if (fifo_data.tag == LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG) {
          // SFLP game rotation: 3x half-precision floats (x, y, z)
          uint16_t qh[3];
          qh[0] = (uint16_t)((fifo_data.data[1] << 8) | fifo_data.data[0]);
          qh[1] = (uint16_t)((fifo_data.data[3] << 8) | fifo_data.data[2]);
          qh[2] = (uint16_t)((fifo_data.data[5] << 8) | fifo_data.data[4]);

          // Convert f16 → f32
          f16_conv.u = lsm6dsv16x_from_f16_to_f32(qh[0]);
          quat_x = f16_conv.f;
          f16_conv.u = lsm6dsv16x_from_f16_to_f32(qh[1]);
          quat_y = f16_conv.f;
          f16_conv.u = lsm6dsv16x_from_f16_to_f32(qh[2]);
          quat_z = f16_conv.f;

          // Derive w from unit quaternion constraint: w²+x²+y²+z² = 1
          float sq_sum = quat_x * quat_x + quat_y * quat_y + quat_z * quat_z;
          quat_w = (sq_sum < 1.0f) ? sqrtf(1.0f - sq_sum) : 0.0f;

          have_quat = true;
        }
      }
    }

    if (!have_quat) {
      // SFLP quaternion not yet available; skip this sample
      continue;
    }

    // ---- Convert quaternion to Euler angles (degrees) ----
    // Roll (X-axis rotation)
    float sinr_cosp = 2.0f * (quat_w * quat_x + quat_y * quat_z);
    float cosr_cosp = 1.0f - 2.0f * (quat_x * quat_x + quat_y * quat_y);
    roll_deg = atan2f(sinr_cosp, cosr_cosp) * 180.0f / (float)M_PI;

    // Pitch (Y-axis rotation)
    float sinp = 2.0f * (quat_w * quat_y - quat_z * quat_x);
    if (fabsf(sinp) >= 1.0f) {
      pitch_deg = copysignf(90.0f, sinp);
    } else {
      pitch_deg = asinf(sinp) * 180.0f / (float)M_PI;
    }

    // Yaw (Z-axis rotation)
    float siny_cosp = 2.0f * (quat_w * quat_z + quat_x * quat_y);
    float cosy_cosp = 1.0f - 2.0f * (quat_y * quat_y + quat_z * quat_z);
    yaw_deg = atan2f(siny_cosp, cosy_cosp) * 180.0f / (float)M_PI;

    // ---- Print sample: counter, timestamp, accel, gyro, Euler ----
    printf("[%5lu] %lld | Accel(mg): %7.2f, %7.2f, %7.2f | "
           "Gyro(mdps): %8.2f, %8.2f, %8.2f | "
           "Euler(deg): %6.1f, %6.1f, %6.1f\n",
           sample_count, sample_timestamp_us,
           accel_mg[0], accel_mg[1], accel_mg[2],
           gyro_mdps[0], gyro_mdps[1], gyro_mdps[2],
           roll_deg, pitch_deg, yaw_deg);

    sample_count++;
  }
}