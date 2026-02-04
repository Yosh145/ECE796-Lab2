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

static const char *TAG = "LSM6DSV16X";

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
  dev_ctx.mdelay = NULL; // Use vTaskDelay if needed
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

  // ========== MILESTONE 2: Configure Sensor for 120 Hz HAODR Mode ==========
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Starting Milestone 2: Basic Sensor Reading");

  // Wait for device to be ready
  vTaskDelay(pdMS_TO_TICKS(10));

  // Configure accelerometer: 120 Hz HAODR, ±4g, high-performance mode
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

  err = lsm6dsv16x_xl_mode_set(&dev_ctx, LSM6DSV16X_XL_HIGH_PERFORMANCE_MD);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to set accelerometer mode (error: %ld)", err);
    return;
  }

  ESP_LOGI(TAG,
           "Accelerometer configured: 120 Hz HAODR, ±4g, high-performance");

  // Configure gyroscope: 120 Hz HAODR, ±2000 dps, high-performance mode
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

  err = lsm6dsv16x_gy_mode_set(&dev_ctx, LSM6DSV16X_GY_HIGH_PERFORMANCE_MD);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to set gyroscope mode (error: %ld)", err);
    return;
  }

  ESP_LOGI(TAG,
           "Gyroscope configured: 120 Hz HAODR, ±2000 dps, high-performance");

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

  // Enable accelerometer and gyroscope batching to FIFO
  err = lsm6dsv16x_fifo_xl_batch_set(&dev_ctx, LSM6DSV16X_XL_BATCHED_AT_120Hz);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to enable accelerometer FIFO batching (error: %ld)", err);
    return;
  }

  err = lsm6dsv16x_fifo_gy_batch_set(&dev_ctx, LSM6DSV16X_GY_BATCHED_AT_120Hz);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to enable gyroscope FIFO batching (error: %ld)", err);
    return;
  }

  // Set FIFO mode to continuous
  err = lsm6dsv16x_fifo_mode_set(&dev_ctx, LSM6DSV16X_STREAM_MODE);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to set FIFO mode (error: %ld)", err);
    return;
  }

  // Set FIFO watermark (trigger when sufficient data available)
  err = lsm6dsv16x_fifo_watermark_set(&dev_ctx, 10);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to set FIFO watermark (error: %ld)", err);
    return;
  }

  ESP_LOGI(TAG, "SFLP enabled: 120 Hz quaternion output, FIFO configured");
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Starting data acquisition (Ctrl+] to stop)...");
  ESP_LOGI(TAG, "Format: [Sample] Timestamp(us) | Accel(mg): X, Y, Z | "
                "Gyro(mdps): X, Y, Z | Euler(deg): Roll, Pitch, Yaw");
  ESP_LOGI(TAG, "");

  // Data acquisition loop
  uint32_t sample_count = 0;
  lsm6dsv16x_all_sources_t status;
  lsm6dsv16x_fifo_out_raw_t fifo_data;
  int16_t accel_raw[3];
  int16_t gyro_raw[3];
  float accel_mg[3];
  float gyro_mdps[3];
  float quat_raw[3]; // Quaternion x, y, z (w calculated from normalization)
  float quat_w, quat_x, quat_y, quat_z;
  float roll_deg, pitch_deg, yaw_deg;
  bool have_accel = false;
  bool have_gyro = false;
  bool have_quat = false;

  while (1) {
    // Check FIFO status
    err = lsm6dsv16x_all_sources_get(&dev_ctx, &status);
    if (err != 0) {
      ESP_LOGW(TAG, "Failed to read status (error: %ld)", err);
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Process FIFO when watermark reached
    if (status.fifo_th) {
      // Capture timestamp for this batch
      int64_t timestamp_us = esp_timer_get_time();

      // Read FIFO entries
      for (int i = 0; i < 50 && err == 0; i++) {
        err = lsm6dsv16x_fifo_out_raw_get(&dev_ctx, &fifo_data);
        if (err != 0) {
          break;
        }

        // Parse FIFO tag to identify data type
        switch (fifo_data.tag) {
        case LSM6DSV16X_XL_NC_TAG: // Accelerometer data
          accel_raw[0] =
              (int16_t)((fifo_data.data[1] << 8) | fifo_data.data[0]);
          accel_raw[1] =
              (int16_t)((fifo_data.data[3] << 8) | fifo_data.data[2]);
          accel_raw[2] =
              (int16_t)((fifo_data.data[5] << 8) | fifo_data.data[4]);

          accel_mg[0] = lsm6dsv16x_from_fs4_to_mg(accel_raw[0]);
          accel_mg[1] = lsm6dsv16x_from_fs4_to_mg(accel_raw[1]);
          accel_mg[2] = lsm6dsv16x_from_fs4_to_mg(accel_raw[2]);
          have_accel = true;
          break;

        case LSM6DSV16X_GY_NC_TAG: // Gyroscope data
          gyro_raw[0] =
              (int16_t)((fifo_data.data[1] << 8) | fifo_data.data[0]);
          gyro_raw[1] =
              (int16_t)((fifo_data.data[3] << 8) | fifo_data.data[2]);
          gyro_raw[2] =
              (int16_t)((fifo_data.data[5] << 8) | fifo_data.data[4]);

          gyro_mdps[0] = lsm6dsv16x_from_fs2000_to_mdps(gyro_raw[0]);
          gyro_mdps[1] = lsm6dsv16x_from_fs2000_to_mdps(gyro_raw[1]);
          gyro_mdps[2] = lsm6dsv16x_from_fs2000_to_mdps(gyro_raw[2]);
          have_gyro = true;
          break;

        case LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG: // Quaternion data
          // SFLP quaternion is stored as 3x 16-bit values (x, y, z)
          // Each value is in Q15 format (-1 to +1 normalized)
          quat_raw[0] =
              (int16_t)((fifo_data.data[1] << 8) | fifo_data.data[0]);
          quat_raw[1] =
              (int16_t)((fifo_data.data[3] << 8) | fifo_data.data[2]);
          quat_raw[2] =
              (int16_t)((fifo_data.data[5] << 8) | fifo_data.data[4]);

          // Convert from Q15 format to float (-1.0 to +1.0)
          quat_x = quat_raw[0] / 32768.0f;
          quat_y = quat_raw[1] / 32768.0f;
          quat_z = quat_raw[2] / 32768.0f;

          // Calculate w from normalization constraint (w² + x² + y² + z² = 1)
          float quat_sq_sum = quat_x * quat_x + quat_y * quat_y + quat_z * quat_z;
          if (quat_sq_sum < 1.0f) {
            quat_w = sqrtf(1.0f - quat_sq_sum);
          } else {
            quat_w = 0.0f; // Handle numerical errors
          }

          // Convert quaternion to Euler angles (in degrees)
          // Roll (X-axis rotation)
          float sinr_cosp = 2.0f * (quat_w * quat_x + quat_y * quat_z);
          float cosr_cosp =
              1.0f - 2.0f * (quat_x * quat_x + quat_y * quat_y);
          roll_deg = atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;

          // Pitch (Y-axis rotation)
          float sinp = 2.0f * (quat_w * quat_y - quat_z * quat_x);
          if (fabsf(sinp) >= 1.0f) {
            pitch_deg =
                copysignf(90.0f, sinp); // Use 90 degrees if out of range
          } else {
            pitch_deg = asinf(sinp) * 180.0f / M_PI;
          }

          // Yaw (Z-axis rotation)
          float siny_cosp = 2.0f * (quat_w * quat_z + quat_x * quat_y);
          float cosy_cosp =
              1.0f - 2.0f * (quat_y * quat_y + quat_z * quat_z);
          yaw_deg = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;

          have_quat = true;
          break;

        default:
          // Unknown or unhandled FIFO tag
          break;
        }

        // When we have all data types, print the combined output
        if (have_accel && have_gyro && have_quat) {
          printf("[%5lu] %lld | Accel: %7.2f, %7.2f, %7.2f | Gyro: %8.2f, "
                 "%8.2f, %8.2f | Euler: %6.1f, %6.1f, %6.1f\n",
                 sample_count, timestamp_us, accel_mg[0], accel_mg[1],
                 accel_mg[2], gyro_mdps[0], gyro_mdps[1], gyro_mdps[2],
                 roll_deg, pitch_deg, yaw_deg);

          sample_count++;
          have_accel = false;
          have_gyro = false;
          have_quat = false;
        }
      }
    }

    // Small delay to prevent CPU hogging
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
