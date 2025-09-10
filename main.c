
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/i2c.h"

// --- Pin Definitions ---
// Motors
const uint MOTOR_A_DIR = 18;
const uint MOTOR_A_STEP = 19;
const uint MOTOR_B_DIR = 16;
const uint MOTOR_B_STEP = 17;
// Peripherals
const uint LED_PIN = 21;
const uint ULTRASONIC_TRIG_PIN = 7;
const uint ULTRASONIC_ECHO_PIN = 6;
const uint MOISTURE_ADC_PIN = 26; // ADC0
const uint ONEWIRE_PIN = 22;      // For DS18B20
// MPU6050 I2C
const uint I2C_SDA_PIN = 4;
const uint I2C_SCL_PIN = 5;
const int MPU6050_ADDR = 0x68;

// --- Constants ---
const float OBSTACLE_DISTANCE_CM = 20.0;
const int STEPS_PER_REVOLUTION = 200;
const int STEP_DELAY_US = 2500; 
const int DATA_LOG_INTERVAL_US = 3000000;

// --- Global Variables ---
enum RoverState { IDLE, MOVING_FORWARD, TURNING };
enum RoverState currentState = IDLE;
enum RoverState previousState = IDLE;
int16_t gyro_z = 0; // Gyro Z-axis reading

// --- MPU6050 Functions ---
void mpu6050_init() {
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    // Wake up MPU6050
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c0, MPU6050_ADDR, buf, 2, false);
}

void mpu6050_read_gyro() {
    uint8_t buffer[6];
    uint8_t reg = 0x43; // Gyro data starts here
    i2c_write_blocking(i2c0, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, MPU6050_ADDR, buffer, 6, false);
    gyro_z = (buffer[4] << 8) | buffer[5];
}

// --- Motor Control Functions ---
void setup_motors() {
  uint pins[] = {MOTOR_A_DIR, MOTOR_A_STEP, MOTOR_B_DIR, MOTOR_B_STEP};
  for (int i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
    gpio_init(pins[i]);
    gpio_set_dir(pins[i], GPIO_OUT);
  }
}

void take_step(bool left_dir, bool right_dir) {
    gpio_put(MOTOR_A_DIR, left_dir);
    gpio_put(MOTOR_B_DIR, right_dir);
    busy_wait_us(100); 
    gpio_put(MOTOR_A_STEP, 1);
    gpio_put(MOTOR_B_STEP, 1);
    sleep_us(STEP_DELAY_US);
    gpio_put(MOTOR_A_STEP, 0);
    gpio_put(MOTOR_B_STEP, 0);
}

void move_forward() {
  take_step(1, 0);
}

// Re-written to use MPU6050 for an accurate turn
void turn_right_90_deg() {
    printf("Turning... ");
    float total_angle = 0;
    uint32_t last_time = time_us_32();

    // Loop until we've turned approximately 90 degrees
    while (total_angle < 88) { // Target slightly less to account for momentum
        take_step(1, 1); // Both motors forward to turn
        
        uint32_t current_time = time_us_32();
        float dt = (current_time - last_time) / 1000000.0f; // Time delta in seconds
        last_time = current_time;

        mpu6050_read_gyro();
        // The gyro gives raw output, we scale it to degrees per second
        // This scaling factor may need tuning (trial and error)
        float scaled_gyro_z = gyro_z / 131.0; 
        total_angle += scaled_gyro_z * dt;
    }
    printf("Turn Complete.\n");
}


// --- Other Sensor Functions ---
float get_distance_cm() {
  gpio_put(ULTRASONIC_TRIG_PIN, 1); sleep_us(10); gpio_put(ULTRASONIC_TRIG_PIN, 0);
  uint32_t timeout = 25000; absolute_time_t wait_start = get_absolute_time();
  while (gpio_get(ULTRASONIC_ECHO_PIN) == 0) { if (absolute_time_diff_us(wait_start, get_absolute_time()) > timeout) return 999; }
  absolute_time_t pulse_start = get_absolute_time();
  while (gpio_get(ULTRASONIC_ECHO_PIN) == 1) { if (absolute_time_diff_us(pulse_start, get_absolute_time()) > timeout) return 999; }
  absolute_time_t pulse_end = get_absolute_time();
  return absolute_time_diff_us(pulse_start, pulse_end) * 0.0343 / 2;
}

bool onewire_reset() {
    gpio_set_dir(ONEWIRE_PIN, GPIO_OUT); gpio_put(ONEWIRE_PIN, 0); busy_wait_us(480);
    gpio_set_dir(ONEWIRE_PIN, GPIO_IN); busy_wait_us(70);
    bool presence = !gpio_get(ONEWIRE_PIN); busy_wait_us(410); return presence;
}
void onewire_write_byte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        gpio_set_dir(ONEWIRE_PIN, GPIO_OUT); gpio_put(ONEWIRE_PIN, 0); busy_wait_us(2);
        if ((byte >> i) & 1) { gpio_set_dir(ONEWIRE_PIN, GPIO_IN); }
        busy_wait_us(60); gpio_set_dir(ONEWIRE_PIN, GPIO_IN); busy_wait_us(2);
    }
}
uint8_t onewire_read_byte() {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        gpio_set_dir(ONEWIRE_PIN, GPIO_OUT); gpio_put(ONEWIRE_PIN, 0); busy_wait_us(2);
        gpio_set_dir(ONEWIRE_PIN, GPIO_IN); busy_wait_us(8);
        byte |= (gpio_get(ONEWIRE_PIN) << i); busy_wait_us(50);
    } return byte;
}
float get_ds18b20_temp() {
    if (!onewire_reset()) return -1000;
    onewire_write_byte(0xCC); onewire_write_byte(0x44); sleep_ms(750);
    if (!onewire_reset()) return -1001;
    onewire_write_byte(0xCC); onewire_write_byte(0xBE);
    uint8_t lsb = onewire_read_byte(); uint8_t msb = onewire_read_byte();
    int16_t raw = (msb << 8) | lsb; return raw / 16.0;
}

float get_moisture() {
  adc_select_input(0);
  uint16_t result = adc_read();
  return 100.0f - (result / 4095.0f * 100.0f);
}

// --- Main Program ---
int main() {
  stdio_init_all();
  sleep_ms(2000);
  printf("Data Rover Initialized (MPU6050 Final Version).\n");
  
  // Setup all peripherals
  setup_motors();
  mpu6050_init();
  gpio_init(LED_PIN); gpio_set_dir(LED_PIN, GPIO_OUT); gpio_put(LED_PIN, 1);
  gpio_init(ULTRASONIC_TRIG_PIN); gpio_set_dir(ULTRASONIC_TRIG_PIN, GPIO_OUT);
  gpio_init(ULTRASONIC_ECHO_PIN); gpio_set_dir(ULTRASONIC_ECHO_PIN, GPIO_IN);
  adc_init(); adc_gpio_init(MOISTURE_ADC_PIN);
  gpio_init(ONEWIRE_PIN);

  absolute_time_t data_log_time = get_absolute_time();
  float last_distance = 0;

  while (true) {
    last_distance = get_distance_cm();
    previousState = currentState;
    if (last_distance < OBSTACLE_DISTANCE_CM) { currentState = TURNING; } 
    else { currentState = MOVING_FORWARD; }
    
    if (currentState != previousState) {
        switch (currentState) {
            case MOVING_FORWARD: printf("Status: Moving Forward\n"); break;
            case TURNING: printf("Status: Obstacle Detected. "); break; // Turning message printed from function
            default: break;
        }
    }

    if (currentState == TURNING) { turn_right_90_deg(); } 
    else { move_forward(); }

    if (absolute_time_diff_us(data_log_time, get_absolute_time()) > DATA_LOG_INTERVAL_US) {
        printf("\n--- Data Log ---\n");
        float temp = get_ds18b20_temp();
        printf("Temperature: %.2f C\n", temp);
        float moisture = get_moisture();
        printf("Moisture:    %.1f %%\n", moisture);
        printf("Distance:    %.1f cm\n", last_distance);
        mpu6050_read_gyro();
        printf("Gyro Z-Axis: %d\n", gyro_z);
        printf("----------------\n\n");
        data_log_time = get_absolute_time();
    }
  }
}

