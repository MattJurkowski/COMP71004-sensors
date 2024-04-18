#include "mbed.h"
#include "HTS221Sensor.h"
#include "LPS22HBSensor.h"
#include "LSM6DSLSensor.h"
#include "lis3mdl_class.h"
#include "VL53L0X.h"

// Define Serial object for virtual COM3 port
serial pc(USBTX, USBRX); // Serial object using USBTX and USBRX pins (virtual COM port)

// objects for various sensors
static DevI2C devI2c(PB_11, PB_10);
static LPS22HBSensor press_temp(&devI2c);
static HTS221Sensor hum_temp(&devI2c);
static LSM6DSLSensor acc_gyro(&devI2c, 0xD4, D4, D5); // high address
static LIS3MDL magnetometer(&devI2c, 0x3C);
static DigitalOut shutdown_pin(PC_6);
static VL53L0X range(&devI2c, &shutdown_pin, PC_7, 0x52);

// functions to print sensor data
void print_t_rh() {
    float value1, value2;
    hum_temp.get_temperature(&value1);
    hum_temp.get_humidity(&value2);

    press_temp.get_temperature(&value1);
    press_temp.get_pressure(&value2);
    int pc; printf("LPS22HB: [temp] %.2f C, [press] %.2f mbar\r\n", value1, value2);
}

void print_mag() {
    int32_t axes[3];
    magnetometer.get_m_axes(axes);
    int pc; printf("LIS3MDL [mag/mgauss]:    %6d, %6d, %6d\r\n", axes[0], axes[1], axes[2]);
}

void print_accel() {
    int32_t axes[3];
    acc_gyro.get_x_axes(axes);
    int pc; printf("LSM6DSL [acc/mg]:        %6d, %6d, %6d\r\n", axes[0], axes[1], axes[2]);
}

void print_gyro() {
    int32_t axes[3];
    acc_gyro.get_g_axes(axes);
    int pc; printf("LSM6DSL [gyro/mdps]:     %6d, %6d, %6d\r\n", axes[0], axes[1], axes[2]);
}

void print_distance() {
    uint32_t distance;
    int status = range.get_distance(&distance);
    if (status == VL53L0X_ERROR_NONE) {
        pc.printf("VL53L0X [mm]:            %u\r\n", distance);
    } else {
        int pc; printf("VL53L0X [mm]:                --\r\n");
    }
}

/* Simple main function */
int main() {
    uint8_t id;
    char key;

    hum_temp.init(NULL);
    press_temp.init(NULL);
    magnetometer.init(NULL);
    acc_gyro.init(NULL);
    range.init_sensor(VL53L0X_DEFAULT_ADDRESS);

    hum_temp.enable();
    press_temp.enable();
    acc_gyro.enable_x();
    acc_gyro.enable_g();

    pc.printf("\033[2J\033[20A");
    pc.printf("\r\n--- Starting new run ---\r\n\r\n");

    hum_temp.read_id(&id);
    pc.printf("HTS221  humidity & temperature    = 0x%X\r\n", id);
    press_temp.read_id(&id);
    pc.printf("LPS22HB pressure & temperature    = 0x%X\r\n", id);
    magnetometer.read_id(&id);
    pc.printf("LIS3MDL magnetometer              = 0x%X\r\n", id);
    acc_gyro.read_id(&id);
    pc.printf("LSM6DSL accelerometer & gyroscope = 0x%X\r\n", id);

    pc.printf("\n\r--- Reading sensor values ---\n\r");
    print_t_rh();
    print_mag();
    print_accel();
    print_gyro();
    print_distance();
    pc.printf("\r\n");

    while (1) {
        // Read key from Serial if available
        if (pc.readable()) {
            key = pc.getc();
            switch (key) {
                case 'a':
                    print_accel();
                    break;
                case 'g':
                    print_gyro();
                    break;
                case 'm':
                    print_mag();
                    break;
                case 'd':
                    print_distance();
                    break;
                case 't':
                    print_t_rh();
                    break;
                default:
                    pc.printf("Invalid key pressed.\r\n");
            }
        }
        wait_us(5000);
    }
}