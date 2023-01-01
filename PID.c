#include <stdio.h>
#include <math.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "bmi160.h"

/* local macro definitions */
/*! I2C interface communication, 1 - Enable; 0- Disable */
#define BMI160_INTERFACE_I2C  1

/*! SPI interface communication, 1 - Enable; 0- Disable */
#define BMI160_INTERFACE_SPI  0

#if (!((BMI160_INTERFACE_I2C == 1) && (BMI160_INTERFACE_SPI == 0)) && \
    (!((BMI160_INTERFACE_I2C == 0) && (BMI160_INTERFACE_SPI == 1))))
#error "Invalid value given for the macros BMI160_INTERFACE_I2C / BMI160_INTERFACE_SPI"
#endif

#define BMI160_SHUTTLE_ID     0x38
#define BMI160_I2C_ADDR       0x69
#define I2C_PORT              i2c0
#define G_ms2                 9.8200000000f
#define RAD_TO_DEG            57.29577951308f        //180/pi
#define COMP_FILT_ALPHA       0.4000000000f

/* BMI160 structs */
/*! @brief This structure containing relevant bmi160 info */
struct bmi160_dev bmi160dev;

/*! @brief variable to hold the bmi160 accel data */
struct bmi160_sensor_data bmi160_accel;

/*! @brief variable to hold the bmi160 gyro data */
struct bmi160_sensor_data bmi160_gyro;

struct pid {
/* PID parameters */
float Kp;
float Ki;
float Kd;
/* Derivative low-pass filter time constant */
float tau ;

/* Output limits */
float limMin;
float limMax;

/* Integrator limits */
float limMinInt;
float limMaxInt;

/* Sample time (in seconds) */
float T;

/* Controller "memory" */
float integrator;
float prevError;            /* Required for integrator */
float differentiator;
float prevMeasurement;      /* Required for differentiator */

/* Controller output */
float out;
}pid_1, pid_2, pid_3, pid_4;

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

//Led test
/*gpio_init(LED_PIN);
gpio_set_dir(LED_PIN, GPIO_OUT);
gpio_put(LED_PIN, 1);*/

/* Function declarations */
int8_t bmi_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint16_t len) {

    uint8_t temp_buf[] = {reg_addr, *buf};

    i2c_write_blocking(I2C_PORT, BMI160_I2C_ADDR, temp_buf, 2, false);

    return 0;
}

int8_t bmi_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint16_t len) {
    
    i2c_write_blocking(I2C_PORT, BMI160_I2C_ADDR, &reg_addr, 1, true); // true to keep master control of bus
    
    i2c_read_blocking(I2C_PORT, BMI160_I2C_ADDR, buf, len, false);
    
    return 0;
}   

void bmi_delay(uint32_t period) {

    sleep_ms(1);
}

/* Function definitions */
static void init_bmi160(void)
{
    bmi160dev.id = BMI160_I2C_ADDR;
    bmi160dev.intf = BMI160_I2C_INTF;
    bmi160dev.write = bmi_write;
    bmi160dev.read = bmi_read;
    bmi160dev.delay_ms = bmi_delay;

    int8_t rslt = BMI160_OK;
    
    rslt = bmi160_init(&bmi160dev);

    sleep_ms(2000);

    if (rslt == BMI160_OK)
    {
        printf("BMI160 initialization success !\n");
        printf("Chip ID 0x%X\n", bmi160dev.chip_id);
    }
    else
    {
        printf("BMI160 initialization failure !\n");
        printf("Code: %d", rslt);
    }
    sleep_ms(1000);
}

static void config_bmi160(void) {
    int8_t rslt = BMI160_OK;
    
    /* Select the Output data rate, range of accelerometer sensor */
    bmi160dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;          //Low pass filter, cutoff @ 684Hz, page 19
    bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    bmi160dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi160dev.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;            //Low pass filter, cutoff @ 890Hz, page 21
    bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi160dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&bmi160dev);

    /* Set power mode*/
    rslt = bmi160_set_power_mode(&bmi160dev);
}

static void config_pwm_2 (int duty_cycle, int period)    {
    gpio_set_function(2, GPIO_FUNC_PWM);                        //Enable pin 2 as pwm
    gpio_pull_up(2);                                            //Enable pull up for gpio 2
    uint slice_num = pwm_gpio_to_slice_num(2);                  //Get pwm channel from GPIO
    pwm_set_enabled(slice_num, true);                           //Turn it on
    pwm_set_wrap(slice_num, period);                            //set wrap point
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);      //set the set point
}

static void config_pwm_3 (int duty_cycle, int period)    {
    gpio_set_function(3, GPIO_FUNC_PWM);                        
    gpio_pull_up(3);                                            
    uint slice_num = pwm_gpio_to_slice_num(3);                  
    pwm_set_enabled(slice_num, true);                           
    pwm_set_wrap(slice_num, period);                            
    pwm_set_chan_level(slice_num, PWM_CHAN_B, duty_cycle);      
}

static void config_pwm_6 (int duty_cycle, int period)    {
    gpio_set_function(6, GPIO_FUNC_PWM);                        
    gpio_pull_up(6);                                            
    uint slice_num = pwm_gpio_to_slice_num(6);                  
    pwm_set_enabled(slice_num, true);                           
    pwm_set_wrap(slice_num, period);                            
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);      
}

static void config_pwm_7 (int duty_cycle, int period)    {
    gpio_set_function(7, GPIO_FUNC_PWM);                        
    gpio_pull_up(7);                                            
    uint slice_num = pwm_gpio_to_slice_num(7);                  
    pwm_set_enabled(slice_num, true);                           
    pwm_set_wrap(slice_num, period);                            
    pwm_set_chan_level(slice_num, PWM_CHAN_B, duty_cycle);      
}

float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = (float)(1 << bit_width) / 2.0f;

    return G_ms2 * val * g_range / half_scale;
}

float pid (struct pid *pid, float setpoint, float measurement) {
    /* Clear controller variables */
    pid->integrator = 0.0f;
    pid->prevError  = 0.0f;
    pid->differentiator  = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->out = 0.0f;
    
    /* Error signal */
    float error = setpoint - measurement;

    /* Proportional */
    float proportional = pid->Kp * error;

    /* Integral */
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

    /* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;
    }

    /* Derivative (band-limited differentiator) */
    pid->differentiator = -(2.0f *pid-> Kd * (measurement - pid->prevMeasurement)  /* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);


    /* Compute output and apply limits */
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {
        pid->out = pid->limMax;
    } else if (pid->out < pid->limMin) {
        pid->out = pid->limMin;
    }

    /* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

    //printf("PID out: %0.3f\n", out);
    //sleep_ms(10);

    return(pid->out);
}

float mma(float thrust, float yaw, float pitch, float roll) {
    float motor_FR;                             //4 motors
    float motor_FL;
    float motor_BR;
    float motor_BL;
    
    motor_FR = thrust + yaw + pitch + roll;     //Motor blending algorithms
    motor_FL = thrust - yaw + pitch - roll;
    motor_BR = thrust - yaw - pitch + roll;
    motor_BL = thrust + yaw - pitch - roll;

    config_pwm_2(motor_FR, 1250);
    config_pwm_3(motor_FL, 1250);
    config_pwm_6(motor_BR, 1250);
    config_pwm_7(motor_BL, 1250);
    
    //printf("\nFR: %0.3f\nFL: %0.3f\nBR: %0.3f\nBL: %0.3f\n", motor_FR, motor_FL, motor_BR, motor_BL);
    //sleep_ms(200);
}

int main(void)  {
    stdio_init_all();                       //Init pico I/O, USB etc..

    i2c_init(I2C_PORT, 400 * 1000);         //Init i2c comm. @ 400kBit/s

    /* Configure the I2C communication */
    gpio_set_function(4, GPIO_FUNC_I2C);    //Set port 4 & 5 to I2C communication
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);                        //Enable pull-up resistors for port 4 & 5
    gpio_pull_up(5);

    sleep_ms(300);

    init_bmi160();                          //Initialize bmi160

    config_bmi160();                        //Set acc./gyro rates etc..

    gpio_init(LED_PIN);                     //Led test (onboard LED on)
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    float phiHat_rad_acc = 0.0f;            //Initial estimate values for acc
    float thetaHat_rad_acc = 0.0f;
    float phiHat_rad = 0.0f;                //Initial estimate values for gyro
    float thetaHat_rad = 0.0f;
    float phiHat_rad_combined = 0.0f;
    float thetaHat_rad_combined = 0.0f;     //Combined values of +-
    signed int accel_x;
    signed int accel_y;
    signed int accel_z;
    int usr_pwm;

    /* PID parameters */
    pid_1.Kp = 2, pid_1.Ki = 0.5, pid_1.Kd = 0.8;
    pid_2.Kp = 2, pid_2.Ki = 0.5, pid_2.Kd = 0.8;
    pid_3.Kp = 2, pid_3.Ki = 0.5, pid_3.Kd = 0.8;
    pid_4.Kp = 2, pid_4.Ki = 0.5, pid_4.Kd = 0.8;

    /* Output limits */
    pid_1.limMin = -1250, pid_1.limMax = 1250;
    pid_2.limMin = -1250, pid_2.limMax = 1250;
    pid_3.limMin = -1250, pid_3.limMax = 1250;
    pid_4.limMin = -1250, pid_4.limMax = 1250;

    /* Integrator limits */
    pid_1.limMinInt = -10, pid_1.limMaxInt = 10;
    pid_2.limMinInt = -10, pid_2.limMaxInt = 10;
    pid_3.limMinInt = -10, pid_3.limMaxInt = 10;
    pid_4.limMinInt = -10, pid_4.limMaxInt = 10;
    
    /* Sample time */
    pid_1.T = 10/6400.0f;
    pid_2.T = 10/6400.0f;
    pid_3.T = 10/6400.0f;
    pid_4.T = 10/6400.0f;

    // Infinite Loop
    while(1)    {
        /* Read Accel and Gyro raw data */
        bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &bmi160_accel, &bmi160_gyro, &bmi160dev);
        
        /* Convert accelerometer values to m/s2 */
        accel_x = lsb_to_ms2(bmi160_accel.x, 2, 16);
        accel_y = lsb_to_ms2(bmi160_accel.y, 2, 16);
        accel_z = lsb_to_ms2(bmi160_accel.z, 2, 16);
        //printf("\nax:%d\tay:%d\taz:%d\n", accel_x, accel_y, accel_z);
        
        /* Convert m/s2 to Euler angles (rad) */
        phiHat_rad_acc = atan2(accel_y, accel_z);
        thetaHat_rad_acc = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z));    //arcsin(x) = arctan(x/sqrt(1-x^2))
        //printf("\nax:%0.1f\tay:%0.1f", phiHat_rad_acc, thetaHat_rad_acc);
        //sleep_ms(200);
        
        /* Convert gyroscope values to Euler rates (rad/s) */
        float phiDot_rps = bmi160_gyro.x + tanf(thetaHat_rad) * (sinf(phiHat_rad) * bmi160_gyro.y + cosf(phiHat_rad) * bmi160_gyro.z);
        float thetaDot_rps = cosf(phiHat_rad) * bmi160_gyro.y - sinf(phiHat_rad) * bmi160_gyro.z;

        /* Complementary filter: Combine acc estimates with gyro integral to get angle estimation */
        phiHat_rad = COMP_FILT_ALPHA * phiHat_rad_acc + (1.0f - COMP_FILT_ALPHA) * (phiHat_rad + (1/6400.0f) * phiDot_rps);     //Roll estimate
        thetaHat_rad = COMP_FILT_ALPHA * thetaHat_rad_acc + (1.0f - COMP_FILT_ALPHA) * (thetaHat_rad + (1/6400.0f) * thetaDot_rps);     //Pitch estimate

        //printf("ax:%0.3f, ay:%0.3f\r\n", phiHat_rad * RAD_TO_DEG, thetaHat_rad * RAD_TO_DEG);

        /*
        pid(&pid_1, 0, phiHat_rad * RAD_TO_DEG);
        pid(&pid_2, 0, thetaHat_rad * RAD_TO_DEG);
        pid(&pid_3, 0, - phiHat_rad * RAD_TO_DEG);
        pid(&pid_4, 0, - thetaHat_rad * RAD_TO_DEG);
        */

        phiHat_rad_combined = pid(&pid_1, 0, phiHat_rad * RAD_TO_DEG);
        thetaHat_rad_combined = pid(&pid_2, 0, thetaHat_rad * RAD_TO_DEG);
        mma(850, 0, thetaHat_rad_combined, phiHat_rad_combined);

        //printf("X: %0.3f\n Y: %0.3f\n", phiHat_rad_combined, thetaHat_rad_combined);
        //sleep_ms(100);
        /*
        printf("Motor 2: %0.3f\n", pid_1.out);
        printf("Motor 3: %0.3f\n", pid_2.out);
        printf("Motor 6: %0.3f\n", pid_3.out);
        printf("Motor 7: %0.3f\n", pid_4.out);
        //sleep_ms(100);
        */

        /*
        config_pwm_2(updateSpeed[0], 1250);
        config_pwm_6(updateSpeed[1], 1250);
        config_pwm_3(updateSpeed[2], 1250);
        config_pwm_7(updateSpeed[3], 1250);

        currentSpeed[0] = updateSpeed[0];
        currentSpeed[1] = updateSpeed[1];
        currentSpeed[2] = updateSpeed[2];
        currentSpeed[3] = updateSpeed[3];
        */
        
        /*
        sleep_ms(5000);
        printf("Enter pwm: ");
        scanf("%d", &usr_pwm);
        printf("\npwm: %d", usr_pwm);
        sleep_ms(100);
        config_pwm_2(usr_pwm, 1250);
        config_pwm_3(usr_pwm, 1250);
        config_pwm_6(usr_pwm, 1250);
        config_pwm_7(usr_pwm, 1250);
        */
    }
}