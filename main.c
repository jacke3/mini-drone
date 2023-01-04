#include <stdio.h>
#include <math.h>
//FREERTOS HEADER FILES
#include "FreeRTOS.h"
#include "task.h"
#include <queue.h>
//PICO-SDK HEADER FILES
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/adc.h"

//GLOBAL VARIABLES
volatile bool time_take = false;
const uint s_velocity = 340;

const uint drone_speed = 100;
const uint drone_distance = 10;

//MOTORS
typedef enum MOTOR_{

    MOTOR1 = 0,
    MOTOR2 = 1,
    MOTOR3 = 4,
    MOTOR4 = 6,

}_MOTOR;

typedef enum MOTORACTION_{

    LOWER = 0,
    HIGHER = 1,
    FORWARD = 2,
    RIGHT_ROTATE = 3,
    LEFT_ROTATE = 4,

}MOTOR_ACTION;
//SENSORS
typedef enum SENSOR_{

    SENSOR1 = 15,
    SENSOR2 = 16,

}_SENSOR;
//PWM VALUES
typedef struct _pwm{

    uint slice_num;
    uint channel_num;
    uint wrap_val;
    uint chan_level;
    uint clock_div;
    bool phase_correction;

} pwm_;
///INTERRUPTS
void time_flag_rise(){ 
   time_take = true;
}
void time_flag_fall(){
   time_take = false;
}
//INITILIZATION
void initialization_pins(){

    gpio_init(SENSOR1);
    gpio_init(SENSOR1 + 1);
    gpio_init(SENSOR2);
    gpio_init(SENSOR2 + 1);

    gpio_set_dir(SENSOR1, GPIO_OUT);
    gpio_set_dir(SENSOR2, GPIO_OUT);
    gpio_set_dir(SENSOR1 + 1, GPIO_IN);
    gpio_set_dir(SENSOR2 + 1, GPIO_IN);
    
    gpio_set_function(MOTOR1, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR2, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR3, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR4, GPIO_FUNC_PWM); 

    ///PID????
}
//SENSOR FUNCTIONS
double measure_distance(double time){
    // Distance (in cm) = (elapsed time * sound velocity (340 m/s))/ 2
    double distance = (time * s_velocity) / 2;
    return distance * 100;
}
int sensor_check(_SENSOR val){

        gpio_put(val,1);
        sleep_us(10);
        gpio_put(val,0);
        gpio_set_irq_enabled_with_callback(val + 1, GPIO_IRQ_EDGE_RISE , true , &time_flag_rise);
        while(time_take == false){

        }
        double start_time = time_us_64();  
        gpio_set_irq_enabled_with_callback(val + 1, GPIO_IRQ_EDGE_FALL , true , &time_flag_fall);
        while(time_take == true){

        }  
        double stop_time = time_us_64();  
        double time = (stop_time - start_time) / 1000000;
        printf("%8f\n",time);
        printf("THE DISTANCE IS %8f CM\n", measure_distance(time)); 
        sleep_ms(3000);  

        return measure_distance(time);   
}
//PWM
void pwm_calculator(pwm_ *motor,int frequency,double duty_cycle){
    // T = 1/f
    double clock_div;
    const double one_clock = 0.000000008;
    double T = 1/(double)frequency;
    int tick = round(T / one_clock);
    int duty = round(tick * duty_cycle);

    if(tick >= 65536){

       clock_div = (double)tick/65536;
       clock_div = ceil(clock_div);
       double new_one_clock = one_clock * clock_div;
       tick = round(T / new_one_clock);
       duty = round(tick * duty_cycle);

       motor ->clock_div = clock_div;
    }

    motor ->chan_level = duty;
    motor ->wrap_val = tick;  
}
void pwm_initialize(pwm_ *object){

    pwm_set_wrap(object ->slice_num, object ->wrap_val); //frekvens
    pwm_set_clkdiv(object ->slice_num, object ->clock_div);
    pwm_set_phase_correct(object ->slice_num, object ->phase_correction);

    pwm_set_chan_level(object ->slice_num, object ->channel_num, object ->chan_level); //duty cycle

}
///MOTOR_FUNCTIONS
void pwm_initialize_MOTOR(pwm_ *motor, int frequency, double duty_cycle, _MOTOR val){

    motor -> slice_num = pwm_gpio_to_slice_num(val);
    motor -> channel_num = pwm_gpio_to_channel(val);
    motor -> phase_correction = false;
    pwm_calculator(motor,frequency,duty_cycle);
    pwm_initialize(motor);
    pwm_set_enabled (motor ->slice_num, true); 
}
void regulate_MOTORS(pwm_ *motor, MOTOR_ACTION COMMAND){

    if(COMMAND = HIGHER){
      motor->chan_level = motor->chan_level + 10;
      pwm_set_chan_level(motor->slice_num, motor->channel_num, motor->chan_level);     
    }   
    else if (COMMAND = LOWER){
      motor->chan_level = motor->chan_level - 10;
      pwm_set_chan_level(motor->slice_num, motor->channel_num, motor->chan_level);
    }
}
void DRONE_FRONT_MOTORS(bool UP,pwm_ *motor3,pwm_ *motor4){
    if(UP == false){
       regulate_MOTORS(motor3,LOWER);
       regulate_MOTORS(motor4,LOWER);
    }
    else if(UP == true){
       regulate_MOTORS(motor3,HIGHER);
       regulate_MOTORS(motor4,HIGHER);
    }
}
void DRONE_BACK_MOTORS(bool UP,pwm_ *motor1,pwm_ *motor2){
    if(UP == false){
       regulate_MOTORS(motor1,LOWER);
       regulate_MOTORS(motor2,LOWER);    
    }
    else if(UP == true){
       regulate_MOTORS(motor1,HIGHER);
       regulate_MOTORS(motor2,HIGHER);
    }
}
void DRONE_ROTATION_LEFT(pwm_ *motor1,pwm_ *motor3){}
void DRONE_ROTATION_RIGHT(pwm_ *motor2,pwm_ *motor4){}
void DRONE_STOP(){}
int LANDING(){}


void CONTROL_DRONE(int c[],int size ,pwm_ *motor1,pwm_ *motor2, pwm_ *motor3,pwm_ *motor4){
     for(int i; i<= size; i++){
        //SETS DRONE DIRECTION 
        if(c[i] == RIGHT_ROTATE){

          break;
        }
        if(c[i] == LEFT_ROTATE){

          break;
        }

        for(int y; y<= drone_speed; y++){
            if(c[i] == LOWER){
                DRONE_FRONT_MOTORS(false,motor3,motor4);    
                DRONE_BACK_MOTORS(false,motor1,motor2);  
            }
            else if(c[i] == HIGHER){
                DRONE_FRONT_MOTORS(true,motor3,motor4);
                DRONE_BACK_MOTORS(true,motor1,motor2);
                }
            else if(c[i] == FORWARD){
                DRONE_FRONT_MOTORS(false,motor3,motor4);
            }
        }
        //DRONE MOVING IN DIRECTION 
        for(int x; x<=drone_distance; x++){ 
            if(c[i] == LOWER){
               sensor_check(SENSOR1);    
            }
            else if(c[i] == FORWARD){
               sensor_check(SENSOR2);  
            }  
            sleep_ms(10);
        }
        //SLOW DOWN DRONE
        for(int z; z<= drone_speed; z++){
            if(c[i] == LOWER){
                DRONE_FRONT_MOTORS(true,motor3,motor4);    
                DRONE_BACK_MOTORS(true,motor1,motor2);  
            }
            else if(c[i] == HIGHER){
                DRONE_FRONT_MOTORS(false,motor3,motor4);
                DRONE_BACK_MOTORS(false,motor1,motor2);
            }
            else if(c[i] == FORWARD){
                DRONE_FRONT_MOTORS(true,motor3,motor4);
            }
        }
    }
}
//VOLTAGE REGULATOR
void voltage_regulator(){

    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);
    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    const float conversion_factor = 3.3f / (1 << 12);
    uint16_t result = adc_read();
    printf("voltage: %f V\n", result, result * conversion_factor);
    sleep_ms(500);  
}
int main(){
    
    pwm_ motor1 = {0,0,0,0,1,false};
    pwm_ *motor1_pointer = &motor1;
    pwm_ motor2 = {0,0,0,0,1,false};
    pwm_ *motor2_pointer = &motor2;
    pwm_ motor3 = {0,0,0,0,1,false};
    pwm_ *motor3_pointer = &motor3;
    pwm_ motor4 = {0,0,0,0,1,false};
    pwm_ *motor4_pointer = &motor4;

    int commands[] = {1,2,0};

    stdio_init_all();
    initialization_pins();
    sleep_ms(5000);

    pwm_initialize_MOTOR(motor1_pointer,100000,0.5,MOTOR1);
    pwm_initialize_MOTOR(motor2_pointer,100000,0.5,MOTOR2);
    pwm_initialize_MOTOR(motor3_pointer,100000,0.5,MOTOR3);
    pwm_initialize_MOTOR(motor4_pointer,100000,0.5,MOTOR4);

    CONTROL_DRONE(commands,sizeof(commands),motor1_pointer,motor2_pointer,motor3_pointer,motor4_pointer);


    /*TaskHandle_t task;
    xTaskCreate(main_task, "MainThread", configMINIMAL_STACK_SIZE, NULL, TEST_TASK_PRIORITY, &task);
    vTaskStartScheduler();*/
}






   
