#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

#define TARGET_DISTANCE 10 //[cm]
#define K_p 1
#define K_i 0

//#define SPEED_LIMIT

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;

    float current_distance, error, integral = 0.0;

    while(1){
        time = chVTGetSystemTime();

        current_distance = get_distance_cm();
        if(current_distance > 0){
        error = current_distance - TARGET_DISTANCE;
		chprintf((BaseSequentialStream *)&SD3, "error = %3.2f \r\n", error);
        integral += error;
        
        speed = ((K_p * error) + (K_i * integral));
        }else{
        	speed = 0;
        }

        // applies the speed from the PI regulator
		right_motor_set_speed(speed);
		left_motor_set_speed(speed);
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
