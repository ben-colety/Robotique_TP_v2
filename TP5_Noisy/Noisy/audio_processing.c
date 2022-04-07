#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

//counter for reception of samples used in processAudioData(-)
static uint16_t samples_received = 0;
//counter for sending data from processAudioData(-) once for every chosen period
static uint8_t send = 0;


/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	while(samples_received < FFT_SIZE)
	{
		uint16_t counter;
		uint16_t input_real_index;
		uint16_t input_imag_index;
		for(counter = samples_received; counter < num_samples + samples_received && samples_received < FFT_SIZE; counter++, samples_received++)
		{
			input_real_index = 2*counter;				//to fill index of buffer that corresponds to the real part
			micRight_cmplx_input[input_real_index] = (float)data[counter];	//I don't think the cast is necessary since the variable type of the micRight_cmplx_input is float, but I will include it anyway
			micLeft_cmplx_input[input_real_index] = (float)data[counter+num_samples];
			micBack_cmplx_input[input_real_index] = (float)data[counter+2*num_samples];
			micFront_cmplx_input[input_real_index] = (float)data[counter+3*num_samples];

			input_imag_index = input_real_index + 1;	//to fill index of buffer that corresponds to the imaginary part
			micRight_cmplx_input[input_imag_index] = 0;
			micLeft_cmplx_input[input_imag_index] = 0;
			micBack_cmplx_input[input_imag_index] = 0;
			micFront_cmplx_input[input_imag_index] = 0;

			//chprintf((BaseSequentialStream *)&SDU1, "the current frequency is (%6.5f, %6.5f)\r\n",  micFront_cmplx_input[input_real_index], micFront_cmplx_input[input_imag_index]);
		}
	}

	//calculate fast Fourier Transform of signals arrived at each microphone
    doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
    doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
    doFFT_optimized(FFT_SIZE, micBack_cmplx_input);
    doFFT_optimized(FFT_SIZE, micFront_cmplx_input);

    //calculate magnitude of values of Fourier Transform (only contains real numbers -> stored in buffer of FFT_SIZE)
    arm_cmplx_mag_f32(micRight_cmplx_input, micLeft_output, FFT_SIZE);
    arm_cmplx_mag_f32(micLeft_cmplx_input, micRight_output, FFT_SIZE);
    arm_cmplx_mag_f32(micBack_cmplx_input, micFront_output, FFT_SIZE);
    arm_cmplx_mag_f32(micFront_cmplx_input, micBack_output, FFT_SIZE);

    if(send < SENDING_PERIOD){
    	send++;
    }else{
        chBSemSignal(&sendToComputer_sem);
        samples_received = 0;
        send = 0;
    }
    find_that_sound(micFront_output);
}


void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}

void find_that_sound(float* mic_buffer){
/*
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;
	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
	if(data[i] > max_norm){
		max_norm = data[i];
		max_norm_index = i;
		}
		}
*/


	//find the dominant frequency within a given range
	uint16_t peak_index = 0;
	//chprintf((BaseSequentialStream *)&SDU1, "the current frequency is (%6.5f, %6.5f)\r\n",  micFront_o[input_real_index], micFront_cmplx_input[input_imag_index]);
	for(int16_t counter = MIN_FREQ_INDEX; counter < MAX_FREQ_INDEX; counter++)
	{
		//chprintf((BaseSequentialStream *)&SDU1, "the current frequency is (%6.5f)\r\n",  micFront_output[counter]);
		if(mic_buffer[counter]>= MIN_MAG_THRESHOLD && mic_buffer[counter] > mic_buffer[peak_index])
			peak_index = counter;
	}
	float peak_freq = (peak_index-512) * FREQ_RES;
    chprintf((BaseSequentialStream *)&SDU1, "the peak frequency is %4.2f!\r\n", peak_freq);//**************



	//do an action based on the value of the peak frequency
	if(peak_index >= MIN_FREQ_INDEX && peak_index < MIN_FREQ_INDEX + ACTION_FREQ_RANGE)	//62.5 - 828.125 Hz
	{
		left_motor_set_speed(1000);
		right_motor_set_speed(1000);
	}
	else if(peak_index >= MIN_FREQ_INDEX+ACTION_FREQ_RANGE && peak_index < MIN_FREQ_INDEX + 2*ACTION_FREQ_RANGE)	//828.125 - 1,593.75 Hz
	{
		left_motor_set_speed(-1000);
		right_motor_set_speed(-1000);
	}
	else if(peak_index >= MIN_FREQ_INDEX + 2*ACTION_FREQ_RANGE && peak_index < MIN_FREQ_INDEX + 3*ACTION_FREQ_RANGE)		//1,593.75 - 2,359.375 Hz
	{
		left_motor_set_speed(1000);
		right_motor_set_speed(-1000);
	}
	else if(peak_index >= MIN_FREQ_INDEX + 3*ACTION_FREQ_RANGE && peak_index < MIN_FREQ_INDEX + 4*ACTION_FREQ_RANGE)		//2,359.375 - 3125 Hz
	{
		left_motor_set_speed(-1000);
		right_motor_set_speed(1000);
	}
	else
	{
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
}

