#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

//number of times the program should NOT send the results of the fourier calculations before sending
//i.e. if the value SENDING_PERIOD is 9, the results will be be sent 1 of 10 cycles of the program
#define SENDING_PERIOD 9

#define MIC_SAMPLE_RATE 16000	//Hz

#define FREQ_RES			(float)((MIC_SAMPLE_RATE/2)/(FFT_SIZE/2))
#define MIN_FREQ_INDEX		4 + FFT_SIZE/2 //62.5 Hz
#define MAX_FREQ_INDEX		200 + FFT_SIZE/2//3.125kHz
#define MIN_MAG_THRESHOLD	10000

#define	NB_ACTIONS 			4
#define	ACTION_FREQ_RANGE 	(MAX_FREQ_INDEX-MIN_FREQ_INDEX)/NB_ACTIONS

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;


void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_send_to_computer(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

//calculates the dominant frequency and does an action depending on what frequency it is
void find_that_sound(float* mic_buffer);

#endif /* AUDIO_PROCESSING_H */
