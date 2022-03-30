#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

#define BLUE_POS		0
#define BLUE_LENGTH		5
#define BLUE_MASK		0b0000000000011111
#define GREEN_POS		5
#define GREEN_LENGTH	6
#define GREEN_MASK		0b0000011111100000
#define RED_POS			11
#define RED_LENGTH		5
#define RED_MASK		0b1111100000000000

#define SIZE_INT8		8

#define SMOOTHING_FACTOR		2
#define DETECTION_THRESHOLD		0.5

static float distance_cm = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	//systime_t time;

    while(1){
    	//time = chVTGetSystemTime();
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
		//time = chVTGetSystemTime() - time;
		//chprintf((BaseSequentialStream *)&SD3,"time = %i \r\n",time);
		//chThdSleepMilliseconds(12);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint8_t send_counter = 0;
	uint16_t line_width_px = 0;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();
		uint16_t img_buff_pixel = 0;
		for(uint16_t pixel = 0; pixel < IMAGE_BUFFER_SIZE; pixel++)
		{
			img_buff_pixel = ((((img_buff_ptr[2*pixel] << SIZE_INT8) | (img_buff_ptr[2*pixel+1]))&GREEN_MASK)); //takes only green from image buffer
			image[pixel] = (uint8_t)(img_buff_pixel >> GREEN_POS); //Truncation
		}
		if(send_counter++%2 == 1 ) SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);

		//measure line width in px
		line_width_px = get_line_width_px(image, IMAGE_BUFFER_SIZE);
		//chprintf((BaseSequentialStream *)&SD3, "line width = %i \r\n", line_width_px);
    }
}

float get_distance_cm(void){
	return distance_cm;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

uint16_t get_line_width_px(uint8_t* image, uint16_t size){
	bool section_measure = false; //indicates if we are measuring the size of a section that could be a line
	uint16_t section_start_temp, section_end_temp = 0;
	uint16_t pixel_local_avg, pixel_global_avg = 0;
	uint16_t line_width_px = 0;

	//calculate average pixel intensity
	for(uint16_t pixel = 0; pixel < size; pixel++)
	{
		pixel_global_avg += image[pixel];
	}
	pixel_global_avg /= IMAGE_BUFFER_SIZE;

	//value smoothing and comparison
	for(uint16_t pixel = 0; pixel < size; pixel++)
	{
		//take local average of pixels symmetrically around the current pixel
		if((pixel <= SMOOTHING_FACTOR) || (pixel >= size - SMOOTHING_FACTOR))
			pixel_local_avg = image[pixel];
		else
		{
			pixel_local_avg = image[pixel];
			for(uint8_t smoothing_index = 1; smoothing_index <= SMOOTHING_FACTOR; smoothing_index++)
			{
				pixel_local_avg += image[pixel-smoothing_index] + image[pixel+smoothing_index];
			}
			pixel_local_avg /= SMOOTHING_FACTOR*2 + 1;
		}

		if(pixel_local_avg <= pixel_global_avg * DETECTION_THRESHOLD && !section_measure)	//possible line section beginning
		{
			section_start_temp = pixel;
			section_measure = true;
		}
		else if(pixel_local_avg <= pixel_global_avg * DETECTION_THRESHOLD && section_measure)	//possible line section end
		{
			section_end_temp = pixel;
		}
		else if(pixel_local_avg > pixel_global_avg * DETECTION_THRESHOLD && section_measure)	//end measure of section
		{
			section_measure = false;
			if(line_width_px < section_end_temp-section_start_temp)
			{
				line_width_px = section_end_temp-section_start_temp;
			}
		}
	}
	return line_width_px;
}
