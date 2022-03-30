#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);
void process_image_start(void);
uint16_t get_line_width_px(uint8_t* image, uint16_t size);
float calculate_distance(uint16_t line_width_px);

#endif /* PROCESS_IMAGE_H */
