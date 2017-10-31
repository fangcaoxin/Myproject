#pragma once
#include <math.h>
/**parameter for camera and lens
* pixel size is 3.5umx3.5um
* resolution 1280x960
* model:WATEC WAT-1100MBD
* lens:
* angle of view 100.7(D)x77.7(H)x57(V)
* model: WATEC M3718BC-12
* the formation image size of lens is larger than image sensor

* the real angle of field 
* horizontal:atan(4.48 / (2 * 3.7)) * 2 
* vetical:atan(3.36 / (2 * 3.7)) * 2
*/
#define FOCUS_LENGTH 3.7
#define H_ANGLE h_angle
#define V_ANGLE v_angle

#define CAMEAR_LED_H_DIS camera_led_h_dis
#define CAMERA_LED_V_DIS camera_led_v_dis

double h_angle = atan(4.48 / (2 * 3.7)) * 2;
double v_angle = atan(3.36 / (2 * 3.7)) * 2;

double camera_led_h_dis = 0.295;
double camera_led_v_dis = 0.105;