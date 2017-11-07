
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

*照明：
SHARP GW5BTJ65K03
 10Wクラス 6500K （前方に本LEDを4個使用）
*/
#define FOCUS_LENGTH 3.7
#define H_ANGLE h_angle
#define V_ANGLE v_angle

#define CAMEAR_LED_H_DIS camera_led_h_dis
#define CAMERA_LED_V_DIS camera_led_v_dis
#define CAMERA_LED_Z_DIS camera_led_z_dis
#define SENOR_WIDTH (double)4.48;
#define SENSOR_HEIGHT (double) 3.36

#define NRER_RED 0.7 /*clear water 0.8~0.85*/
#define NRER_GREEN 0.8 /*clear water 0.93~0.97*/
#define NRER_BLUE 0.7 /*clear water 0.95~0.99 */


double h_angle = atan(4.48 / (2 * 3.7)) * 2;
double v_angle = atan(3.36 / (2 * 3.7)) * 2;

double camera_led_h_dis = 0.295;
double camera_led_v_dis = 0.105;
double camera_led_z_dis = 0.196;