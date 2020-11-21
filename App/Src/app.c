#include "app.h"
#include "DD_Gene.h"
#include "DD_RCDefinition.h"
#include "SystemTaskManager.h"
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "MW_GPIO.h"
#include "MW_IWDG.h"
#include "message.h"
#include "MW_flash.h"
#include "constManager.h"
#include "trapezoid_ctrl.h"

static char *testmode_name[] = {
  "MANUAL_SUSPENSION",
  "AUTO_FIRSTMECHA_UP",
  "AUTO_FIRSTMECHA_DOWN",
  "AUTO_SHEETS",
  "AUTO_SHEETS_1ZONE_RETURN",
  "AUTO_TOWEL_ALL",
  "AUTO_TOWEL_ALL_CLIP",
  "AUTO_TOWEL_3",
  "AUTO_TOWEL_2",
  "AUTO_TOWEL_1",
  "AUTO_SHEETS_TOWEL",
  "AUTO_TEST",
  "NO_OPERATION",
  "STOP_EVERYTHING",
};
/********/
static
int duty_adjust(int duty, int omni_num);
static
int32_t absolute_distance(int32_t position[2]);
static
int auto_omni_suspension(int x, int y, int w, int max_duty);
static
int32_t go_to_target(int32_t target_position[3], int32_t now_position[3], bool not_stop);
static
Arm_MovingSituation_t arm_spin_to_target(Arm_MovingTarget_t target, bool reset);
static
int odmetry_position(int32_t position[3], bool adjust_flag[3], int32_t adjust_value[3], bool reset);
static
int manual_omni_suspension(void);
static
int spin_xy_point(int32_t xy_point[2], double degree);
static 
int duty_check(void);
static
void raspi_switch_ctrl(int switch_data[RASPI_SWITCH_NUM]);
static
void all_motor_stop(void);
static 
int ABSystem(void);
static
int LEDSystem(void);
/*メモ
 *g_ab_h...ABのハンドラ
 *g_md_h...MDのハンドラ
 *
 *g_rc_data...RCのデータ
 */

int appInit(void){

	ad_init();

	/*GPIO の設定などでMW,GPIOではHALを叩く*/
	return EXIT_SUCCESS;
}

/*application tasks*/
int appTask(void){
	int ret=0;
	int i;
	int32_t encoder_count = 0;
	static int32_t recent_system_count = 0;

	static unsigned int target_count = 0;
	static bool circle_flag = false, cross_flag = false;
	static int target_duty=0,duty;

	int32_t adjust[3] = {};
	bool adjust_flag[3] = {false, false, false};
	static int32_t now_position[3] = {};
	int32_t target_position[3] = {};

	static int raspi_switch_data[RASPI_SWITCH_NUM] = {};

	if(!__RC_ISPRESSED_CIRCLE(g_rc_data)) circle_flag = true;
	if(!__RC_ISPRESSED_CROSS(g_rc_data)) cross_flag = true;

	if(__RC_ISPRESSED_CIRCLE(g_rc_data) && circle_flag){ 
		target_count++;
		circle_flag = false;
	}
	if(__RC_ISPRESSED_CROSS(g_rc_data) && cross_flag){
		target_count--;
		cross_flag = false;
	}

	//target_duty = (int)((double)target_count*(200.0/3.0));

	//odmetry_position(now_position, adjust_flag, adjust, false);

	if(__RC_ISPRESSED_CIRCLE(g_rc_data)){
		target_position[0] = 3000;
		target_position[1] = 0;
		target_position[2] = 18000;
	}
	if(__RC_ISPRESSED_CROSS(g_rc_data)){
		target_position[0] = 1500;
		target_position[1] = -300;
		target_position[2] = 9000;
	}
	if(__RC_ISPRESSED_SQARE(g_rc_data)){
		target_position[0] = -3000;
		target_position[1] = 0;
		target_position[2] = 18000;
	}
	if(__RC_ISPRESSED_TRIANGLE(g_rc_data)){
		target_position[0] = -3000;
		target_position[1] = 800;
		target_position[2] = 18000;
	}
	if(__RC_ISPRESSED_L1(g_rc_data)){
		target_position[0] = 0;
		target_position[1] = 500;
		target_position[2] = 0;
	}

	//go_to_target(target_position,now_position,false);

	if(__RC_ISPRESSED_L2(g_rc_data)){
		adjust_flag[0] = true;
		adjust_flag[1] = true;
		adjust_flag[2] = true;
		adjust[0] = 0;
		adjust[1] = 0;
		adjust[2] = 0;
		odmetry_position(now_position, adjust_flag, adjust, false);
		adjust_flag[0] = false;
		adjust_flag[1] = false;
		adjust_flag[2] = false;
	}
	
	int rc_analogdata = DD_RCGetRY(g_rc_data);
	static int test_duty = 0;

	if(test_duty >= 10000){
		test_duty = 0;
	}
	for(i=0;i<DD_NUM_OF_MD;i++){
		if(abs(rc_analogdata)==0){
			g_md_h[i].mode = D_MMOD_FORWARD;
    	  	g_md_h[i].duty = 0;
    	}
    	else{
			test_duty++;
    	  	if(rc_analogdata > 0){
			/*前後の向き判定*/
				g_md_h[i].mode = D_MMOD_FORWARD;
    	  	}
    	  	else{
				g_md_h[i].mode = D_MMOD_BACKWARD;
    	  	}
    	  	/*絶対値を取りDutyに格納*/
    	  	g_md_h[i].duty = abs(rc_analogdata) * MD_GAIN;
    	}
	}

	//manual_omni_suspension();
	//duty_check();

	/*if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
		encoder_count = DD_encoder1Get_int32();
		MW_printf("encoder_count:[%d]  time:[%d]  target:[%d]\n",encoder_count,g_SY_system_counter-recent_system_count,target_duty);
		if(abs(encoder_count) > target_duty){
			if(abs( abs(encoder_count) - target_duty) > 30){
				duty -= 50;
			}else{
				duty--;
			}
		}else if(abs(encoder_count) < target_duty){
			if(abs( abs(encoder_count) - target_duty) > 30){
				duty += 50;
			}else{
				duty++;
			}
		}
		for(i=0; i<4; i++){
			g_md_h[i].mode = D_MMOD_FORWARD;
			g_md_h[i].duty = duty;
			if(g_md_h[i].duty >= 9900){
				g_md_h[i].duty = 9900;
			}
		}
		DD_encoder1reset();
		encoder_count = 0;
		recent_system_count = g_SY_system_counter;
	}*/

	//if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
	//	MW_printf("encoder_count:[%d]  time:[%d]\n",encoder_count,g_SY_system_counter-recent_system_count);
	//}

#if USE_RASPI_CONTROL
	raspi_switch_ctrl(raspi_switch_data);
#endif

	return EXIT_SUCCESS;
}

static
int32_t go_to_target(int32_t target_position[3], int32_t now_position[3], bool not_stop){
	//MovingSituation_t return_situation;
	static int32_t re_now_position[2] = {};
	int32_t distance;
	int input_duty[3] = {};
	int input_max_duty_cal[2] = {};
	int input_max_duty = 0;
	int i;
	static int recent_input_max_duty = 0;
	static int now_degree = 0, target_degree = 0;
	static int abs_degree_to_target = 0;
	int turn_destination_coeff = 1;

	for(i=0; i<2; i++){
		re_now_position[i] = now_position[i] - target_position[i]; //目標を(0,0)とした時の今のポジションを計算
	}

	spin_xy_point(re_now_position, (double)(now_position[2])/100.0); //今の角度に従って目標座標(角度)を調整

	// 0〜360 に変換
	if(now_position[2] < 0){
		now_degree = 18000 + (18000 + now_position[2]);
	}else{
		now_degree = now_position[2];
	}
	if(target_position[2] < 0){
		target_degree = 18000 + (18000 + target_position[2]);
	}else{
		target_degree = target_position[2];
	}

	if(abs(target_degree - now_degree) > 18000){
		if(target_degree-now_degree > 0){
			abs_degree_to_target = 36000 - target_degree + now_degree;
			turn_destination_coeff = -1;
		}else{
			abs_degree_to_target = 36000 - now_degree + target_degree;
			turn_destination_coeff = 1;
		}
	}else{
		if(target_degree-now_degree > 0){
			turn_destination_coeff = 1;
		}else{
			turn_destination_coeff = -1;
		}
		abs_degree_to_target = abs(target_degree-now_degree);
	}

	if(abs_degree_to_target >= SPIN_INPUT_MAX_DEGREE){
		input_duty[2] = SPIN_INPUT_MAX_VALUE;
	}else if(abs_degree_to_target < SPIN_STOP_RANGE){
		input_duty[2] = 0;
	}else{
		input_duty[2] = SPIN_DUTY_GET(abs_degree_to_target);
	}
	input_duty[2] *= turn_destination_coeff;

	distance = absolute_distance(re_now_position); //距離を計算(常に正)
	if(not_stop){
		input_max_duty_cal[0] = DUTY_MAX_VALUE;
	}else{
		if(distance >= DUTY_MAX_DISTANCE){
			input_max_duty_cal[0] = DUTY_MAX_VALUE;
		}else{
			input_max_duty_cal[0] = STRAIGHT_DUTY_GET(distance);
		}
	}
	input_max_duty_cal[1] = abs(input_duty[2])*INPUT_MAX_DUTY_COMPARE_COEFF;

	//x,y入力デューティを最大値にあわせて調整
	if(distance < MOVE_STOP_RANGE){
		input_duty[0] = 0;
		input_duty[1] = 0;
		input_max_duty_cal[0] = 0;
	}else if(abs(re_now_position[0]) >= abs(re_now_position[1])){
		for(i=0; i<2; i++){
			input_duty[i] = -(int)((double)re_now_position[i] * fabs(((double)XY_TO_INPUT_MAX_VALUE/(double)re_now_position[0])));
		}
	}else{
		for(i=0; i<2; i++){
			input_duty[i] = -(int)((double)re_now_position[i] * fabs(((double)XY_TO_INPUT_MAX_VALUE/(double)re_now_position[1])));
		}
	}

	if(input_max_duty_cal[0] > input_max_duty_cal[1]){
		input_max_duty = input_max_duty_cal[0];
	}else{
		input_max_duty = input_max_duty_cal[1];
	}
	if(input_max_duty-recent_input_max_duty > ACCELARATING_COEFF){
		input_max_duty = recent_input_max_duty + ACCELARATING_COEFF;
	}
	recent_input_max_duty = input_max_duty;
	

	auto_omni_suspension(input_duty[0],input_duty[1],input_duty[2],input_max_duty);

	return distance;
}

static
int32_t absolute_distance(int32_t position[2]){
	return (int)(sqrt(position[0]*position[0]+position[1]*position[1]));
}

static
Arm_MovingSituation_t arm_spin_to_target(Arm_MovingTarget_t target, bool reset){
	const target_to_degree[6] = {FALL_MECHA_ENC,NEAR_ROBOT_ENC,UNE_150_ENC,UNE_200_ENC,UNE_250_ENC,NEAR_UNE_ENC};
	Arm_MovingSituation_t return_situation;
	int32_t encoder_val = DD_encoder1Get_int32();
	static int degree_to_target;


	if(reset){
		DD_encoder1reset();
		return RESET_ARM_FUNC;
	}

	degree_to_target = encoder_val-target_to_degree[target]; //マイナスなら、右回転

}

static
int spin_xy_point(int32_t xy_point[2], double degree /*普通の角度180.0*/){
	int32_t recent_point[2];
	double rad;

	rad = degree * M_PI/180.0;
	recent_point[0] = xy_point[0];
	recent_point[1] = xy_point[1];

	xy_point[0] = (int)((double)recent_point[0]*cos(rad) - (double)recent_point[1]*sin(rad));
	xy_point[1] = (int)((double)recent_point[0]*sin(rad) + (double)recent_point[1]*cos(rad));

	return 0;
}

static
int odmetry_position(int32_t position[3], bool adjust_flag[3], int32_t adjust_value[3], bool reset){

	static int32_t adjust[3] = {};
	bool adjusted = false;
	int i;

#if DD_NUM_OF_SS
	position[0] = g_ss_h[I2C_ODMETRY].data[2] + (g_ss_h[I2C_ODMETRY].data[3])*256 + ( (0b01111111)&(g_ss_h[I2C_ODMETRY].data[4]) ) *65536;
    if(( ((0b10000000)&(g_ss_h[I2C_ODMETRY].data[4]))>>7 ) == 1){
      position[0] *= -1;
    }
    position[1] = g_ss_h[I2C_ODMETRY].data[5] + (g_ss_h[I2C_ODMETRY].data[6])*256 + ( (0b01111111)&(g_ss_h[I2C_ODMETRY].data[7]) ) *65536;
    if(( ((0b10000000)&(g_ss_h[I2C_ODMETRY].data[7]))>>7 ) == 1){
      position[1] *= -1;
    }
    position[2] = g_ss_h[I2C_ODMETRY].data[0] + ( (0b01111111)&(g_ss_h[I2C_ODMETRY].data[1]) ) * 256;
    if(( ((0b10000000)&(g_ss_h[I2C_ODMETRY].data[1]))>>7 ) == 1){
      position[2] *= -1;
    }
#endif

	if(reset){
		for(i=0; i<3; i++){
			position[i] = 0;
			adjust[i] = position[i];
		}
		return 0;
	}

	for(i=0; i<3; i++){
		if(adjust_flag[i]){
			adjust[i] = position[i] - adjust_value[i];
			position[i] = adjust_value[i];
			adjusted = true;
		}
	}
	for(i=0;i<3;i++){
		position[i] -= adjust[i];
	}
	if(adjusted){
		return 0;
	}

	if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
		MW_printf("\nposition[x][y][w] : [%10d][%10d][%4d.%2d]\n",position[0],position[1],position[2]/100,abs(position[2]%100));
	}

	return 0;
}

#if DD_NUM_OF_MD
static
int auto_omni_suspension(int x, int y, int w, int max_duty){
	const int cal_array[4][3] = {
		{ 100, -100, 100},
		{-100, -100, 100},
		{-100,  100, 100},
		{ 100,  100, 100},
	};
	int temp_array[4];
	int cal_duty[4];
	int in_array[3];
	int max_cal_value = 0;
	double coefficient;
	int i=0,j=0;

	in_array[0] = x;
	in_array[1] = y;
	in_array[2] = w;

	for(i=0; i<4; i++){
		temp_array[i] = 0;
		for(j=0; j<3; j++){
			temp_array[i] += in_array[j]*cal_array[i][j];
		}
	}

	for(i=0; i<4; i++){
		if(max_cal_value <= abs(temp_array[i])){
			max_cal_value = abs(temp_array[i]);
		}
	}

	coefficient = (double)max_duty/(double)max_cal_value;

	for(i=0; i<4; i++){
		cal_duty[i] = (int)( (double)temp_array[i] * coefficient );
	}

	for(i=0; i<4; i++){
		g_md_h[i].duty = abs(cal_duty[i]);
		if(cal_duty[i] < 0){
			g_md_h[i].mode = D_MMOD_BACKWARD;
		}else if(cal_duty[i] > 0){
			g_md_h[i].mode = D_MMOD_FORWARD;
		}else{
			g_md_h[i].mode = D_MMOD_BRAKE;
		}
	}

	return EXIT_SUCCESS;
}
#endif

static
int duty_adjust(int duty, int omni_num){
	const double coeff_forward[4][2] = {
		{0.8961426939,416.180},
		{0.8791354512,399.575},
		{0.8864499033,400.469},
		{0.9043946380,378.819},
	};
	const double coeff_reverse[4][2] = {
		{0.8939177467,397.066},
		{0.8748625036,336.814},
		{0.8686693640,365.415},
		{0.8929280395,337.306},
	};
	double cal_duty;

	if(duty > 0){
		cal_duty = ((double)duty * coeff_forward[omni_num][0]) + coeff_forward[omni_num][1];
	}else if(duty < 0){
		cal_duty = ((double)duty * coeff_reverse[omni_num][0]) - coeff_reverse[omni_num][1];
	}else{
		cal_duty = 0.0;
	}

	if(fabs(cal_duty) >= 9999.0){
		if(cal_duty < 0.0){
			cal_duty = -9999.0;
		}else if(cal_duty > 0.0){
			cal_duty = 9999.0;
		}
	}

	return (int)cal_duty;
}

#if DD_NUM_OF_MD
static
void all_motor_stop(void){
	int i;
	for(i=0;i<4;i++){
		g_md_h[i].duty = 0;
		g_md_h[i].mode = D_MMOD_BRAKE;
	}
}
#endif

static
void raspi_switch_ctrl(int switch_data[RASPI_SWITCH_NUM]){
	int i,j;
	int8_t bit;
	for(i=0;i<8;i++){
		bit = 0b00000001;
		for(j=0;j<8;j++){
			bit << j;
			if((raspi_control_rcv[i]&bit)==1){
				switch_data[i*8+j] = 1;
			}else{
				switch_data[i*8+j] = 0;
			}
		}
	}
}

#if DD_NUM_OF_MD
static
int manual_omni_suspension(void){
	const int cal_array[4][3] = {
		{ 1, -1, 1},
		{-1, -1, 1},
		{-1,  1, 1},
		{ 1,  1, 1},
	};
	int temp_array[4];
	int cal_duty[4];
	int in_array[3];
	int i=0,j=0;

	in_array[0] = DD_RCGetLX(g_rc_data);
	in_array[1] = -DD_RCGetLY(g_rc_data);
	in_array[2] = DD_RCGetRX(g_rc_data);

	for(i=0; i<4; i++){
		temp_array[i] = 0;
		for(j=0; j<3; j++){
			temp_array[i] += in_array[j]*cal_array[i][j];
		}
		cal_duty[i] = temp_array[i] * 50;
		cal_duty[i] = duty_adjust(cal_duty[i],i);
		g_md_h[i].duty = abs(cal_duty[i]);
		if(cal_duty[i] < 0){
			g_md_h[i].mode = D_MMOD_BACKWARD;
		}else if(cal_duty[i] > 0){
			g_md_h[i].mode = D_MMOD_FORWARD;
		}else{
			g_md_h[i].mode = D_MMOD_BRAKE;
		}
	}

	return EXIT_SUCCESS;
}
#endif

#if DD_NUM_OF_MD
static int duty_check(void){
	static unsigned int count = 0;
	static bool circle_flag = false, cross_flag = false;
	int i;

	if(!__RC_ISPRESSED_CIRCLE(g_rc_data)) circle_flag = true;
	if(!__RC_ISPRESSED_CROSS(g_rc_data)) cross_flag = true;

	if(__RC_ISPRESSED_CIRCLE(g_rc_data) && circle_flag){ 
		count++;
		circle_flag = false;
	}
	if(__RC_ISPRESSED_CROSS(g_rc_data) && cross_flag){
		count--;
		cross_flag = false;
	}
	for(i=0; i<4; i++){
		g_md_h[i].mode = D_MMOD_FORWARD;
		g_md_h[i].duty = count * 200;
		if(g_md_h[i].duty >= 9900){
			g_md_h[i].duty = 9900;
		}
	}

	return 0;
}
#endif