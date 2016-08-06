#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include <signal.h>
#include <curses.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <stdlib.h>

#define frequency 25000000.0
#define LED0 0x6			//LED0 start register
#define LED0_ON_L 0x6		//LED0 output and brightness control byte 0
#define LED0_ON_H 0x7		//LED0 output and brightness control byte 1
#define LED0_OFF_L 0x8		//LED0 output and brightness control byte 2
#define LED0_OFF_H 0x9		//LED0 output and brightness control byte 3
#define LED_MULTIPLYER 4	// For the other 15 channels
#define D 1100.0
#define P 11.0
#define I 0.015
#define P_yaw 1.0
#define A 0.00001

int execute,pwm;
float x_roll_final = 0.0;
float y_pitch_final = 0.0;

//init the pwm board
void init_pwm(int pwm)
{
    float freq =400.0*.95;
    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    uint8_t prescale = floor(prescaleval+0.5);
    int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
    int sleep	= settings | 0x10;
    int wake 	= settings & 0xef;
    int restart = wake | 0x80;
    wiringPiI2CWriteReg8(pwm, 0x00, sleep);
    wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
    wiringPiI2CWriteReg8(pwm, 0x00, wake);
    wiringPiI2CWriteReg8(pwm, 0x00, restart|0x20);
}

//turn on the motor
void init_motor(int pwm, uint8_t channel)
{
	int on_value=0;

	int time_on_us=900;
	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

  time_on_us=1200;
  off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1000;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);
}

//set the pwm value of the motor
void set_PWM(int pwm, uint8_t channel, float time_on_us)
{
	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
}

//when cntrl+c pressed, kill motors
void trap(int signal)
{

  // turn off  all 4 motors!!
  for(int i=0; i<=100; i++) {
      set_PWM(pwm,0,1000);
      set_PWM(pwm,1,1000);
      set_PWM(pwm,2,1000);
      set_PWM(pwm,3,1000);
  }
  execute=0;
  printf("ending program\n\r");
}

struct data
{
	int keypress;
	float pitch;
	float roll;
	float yaw;
	float thrust;
    int sequence_num;
};


int main (int argc, char *argv[])
{
    //open file for writing to if needed
	FILE *fr;
  FILE *fp;
	fr=fopen("data_roll.txt","a+");
  fp=fopen("data_pitch.txt","a+");
	execute=1;

	signal(SIGINT, &trap);
  struct timespec te;
  int address;
  int mag,imu;
  // int data;
  int display;

  wiringPiSetup () ;

  //setup for pwm
  pwm=wiringPiI2CSetup (0x40);  //connect pwm board to imu

  // dont forget to add initailization for imu here!
  imu = wiringPiI2CSetup(0x6B);

  wiringPiI2CWriteReg8(imu, 0x10, 0xC0); // gyro
  wiringPiI2CWriteReg8(imu, 0x20, 0xC0); // accel
  wiringPiI2CWriteReg8(imu, 0x10, 0xC8); //turn on and set gyro output
  wiringPiI2CWriteReg8(imu, 0x20, 0xD8); //turn on and set accel output

  printf("0x%x\n", wiringPiI2CReadReg8(imu,0x0F));

  //code
  float sum = 0.0;
  float g_scale = 8/32768.0;

  float pitch_angle_gyro=0.0;
  float roll_angle_gyro=0.0;
  float yaw_angle_gyro=0.0;
  long time_curr;
  long time_prev=0;
  long time_end=0;
  struct timeval tv;
  gettimeofday(&tv,NULL);
  long prev_seq_time=tv.tv_sec*1000LL+tv.tv_usec/1000;
  long curr_abs_time=tv.tv_sec*1000LL+tv.tv_usec/1000;
  float delta_x_rotate=0.0;
  float delta_y_rotate=0.0;
  float delta_z_rotate=0.0;
  int neutral_power = 1400;
  float mpw0=0.0, mpw1 = 0.0;


  // D controller

  float mdw0=0.0, mdw1 = 0.0;

  //PD controller
  float mpdw0=0.0, mpdw1 = 0.0;

  //PID controller
  float roll_prev=0.0, roll_curr=0.0, roll_vel = 0.0;
  float pitch_prev=0.0, pitch_curr=0.0, pitch_vel = 0.0;
  float yaw_angle_prev=0.0, yaw_angle_curr=0.0;
  float mpidw0=0.0, mpidw1=0.0, mpidw2=0.0, mpidw3=0.0; // 0 & 1 are roll; 2 & 3 are pitch
  float power0=0.0, power1=0.0, power2=0.0, power3=0.0; // 0 & 1 are roll; 2 & 3 are pitch
  float i_error=0.0, i_term=0.0; // roll
  float pitch_i_error = 0.0, pitch_i_term=0.0; // pitch
  float desired_roll = 0.0;
  float roll_error=0.0;
  float desired_pitch = 0.0;
  float pitch_error=0.0;
  float desired_yaw = 0.0;
  float yaw_error=0.0;

  int x_rate, y_rate, z_rate, x_linear, y_linear, z_linear;

  //Calibration Variables
  int calibration_bool = 0;
  int calibration_counter = 0;
  float total_roll_accel = 0.0;
  float total_pitch_accel = 0.0;
  float total_pitch_gyro = 0.0;
  float total_roll_gyro = 0.0;
  float total_yaw_rate = 0.0;

  float cal_roll_accel = 0.0;
  float cal_pitch_accel = 0.0;
  float cal_pitch_gyro = 0.0;
  float cal_roll_gyro = 0.0;
  float cal_yaw_rate = 0.0;

  int start = 0;

  //shared memory init
  int segment_id;
  data* shared_memory;
  struct shmid_ds shmbuffer;
  int segment_size;
  const int shared_segment_size = 0x6400;
  int smhkey=33222;
  float thrust_val = 0.0;
  int seq_prev;
  // int ch = 0;

  /* Allocate a shared memory segment.  */
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666);
  /* Attach the shared memory segment.  */
  shared_memory = (data*) shmat (segment_id, 0, 0);
  printf ("shared memory attached at address %p\n", shared_memory);
  /* Determine the segment's size. */
  shmctl (segment_id, IPC_STAT, &shmbuffer);
  segment_size  =               shmbuffer.shm_segsz;
  printf ("segment size: %d\n", segment_size);



  if(imu==-1||pwm==-1)
  {
    printf("cant connect to I2C device %d %d\n",imu,pwm);
    return -1;
  }
  else
  {
    //start the pwm
    init_pwm(pwm);

    //init motors
    init_motor(pwm,0);
    init_motor(pwm,1);
    init_motor(pwm,2);
    init_motor(pwm,3);

    while(execute==1)
    {
      //get current time in nanoseconds
      timespec_get(&te,TIME_UTC);
      time_curr=te.tv_nsec;

      x_rate=wiringPiI2CReadReg16(imu, 0x18);
      y_rate=wiringPiI2CReadReg16(imu, 0x1A);
      z_rate=wiringPiI2CReadReg16(imu, 0x1C);

      x_linear = wiringPiI2CReadReg16(imu, 0x28);
      y_linear = wiringPiI2CReadReg16(imu, 0x2A);
      z_linear = wiringPiI2CReadReg16(imu, 0x2C);

      //convert to 2's complement
      if(x_rate>0x8000)
      {
        x_rate=x_rate ^ 0xffff;
        x_rate=-x_rate-1;
      }
      if(y_rate>0x8000)
      {
        y_rate=y_rate ^ 0xffff;
        y_rate=-y_rate-1;
      }
      if(z_rate>0x8000)
      {
        z_rate=z_rate ^ 0xffff;
        z_rate=-z_rate-1;
      }
      if(x_linear>0x8000)
      {
        x_linear=x_linear ^ 0xffff;
        x_linear=-x_linear-1;
      }
      if(y_linear>0x8000)
      {
        y_linear=y_linear ^ 0xffff;
        y_linear=-y_linear-1;
      }
      if(z_linear>0x8000)
      {
        z_linear=z_linear ^ 0xffff;
        z_linear=-z_linear-1;
      }

      //calibration value
      // z_rate += 2;

      //convert to dps
      float x_angle_dps=x_rate*(500/32768.0);
      float y_angle_dps=y_rate*(500/32768.0);
      float z_angle_dps=z_rate*(500/32768.0);
      float x_linear_g = x_linear * g_scale;
      float y_linear_g = y_linear * g_scale;
      float z_linear_g = z_linear * g_scale;

      float x_roll = atan2(y_linear_g, z_linear_g);
      float y_pitch = atan2(x_linear_g, z_linear_g);
      x_roll = -x_roll * 180/M_PI;
      y_pitch = y_pitch * 180/M_PI;

      float x_roll_accel = x_roll;
      float y_pitch_accel = y_pitch;

      //compute time since last execution
      float diff=time_curr-time_prev;
      //check for rollover
      if(diff<=0)
      {
          diff+=1000000000;
      }
      //convert to seconds
      diff=diff/1000000000;

      // PID controller (roll & pitch)
      data s=*shared_memory;
      desired_roll = s.roll;
      desired_pitch = s.pitch;
      thrust_val = s.thrust;
      desired_yaw = s.yaw*180.0;

      float yaw_limit = 90.0;
      if(desired_yaw > yaw_limit) {
        desired_yaw = yaw_limit;
      }

      if(desired_yaw < -yaw_limit) {
        desired_yaw = -yaw_limit;
      }

      // printf("%f, %f\n", z_angle_dps, desired_yaw);


      if(thrust_val > 1580.0){
        thrust_val = 1580.0;
      }

      //Calibration
      if(s.keypress == 35) {
        calibration_bool = 1;
        total_roll_accel =0.0;
        total_pitch_accel =0.0;
        total_pitch_gyro =0.0;
        total_roll_gyro =0.0;
        total_yaw_rate= 0.0;
        calibration_counter = 0;
      }
      if(calibration_bool == 1){
        calibration_counter++;
        //+= raw values
        //roll_angle_gyro, x_roll_accel, x_roll_final, pitch_angle_gyro, y_pitch_accel, y_pitch_final);
        total_roll_accel += x_roll_accel;
        total_pitch_accel += y_pitch_accel;
        total_pitch_gyro += y_angle_dps;
        total_roll_gyro += x_angle_dps;
        total_yaw_rate += z_angle_dps;
      }
      if(calibration_counter>99){
        calibration_bool = 0;
        calibration_counter = 0;
        cal_roll_accel = total_roll_accel/100. ;
        cal_pitch_accel = total_pitch_accel/100.;
        cal_pitch_gyro = total_pitch_gyro/100.;
        cal_roll_gyro = total_roll_gyro/100.;
        cal_yaw_rate = total_yaw_rate/100.;
        x_roll_final = 0;
        y_pitch_final = 0;

        printf("%f %f %f %f %f \n", cal_roll_accel, cal_pitch_accel, cal_pitch_gyro, cal_roll_gyro, cal_yaw_rate);
      }
    //   if (display%100==0){
    //   printf("%f %f\n", x_roll_final, y_pitch_final);
    //   //printf("%f %f %f %f %f \n", cal_roll_accel, cal_pitch_accel, cal_pitch_gyro, cal_roll_gyro, cal_yaw_rate);

    // }

      x_angle_dps = x_angle_dps - cal_roll_gyro;
      y_angle_dps = y_angle_dps - cal_pitch_gyro;
      z_angle_dps = z_angle_dps - cal_yaw_rate;

      //compute amount of rotation since last execution
      delta_x_rotate=x_angle_dps*diff;
      delta_y_rotate=y_angle_dps*diff;
      delta_z_rotate=z_angle_dps*diff;

      //compute yaw
      // yaw_angle_gyro += delta_z_rotate;
      // pitch_angle_gyro += delta_y_rotate;
      // roll_angle_gyro += delta_x_rotate;

      // Complimentary filter
      // float A = 0.001;
      x_roll_final = (x_roll_accel- cal_roll_accel)*A + (1.0-A)*(delta_x_rotate + x_roll_final);
      y_pitch_final = (y_pitch_accel - cal_pitch_accel)*A + (1.0-A)*(delta_y_rotate + y_pitch_final);

      //fprintf(f, "%f, %f, %f, %f, %f, %f\n\r", roll_angle_gyro, x_roll_accel, x_roll_final, pitch_angle_gyro, y_pitch_accel, y_pitch_final);

      // if(display%200==0) {
      //   printf("%f\n", fabs(x_roll_final));
      // }
      if (fabs(x_roll_final) > 30.0 || fabs(y_pitch_final) > 30.0 || fabs(z_angle_dps)>180.0) {
        for(int i=0; i<=100; i++) {
          set_PWM(pwm,0,1000);
          set_PWM(pwm,1,1000);
          set_PWM(pwm,2,1000);
          set_PWM(pwm,3,1000);
        }



        execute=0;
        printf("ending program for angle exceed\n\r");
        return 0;
      }

      // // P Controller
      // roll_error = x_roll_final - desired_roll;
      // mpw0 = neutral_power - roll_error*P;
      // mpw1 = neutral_power + roll_error*P;
      //
      // // D controller
      // roll_curr = x_roll_final;
      // roll_vel = roll_curr - roll_prev;
      // mdw0 = 1350 - roll_vel*D;
      // mdw1 = 1350 + roll_vel*D;
      //
      // // PD controller
      // mpdw0 = 1400 - roll_vel*D - roll_error*P;
      // mpdw1 = 1400 + roll_vel*D + roll_error*P;
      // if (mpdw0 >= 1500) { mpdw0 = 1500; }
      // if (mpdw1 >= 1500) { mpdw1 = 1500; }

      // // PID controller (roll & pitch)
      // data s=*shared_memory;
      // desired_roll = s.roll;
      // desired_pitch = s.pitch;
      // thrust_val = s.thrust;
      // desired_yaw = s.yaw;



      if(s.keypress == 32) {
        // turn off motors
        for(int i=0; i<=100; i++) {
            set_PWM(pwm,0,1000);
            set_PWM(pwm,1,1000);
            set_PWM(pwm,2,1000);
            set_PWM(pwm,3,1000);
        }
        execute=0;
        printf("Spacebar pressed, ending program...\n\r");
        return 0;
        // exit program
      }

      // printf("%f %f\n", x_roll_final, roll_error);
      roll_error = x_roll_final - desired_roll;
      roll_curr = x_roll_final;
      roll_vel = roll_curr - roll_prev;
      i_term += roll_error*I;

      pitch_error = y_pitch_final - desired_pitch;
      pitch_curr = y_pitch_final;
      pitch_vel = pitch_curr - pitch_prev;
      pitch_i_term += pitch_error*I;
      if (display%100==0){
      // printf("roll: %f %f\n", x_roll_final, desired_roll);
      // printf("pitch: %f %f\n", y_pitch_final, desired_pitch);
    }

      // P controller (yaw rate)


      // desired_yaw = 0.0;

      if (i_term > 50.0) { i_term = 50.0; }
      if (i_term < -50.0) { i_term = -50.0; }

      if (pitch_i_term > 50.0) { pitch_i_term = 50.0; }
      if (pitch_i_term < -50.0) { pitch_i_term = -50.0; }

      mpidw0 = thrust_val - roll_vel*D - roll_error*P - i_term - (z_angle_dps - desired_yaw) * P_yaw;
      mpidw1 = thrust_val + roll_vel*D + roll_error*P + i_term - (z_angle_dps - desired_yaw) * P_yaw;

      mpidw2 = thrust_val - pitch_vel*D - pitch_error*P - pitch_i_term + (z_angle_dps - desired_yaw) * P_yaw;
      mpidw3 = thrust_val + pitch_vel*D + pitch_error*P + pitch_i_term + (z_angle_dps - desired_yaw) * P_yaw;

      if (mpidw0 >= 1700) { mpidw0 = 1700; }
      if (mpidw1 >= 1700) { mpidw1 = 1700; }

      if (mpidw0 <= 1000) { mpidw0 = 1000; }
      if (mpidw1 <= 1000) { mpidw1 = 1000; }

      if (mpidw2 >= 1700) { mpidw2 = 1700; }
      if (mpidw3 >= 1700) { mpidw3 = 1700; }

      if (mpidw2 <= 1000) { mpidw2 = 1000; }
      if (mpidw3 <= 1000) { mpidw3 = 1000; }

      // if (mpw0 >= 1300) { mpw0 = 1300; }
      // if (mpw1 >= 1300) { mpw1 = 1300; }
      //
      // if (mdw0 >= 1450) { mdw0 = 1450; }
      // if (mdw1 >= 1450) { mdw1 = 1450; }



      // fprintf(f, "%f %f\n\r", x_roll_final, desired_roll);
      // fprintf(fp, "%f %f\n\r", y_pitch_final, desired_pitch);
      // fprintf(f, "%f %f %f %f %f %f\n\r", roll_angle_gyro, x_roll_accel, x_roll_final, pitch_angle_gyro, y_pitch_accel, y_pitch_final);

      if(s.keypress == 33) {
        start=0;
      }

      if(s.keypress == 34) {
        start=1;
      }

    //   if(s.keypress == '3') {
    //     calibration_bool = 1;
    //     total_roll_accel =0.0;
    //     total_pitch_accel =0.0;
    //     total_pitch_gyro =0.0;
    //     total_roll_gyro =0.0;
    //     total_yaw_rate= 0.0;
    //     calibration_counter = 0;
    //   }
    //   if(calibration_bool == 1){
    //     calibration_counter++;
    //     //+= raw values
    //     //roll_angle_gyro, x_roll_accel, x_roll_final, pitch_angle_gyro, y_pitch_accel, y_pitch_final);
    //     total_roll_accel += x_roll_accel;
    //     total_pitch_accel += y_pitch_accel;
    //     total_pitch_gyro += y_angle_dps;
    //     total_roll_gyro += x_angle_dps;
    //     total_yaw_rate += z_angle_dps;
    //   }
    //   if(calibration_counter>99){
    //     calibration_bool = 0;
    //     calibration_counter = 0;
    //     cal_roll_accel = total_roll_accel/100. ;
    //     cal_pitch_accel = total_pitch_accel/100.;
    //     cal_pitch_gyro = total_pitch_gyro/100.;
    //     cal_roll_gyro = total_roll_gyro/100.;
    //     cal_yaw_rate = total_yaw_rate/100.;

    //     printf("%f %f %f %f %f \n", cal_roll_accel, cal_pitch_accel, cal_pitch_gyro, cal_roll_gyro, cal_yaw_rate);
    //   }
    //   if (display%100==0){
    //   printf("%f %f\n", x_roll_final, y_pitch_final);
    //   //printf("%f %f %f %f %f \n", cal_roll_accel, cal_pitch_accel, cal_pitch_gyro, cal_roll_gyro, cal_yaw_rate);

    // }
      if(execute==1 && start ==1)//if cntrl+c has not been pressed
      {

        //set motors
        set_PWM(pwm,0,mpidw0);
        set_PWM(pwm,1,mpidw1);
        set_PWM(pwm,2,mpidw2);
        set_PWM(pwm,3,mpidw3);
      }
      else
      {
        // turn off motors
        // for(int i=0; i<=100; i++) {
            set_PWM(pwm,0,1000);
            set_PWM(pwm,1,1000);
            set_PWM(pwm,2,1000);
            set_PWM(pwm,3,1000);
        // }
        // execute=0;
        // printf("ending program\n\r");
        // return 0;
        // exit program
      }

      display++;
      timespec_get(&te,TIME_UTC);
      time_end=te.tv_nsec;



      if (s.sequence_num != seq_prev) {
        gettimeofday(&tv,NULL);
        prev_seq_time=tv.tv_sec*1000LL+tv.tv_usec/1000;

      }
      gettimeofday(&tv,NULL);
      curr_abs_time=tv.tv_sec*1000LL+tv.tv_usec/1000;

      if(curr_abs_time - prev_seq_time > 1000) {
        // turn off motors
        for(int i=0; i<=100; i++) {
            set_PWM(pwm,0,1000);
            set_PWM(pwm,1,1000);
            set_PWM(pwm,2,1000);
            set_PWM(pwm,3,1000);
        }
        execute=0;
        printf("ending program for time exceed in con\n\r");
        return 0;
        // exit program
      }

      if (time_end - time_curr > (long) (10000000)) {
        // turn off motors
        for(int i=0; i<=100; i++) {
            set_PWM(pwm,0,1000);
            set_PWM(pwm,1,1000);
            set_PWM(pwm,2,1000);
            set_PWM(pwm,3,1000);
        }
        execute=0;
        printf("ending program for time exceed\n\r");
        return 0;
        // exit program
      }
      time_prev = time_curr;
      roll_prev = roll_curr;
      pitch_prev = pitch_curr;
      seq_prev = s.sequence_num;


    }

  }

  return 0;
}
