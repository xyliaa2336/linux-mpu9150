////////////////////////////////////////////////////////////////////////////
//
//  This file is part of linux-mpu9150
//
//  Copyright (c) 2013 Pansenti, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <errno.h>
#include <sys/time.h>

#include "./MQTT_stuff/src/MQTTAsync.h"
#include "mpu9150.h"
#include "linux_glue.h"
#include "local_defaults.h"



#define ADDRESS     "tcp://m2m.eclipse.org:1883"
#define CLIENTID    "RPi_71"
#define TOPIC       "MQTT_MPU"
#define PAYLOAD     "Hello World!"
#define QOS         1
#define TIMEOUT     10000L

#define MPU_MSG_NUM  1
#define MPU_MSG_LENGTH  26// 8(timestamp) + 16(quaternion) + 2(temperature) 

volatile MQTTAsync_token deliveredtoken;

int set_cal(int mag, char *cal_file);
void read_loop(unsigned int sample_rate);
void mpu_add_msg(mpudata_t *mpu);
void print_fused_euler_angles(mpudata_t *mpu);
void print_fused_quaternion(mpudata_t *mpu);
void print_calibrated_accel(mpudata_t *mpu);
void print_calibrated_mag(mpudata_t *mpu);
void register_sig_handler();
void sigint_handler(int sig);

int done;
 
int finished = 0;

struct timeval tv;
int msg_cnt=0;

char mpu_msg[MPU_MSG_LENGTH];

	MQTTAsync client;
	MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
	MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
	MQTTAsync_token token;

	MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;

char msg[10]="0123456789";


void connlost(void *context, char *cause)
{
	MQTTAsync client = (MQTTAsync)context;
	MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
	int rc;

	printf("\nConnection lost\n");
	printf("     cause: %s\n", cause);

	printf("Reconnecting\n");

	//change this
	conn_opts.keepAliveInterval = 700;

	conn_opts.cleansession = 1;
	if ((rc = MQTTAsync_connect(client, &conn_opts)) != MQTTASYNC_SUCCESS)
	{
		printf("Failed to start connect, return code %d\n", rc);
 		finished = 1;
	}
}


void onDisconnect(void* context, MQTTAsync_successData* response)
{
	printf("Successful disconnection\n");
	finished = 1;
}

void onSendAgain(void* context, MQTTAsync_successData* response)
{
	MQTTAsync client = (MQTTAsync)context;
	MQTTAsync_disconnectOptions opts = MQTTAsync_disconnectOptions_initializer;
	MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
	int rc;

	printf("Message with token value %d delivery confirmed\n", response->token);

	opts.onSuccess = onDisconnect;
	opts.context = client;
	if ((rc = MQTTAsync_disconnect(client, &opts)) != MQTTASYNC_SUCCESS)
	{
		printf("Failed to start sendMessage, return code %d\n", rc);
		exit(-1);	
	}
}


void onSend(void* context, MQTTAsync_successData* response)
{
	MQTTAsync client = (MQTTAsync)context;
	MQTTAsync_disconnectOptions opts = MQTTAsync_disconnectOptions_initializer;
	MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
	int rc;

	printf("Message with token value %d delivery confirmed\n", response->token);

	opts.onSuccess = onSendAgain;//onDisconnect;
	opts.context = client;
	
	if ((rc = MQTTAsync_sendMessage(client, TOPIC, &pubmsg, &opts)) != MQTTASYNC_SUCCESS)
	{
		printf("Failed to start sendMessage, return code %d\n", rc);
 		exit(-1);	
	}

	/*if ((rc = MQTTAsync_disconnect(client, &opts)) != MQTTASYNC_SUCCESS)
	{
		printf("Failed to start sendMessage, return code %d\n", rc);
		exit(-1);	
	}*/
}


void onConnectFailure(void* context, MQTTAsync_failureData* response)
{
	printf("Connect failed, rc %d\n", response ? response->code : 0);
	finished = 1;
}


void onConnect(void* context, MQTTAsync_successData* response)
{
	MQTTAsync client = (MQTTAsync)context;
	MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
	MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
	int rc;

	printf("Successful connection\n");
	
	opts.onSuccess = onSend;
	opts.context = client;

	//pubmsg.payload = PAYLOAD;
	//pubmsg.payloadlen = strlen(PAYLOAD);
	pubmsg.qos = QOS;
	pubmsg.retained = 0;
	deliveredtoken = 0;

	/*if ((rc = MQTTAsync_sendMessage(client, TOPIC, &pubmsg, &opts)) != MQTTASYNC_SUCCESS)
	{
		printf("Failed to start sendMessage, return code %d\n", rc);
 		exit(-1);	
	}*/
}

void usage(char *argv_0)
{
	printf("\nUsage: %s [options]\n", argv_0);
	printf("  -b <i2c-bus>          The I2C bus number where the IMU is. The default is 1 to use /dev/i2c-1.\n");
	printf("  -s <sample-rate>      The IMU sample rate in Hz. Range 2-50, default 10.\n");
	printf("  -y <yaw-mix-factor>   Effect of mag yaw on fused yaw data.\n");
	printf("                           0 = gyro only\n");
	printf("                           1 = mag only\n");
	printf("                           > 1 scaled mag adjustment of gyro data\n");
	printf("                           The default is 4.\n");
	printf("  -a <accelcal file>    Path to accelerometer calibration file. Default is ./accelcal.txt\n");
	printf("  -m <magcal file>      Path to mag calibration file. Default is ./magcal.txt\n");
	printf("  -v                    Verbose messages\n");
	printf("  -h                    Show this help\n");

	printf("\nExample: %s -b3 -s20 -y10\n\n", argv_0);
	
	exit(1);
}

int publish_main(void)
{
	MQTTAsync client;
	MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
	MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
	MQTTAsync_token token;
	int rc;
	MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;

	MQTTAsync_create(&client, ADDRESS, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL);

	MQTTAsync_setCallbacks(client, NULL, connlost, NULL, NULL);

	conn_opts.keepAliveInterval = 700;
	conn_opts.cleansession = 1;
	conn_opts.onSuccess = onConnect;
	conn_opts.onFailure = onConnectFailure;
	conn_opts.context = client;
	if ((rc = MQTTAsync_connect(client, &conn_opts)) != MQTTASYNC_SUCCESS)
	{
		printf("Failed to start connect, return code %d\n", rc);
		exit(-1);	
	}

	printf("Waiting for publication of %s\n"
         "on topic %s for client with ClientID: %s\n",
         PAYLOAD, TOPIC, CLIENTID);
	/*while (!finished)
		#if defined(WIN32)
			Sleep(100);
		#else
			usleep(10000L);
		#endif
	*/
sleep(3);
int i=0;
	while(i<5){
		i++;
		pubmsg.payload = msg;//PAYLOAD;
		pubmsg.payloadlen = strlen(msg);//PAYLOAD);

		msg[0]=msg[0]+1;
		if ((rc = MQTTAsync_sendMessage(client, TOPIC, &pubmsg, &opts)) != MQTTASYNC_SUCCESS)
		{
			printf("Failed to start sendMessage, return code %d\n", rc);
	 		exit(-1);	
		}

	}
	sleep(3600);
	MQTTAsync_destroy(&client);
 	return rc;
}

void MQTT_init(void)
{

	int rc=0;;

	MQTTAsync_create(&client, ADDRESS, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL);

	MQTTAsync_setCallbacks(client, NULL, connlost, NULL, NULL);

	conn_opts.keepAliveInterval = 700;
	conn_opts.cleansession = 1;
	conn_opts.onSuccess = onConnect;
	conn_opts.onFailure = onConnectFailure;
	conn_opts.context = client;
	if ((rc = MQTTAsync_connect(client, &conn_opts)) != MQTTASYNC_SUCCESS)
	{
		printf("Failed to start connect, return code %d\n", rc);
		exit(-1);	
	}

	printf("Waiting for publication of %s\n"
         "on topic %s for client with ClientID: %s\n",
         PAYLOAD, TOPIC, CLIENTID);
	/*while (!finished)
		#if defined(WIN32)
			Sleep(100);
		#else
			usleep(10000L);
		#endif
	*/
	sleep(3);
}

int publish(void* msg_p)
{
	int rc=0;

		pubmsg.payload = msg_p;//PAYLOAD;
		pubmsg.payloadlen = MPU_MSG_LENGTH;//PAYLOAD;

		if ((rc = MQTTAsync_sendMessage(client, TOPIC, &pubmsg, &opts)) != MQTTASYNC_SUCCESS)
		{
			printf("Failed to start sendMessage, return code %d\n", rc);
	 		exit(-1);	
		}

	// /sleep(3600);
	//MQTTAsync_destroy(&client);
 	return rc;
}

int main(int argc, char **argv)
{
	int opt, len;
	int i2c_bus = DEFAULT_I2C_BUS;
	int sample_rate = DEFAULT_SAMPLE_RATE_HZ;
	int yaw_mix_factor = DEFAULT_YAW_MIX_FACTOR;
	int verbose = 0;
	char *mag_cal_file = NULL;
	char *accel_cal_file = NULL;




	MQTT_init();
	
	
	while ((opt = getopt(argc, argv, "b:s:y:a:m:vh")) != -1) {
		switch (opt) {
		case 'b':
			i2c_bus = strtoul(optarg, NULL, 0);
			
			if (errno == EINVAL)
				usage(argv[0]);
			
			if (i2c_bus < MIN_I2C_BUS || i2c_bus > MAX_I2C_BUS)
				usage(argv[0]);

			break;
		
		case 's':
			sample_rate = strtoul(optarg, NULL, 0);
			
			if (errno == EINVAL)
				usage(argv[0]);
			
			if (sample_rate < MIN_SAMPLE_RATE || sample_rate > MAX_SAMPLE_RATE)
				usage(argv[0]);

			break;

		case 'y':
			yaw_mix_factor = strtoul(optarg, NULL, 0);
			
			if (errno == EINVAL)
				usage(argv[0]);
			
			if (yaw_mix_factor < 0 || yaw_mix_factor > 100)
				usage(argv[0]);

			break;

		case 'a':
			len = 1 + strlen(optarg);

			accel_cal_file = (char *)malloc(len);

			if (!accel_cal_file) {
				perror("malloc");
				exit(1);
			}

			strcpy(accel_cal_file, optarg);
			break;

		case 'm':
			len = 1 + strlen(optarg);

			mag_cal_file = (char *)malloc(len);

			if (!mag_cal_file) {
				perror("malloc");
				exit(1);
			}

			strcpy(mag_cal_file, optarg);
			break;

		case 'v':
			verbose = 1;
			break;

		case 'h':
		default:
			usage(argv[0]);
			break;
		}
	}

	register_sig_handler();

	mpu9150_set_debug(verbose);

	if (mpu9150_init(i2c_bus, sample_rate, yaw_mix_factor))
		exit(1);

	set_cal(0, accel_cal_file);
	set_cal(1, mag_cal_file);

	if (accel_cal_file)
		free(accel_cal_file);

	if (mag_cal_file)
		free(mag_cal_file);

	read_loop(sample_rate);

	mpu9150_exit();
	MQTTAsync_destroy(&client);

	return 0;
}

void read_loop(unsigned int sample_rate)
{
	unsigned long loop_delay;
	mpudata_t mpu;								/*
									typedef struct {
									short rawGyro[3];
									short rawAccel[3];
									long rawQuat[4];
									unsigned long dmpTimestamp;

									short rawMag[3];
									unsigned long magTimestamp;

									short Temp[3];
	
									short calibratedAccel[3];
									short calibratedMag[3];

									quaternion_t fusedQuat;
									vector3d_t fusedEuler;

									float lastDMPYaw;
									float lastYaw;
								} mpudata_t;*/

	memset(&mpu, 0, sizeof(mpudata_t));

	if (sample_rate == 0)
		return;

	loop_delay = (1000 / sample_rate) - 2;

	printf("\nEntering read loop (ctrl-c to exit)\n\n");

	linux_delay_ms(loop_delay);
	


	while (!done) {
		while(msg_cnt<MPU_MSG_NUM && !done){
			if (mpu9150_read(&mpu) == 0) {
				 mpu_add_msg(&mpu);

				// print_fused_euler_angles(&mpu);
				 print_fused_quaternions(&mpu);
				// print_calibrated_accel(&mpu);
				// print_calibrated_mag(&mpu);
			}
			linux_delay_ms(loop_delay);
			
		}
		msg_cnt=0;
		publish(mpu_msg);
		//publish(msg);
		
	}

	printf("\n\n");
}


void mpu_add_msg(mpudata_t *mpu)
{
	short temperature;
	float ft;

	//time stamp
	gettimeofday(&tv, NULL); //get time!!
	memcpy (&mpu_msg[0], &tv, 8); //4byte for each sec and usec

	//mpu_msg[0]=12;
	//mpu_msg[1]=16;

	//quaternions
	//mpu_msg[2]=mpu->fusedQuat[QUAT_W];
	//mpu_msg[3]=mpu->fusedQuat[QUAT_X];
	//mpu_msg[4]=mpu->fusedQuat[QUAT_Y];
	//mpu_msg[5]=mpu->fusedQuat[QUAT_Z];

	memcpy (&mpu_msg[8], &(mpu->fusedQuat[QUAT_W]), 4); 
	memcpy (&mpu_msg[12], &(mpu->fusedQuat[QUAT_X]), 4); 
	memcpy (&mpu_msg[16], &(mpu->fusedQuat[QUAT_Y]), 4); 
	memcpy (&mpu_msg[20], &(mpu->fusedQuat[QUAT_Z]), 4); 

	//temperature
	mpu_get_temperature(&temperature,NULL);

	memcpy (&mpu_msg[24], &temperature, 2); 
	ft=temperature/340.0f+35.0f;
	printf("%d\n", temperature);
	msg_cnt++;
		
}



void print_fused_euler_angles(mpudata_t *mpu)
{
	printf("\rX: %0.0f Y: %0.0f Z: %0.0f        ",
			mpu->fusedEuler[VEC3_X] * RAD_TO_DEGREE, 
			mpu->fusedEuler[VEC3_Y] * RAD_TO_DEGREE, 
			mpu->fusedEuler[VEC3_Z] * RAD_TO_DEGREE);

	fflush(stdout);
}

void print_fused_quaternions(mpudata_t *mpu)
{
	printf("\rW: %0.2f X: %0.2f Y: %0.2f Z: %0.2f        ",
			mpu->fusedQuat[QUAT_W],
			mpu->fusedQuat[QUAT_X],
			mpu->fusedQuat[QUAT_Y],
			mpu->fusedQuat[QUAT_Z]);
	//printf("%d\n",sizeof(mpu->fusedQuat[QUAT_W]) );
	fflush(stdout);
}

void print_calibrated_accel(mpudata_t *mpu)
{
	printf("\rX: %05d Y: %05d Z: %05d        ",
			mpu->calibratedAccel[VEC3_X], 
			mpu->calibratedAccel[VEC3_Y], 
			mpu->calibratedAccel[VEC3_Z]);

	fflush(stdout);
}

void print_calibrated_mag(mpudata_t *mpu)
{
	printf("\rX: %03d Y: %03d Z: %03d        ",
			mpu->calibratedMag[VEC3_X], 
			mpu->calibratedMag[VEC3_Y], 
			mpu->calibratedMag[VEC3_Z]);

	fflush(stdout);
}

int set_cal(int mag, char *cal_file)
{
	int i;
	FILE *f;
	char buff[32];
	long val[6];
	caldata_t cal;

	if (cal_file) {
		f = fopen(cal_file, "r");
		
		if (!f) {
			perror("open(<cal-file>)");
			return -1;
		}
	}
	else {
		if (mag) {
			f = fopen("./magcal.txt", "r");
		
			if (!f) {
				printf("Default magcal.txt not found\n");
				return 0;
			}
		}
		else {
			f = fopen("./accelcal.txt", "r");
		
			if (!f) {
				printf("Default accelcal.txt not found\n");
				return 0;
			}
		}		
	}

	memset(buff, 0, sizeof(buff));
	
	for (i = 0; i < 6; i++) {
		if (!fgets(buff, 20, f)) {
			printf("Not enough lines in calibration file\n");
			break;
		}

		val[i] = atoi(buff);

		if (val[i] == 0) {
			printf("Invalid cal value: %s\n", buff);
			break;
		}
	}

	fclose(f);

	if (i != 6) 
		return -1;

	cal.offset[0] = (short)((val[0] + val[1]) / 2);
	cal.offset[1] = (short)((val[2] + val[3]) / 2);
	cal.offset[2] = (short)((val[4] + val[5]) / 2);

	cal.range[0] = (short)(val[1] - cal.offset[0]);
	cal.range[1] = (short)(val[3] - cal.offset[1]);
	cal.range[2] = (short)(val[5] - cal.offset[2]);
	
	if (mag) 
		mpu9150_set_mag_cal(&cal);
	else 
		mpu9150_set_accel_cal(&cal);

	return 0;
}

void register_sig_handler()
{
	struct sigaction sia;

	bzero(&sia, sizeof sia);
	sia.sa_handler = sigint_handler;

	if (sigaction(SIGINT, &sia, NULL) < 0) {
		perror("sigaction(SIGINT)");
		exit(1);
	} 
}

void sigint_handler(int sig)
{
	done = 1;
}
