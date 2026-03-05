#include "mit_protocol.h"
#include "pid_control.h"
#include "common.h"
#include "can_thread.h"
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include "math_ops.h"
#include "math.h"
// #include <linux/can.h>

#define MOTOR_ID  1

#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U /* error message frame */

typedef struct
{
    // unsigned int can_id;
    // unsigned int can_dlc;
    // unsigned char data[CAN_MAX_DLC];
	canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	__u8    can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
	__u8    __pad;   /* padding */
	__u8    __res0;  /* reserved / padding */
	__u8    __res1;  /* reserved / padding */
	__u8    data[CAN_MAX_DLEN] __attribute__((aligned(8)));
}can_frame  __attribute__((__aligned__(1)));

typedef struct 
{
    float max_vel;
    float max_pos;
    float max_torque;
}mit_param;

typedef struct 
{
    joint_control control;
    joint_state state;
    pid_control pid;
    mHandle can;
    mit_param param;
    can_frame can_tx;
    int motor_enable;
    pthread_spinlock_t lock;
}motor_mit;

motor_mit g_motor[1];

int motor_test_can_call(void *arg, void *arg2, char *buf, int len)
{
    can_frame *frame = (can_frame *)buf;
    motor_mit *tmp = arg;

    int p_int, v_int, i_int;
    float p, v, t;

    motor_mit *motor = NULL;

    if (frame->can_id == (MOTOR_ID | 0x10))
    {
        motor = &tmp[0];
    }else{
        return 0;
    }

    p_int = (frame->data[1] << 8) | frame->data[2];
    v_int = (frame->data[3] << 4) | (frame->data[4] >> 4);
    i_int = ((frame->data[4] & 0xF) << 8) | frame->data[5];
    /// convert uints to floats ///
    p = uint_to_float(p_int, -motor->param.max_pos, motor->param.max_pos, 16);
    v = uint_to_float(v_int, -motor->param.max_vel, motor->param.max_vel, 12);
    t = uint_to_float(i_int, -motor->param.max_torque, motor->param.max_torque, 12);

    motor->state.p = p;
    motor->state.v = v;
    motor->state.t = t;

    return 0;
}

void *commu_thread(void *arg)
{
    int enable_last[1] = {0};
    motor_mit *tmp = arg;

    while (1)
    {
        delay_ms(5);

        for (size_t i = 0; i < 1; i++)
        {
            motor_mit *motor = &tmp[i];

            if(!enable_last[i] && motor->motor_enable)
            {
                EnterMotorMode(motor->can_tx.data);
            }
            else if(enable_last[i] && !motor->motor_enable)
            {
                ExitMotorMode(motor->can_tx.data);
            }
            else
            {
                pack_cmd(motor->can_tx.data, &motor->control, 
                    motor->param.max_pos, -motor->param.max_pos, 
                    motor->param.max_vel, -motor->param.max_vel, 
                    motor->param.max_torque, -motor->param.max_torque);
            }
            
            can_commu_send(motor->can, (char *)&motor->can_tx, sizeof(can_frame));
            
            enable_last[i] = motor->motor_enable;
            usleep(1000*1);
            
            enable_last[i] = motor->motor_enable;
            usleep(1000*1);
        }
    }

    return NULL;
}

int motor_init()
{
    memset(g_motor, 0, sizeof(g_motor));

    g_motor[0].param.max_pos = 12.5;
    g_motor[0].param.max_vel = 45;
    g_motor[0].param.max_torque = 40;  // 根据测试电机修改 
    g_motor[0].motor_enable = 0;
    g_motor[0].control.t_ff = 0.0;

    g_motor[0].can = can_commu_init("can0", "can0", 0, 0xffff, motor_test_can_call, (void *)g_motor, NULL, 0);
    if (!g_motor[0].can)
        return -1;

    pthread_spin_init(&g_motor[0].lock, 0);
    g_motor[0].can_tx.can_dlc = 8;
    g_motor[0].can_tx.can_id = 1;

    pthread_t thread;
    if (-1 == pthread_create(&thread, NULL, commu_thread, (void *)g_motor))
    {
        printf("commu thread create failed.\n");
        return -1;
    }  

    return 0;
}

int motor_enable(){
    g_motor[0].motor_enable = 1;
    g_motor[0].control.t_ff = 3.0;

    return 0;
}

int motor_disable(){
    g_motor[0].motor_enable = 0;
    g_motor[0].control.t_ff = 0.0;

    return 0;
}
