#include "mit_protocol.h"
#include "pid_control.h"
#include "common.h"
#include "torque_sensor.h"
#include "can_thread.h"
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include "math_ops.h"
#include "finsh.h"
#include "math.h"
// #include <linux/can.h>

#define MOTOR_1_ID 1
#define MOTOR_2_ID 2

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
    int protocol;
}motor_mit;

#define MOTOR_NUM   2
motor_mit g_motor[MOTOR_NUM];

int vsec_pack_current(uint8_t *data, float current)
{
    if (fabs(current)  >= 420)
    {
        printf("error current:%.3f\n", current);
        return -1;
    }
    

    int temp = current * 1000;

    uint8_t *p = (uint8_t *)&temp;

    data[3] = p[0];
    data[2] = p[1];
    data[1] = p[2];
    data[0] = p[3];


    return 0;
}

int motor_enable(motor_mit *motor, int enable)
{
    // pthread_spin_lock(&motor->lock);
    motor->motor_enable = enable;
    // pthread_spin_unlock(&motor->lock);
 
    return 0;
}

int set_motor_tor(motor_mit *motor, float tor)
{
    // pthread_spin_lock(&motor->lock);
    motor->control.t_ff = tor;
    // pthread_spin_unlock(&motor->lock);

    return 0;
}

int set_vesc_tor(motor_mit *motor, float tor)
{
    // pthread_spin_lock(&motor->lock);
    motor->control.t_ff = tor;
    // pthread_spin_unlock(&motor->lock);

    return 0;
}

int set_motor_vel(motor_mit *motor, float vel)
{
    // pthread_spin_lock(&motor->lock);
    motor->control.v_des = vel;
    // pthread_spin_unlock(&motor->lock);

    return 0;
}

int motor_test_can_call(void *arg, void *arg2, char *buf, int len)
{
    // return 0;
    // printf("can recv\n");

    can_frame *frame = (can_frame *)buf;
    motor_mit *tmp = arg;
    int p_int, v_int, i_int;
    float p, v, t;
    motor_mit *motor = NULL;

    // printf("can id:%d, dlc:%d\n", frame->can_id, frame->can_dlc);

    if (frame->data[0] == MOTOR_1_ID)
    {
        motor = &tmp[0];
    }
    else if((frame->data[0] & 0xf) == (MOTOR_2_ID&0xf))
    {
       motor = &tmp[1];
    }
    else
    {
        printf("%c", frame->data[0]);
        return 0;
    }

    // printf("motor msg\n");
    p_int = (frame->data[1] << 8) | frame->data[2];
    v_int = (frame->data[3] << 4) | (frame->data[4] >> 4);
    i_int = ((frame->data[4] & 0xF) << 8) | frame->data[5];
    /// convert uints to floats ///
    p = uint_to_float(p_int, -motor->param.max_pos, motor->param.max_pos, 16);
    v = uint_to_float(v_int, -motor->param.max_vel, motor->param.max_vel, 12);
    t = uint_to_float(i_int, -motor->param.max_torque, motor->param.max_torque, 12);

// pthread_spin_lock(&motor->lock);
    motor->state.p = p;
    motor->state.v = v;
    motor->state.t = t;

// pthread_spin_unlock(&motor->lock);

    return 0;

}

void *commu_thread(void *arg)
{
    int enable_last[MOTOR_NUM] = {0};
    motor_mit *tmp = arg;

    while (1)
    {
        
        //usleep(1000*10);
        delay_ms(10);

        // motor_mit *motor = &tmp[0];

        // pthread_spin_lock(&motor->lock);

        // vsec_pack_current(motor->can_tx.data, motor->control.t_ff);

        // can_commu_send(motor->can, (char *)&motor->can_tx, sizeof(can_frame));

        // pthread_spin_unlock(&motor->lock);

        // continue;


        for (size_t i = 0; i < MOTOR_NUM; i++)
        {
            motor_mit *motor = &tmp[i];

            if (motor->protocol == 0)
            {
                // pthread_spin_lock(&motor->lock);

                if(!enable_last[i] && motor->motor_enable)
                {
                    //enter;
                    printf("enable\n");
                    EnterMotorMode(motor->can_tx.data);
                }
                else if(enable_last[i] && !motor->motor_enable)
                {
                    //exit
                    printf("disenable\n");
                    ExitMotorMode(motor->can_tx.data);
                }
                else
                {
                    pack_cmd_new(motor->can_tx.data, &motor->control, 
                    motor->param.max_pos, -motor->param.max_pos, 
                    motor->param.max_vel, -motor->param.max_vel, 
                    motor->param.max_torque, -motor->param.max_torque);
                }
                
                can_commu_send(motor->can, (char *)&motor->can_tx, sizeof(can_frame));
                
                // pthread_spin_unlock(&motor->lock);
                
                enable_last[i] = motor->motor_enable;
                usleep(1000*1);
            }
            else if (motor->protocol == 1)
            {
                // pthread_spin_lock(&motor->lock);

                if(!enable_last[i] && motor->motor_enable)
                {
                    //enter;
                    EnterMotorMode(motor->can_tx.data);
                }
                else if(enable_last[i] && !motor->motor_enable)
                {
                    //exit
                    ExitMotorMode(motor->can_tx.data);
                }
                else
                {
                    pack_cmd_vel(motor->can_tx.data, &motor->control, 
                    motor->param.max_vel, -motor->param.max_vel);

                }
                
                can_commu_send(motor->can, (char *)&motor->can_tx, sizeof(can_frame));
                
                // pthread_spin_unlock(&motor->lock);
                
                enable_last[i] = motor->motor_enable;
                usleep(1000*1);
            }
            else if (motor->protocol == 2)
            {
                // pthread_spin_lock(&motor->lock);

                if(!enable_last[i] && motor->motor_enable)
                {
                    //enter;
                    EnterMotorMode(motor->can_tx.data);
                }
                else if(enable_last[i] && !motor->motor_enable)
                {
                    //exitv
                    ExitMotorMode(motor->can_tx.data);
                }
                else
                {
                    if(motor->motor_enable){
                        float KT = 0.07f;
                        float GR = (7056.f/361.f);

                        g_motor[1].pid.input = motor->state.v;
                        pid_caculate(&g_motor[1].pid);
                        // printf("%.2f, %.2f, %.2f, %.2f\n", g_motor[1].pid.error, g_motor[1].pid.output, g_motor[1].pid.input, g_motor[1].pid.des);
                        // printf("%.2f\n", g_motor[1].pid.output);

                        motor->control.t_ff = g_motor[1].pid.output * GR * KT;
                    }else{
                        motor->control.t_ff = 0;

                        // g_motor[1].pid.input = motor->state.v;
                        // pid_caculate(&g_motor[1].pid);
                        // printf("%.2f\n", g_motor[1].pid.output);
                    }

                    pack_cmd_new(motor->can_tx.data, &motor->control, 
                    motor->param.max_pos, -motor->param.max_pos, 
                    motor->param.max_vel, -motor->param.max_vel, 
                    motor->param.max_torque, -motor->param.max_torque);

                }
                
                can_commu_send(motor->can, (char *)&motor->can_tx, sizeof(can_frame));
                
                // pthread_spin_unlock(&motor->lock);
                
                enable_last[i] = motor->motor_enable;
                usleep(1000*1);
            }
            
            
            usleep(1000*10);
        }
    }
    

    return NULL;
}

int motor_test_init()
{
    memset(g_motor, 0, sizeof(g_motor));

    if(dy200_init("/dev/ttyUSB0", 115200))
        return -1;

    g_motor[0].param.max_pos = 12.5;
    g_motor[0].param.max_vel = 45;
    g_motor[0].param.max_torque = 160;
    g_motor[0].protocol = 0;

    g_motor[1].param.max_pos = 12.5;
    g_motor[1].param.max_vel = 45;
    g_motor[1].param.max_torque = 160;
    g_motor[1].protocol = 0;

    g_motor[0].can = can_commu_init("can0", "can0", 0, 0xffff, motor_test_can_call, (void *)g_motor, NULL, 0);
    if (!g_motor[0].can)
        return -1;

    g_motor[1].can = g_motor[0].can;

    pid_zero(&g_motor[1].pid);

    g_motor[1].control.kp = 0.0;
    g_motor[1].control.kd = 0.0;
    g_motor[1].pid.kp = 1.5;
    g_motor[1].pid.ki = 0.25;
    g_motor[1].pid.output_max = 90.0;
    g_motor[1].pid.error_all_max = g_motor[1].pid.output_max / g_motor[1].pid.ki;
    g_motor[1].pid.error_max = 10.0;

    pthread_spin_init(&g_motor[0].lock, 0);
    g_motor[0].can_tx.can_dlc = 8;
    g_motor[0].can_tx.can_id = MOTOR_1_ID;

    pthread_spin_init(&g_motor[1].lock, 0);
    g_motor[1].can_tx.can_dlc = 8;
    g_motor[1].can_tx.can_id = MOTOR_2_ID;

    pthread_t t_id2;
    if (-1 == pthread_create(&t_id2, NULL, commu_thread, (void *)g_motor))
    {
        printf("commu thread create failed.\n");
        return -1;
    }  


    return 0;
}

int s_torq(int argc, char *argv[])
{
    float tor =0;
    if (argc == 2)
    {
        tor = atof(argv[1]);
    }

// pthread_spin_lock(&g_motor[0].lock);
//     g_motor[0].control.t_ff = tor;
set_motor_tor(&g_motor[0], tor);
// pthread_spin_unlock(&g_motor[0].lock);

return 0;
}
FINSH_FUNCTION_EXPORT(s_torq, s_torq);

int s2_torq(int argc, char *argv[])
{
    float tor =0;
    if (argc == 2)
    {
        tor = atof(argv[1]);
    }

// pthread_spin_lock(&g_motor[0].lock);
//     g_motor[0].control.t_ff = tor;
set_motor_tor(&g_motor[1], tor);
// pthread_spin_unlock(&g_motor[0].lock);

return 0;
}
FINSH_FUNCTION_EXPORT(s2_torq, s2_torq);

int s_vel(int argc, char *argv[])
{
    float vel =0;
    if (argc == 2)
    {
        vel = atof(argv[1]);
    }

// pthread_spin_lock(&g_motor[0].lock);
//     g_motor[0].control.t_ff = tor;
set_motor_vel(&g_motor[1], vel);
// pthread_spin_unlock(&g_motor[0].lock);

return 0;
}
FINSH_FUNCTION_EXPORT(s_vel, s_vel);

int s_pid_vel(int argc, char *argv[])
{
    float vel =0;
    if (argc == 2)
    {
        vel = atof(argv[1]);
    }

    g_motor[1].pid.des = vel;
    // g_motor[1].control.v_des = vel;

    return 0;
}
FINSH_FUNCTION_EXPORT(s_pid_vel, s_pid_vel);

int m_enable(int argc, char *argv[])
{
    int enable =0;
    if (argc != 2)
        return -1;

    enable = atoi(argv[1]);


// pthread_spin_lock(&g_motor[0].lock);
    motor_enable( &g_motor[0], enable);
// pthread_spin_unlock(&g_motor[0].lock);

return 0;
}
FINSH_FUNCTION_EXPORT(m_enable, m_enable);


int m_enable2(int argc, char *argv[])
{
    int enable =0;
    if (argc != 2)
        return -1;

    enable = atoi(argv[1]);


// pthread_spin_lock(&g_motor[1].lock);
    motor_enable( &g_motor[1], enable);
// pthread_spin_unlock(&g_motor[1].lock);

return 0;
}
FINSH_FUNCTION_EXPORT(m_enable2, m_enable2);


int p_info(int argc, char *argv[])
{
    float torque,speed,power;

    printf("print info\n");

    get_dy200_info(&torque, &speed, &power);
    printf("torque:%.3f, speed:%.3f, power:%.3f\n", torque, speed, power);

// pthread_spin_lock(&g_motor[0].lock);
    printf("motor pos:%.5f, vel:%.5f, tor:%.5f\n", g_motor[0].state.p, g_motor[0].state.v, g_motor[0].state.t);
// pthread_spin_unlock(&g_motor[0].lock);

// pthread_spin_lock(&g_motor[1].lock);
    printf("motor pos:%.5f, vel:%.5f, tor:%.5f\n", g_motor[1].state.p, g_motor[1].state.v, g_motor[1].state.t);
// pthread_spin_unlock(&g_motor[1].lock);

return 0;
 
}
FINSH_FUNCTION_EXPORT(p_info, p_info);

int tor_test(int argc, char *argv[])
{

    float torque,speed,power;
        
    

    motor_enable( &g_motor[0], 1);

    printf("current, torque\n");

    float current = 0;
    int times = 40 * 10 * 2;
    for (int i = 0; i < times; i++)
    {
        current += (0.1 * (i >= (times/2) ? -1 : 1));
        set_motor_tor(&g_motor[0], current);
        usleep(1000*10);
        get_dy200_info(&torque, &speed, &power);
        printf("%.3f, %.3f\n", g_motor[0].state.t, torque*-1);
    }

    motor_enable( &g_motor[0], 0);

    return 0;
}
FINSH_FUNCTION_EXPORT(tor_test, tor_test);

int ttor_test_new(int argc, char *argv[])
{
    float torque,speed,power;
    float KT = 0.07f;
    float GR = (7056.f/361.f);

    motor_enable( &g_motor[0], 1);
    // motor_enable( &g_motor[1], 1);

    // s_pid_vel(&g_motor[1], 10);

    usleep(1000*200);
 
    printf("current, torque\n");

    float current = 0;
    int times = 1000 * 2;
    for (int i = 0; i < times; i++)
    {
        current += (90.0f * 2.0f / times * (i >= (times/2) ? -1 : 1));
        set_motor_tor(&g_motor[0], current*KT*GR);
        usleep(1000*10);
        get_dy200_info(&torque, &speed, &power);
        if(i <= (times/2) && i >=50){
            printf("%.3f, %.3f\n", current, torque);
        }
    }

    // s_pid_vel(&g_motor[1], 0);

    motor_enable( &g_motor[0], 0);
    // motor_enable( &g_motor[1], 0);

    return 0;
}
FINSH_FUNCTION_EXPORT(ttor_test_new, ttor_test_new);

int a_max_speed_tor_test(int argc, char *argv[])
{
    float torque,speed,power;
    float KT = 0.07;
    float GR = (7056.f/361.f);

    motor_enable( &g_motor[0], 1);
    motor_enable( &g_motor[1], 1);

    printf("torque, speed\n");

    float current1 = 0;
    float current2 = 0;
    float target_current = 10.0f;
    int times = 1000 * 2;

    for (int i = 0; i < times / 2; i++)
    {
        current1 += (target_current * 2.0f / times * (i >= (times/2) ? -1 : 1));
        set_motor_tor(&g_motor[0], current1*KT*GR);
        usleep(1000*10);
    }

    printf("%.3f\n", current1);

    for (int i = 0; i < times; i++)
    {
        current2 += (target_current * 2.0f / times * (i >= (times/2) ? -1 : 1));
        set_motor_tor(&g_motor[1], current2*KT*GR);
        usleep(1000*10);
        get_dy200_info(&torque, &speed, &power);
        if(i <= (times/2) && i >=50){
            printf("%.3f, %.3f\n", torque, speed);
        }

        // if(i <= (times/2)){
        //     printf("current2%.3f\n", current2);
        // }
    }

    for (int i = 0; i < times / 2; i++)
    {
        current1 -= (target_current * 2.0f / times * (i >= (times/2) ? -1 : 1));
        set_motor_tor(&g_motor[0], current1*KT*GR);
        usleep(1000*10);
    }

    printf("%.3f\n", current1);

    // motor_enable( &g_motor[0], 0);
    motor_enable( &g_motor[1], 0);

    return 0;
}
FINSH_FUNCTION_EXPORT(a_max_speed_tor_test, a_max_speed_tor_test);

int vesc_tor_test(int argc, char *argv[])
{
    float torque,speed,power;
        
    float current = 0;

    float max = 50;
    float sec = 2;

    if (argc >= 2)
    {
        max = atof(argv[1]);
        if (argc == 3)
        {
            sec = atof(argv[2]);
        }
    }

    if (max >= 420 || sec >= 30)
    {
        printf("wrong param, max:%.3f sec:%.3f\n", max, sec);
    }

    printf("test param, max:%.3f sec:%.3f\n", max, sec);

#define FILE_BUF 64
    FILE *file = fopen("./test.csv", "w");
    char bufer[FILE_BUF] = {0};

    printf("\n*****************test start*************\n\n");

    //memset(bufer, 0, FILE_BUF);
    sprintf(bufer, "current, torque\n");
    fwrite(bufer, 1, strlen(bufer), file);
    printf("%s", bufer);

    float interval = 0.01;

    int times = sec / interval;

    float step = max / (float)times;

    int loop_num = times * 2;

    for (int i = 0; i < loop_num; i++)
    {
        current += ( step * (i <= times ? 1 : -1) );

        set_motor_tor(&g_motor[0], current);

        //usleep(interval * 1000 * 1000);
        delay_us(interval * 1000 * 1000);

        get_dy200_info(&torque, &speed, &power);

        memset(bufer, 0, FILE_BUF);
        sprintf(bufer, "%.3f, %.3f\n", current, torque);
        fwrite(bufer, 1, strlen(bufer), file);
        printf("%s", bufer);

    }

    // motor_enable( &g_motor[0], 0);
    fclose(file);

    return 0;
}
FINSH_FUNCTION_EXPORT(vesc_tor_test, vesc_tor_test);

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/sysinfo.h>
static __inline int kbhit(void)
{

    fd_set rfds;
    struct timeval tv;
    int retval;

    /* Watch stdin (fd 0) to see when it has input. */
    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    /* Wait up to five seconds. */
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    retval = select(1, &rfds, NULL, NULL, &tv);
    /* Don't rely on the value of tv now! */

    if (retval == -1)
    {
        perror("select()");
        return 0;
    }
    else if (retval)
        return 1;
    /* FD_ISSET(0, &rfds) will be true. */
    else
        return 0;
    return 0;
}

int dir_test(int argc, char *argv[])
{
    float vel = -1;
    float tor = 10;
    int time = 2;

    if (argc >= 3)
    {
        vel = atof(argv[1]);
        tor = atof(argv[2]);
    }
    if (argc >= 4)
    {
        time = atoi(argv[3]);
    }

    printf("vel:%f tor:%f time:%d\n", vel, tor, time);
    
    motor_enable( &g_motor[0], 1);
    motor_enable( &g_motor[1], 1);

    while (!kbhit())
    {
        set_motor_tor(&g_motor[0], 0);
        set_motor_vel(&g_motor[1], vel);
        usleep(1000*100);
        set_motor_tor(&g_motor[0], tor);

        sleep(time);
        if (kbhit())
            break;

        set_motor_tor(&g_motor[0], 0);
        set_motor_vel(&g_motor[1], -vel);
        usleep(1000*100);
        set_motor_tor(&g_motor[0], -tor);

        sleep(time);
        if (kbhit())
            break;  
    }


    motor_enable( &g_motor[0], 0);
    motor_enable( &g_motor[1], 0);
    usleep(1000*100);

    return 0;
}
FINSH_FUNCTION_EXPORT(dir_test, dir_test);

int dir_test2(int argc, char *argv[])
{
    float vel = -1;
    float tor = 10;
    int time = 2;

    if (argc >= 3)
    {
        vel = atof(argv[1]);
        tor = atof(argv[2]);
    }
    if (argc >= 4)
    {
        time = atoi(argv[3]);
    }
    


    motor_enable( &g_motor[0], 1);
    motor_enable( &g_motor[1], 1);

    while (!kbhit())
    {
        set_motor_tor(&g_motor[0], tor);
        set_motor_vel(&g_motor[1], vel);

        sleep(time);
        if (kbhit())
            break;

        set_motor_tor(&g_motor[0], -tor*0.5);
        set_motor_vel(&g_motor[1], vel);     

        sleep(time);
        if (kbhit())
            break;  
    }
    

        



    motor_enable( &g_motor[0], 0);
    motor_enable( &g_motor[1], 0);

    return 0;
}
FINSH_FUNCTION_EXPORT(dir_test2, dir_test2);