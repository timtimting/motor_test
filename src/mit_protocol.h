#ifndef _MIT_PROTOCOL_H_
#define _MIT_PROTOCOL_H_

#include <stdint.h>
#include <linux/can.h>
// #include "robot_hw.h"
#include "common.h"

/// Value Limits ///
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

typedef struct
{
    float p_des, v_des, kp, kd, t_ff;
}joint_control;

typedef struct
{
    float p, v, t;
}joint_state;

void Zero(struct can_frame *msg);
void EnterMotorMode(uint8_t *data);
void ExitMotorMode(uint8_t *data);
void pack_cmd(uint8_t *data, joint_control *joint, 
                    float p_max, float p_min, float v_max, float v_min, float t_max, float t_min);
void pack_cmd_vel(uint8_t *data, joint_control *joint, float v_max, float v_min);

#endif