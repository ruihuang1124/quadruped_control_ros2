/*!
* @file rt_usb_cdc.h
* @brief USB communication to USB2CAN board
* @author by TianYang TANG, Rui HUANG.
*/

#ifndef _rt_usb_cdc
#define _rt_usb_cdc

#ifdef linux


// incredibly obscure bug in SPI_IOC_MESSAGE macro is fixed by this
#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif


#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
}
#endif

#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <sstream>
// #include "leg_can_command_t.hpp"
// #include "leg_can_data_t.hpp"
// #include <sensor_msgs/msg/joint_state.hpp>
// #include "custom_msgs/msg/actuator_cmds.hpp"
#include "custom_msgs/msg/joint_commands.hpp"
#include "custom_msgs/msg/joint_states.hpp"

/*
 * Arcdog quadruped robot & HT motor LIMITS.
 */
#define LEG_AMOUNT 4
#define P_MIN_HT -95.5f
#define P_MAX_HT 95.5f
#define V_MIN_HT -45.0f
#define V_MAX_HT 45.0f
#define KP_MIN_HT 0.0f
#define KP_MAX_HT 500.0f
#define KD_MIN_HT 0.0f
#define KD_MAX_HT 5.0f
#define T_MIN_HT_03 -18.0f
#define T_MAX_HT_03 18.0f

#define T_MIN_HT_04 -18.0f
#define T_MAX_HT_04 18.0f

// DM motor LIMITS.
#define P_MIN_DM -12.5f
#define P_MAX_DM 12.5f
#define V_MIN_DM -45.0f
#define V_MAX_DM 45.0f
#define KP_MIN_DM 0.0f
#define KP_MAX_DM 500.0f
#define KD_MIN_DM 0.0f
#define KD_MAX_DM 5.0f
#define T_MIN_DM -54.0f
#define T_MAX_DM 54.0f

// DM motor LIMITS.
#define P_MIN_LK 0.0f
#define P_MAX_LK 6.2831852f
#define V_MIN_LK -30.0f
#define V_MAX_LK 30.0f
#define KP_MIN_LK 0.0f
#define KP_MAX_LK 500.0f
#define KD_MIN_LK 0.0f
#define KD_MAX_LK 100.0f
#define T_MIN_LK -33.0f
#define T_MAX_LK 33.0f

// DM2325 motor LIMITS.
#define P_MIN_DM2325 -2500.0f
#define P_MAX_DM2325 2500.0f
#define V_MIN_DM2325 -200.0f
#define V_MAX_DM2325 200.0f
#define KP_MIN_DM2325 0.0f
#define KP_MAX_DM2325 500.0f
#define KD_MIN_DM2325 0.0f
#define KD_MAX_DM2325 5.0f
#define T_MIN_DM2325 -10.0f
#define T_MAX_DM2325 10.0f


// original value: 0.364
// #define ARCDOG_K_ABAD_OFFSET_POS_0  0.3425f  //0.364+(0-(0.019+0.024)*0.5)
// #define ARCDOG_K_ABAD_OFFSET_POS_1  0.3465f  //0.364+(0-(0.019+0.016)*0.5)
// #define ARCDOG_K_ABAD_OFFSET_POS_2  0.334f  //0.364+(0-(0.030+0.030)*0.5)
// #define ARCDOG_K_ABAD_OFFSET_POS_3  0.385f  //0.364+(0-(0.024+0.027)*0.5)

// // original value: 1.219
// #define ARCDOG_K_HIP_OFFSET_POS_0   1.207f  // 1.219+(1.368-(1.380+1.380)*0.5)
// #define ARCDOG_K_HIP_OFFSET_POS_1   1.226f  // 1.219+(1.368-(1.360+1.362)*0.5)
// #define ARCDOG_K_HIP_OFFSET_POS_2   1.230f  // 1.219+(1.368-(1.354+1.360)*0.5)
// #define ARCDOG_K_HIP_OFFSET_POS_3   1.2345f  // 1.219+(1.368-(1.351+1.354)*0.5)

// // original value: -2.841
// //GearRatio = 7.65
// #define ARCDOG_K_KNEE_OFFSET_POS_0   -2.858f    // -2.841-(1.057-(1.040+1.040)*0.5)
// #define ARCDOG_K_KNEE_OFFSET_POS_1   -2.8305f    // -2.841-(1.057-(1.066+1.069)*0.5)
// #define ARCDOG_K_KNEE_OFFSET_POS_2   -2.896f   // -2.841-(1.057-(1.002+1.002)*0.5)
// #define ARCDOG_K_KNEE_OFFSET_POS_3   -2.8155f    // -2.841-(1.057-(1.084+1.081)*0.5)


// original value: -0.029
#define ARCDOG_K_ABAD_OFFSET_POS_0  -0.8350f
#define ARCDOG_K_ABAD_OFFSET_POS_1  -0.8350f
#define ARCDOG_K_ABAD_OFFSET_POS_2  -0.8350f
#define ARCDOG_K_ABAD_OFFSET_POS_3  -0.8350f

// // original value: 1.219
// #define ARCLAB_K_HIP_OFFSET_POS_0   1.207f  // 1.219+(1.368-(1.380+1.380)*0.5)
// #define ARCLAB_K_HIP_OFFSET_POS_1   1.226f  // 1.219+(1.368-(1.360+1.362)*0.5)
// #define ARCLAB_K_HIP_OFFSET_POS_2   1.230f  // 1.219+(1.368-(1.354+1.360)*0.5)
// #define ARCLAB_K_HIP_OFFSET_POS_3   1.2345f  // 1.219+(1.368-(1.351+1.354)*0.5)

// original value: 1.319
#define ARCDOG_K_HIP_OFFSET_POS_0   0.7093f
#define ARCDOG_K_HIP_OFFSET_POS_1   0.7093f
#define ARCDOG_K_HIP_OFFSET_POS_2   0.7093f
#define ARCDOG_K_HIP_OFFSET_POS_3   0.7093f

// // original value: -2.841
// //GearRatio = 7.65
// #define ARCLAB_K_KNEE_OFFSET_POS_0   -2.858f    // -2.841-(1.057-(1.040+1.040)*0.5)
// #define ARCLAB_K_KNEE_OFFSET_POS_1   -2.8305f    // -2.841-(1.057-(1.066+1.069)*0.5)
// #define ARCLAB_K_KNEE_OFFSET_POS_2   -2.896f   // -2.841-(1.057-(1.002+1.002)*0.5)
// #define ARCLAB_K_KNEE_OFFSET_POS_3   -2.8155f    // -2.841-(1.057-(1.084+1.081)*0.5)

// original value: -2.884
//GearRatio = 7.65
#define ARCDOG_K_KNEE_OFFSET_POS_0   -2.280f
#define ARCDOG_K_KNEE_OFFSET_POS_1   -2.280f
#define ARCDOG_K_KNEE_OFFSET_POS_2   -2.280f
#define ARCDOG_K_KNEE_OFFSET_POS_3   -2.280f

#define ARCDOG_K_PRISMATIC_OFFSET_POS_0   0.16f
#define ARCDOG_K_PRISMATIC_OFFSET_POS_1   0.16f
#define ARCDOG_K_PRISMATIC_OFFSET_POS_2   0.16f
#define ARCDOG_K_PRISMATIC_OFFSET_POS_3   0.16f


/*!
* USB command compose message
*/
typedef struct
{
    /* data */
    uint8_t leg;
    uint8_t dataA[8];
    uint8_t dataB[8];
    uint8_t dataC[8];
    uint8_t dataD[8];
} CAN_HOST_DATA;

/*!
* USB data raw message
*/
typedef struct
{
    /* data */
    uint8_t leg;
    uint8_t dataA[8];
    uint8_t dataB[8];
    uint8_t dataC[8];
    uint8_t dataD[8];
} CAN_SLAVE_DATA;

typedef struct
{
 /* data */
 uint16_t p_des;
 uint16_t v_des;
 uint16_t kp;
 uint16_t kd;
 uint16_t t_ff;
} CAN_COMMAND;

typedef struct
{
 /* data */
 uint8_t ID;
 uint8_t ERR;
 uint16_t POS;
 uint16_t VEL;
 uint16_t T;
} CAN_DATA;

/*!
* leg command message
*/
typedef struct {
 float q_des_abad;
 float q_des_hip;
 float q_des_knee;
 float q_des_prismatic;
 float qd_des_abad;
 float qd_des_hip;
 float qd_des_knee;
 float qd_des_prismatic;
 float kp_abad;
 float kp_hip;
 float kp_knee;
 float kp_prismatic;
 float kd_abad;
 float kd_hip;
 float kd_knee;
 float kd_prismatic;
 float tau_abad_ff;
 float tau_hip_ff;
 float tau_knee_ff;
 float tau_prismatic_ff;
} LEG_COMMAND_T;

/*!
* leg data message
*/
typedef struct {
 float q_abad;
 float q_hip;
 float q_knee;
 float q_prismatic;
 float qd_abad;
 float qd_hip;
 float qd_knee;
 float qd_prismatic;
 float tau_abad;
 float tau_hip;
 float tau_knee;
 float tau_prismatic;
} LEG_DATA_T;

// extern int tty_descriptor;
// extern const char *tty_path;

int init_usb();

void usb_send_receive(CAN_HOST_DATA* command, CAN_SLAVE_DATA* data, int tty_descriptor);
void usb_send_receive(LEG_COMMAND_T* leg_command, LEG_DATA_T* leg_data, int tty_descriptor,uint8_t leg);
void usb_send_receive(custom_msgs::msg::JointCommands* leg_command, custom_msgs::msg::JointStates* leg_data, int tty_descriptor);
// void usb_send_receive(leg_can_command_t* leg_command, leg_can_data_t* leg_data, int tty_descriptor);
int usb_driver_start();
void usb_driver_run(int tty_descriptor, bool motor_mode_flag);
void usb_driver_run(int tty_descriptor, bool motor_mode_flag, uint64_t current_iteration);
void usb_driver_end(int tty_descriptor);


CAN_HOST_DATA* get_can_host_data();
LEG_DATA_T* get_leg_data_struct();
LEG_COMMAND_T* get_leg_command_struct();



// leg_can_data_t* get_leg_data_lcmtype();
// leg_can_command_t* get_leg_command_lcmtype();
custom_msgs::msg::JointCommands* get_joint_commands_msgtype();
custom_msgs::msg::JointStates* get_joint_states_msgtype();



#endif // END of #ifdef linux

#endif