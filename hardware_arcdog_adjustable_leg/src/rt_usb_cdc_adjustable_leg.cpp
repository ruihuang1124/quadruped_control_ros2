/*!
 * @file rt_usb_cdc.h
 * @brief USB communication to USB2CDC board
 * @author by TianYang TANG, Rui HUANG.
 */
//using datatypes in IEEE standard
#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE

// #include "hardware_arcdog/rt_usb_cdc.h"
#include "hardware_arcdog_adjustable_leg/rt_usb_cdc_adjustable_leg.h"
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <iomanip> 
// #include <lcm/lcm-cpp.hpp>
#include <cstdio> 
#include <cmath> // Added for math functions if needed


/* Input buffer size.  Use a power of two larger than 512; I'd recommend 4096 - 65536.
 */
#ifndef BUFFER_SIZE
#define BUFFER_SIZE 512
#endif

/* Units of MB: 1000000 or 1048576
 */
#ifndef MEGA
#define MEGA 1000000
#endif

// for DM2325 and adjustable leg
#define GEAR_RATIO_1    25.0f   // 一级减速比(DM2325电机)
#define GEAR_RATIO_2    1/2.5f    // 二级减速比（伸缩腿齿轮）
#define SCREW_LEAD      0.005f  // 丝杠导程 (5mm = 0.005m)
#define TOTAL_REDUCTION (GEAR_RATIO_1 * GEAR_RATIO_2) 

static uint32_t frame = 0;
static bool recieve_flag=0;


// only used for actual robot
const float arcdog_abad_side_sign[4] = {-1, -1, 1, 1};
const float arcdog_hip_side_sign[4] = {1, -1, 1, -1};
// const float arcdog_knee_side_sign[4] = {-1, 1, -1, 1};
const float arcdog_knee_side_sign[4] = {-1, 1, -1, 1};
float arcdog_abad_offset[4] = {ARCDOG_K_ABAD_OFFSET_POS_0, -ARCDOG_K_ABAD_OFFSET_POS_1, ARCDOG_K_ABAD_OFFSET_POS_2, -ARCDOG_K_ABAD_OFFSET_POS_3};
float arcdog_hip_offset[4] = {ARCDOG_K_HIP_OFFSET_POS_0, ARCDOG_K_HIP_OFFSET_POS_1, ARCDOG_K_HIP_OFFSET_POS_2, ARCDOG_K_HIP_OFFSET_POS_3};
float arcdog_knee_offset[4] = {ARCDOG_K_KNEE_OFFSET_POS_0, ARCDOG_K_KNEE_OFFSET_POS_1, ARCDOG_K_KNEE_OFFSET_POS_2, ARCDOG_K_KNEE_OFFSET_POS_3};
float arcdog_prismatic_offset[4] = {ARCDOG_K_PRISMATIC_OFFSET_POS_0, ARCDOG_K_PRISMATIC_OFFSET_POS_1, ARCDOG_K_PRISMATIC_OFFSET_POS_2, ARCDOG_K_PRISMATIC_OFFSET_POS_3};

//const float arcdog_abad_side_sign[4] = {1, 1, 1, 1};
//const float arcdog_hip_side_sign[4] = {1, 1, 1, 1};
//const float arcdog_knee_side_sign[4] = {1, 1, 1, 1};
//float arcdog_abad_offset[4] = {0, 0, 0, 0};
//float arcdog_hip_offset[4] = {0, 0, 0, 0};
//float arcdog_knee_offset[4] = {0, 0, 0, 0};

/* Xorshift64* pseudo-random number generator.  Zero state is invalid. For CAN test
 */
static uint64_t prng_state = 0;
#define USB_FRAME_SIZE 33


static CAN_HOST_DATA host_data[LEG_AMOUNT];
static CAN_HOST_DATA host_data_last[LEG_AMOUNT];
static CAN_SLAVE_DATA slave_data[LEG_AMOUNT];
static CAN_COMMAND raw_motor_cmd_t[LEG_AMOUNT];
static CAN_DATA raw_motor_data_t[LEG_AMOUNT];
static LEG_COMMAND_T leg_command_drv_struct[LEG_AMOUNT];
static LEG_DATA_T leg_data_drv_struct[LEG_AMOUNT];

// leg_can_command_t leg_command_drv_lcmtype;
// leg_can_data_t leg_data_drv_lcmtype;

custom_msgs::msg::JointCommands joint_commands_drv_msgtype;
custom_msgs::msg::JointStates joint_states_drv_msgtype;


pthread_mutex_t usb_mutex;

void printHexArray(const uint8_t *array, size_t size)
{
    std::ostringstream oss;
    oss << "Data: [";

    for (size_t i = 0; i < size; ++i)
    {
        oss << "0x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(array[i]);
        if (i < size - 1)
        {
            oss << ", ";
        }
    }
    oss << "]";

}

/* When 'done' becomes nonzero, it is time to stop the measurement.
 */
static volatile sig_atomic_t done = 0;

/* Signal handler for 'done'.
 */
static void handle_done(int signum)
{
    // Silence unused variable warning; generates no code.
    (void)signum;

    done = 1;
}

static int install_done(int signum)
{
    struct sigaction act;
    memset(&act, 0, sizeof act);
    sigemptyset(&act.sa_mask);
    act.sa_handler = handle_done;
    act.sa_flags = 0; // Specifically, NO SA_RESTART flag.
    return sigaction(signum, &act, NULL);
}

/* One-second interval timer.  Simply sets 'update' to nonzero.
 */
#ifndef UPDATE_SIGNAL
#define UPDATE_SIGNAL (SIGRTMIN + 0)
#endif

static timer_t update_timer;
static volatile sig_atomic_t update = 0;

static void handle_update(int signum)
{
  (void)signum;
  update = 1;
}

static int install_update(void)
{
  struct itimerspec spec;
  struct sigevent ev;
  struct sigaction act;

  memset(&act, 0, sizeof act);
  sigemptyset(&act.sa_mask);
  act.sa_handler = handle_update;
  act.sa_flags = SA_RESTART;
  if (sigaction(UPDATE_SIGNAL, &act, NULL) == -1)
    return -1;

  ev.sigev_notify = SIGEV_SIGNAL;
  ev.sigev_signo = UPDATE_SIGNAL;
  ev.sigev_value.sival_ptr = NULL;
  if (timer_create(CLOCK_BOOTTIME, &ev, &update_timer) == -1)
    return -1;

  spec.it_value.tv_sec = 1; // One second to first update
  spec.it_value.tv_nsec = 0;
  spec.it_interval.tv_sec = 1; // Repeat at one second intervals
  spec.it_interval.tv_nsec = 0;
  if (timer_settime(update_timer, 0, &spec, NULL) == -1)
    return -1;

  return 0;
}

/* USB serial port device handling.
 */
static struct termios tty_settings;
// int tty_descriptor = -1;
const char *tty_path = NULL;


/* CAN data structure declaration.
 */

// CAN_HOST_DATA can_host_data;
// CAN_SLAVE_DATA can_slave_data;

static void tty_cleanup(int tty_descriptor)
{
    if (tty_descriptor != -1)
    {
        if (tcsetattr(tty_descriptor, TCSANOW, &tty_settings) == -1)
            fprintf(stderr, "Warning: %s: Cannot reset original termios settings: %s.\n", tty_path, strerror(errno));

        tcflush(tty_descriptor, TCIOFLUSH);

        if (close(tty_descriptor) == -1)
            fprintf(stderr, "Warning: %s: Error closing device: %s.\n", tty_path, strerror(errno));

        tty_descriptor = -1;
    }
}

static int tty_open(const char *path,int &tty_descriptor)
{
    struct termios raw;
    int fd;

    // NULL or empty path is invalid.But in this situation the default path will be used
    if (!path || !*path)
    {
        // errno = ENOENT;
        // return -1;
        path = "/dev/robot_can";
    }

    // Fail if tty is already open.
    if (tty_descriptor != -1)
    {
        errno = EALREADY;
        return -1;
    }

    // Open the tty device.
    do
    {
        fd = open(path, O_RDWR | O_NOCTTY | O_CLOEXEC);
    } while (fd == -1 && errno == EINTR);
    if (fd == -1)
        return -1;

    // Set exclusive mode, so that others cannot open the device while we have it open.
    if (ioctl(fd, TIOCEXCL) == -1)
        fprintf(stderr, "Warning: %s: Cannot get exclusive access on tty device: %s.\n", path, strerror(errno));

    // Drop any already pending data.
    tcflush(fd, TCIOFLUSH);

    // Obtain current termios settings.
    if (tcgetattr(fd, &raw) == -1 || tcgetattr(fd, &tty_settings) == -1)
    {
        fprintf(stderr, "%s: Cannot get termios settings: %s.\n", path, strerror(errno));
        close(fd);
        errno = 0; // Already reported
        return -1;
    }

    // Raw 8-bit mode: no post-processing or special characters, 8-bit data.
    raw.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | INPCK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IUCLC | IUTF8);
    raw.c_oflag &= ~(OPOST);
    raw.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    raw.c_cflag &= ~(CSIZE | PARENB | CLOCAL);
    raw.c_cflag |= CS8 | CREAD | HUPCL;
    // Blocking reads.
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &raw) == -1)
    {
        fprintf(stderr, "%s: Cannot set termios settings: %s.\n", path, strerror(errno));
        close(fd);
        errno = 0; // Already reported
        return -1;
    }

    // Drop any already pending data, again.  Just to make sure.
    tcflush(fd, TCIOFLUSH);

    // Everything seems to be in order.  Update state for tty_cleanup(), and return success.
    tty_descriptor = fd;
    tty_path = path;
    return 0;
}

static inline double seconds_between(const struct timespec after, const struct timespec before)
{
  return (double)(after.tv_sec - before.tv_sec) + (double)(after.tv_nsec - before.tv_nsec) / 1000000000.0;
}

int init_usb()
{
int tty_descriptor = -1;
tty_open(NULL,tty_descriptor);
return tty_descriptor;
}

int usb_driver_start() {

  // size_t command_size = sizeof(spi_command_t);
  // size_t data_size = sizeof(spi_data_t);
  //
  // memset(&spi_command_drv, 0, sizeof(spi_command_drv));
  // memset(&spi_data_drv, 0, sizeof(spi_data_drv));

  //sudo now.
  if (pthread_mutex_init(&usb_mutex, NULL) != 0) printf("[ERROR: RT USB CDC] Failed to create usb cdc data mutex\n");

  // if (command_size != K_EXPECTED_COMMAND_SIZE) {
  //   printf("[RT USB CDC] Error command size is %ld, expected %d\n", command_size, K_EXPECTED_COMMAND_SIZE);
  // } else
  //   printf("[RT USB CDC] command size good\n");
  //
  // if (data_size != K_EXPECTED_DATA_SIZE) {
  //   printf("[RT USB CDC] Error data size is %ld, expected %d\n", data_size, K_EXPECTED_DATA_SIZE);
  // } else
  //   printf("[RT USB CDC] data size good\n");

  printf("[RT USB CDC] Open\n");

  const size_t buffer_size = BUFFER_SIZE;
  size_t buffer_have = 0;
  unsigned char *buffer_data = NULL;
  struct timespec started, mark;
  uint64_t received_before = 0; // Received till mark
  uint64_t received = 0;        // Received after mark
  uint64_t sequence = 0;
  uint64_t seed;
  uint8_t ros_can_trans[USB_FRAME_SIZE] = {0};
  static unsigned char request[USB_FRAME_SIZE];


  // if (install_done(SIGHUP) ||
  //   install_done(SIGINT) ||
  //   install_done(SIGTERM))
  // {
  //   fprintf(stderr, "Cannot install signal handlers: %s.\n", strerror(errno));
  // }
  printf("Attempting to open device: /dev/robot_can\n");
  buffer_data = static_cast<unsigned char *>(malloc(buffer_size + 4));
  if (!buffer_data)
  {
    fprintf(stderr, "Not enough memory available for a %zu-byte input buffer.\n", buffer_size);
  }
  int tty_descriptor = init_usb();
  if (tty_descriptor)
  {
    if (errno)
      fprintf(stderr, "%s: Cannot open device: /dev/robot_can.\n", strerror(errno));
  }


  // if (install_update())
  // {
  //   fprintf(stderr, "Cannot create a periodic update signal: %s.\n", strerror(errno));
  //   tty_cleanup(tty_descriptor);
  // }

  // if (clock_gettime(CLOCK_BOOTTIME, &started) == -1)
  // {
  //   fprintf(stderr, "Cannot read BOOTTIME clock: %s.\n", strerror(errno));
  //   tty_cleanup(tty_descriptor);
  // }
  // else
  //   mark = started;

  return tty_descriptor;
}


void usb_send_receive(CAN_HOST_DATA* command, CAN_SLAVE_DATA* data,int tty_descriptor)
{
  // printf("command leg %d is:\n", command->leg);
  // for (int i = 0; i < 8; ++i) {
  //   printf("%X ", command->dataA[i]);
  //   printf("%X ", command->dataB[i]);
  //   printf("%X ", command->dataC[i]);
  //   printf("%X ", command->dataD[i]);
  // }
  // printf("\n");

  if(tty_descriptor == -1 || command == NULL || data==NULL)
    {
        return;
    }
    static unsigned char *buffer_data = NULL;
    const size_t buffer_size = BUFFER_SIZE;
    static size_t buffer_have = 0;
    static unsigned char request[USB_FRAME_SIZE];
    memcpy(request, command, USB_FRAME_SIZE);
//    if (request[0] == 0x00){
//        printf("request is:\n");
//        for (int i = 0; i < USB_FRAME_SIZE; ++i) {
//      printf("%X ",request[i]);
//        }
//        printf("\n");
//    }
    //    std::cout<<std::hex<<request<<"\n";

    // // ================= [DEBUG START: 底层 IO 检查] =================
    // // 检查点 4: 确认底层 buffer 内容
    // // printf("[4] USB Write Buffer (First 16 bytes):\n  ");
    // if (request[0] == 0x03)
    // {
    //   for (int i = 0; i < 16; ++i) printf("%02X ", request[i]);
    //   printf("\n");
    // }
    // // ================= [DEBUG END] =================
    const unsigned char *const q = request + USB_FRAME_SIZE;
    const unsigned char *p = request;
    ssize_t n;
    if (buffer_data == NULL) 
    {
        buffer_data = static_cast<unsigned char *>(malloc(buffer_size + 4));
    }
    if (!buffer_data)
    {
        fprintf(stderr, "Not enough memory available for a %zu-byte input buffer.\n", buffer_size);
        return ;
    }
    while (p < q)
    {
        n = write(tty_descriptor, p, (size_t)(q - p));
        if (n > 0)
        {
            p += n;
        }
        else if (n != -1)
        {
            fprintf(stderr, "%s: Invalid write (%zd)\n", tty_path, n);
            break;
        }
        else if (errno != EINTR)
        {
            fprintf(stderr, "%s: Write error: %s.\n", tty_path, strerror(errno));
            break;
        }
    }
    if (p != q)
    {
        tty_cleanup(tty_descriptor);
        return;
    }

            // Receive more data?
        if (buffer_have < buffer_size)
        {
            ssize_t n = read(tty_descriptor, buffer_data + buffer_have, buffer_size - buffer_have);

            // // ================= [DEBUG START: 读取检查] =================
            // // 检查点 5: 确认 read 是否读到了东西
            // if (n > 0) {
            //     // printf("[5] USB Read Success: %zd bytes received.\n", n);
            //     // 打印刚收到的前几个字节看看是不是乱码
            //     // if (buffer_data[buffer_have] == 0X01)
            //     // {
            //     //   printf("    Rx Head: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", 
            //     //        buffer_data[buffer_have], buffer_data[buffer_have+1], buffer_data[buffer_have+2], buffer_data[buffer_have+3],
            //     //        buffer_data[buffer_have+4], buffer_data[buffer_have+5], buffer_data[buffer_have+6], buffer_data[buffer_have+7],
            //     //        buffer_data[buffer_have+8]);
            //     // }
            //     // printf("[5] USB Read Data Analysis (Hex):\n");
            //     if (buffer_data[buffer_have] == 0X01)
            //     {
            //     for (int i = 0; i < 33; ++i) {
            //         // 打印十六进制
            //         printf("%02X ", (unsigned char)buffer_data[i]);
                    
            //         // 尝试按 8 字节（假设一个电机8字节）换行，方便观察规律
            //         // 如果有帧头（假设1字节），可以尝试 (i - 1 + 1) % 8 == 0 这种逻辑
            //         // 这里简单每 8 个换一行
            //         if ((i + 1) % 8 == 0) printf(" | "); 
            //     }
            //     printf("\n");   
            //     }
            // } else if (n == 0) {
            //     printf("[5] USB Read: 0 bytes (No data from slave).\n");
            // }
            // // ================= [DEBUG END] =================

            if (n > 0)
            {
                buffer_have += n;
            }
            else if (n != -1)
            {
                fprintf(stderr, "%s: Unexpected read error (%zd).\n", tty_path, n);
                tty_cleanup(tty_descriptor);
                return ;
            }
            else if (errno != EINTR)
            {
                fprintf(stderr, "%s: Read error: %s.\n", tty_path, strerror(errno));
                tty_cleanup(tty_descriptor);
                return;
            }
        }
                // Verify all full words thus far received.
        if (buffer_have > 3)
        {
            const unsigned char *next = buffer_data;
            const unsigned char *const ends = buffer_data + buffer_have;

            while (next + USB_FRAME_SIZE <= ends)
            {
                uint64_t u = 0;
                memcpy(data, next, USB_FRAME_SIZE);
                next += USB_FRAME_SIZE;
            }

            if (next < ends)
            {
                memmove(buffer_data, next, (size_t)(ends - next));
                buffer_have = (size_t)(ends - next);
            }
            else
            {
                buffer_have = 0;
            }
        }

}


// 函数1：将 D 数组的数据填充到 CAN_COMMAND 结构体中
void array_to_struct(uint8_t D[8], CAN_DATA *data) {
    // Extract ID and ERR from D[0]
    data->ID = D[0] & 0x0F;           // ID is the least significant 4 bits
    data->ERR = (D[0] >> 4) & 0x0F;   // ERR is the next 4 bits

    // Extract POS from D[1] and D[2]
    data->POS = ((uint16_t)D[1] << 8) | D[2];

    // Extract VEL from D[3] and D[4]
    data->VEL = (((uint16_t)(D[3] & 0xFF) << 4) | ((uint16_t)D[4] >> 4)) & 0x0FFF; // VEL is 12 bits

    // Extract T from D[4] and D[5]
    data->T = (((uint16_t)(D[4] & 0x0F) << 8) | D[5]) & 0x0FFF; // T is 12 bits
}
// 函数2：将 CAN_COMMAND 结构体的数据填充到 D 数组中
void struct_to_array(CAN_COMMAND *cmd, uint8_t D[8]) {
    D[0] = (cmd->p_des >> 8) & 0xFF;       // p_des高8位
    D[1] = cmd->p_des & 0xFF;              // p_des低8位
    D[2] = (cmd->v_des >> 4) & 0xFF;       // 右
    D[3] = (((cmd->v_des & 0x0F)<<4) | ((cmd->kp>>8)&0x0F) &0x0F); // v_des低4位 & Kp高4位
    D[4] = cmd->kp & 0xFF;                  // Kp低8位
    D[5] = (cmd->kd >> 4) & 0xFF;          // Kd高4位
    D[6] = (((cmd->kd & 0x0F)<<4)| ((cmd->t_ff >> 8) & 0x0F)) & 0xFF; // Kd低4位 & t_ff高4位
    D[7] = cmd->t_ff & 0xFF;               // t_ff低8位
}

 float uint_to_float(int x_int, float x_min, float x_max, int bits){
 /// converts unsigned int to float, given range and number of bits ///
 float span = x_max - x_min;
 float offset = x_min;
 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
 }

int float_to_uint(float x, float x_min, float x_max, int bits){
 // Converts a float to an unsigned int, given range and number of bits
 float span = x_max - x_min;
 float offset = x_min;
 return (int) ((x-offset)*((float)((1<<bits)-1))/span);
 }

void leg_command_to_can_command(LEG_COMMAND_T *leg_cmd, CAN_COMMAND *can_cmd) {
    can_cmd[0].p_des = float_to_uint(leg_cmd->q_des_abad, P_MIN_LK, P_MAX_LK,16);
    can_cmd[0].v_des = float_to_uint(leg_cmd->qd_des_abad, V_MIN_LK, V_MAX_LK,12);
    can_cmd[0].kp = float_to_uint(leg_cmd->kp_abad, KP_MIN_LK, KP_MAX_LK, 12);
    can_cmd[0].kd = float_to_uint(leg_cmd->kd_abad, KD_MIN_LK, KD_MAX_LK, 12);
    can_cmd[0].t_ff = float_to_uint(leg_cmd->tau_abad_ff, T_MIN_LK, T_MAX_LK, 12);

    can_cmd[1].p_des = float_to_uint(leg_cmd->q_des_hip, P_MIN_LK, P_MAX_LK,16);
    can_cmd[1].v_des = float_to_uint(leg_cmd->qd_des_hip, V_MIN_LK, V_MAX_LK,12);
    can_cmd[1].kp = float_to_uint(leg_cmd->kp_hip, KP_MIN_LK, KP_MAX_LK, 12);
    can_cmd[1].kd = float_to_uint(leg_cmd->kd_hip, KD_MIN_LK, KD_MAX_LK, 12);
    can_cmd[1].t_ff = float_to_uint(leg_cmd->tau_hip_ff, T_MIN_LK, T_MAX_LK, 12);

    can_cmd[2].p_des = float_to_uint(leg_cmd->q_des_knee, P_MIN_DM, P_MAX_DM,16);
    can_cmd[2].v_des = float_to_uint(leg_cmd->qd_des_knee, V_MIN_DM, V_MAX_DM,12);
    can_cmd[2].kp = float_to_uint(leg_cmd->kp_knee, KP_MIN_DM, KP_MAX_DM, 12);
    can_cmd[2].kd = float_to_uint(leg_cmd->kd_knee, KD_MIN_DM, KD_MAX_DM, 12);
    can_cmd[2].t_ff = float_to_uint(leg_cmd->tau_knee_ff, T_MIN_DM, T_MAX_DM, 12);

    // ------------------------------------------------------
    // Prismatic Joint - Inverse Mapping (Command -> Motor)
    // ------------------------------------------------------
    // Calculate conversion factor (Motor Radians -> Linear Meters)
    // Formula: (1 / Total_Reduction) * (Lead / 2PI)
    float rot_to_linear_factor = (1.0f / TOTAL_REDUCTION) * (SCREW_LEAD / (2.0f * 3.14159265f));
    
    float direction_sign = -1.0f; 
    
    float motor_q_des, motor_qd_des, motor_tau_ff, motor_kp, motor_kd;

    if (rot_to_linear_factor != 0.0f) {
        // Position: Meters -> Radians
        // q_motor = q_linear / K
        motor_q_des = (leg_cmd->q_des_prismatic / rot_to_linear_factor) * direction_sign;

        // Velocity: m/s -> Rad/s
        // qd_motor = qd_linear / K
        motor_qd_des = (leg_cmd->qd_des_prismatic / rot_to_linear_factor) * direction_sign;

        // Force -> Torque
        // P = F*v = T*w  => F * (w*K) = T * w => T = F * K
        // Torque (Nm) = Force (N) * rot_to_linear_factor
        motor_tau_ff = (leg_cmd->tau_prismatic_ff * rot_to_linear_factor) * direction_sign;

        // Kp Mapping (Linear Stiffness N/m -> Rotational Stiffness Nm/rad)
        // F = Kp_lin * x
        // (T / K) = Kp_lin * (theta * K)
        // T = (Kp_lin * K^2) * theta
        // Kp_rot = Kp_lin * K^2
        // motor_kp = leg_cmd->kp_prismatic * (rot_to_linear_factor * rot_to_linear_factor);
        motor_kp = leg_cmd->kp_prismatic;

        // Kd Mapping (Linear Damping Ns/m -> Rotational Damping Nms/rad)
        // Similar to Kp: Kd_rot = Kd_lin * K^2
        // motor_kd = leg_cmd->kd_prismatic * (rot_to_linear_factor * rot_to_linear_factor);
        motor_kd = leg_cmd->kd_prismatic;
    } else {
        motor_q_des = 0.0f;
        motor_qd_des = 0.0f;
        motor_tau_ff = 0.0f;
        motor_kp = 0.0f;
        motor_kd = 0.0f;
    }

    can_cmd[3].p_des = float_to_uint(motor_q_des, P_MIN_DM2325, P_MAX_DM2325, 16);
    can_cmd[3].v_des = float_to_uint(motor_qd_des, V_MIN_DM2325, V_MAX_DM2325, 12);
    can_cmd[3].kp = float_to_uint(motor_kp, KP_MIN_DM2325, KP_MAX_DM2325, 12);
    can_cmd[3].kd = float_to_uint(motor_kd, KD_MIN_DM2325, KD_MAX_DM2325, 12);
    can_cmd[3].t_ff = float_to_uint(motor_tau_ff, T_MIN_DM2325, T_MAX_DM2325, 12);
}

void can_data_to_leg_data(CAN_DATA *can_data, LEG_DATA_T *leg_data) {
    leg_data->q_abad = uint_to_float(can_data[0].POS, P_MIN_LK, P_MAX_LK,16);
    leg_data->qd_abad = uint_to_float(can_data[0].VEL, V_MIN_LK, V_MAX_LK,12);
    leg_data->tau_abad = uint_to_float(can_data[0].T, T_MIN_LK, T_MAX_LK,12);

    leg_data->q_hip = uint_to_float(can_data[1].POS, P_MIN_LK, P_MAX_LK,16);
    leg_data->qd_hip = uint_to_float(can_data[1].VEL, V_MIN_LK, V_MAX_LK,12);
    leg_data->tau_hip = uint_to_float(can_data[1].T, T_MIN_LK, T_MAX_LK,12);

    leg_data->q_knee = uint_to_float(can_data[2].POS, P_MIN_DM, P_MAX_DM,16);
    leg_data->qd_knee = uint_to_float(can_data[2].VEL, V_MIN_DM, V_MAX_DM,12);
    leg_data->tau_knee = uint_to_float(can_data[2].T, T_MIN_DM, T_MAX_DM,12);

    // leg_data->q_prismatic = uint_to_float(can_data[3].POS, P_MIN_DM2325, P_MAX_DM2325,16);
    // leg_data->qd_prismatic = uint_to_float(can_data[3].VEL, V_MIN_DM2325, V_MAX_DM2325,12);
    // leg_data->tau_prismatic = uint_to_float(can_data[3].T, T_MIN_DM2325, T_MAX_DM2325,12);

    // ------------------------------------------------------
    // Prismatic Joint - Linear Mapping Modification
    // ------------------------------------------------------
    
    // Step 1: Restore the raw motor data in Radians (rad) and Rad/s
    float motor_q_rad = uint_to_float(can_data[3].POS, P_MIN_DM2325, P_MAX_DM2325, 16);
    float motor_qd_rad = uint_to_float(can_data[3].VEL, V_MIN_DM2325, V_MAX_DM2325, 12);
    float motor_tau_Nm = uint_to_float(can_data[3].T, T_MIN_DM2325, T_MAX_DM2325, 12);

    // Step 2: Calculate conversion factor (Motor Radians -> Linear Meters)
    // Formula: (1 / Total_Reduction) * (Lead / 2PI)
    // This converts "Motor Rotation" to "Lead Screw Linear Motion"
    float rot_to_linear_factor = (1.0f / TOTAL_REDUCTION) * (SCREW_LEAD / (2.0f * 3.14159265f));
    float direction_sign = -1.0f;

    // Step 3: Apply mapping
    // Position: Radians -> Meters (m)
    leg_data->q_prismatic = motor_q_rad * rot_to_linear_factor * direction_sign;

    // Velocity: Radians/sec -> Meters/sec (m/s)
    leg_data->qd_prismatic = motor_qd_rad * rot_to_linear_factor * direction_sign;
    
    // Supplement: Force mapping
    // Based on energy conservation P = F*v = T*w, we know F = T * (w/v)
    // The ratio (w/v) is exactly the inverse of rot_to_linear_factor.
    // Therefore: Linear Force (N) = Motor Torque (Nm) / rot_to_linear_factor
    if (rot_to_linear_factor != 0.0f) {
        leg_data->tau_prismatic = motor_tau_Nm / rot_to_linear_factor; 
    } else {
        leg_data->tau_prismatic = 0.0f;
    }


}

// void leg_can_command_struct_to_lcm(LEG_COMMAND_T *leg_cmd_struct, leg_can_command_t *leg_can_command_lcm) {
//   for (int leg = 0; leg < LEG_AMOUNT; leg++) {
//     leg_can_command_lcm->q_des_abad[leg] = leg_cmd_struct[leg].q_des_abad;
//     leg_can_command_lcm->q_des_hip[leg] = leg_cmd_struct[leg].q_des_hip;
//     leg_can_command_lcm->q_des_knee[leg] = leg_cmd_struct[leg].q_des_knee;
//     leg_can_command_lcm->q_des_prismatic[leg] = leg_cmd_struct[leg].q_des_prismatic;

//     leg_can_command_lcm->qd_des_abad[leg] = leg_cmd_struct[leg].qd_des_abad;
//     leg_can_command_lcm->qd_des_hip[leg] = leg_cmd_struct[leg].qd_des_hip;
//     leg_can_command_lcm->qd_des_knee[leg] = leg_cmd_struct[leg].qd_des_knee;
//     leg_can_command_lcm->qd_des_prismatic[leg] = leg_cmd_struct[leg].qd_des_prismatic;

//     leg_can_command_lcm->kp_abad[leg] = leg_cmd_struct[leg].kp_abad;
//     leg_can_command_lcm->kp_hip[leg] = leg_cmd_struct[leg].kp_hip;
//     leg_can_command_lcm->kp_knee[leg] = leg_cmd_struct[leg].kp_knee;
//     leg_can_command_lcm->kp_prismatic[leg] = leg_cmd_struct[leg].kp_prismatic;

//     leg_can_command_lcm->kd_abad[leg] = leg_cmd_struct[leg].kd_abad;
//     leg_can_command_lcm->kd_hip[leg] = leg_cmd_struct[leg].kd_hip;
//     leg_can_command_lcm->kd_knee[leg] = leg_cmd_struct[leg].kd_knee;
//     leg_can_command_lcm->kd_prismatic[leg] = leg_cmd_struct[leg].kd_prismatic;

//     leg_can_command_lcm->tau_abad_ff[leg] = leg_cmd_struct[leg].tau_abad_ff;
//     leg_can_command_lcm->tau_hip_ff[leg] = leg_cmd_struct[leg].tau_hip_ff;
//     leg_can_command_lcm->tau_knee_ff[leg] = leg_cmd_struct[leg].tau_knee_ff;
//     leg_can_command_lcm->tau_prismatic_ff[leg] = leg_cmd_struct[leg].tau_prismatic_ff;
//   }
// }


void leg_can_command_msg_to_struct(custom_msgs::msg::JointCommands *leg_can_command_msg, LEG_COMMAND_T *leg_cmd_struct) {
  for (int leg = 0; leg < LEG_AMOUNT; leg++) {
    leg_cmd_struct[leg].q_des_abad = (leg_can_command_msg->q_des_abad[leg] - arcdog_abad_offset[leg]) * arcdog_abad_side_sign[leg];
    leg_cmd_struct[leg].q_des_hip = (leg_can_command_msg->q_des_hip[leg] - arcdog_hip_offset[leg]) * arcdog_hip_side_sign[leg];
    leg_cmd_struct[leg].q_des_knee = (leg_can_command_msg->q_des_knee[leg] - arcdog_knee_offset[leg]) * arcdog_knee_side_sign[leg];
    leg_cmd_struct[leg].q_des_prismatic = leg_can_command_msg->q_des_prismatic[leg] - arcdog_prismatic_offset[leg];
    // leg_cmd_struct[leg].q_des_prismatic = leg_can_command_msg->q_des_prismatic[leg];

    leg_cmd_struct[leg].qd_des_abad = leg_can_command_msg->qd_des_abad[leg] * arcdog_abad_side_sign[leg];
    leg_cmd_struct[leg].qd_des_hip = leg_can_command_msg->qd_des_hip[leg] * arcdog_hip_side_sign[leg];
    leg_cmd_struct[leg].qd_des_knee = leg_can_command_msg->qd_des_knee[leg] * arcdog_knee_side_sign[leg];
    leg_cmd_struct[leg].qd_des_prismatic = leg_can_command_msg->qd_des_prismatic[leg];

    leg_cmd_struct[leg].kp_abad = leg_can_command_msg->kp_abad[leg];
    leg_cmd_struct[leg].kp_hip = leg_can_command_msg->kp_hip[leg];
    leg_cmd_struct[leg].kp_knee = leg_can_command_msg->kp_knee[leg];
    leg_cmd_struct[leg].kp_prismatic = leg_can_command_msg->kp_prismatic[leg];

    leg_cmd_struct[leg].kd_abad = leg_can_command_msg->kd_abad[leg];
    leg_cmd_struct[leg].kd_hip = leg_can_command_msg->kd_hip[leg];
    leg_cmd_struct[leg].kd_knee = leg_can_command_msg->kd_knee[leg];
    leg_cmd_struct[leg].kd_prismatic = leg_can_command_msg->kd_prismatic[leg];

    leg_cmd_struct[leg].tau_abad_ff = leg_can_command_msg->tau_abad_ff[leg] * arcdog_abad_side_sign[leg];
    leg_cmd_struct[leg].tau_hip_ff = leg_can_command_msg->tau_hip_ff[leg] * arcdog_hip_side_sign[leg];
    leg_cmd_struct[leg].tau_knee_ff = leg_can_command_msg->tau_knee_ff[leg] * arcdog_knee_side_sign[leg];
    leg_cmd_struct[leg].tau_prismatic_ff = leg_can_command_msg->tau_prismatic_ff[leg];
  }
}

void leg_can_data_struct_to_msg(LEG_DATA_T *leg_data_struct, custom_msgs::msg::JointStates *leg_can_data_msg) {
  for (int leg = 0; leg < LEG_AMOUNT; leg++) {
    leg_can_data_msg->q_abad[leg] = leg_data_struct[leg].q_abad * arcdog_abad_side_sign[leg] + arcdog_abad_offset[leg];
    leg_can_data_msg->q_hip[leg] = leg_data_struct[leg].q_hip * arcdog_hip_side_sign[leg] + arcdog_hip_offset[leg];
    leg_can_data_msg->q_knee[leg] = leg_data_struct[leg].q_knee * arcdog_knee_side_sign[leg] + arcdog_knee_offset[leg];
    leg_can_data_msg->q_prismatic[leg] = leg_data_struct[leg].q_prismatic + arcdog_prismatic_offset[leg];
    // leg_can_data_msg->q_prismatic[leg] = leg_data_struct[leg].q_prismatic;

    leg_can_data_msg->qd_abad[leg] = leg_data_struct[leg].qd_abad * arcdog_abad_side_sign[leg];
    leg_can_data_msg->qd_hip[leg] = leg_data_struct[leg].qd_hip * arcdog_hip_side_sign[leg];
    leg_can_data_msg->qd_knee[leg] = leg_data_struct[leg].qd_knee * arcdog_knee_side_sign[leg];
    leg_can_data_msg->qd_prismatic[leg] = leg_data_struct[leg].qd_prismatic;

    leg_can_data_msg->tau_abad[leg] = leg_data_struct[leg].tau_abad * arcdog_abad_side_sign[leg];
    leg_can_data_msg->tau_hip[leg] = leg_data_struct[leg].tau_hip * arcdog_hip_side_sign[leg];
    leg_can_data_msg->tau_knee[leg] = leg_data_struct[leg].tau_knee * arcdog_knee_side_sign[leg];
    leg_can_data_msg->tau_prismatic[leg] = leg_data_struct[leg].tau_prismatic;
  }
}

// void leg_can_data_lcm_to_struct(leg_can_data_t *leg_can_data_lcm, LEG_DATA_T *leg_data_structure) {
//   for (int leg = 0; leg < LEG_AMOUNT; leg++) {
//     leg_data_structure[leg].q_abad = leg_can_data_lcm->q_abad[leg];
//     leg_data_structure[leg].q_hip = leg_can_data_lcm->q_hip[leg];
//     leg_data_structure[leg].q_knee = leg_can_data_lcm->q_knee[leg];
//     leg_data_structure[leg].q_prismatic = leg_can_data_lcm->q_prismatic[leg];

//     leg_data_structure[leg].qd_abad = leg_can_data_lcm->qd_abad[leg];
//     leg_data_structure[leg].qd_hip = leg_can_data_lcm->qd_hip[leg];
//     leg_data_structure[leg].qd_knee = leg_can_data_lcm->qd_knee[leg];
//     leg_data_structure[leg].qd_prismatic = leg_can_data_lcm->qd_prismatic[leg];

//     leg_data_structure[leg].tau_abad = leg_can_data_lcm->tau_abad[leg];
//     leg_data_structure[leg].tau_hip = leg_can_data_lcm->tau_hip[leg];
//     leg_data_structure[leg].tau_knee = leg_can_data_lcm->tau_knee[leg];
//     leg_data_structure[leg].tau_prismatic = leg_can_data_lcm->tau_prismatic[leg];
//   }
// }

void usb_send_receive(LEG_COMMAND_T *leg_command, LEG_DATA_T *leg_data, int tty_descriptor,uint8_t leg) {
  if(tty_descriptor == -1 || leg_command == NULL || leg_data==NULL)
  {
    return;
  }

  // for (int i = 0; i < 4; i++) {
  //   host_data_last[i] = host_data[i];
  // }

  // for (int leg = 0; leg < 4; ++leg) {
  //   printf("command leg %d is:\n", leg);
  //   std::cout << leg_command[leg].kd_abad << " " << leg_command[leg].kd_hip << " " << leg_command[leg].kd_knee << " "
  //             << leg_command[leg].kd_prismatic << " " << leg_command[leg].kp_abad << " " << leg_command[leg].kp_hip << " "
  //             << leg_command[leg].kp_knee << " " << leg_command[leg].kp_prismatic << " " << leg_command[leg].q_des_abad << " "
  //             << leg_command[leg].q_des_hip << " " << leg_command[leg].q_des_knee << " " << leg_command[leg].q_des_prismatic << " "
  //             << leg_command[leg].qd_des_abad << " " << leg_command[leg].qd_des_hip << " " << leg_command[leg].qd_des_knee << " "
  //             << leg_command[leg].qd_des_prismatic << " " << leg_command[leg].tau_abad_ff << " " << leg_command[leg].tau_hip_ff << " "
  //             << leg_command[leg].tau_knee_ff << " " << leg_command[leg].tau_prismatic_ff << " ";
  //   printf("\n");
  // }

  // for(int i=0;i<4;i++) {
    leg_command_to_can_command(&leg_command[leg], raw_motor_cmd_t);
    struct_to_array(&raw_motor_cmd_t[0], host_data[leg].dataA);
    struct_to_array(&raw_motor_cmd_t[1], host_data[leg].dataB);
    struct_to_array(&raw_motor_cmd_t[2], host_data[leg].dataC);
    struct_to_array(&raw_motor_cmd_t[3], host_data[leg].dataD);
    host_data[leg].leg = static_cast<uint8_t>(leg);

    // if(leg == 1)
    // {
    //   // ================= [DEBUG START: 发送路径检查] =================
    //   fprintf(stderr, "\n========= LEG %d SEND DEBUG =========\n", leg);
      
    //   // 检查点 1: 物理指令 (确保上层传下来的指令不是全0)
    //   fprintf(stderr,"[1] Physical CMD:\n");
    //   fprintf(stderr,"  ABAD: P_des:%.3f V_des:%.3f Kp:%.3f Kd:%.3f T_ff:%.3f\n", 
    //         leg_command[leg].q_des_abad, leg_command[leg].qd_des_abad, leg_command[leg].kp_abad, leg_command[leg].kd_abad, leg_command[leg].tau_abad_ff);
    //   fprintf(stderr,"  PRIS: P_des:%.3f V_des:%.3f Kp:%.3f Kd:%.3f T_ff:%.3f\n", 
    //         leg_command[leg].q_des_prismatic, leg_command[leg].qd_des_prismatic, leg_command[leg].kp_prismatic, leg_command[leg].kd_prismatic, leg_command[leg].tau_prismatic_ff);

    //   // 检查点 2: 转换后的原始 HEX 值 (确保转换函数工作正常，没有溢出或全0)
    //   // 假设 raw_motor_cmd_t 结构体里有 p_des, v_des 等成员，请根据实际结构体成员名调整
    //   // 这里打印前两个电机作为示例
    //   // fprintf(stderr,"[2] Raw HEX CMD:\n");
    //   // fprintf(stderr,"  Motor 0 (ABAD): P:0x%X V:0x%X Kp:0x%X Kd:0x%X T:0x%X\n", 
    //   //        raw_motor_cmd_t[0].p_des, raw_motor_cmd_t[0].v_des, raw_motor_cmd_t[0].kp, raw_motor_cmd_t[0].kd, raw_motor_cmd_t[0].tau_ff);

    //   // 检查点 3: 最终发送的字节流 (这是真正发给 USB 的数据)
    //   fprintf(stderr,"[3] Host Data Bytes (DataA - ABAD):\n  ");
    //   for(int k=0; k<8; k++) fprintf(stderr,"%02X ", host_data[leg].dataA[k]);
    //   fprintf(stderr,"\n");
    //   fprintf(stderr,"[3] Host Data Bytes (DataA - ABAD):\n  ");
    //   for(int k=0; k<8; k++) fprintf(stderr,"%02X ", host_data[leg].dataB[k]);
    //   fprintf(stderr,"\n");
    //   fprintf(stderr,"[3] Host Data Bytes (DataA - ABAD):\n  ");
    //   for(int k=0; k<8; k++) fprintf(stderr,"%02X ", host_data[leg].dataC[k]);
    //   fprintf(stderr,"\n");
    //   fprintf(stderr,"[3] Host Data Bytes (DataD - Prismatic):\n  ");
    //   for(int k=0; k<8; k++) fprintf(stderr,"%02X ", host_data[leg].dataD[k]);
    //   fprintf(stderr,"\n");
    //   // ================= [DEBUG END] =================

    // }

    usb_send_receive(&host_data[leg],&slave_data[leg],tty_descriptor);

    // if(leg == 1)
    // {
    //   printf("\033[8A");
    //   printf("[6] Slave Data Bytes (DataA - ABAD):\n  ");
    //   for(int k=0; k<8; k++) printf("%02X ", slave_data[leg].dataA[k]);
    //   printf("\n");
    //   printf("[6] Slave Data Bytes (DataB - HIP):\n  ");
    //   for(int k=0; k<8; k++) printf("%02X ", slave_data[leg].dataB[k]);
    //   printf("\n");
    //   printf("[6] Slave Data Bytes (DataC - KNEE):\n  ");
    //   for(int k=0; k<8; k++) printf("%02X ", slave_data[leg].dataC[k]);
    //   printf("\n");
    //   printf("[6] Slave Data Bytes (DataD - PRIS):\n  ");
    //   for(int k=0; k<8; k++) printf("%02X ", slave_data[leg].dataD[k]);
    //   printf("\n");
    // }

    array_to_struct(slave_data[leg].dataA, &raw_motor_data_t[0]) ;
    array_to_struct(slave_data[leg].dataB, &raw_motor_data_t[1]) ;
    array_to_struct(slave_data[leg].dataC, &raw_motor_data_t[2]) ;
    array_to_struct(slave_data[leg].dataD, &raw_motor_data_t[3]);

    // // ================= [DEBUG START: 解包检查] =================
    // // 检查点 7: 检查解包后的原始整数值 (ID, POS, VEL, TOR)
    // // 假设 raw_motor_data_t 有 ID, p, v, t 成员
    // if(leg == 1)
    // {
    //   printf("\033[5A");
    //   printf("[7] Raw HEX Feedback:\n");
    //   printf("  Motor 0 (ABAD): ID:0x%X P:0x%X V:0x%X\n", raw_motor_data_t[0].ID, raw_motor_data_t[0].POS, raw_motor_data_t[0].VEL);
    //   printf("  Motor 1 (HIP): ID:0x%X P:0x%X V:0x%X\n", raw_motor_data_t[1].ID, raw_motor_data_t[1].POS, raw_motor_data_t[1].VEL);
    //   printf("  Motor 2 (KNEE): ID:0x%X P:0x%X V:0x%X\n", raw_motor_data_t[2].ID, raw_motor_data_t[2].POS, raw_motor_data_t[2].VEL);
    //   printf("  Motor 3 (PRIS): ID:0x%X P:0x%X V:0x%X\n", raw_motor_data_t[3].ID, raw_motor_data_t[3].POS, raw_motor_data_t[3].VEL);
    // }
    // // ================= [DEBUG END] =================

    // if (host_data_last[i].dataA[0] != host_data[i].dataA[0] || host_data_last[i].dataA[1] != host_data[i].dataA[1] ||
    //     host_data_last[i].dataA[2] != host_data[i].dataA[2] || host_data_last[i].dataA[3] != host_data[i].dataA[3] ||
    //     host_data_last[i].dataA[4] != host_data[i].dataA[4] || host_data_last[i].dataA[5] != host_data[i].dataA[5] ||
    //     host_data_last[i].dataA[6] != host_data[i].dataA[6] || host_data_last[i].dataA[7] != host_data[i].dataA[7]) {
    //             //      printf("\nID:%X ,ERR:%X, POS:%f, VEL:%f,
    //             //      Tor:%f",raw_motor_data_t[0].ID,raw_motor_data_t[0].ERR,uint_to_float(raw_motor_data_t[0].POS,P_MIN,P_MAX,16),uint_to_float(raw_motor_data_t[0].VEL,V_MIN,V_MAX,12),uint_to_float(raw_motor_data_t[0].T,T_MIN,T_MAX,12));
    //             //      printf("\nID:%X ,ERR:%X, POS:%f, VEL:%f,
    //             //      Tor:%f",raw_motor_data_t[1].ID,raw_motor_data_t[1].ERR,uint_to_float(raw_motor_data_t[1].POS,P_MIN,P_MAX,16),uint_to_float(raw_motor_data_t[1].VEL,V_MIN,V_MAX,12),uint_to_float(raw_motor_data_t[1].T,T_MIN,T_MAX,12));
    //             //      printf("\nID:%X ,ERR:%X, POS:%f, VEL:%f,
    //             //      Tor:%f",raw_motor_data_t[2].ID,raw_motor_data_t[2].ERR,uint_to_float(raw_motor_data_t[2].POS,P_MIN,P_MAX,16),uint_to_float(raw_motor_data_t[2].VEL,V_MIN,V_MAX,12),uint_to_float(raw_motor_data_t[2].T,T_MIN,T_MAX,12));
    //             //      printf("\nID:%X ,ERR:%X, POS:%f, VEL:%f,
    //             //      Tor:%f",raw_motor_data_t[3].ID,raw_motor_data_t[3].ERR,uint_to_float(raw_motor_data_t[3].POS,P_MIN,P_MAX,16),uint_to_float(raw_motor_data_t[3].VEL,V_MIN,V_MAX,12),uint_to_float(raw_motor_data_t[3].T,T_MIN,T_MAX,12));
    //             printf("\nLeg %d, Last Host DATA A: POS:%X, POS:%X", i, host_data_last[i].dataA[0], host_data_last[i].dataA[1]);
    //             printf("\nLeg %d, Last Host DATA B: POS:%X, POS:%X", i, host_data_last[i].dataB[0], host_data_last[i].dataB[1]);
    //             printf("\nLeg %d, Last Host DATA C: POS:%X, POS:%X", i, host_data_last[i].dataC[0], host_data_last[i].dataC[1]);
    //             printf("\nLeg %d, Host DATA A: POS:%X, POS:%X", i, host_data[i].dataA[0], host_data[i].dataA[1]);
    //             printf("\nLeg %d, Host DATA B: POS:%X, POS:%X", i, host_data[i].dataB[0], host_data[i].dataB[1]);
    //             printf("\nLeg %d, Host DATA C: POS:%X, POS:%X", i, host_data[i].dataC[0], host_data[i].dataC[1]);
    // }
    can_data_to_leg_data(raw_motor_data_t, &leg_data[leg]);
  // }

  // ================= [DEBUG START: 最终结果检查] =================
    // 检查点 8: 最终物理数据
    // if(leg == 1)
    // {
    //   printf("\033[6A");
    //   printf("[8] Final Leg Data:\n");
    //   printf("  ABAD: Pos:%.3f Vel:%.3f Tor:%.3f\n", leg_data[leg].q_abad, leg_data[leg].qd_abad, leg_data[leg].tau_abad);
    //   printf("  HIP: Pos:%.3f Vel:%.3f Tor:%.3f\n", leg_data[leg].q_hip, leg_data[leg].qd_hip, leg_data[leg].tau_hip);
    //   printf("  KNEE: Pos:%.3f Vel:%.3f Tor:%.3f\n", leg_data[leg].q_knee, leg_data[leg].qd_knee, leg_data[leg].tau_knee);
    //   printf("  PRIS: Pos:%.3f Vel:%.3f Tor:%.3f\n", leg_data[leg].q_prismatic, leg_data[leg].qd_prismatic, leg_data[leg].tau_prismatic);
    //   printf("======================================\n");
    // }
    // ================= [DEBUG END] =================

  // printf("\nLEGID: 1 ABAD, POS:%f, VEL:%f, Tor:%f",leg_data[1].q_abad,leg_data[1].qd_abad,leg_data[1].tau_abad);
  // printf("\nLEGID: 1 HIP,  POS:%f, VEL:%f, Tor:%f",leg_data[1].q_hip,leg_data[1].qd_hip,leg_data[1].tau_hip);
  // printf("\nLEGID: 1 KNEE, POS:%f, VEL:%f, Tor:%f",leg_data[1].q_knee,leg_data[1].qd_knee,leg_data[1].tau_knee);
}

void usb_send_receive(custom_msgs::msg::JointCommands* leg_command, custom_msgs::msg::JointStates* leg_data, int tty_descriptor){
  leg_can_command_msg_to_struct(leg_command,leg_command_drv_struct);
  for(int i=0;i<4;i++)
  {
  usb_send_receive(leg_command_drv_struct,leg_data_drv_struct,tty_descriptor,i);
  usleep(100);
  }
  leg_can_data_struct_to_msg(leg_data_drv_struct,leg_data);
}


void usb_driver_run(int tty_descriptor, bool motor_mode_flag){
  // do usb cdc can board calculations
  // in here, the driver is good
  pthread_mutex_lock(&usb_mutex);
  if(!motor_mode_flag) {
    for(int i=0;i<4;i++) {
      host_data[i].leg = static_cast<uint8_t>(i);
      usb_send_receive(&host_data[i],&slave_data[i],tty_descriptor);
      array_to_struct(slave_data[i].dataA, &raw_motor_data_t[0]) ;
      array_to_struct(slave_data[i].dataB, &raw_motor_data_t[1]) ;
      array_to_struct(slave_data[i].dataC, &raw_motor_data_t[2]) ;
      array_to_struct(slave_data[i].dataD, &raw_motor_data_t[3]) ;
      can_data_to_leg_data(raw_motor_data_t, &leg_data_drv_struct[i]) ;
    }
    leg_can_data_struct_to_msg(leg_data_drv_struct,&joint_states_drv_msgtype);
  }else {
    usb_send_receive(&joint_commands_drv_msgtype,&joint_states_drv_msgtype,tty_descriptor);
  }

  pthread_mutex_unlock(&usb_mutex);
}


void usb_driver_run(int tty_descriptor, bool motor_mode_flag, uint64_t current_iteration){
    // std::cerr<<"usb_driver_run started!!!"<<std::endl;
  // do usb cdc can board calculations
  // in here, the driver is good
  pthread_mutex_lock(&usb_mutex);
  if(!motor_mode_flag) {
    for(int i=0;i<4;i++) {
      host_data[i].leg = static_cast<uint8_t>(i);
      usb_send_receive(&host_data[i],&slave_data[i],tty_descriptor);
      array_to_struct(slave_data[i].dataA, &raw_motor_data_t[0]) ;
      array_to_struct(slave_data[i].dataB, &raw_motor_data_t[1]) ;
      array_to_struct(slave_data[i].dataC, &raw_motor_data_t[2]) ;
      array_to_struct(slave_data[i].dataD, &raw_motor_data_t[3]) ;
      can_data_to_leg_data(raw_motor_data_t, &leg_data_drv_struct[i]) ;
    }
    leg_can_data_struct_to_msg(leg_data_drv_struct,&joint_states_drv_msgtype);
  }else {
    usb_send_receive(&joint_commands_drv_msgtype,&joint_states_drv_msgtype,tty_descriptor);
  }
  pthread_mutex_unlock(&usb_mutex);
  if (current_iteration == 1500) {
      // std::cerr<<"usb_driver_run started!!!"<<std::endl;
      // std::cerr<<"usb_driver_run started!!!"<<std::endl;
      // std::cerr<<"usb_driver_run started!!!"<<std::endl;
    for (int i = 0; i < 4; ++i) {
      arcdog_abad_offset[i] = 2 * arcdog_abad_offset[i] - joint_states_drv_msgtype.q_abad[i];
      arcdog_hip_offset[i] = 2 * arcdog_hip_offset[i] - joint_states_drv_msgtype.q_hip[i];
      arcdog_knee_offset[i] = 2 * arcdog_knee_offset[i] - joint_states_drv_msgtype.q_knee[i];
      arcdog_prismatic_offset[i] = 2 * arcdog_prismatic_offset[i] - joint_states_drv_msgtype.q_prismatic[i];
    }
    std::cerr<<"Finish setting offset for arcdog's motor! NO ACTIVE ON ARCDOG MINI"<<std::endl;
  }
}


LEG_DATA_T* get_leg_data_struct() { return leg_data_drv_struct;
}


LEG_COMMAND_T* get_leg_command_struct() {
  return leg_command_drv_struct;
}

// leg_can_data_t* get_leg_data_lcmtype() { return &leg_data_drv_lcmtype;
// }


// leg_can_command_t* get_leg_command_lcmtype() {
//   return &leg_command_drv_lcmtype;
// }


custom_msgs::msg::JointCommands* get_joint_commands_msgtype() {
    return &joint_commands_drv_msgtype;
}

custom_msgs::msg::JointStates* get_joint_states_msgtype() {
    return &joint_states_drv_msgtype;
}


CAN_HOST_DATA* get_can_host_data() {
  return host_data;
}

void usb_driver_end(int tty_descriptor){
  tty_cleanup(tty_descriptor);
}
