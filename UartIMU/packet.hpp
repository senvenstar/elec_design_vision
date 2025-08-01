#ifndef SENOSR_IMU_UARTIMU_PACKET_H
#define SENOSR_IMU_UARTIMU_PACKET_H

/**
 * @brief 下行控制指令
 *
 * @note  包含角度信息和供预测的角速度信息
 */
#define GIMAdvv_CMD_ID 0x1026
typedef struct __attribute__((packed))
{
    uint8_t valid;
    float yaw_error;
    float pitch_error;
    float yaw_speed;
    float pitch_speed;
} advv_detection_t;


/**
 * @brief 下行心跳包
 *
 * @note  自瞄状态信息 10Hz
 */
#define STS_CMD_ID 0x0500
typedef struct __attribute__((packed))
{
    uint8_t mode;
} detection_sts_t;

/**
 * @brief IMU位姿数据
 * 
 */
#define CMD_MCU_DATA 0x1027
typedef struct __attribute__((packed))
{
    float cur_yaw;
    float cur_pitch;
} pc_mcu_data_t;

/**
 * @brief 赛场信息
 * 
 */
#define CMD_ROBOT_DATA 0x1022
typedef struct __attribute__((packed)) 
{ 
    uint16_t red_1_robot_HP;   
    uint16_t red_2_robot_HP;   
    uint16_t red_3_robot_HP;   
    uint16_t red_4_robot_HP;   
    uint16_t red_5_robot_HP;   
    uint16_t red_7_robot_HP;   
    uint16_t red_outpost_HP; 
    uint16_t red_base_HP;   
    uint16_t blue_1_robot_HP;   
    uint16_t blue_2_robot_HP;   
    uint16_t blue_3_robot_HP;   
    uint16_t blue_4_robot_HP;   
    uint16_t blue_5_robot_HP;   
    uint16_t blue_7_robot_HP;   
    uint16_t blue_outpost_HP; 
    uint16_t blue_base_HP;
    uint8_t robot_id;
} robot_data_t;

#endif //SENOSR_IMU_UARTIMU_PACKET_
