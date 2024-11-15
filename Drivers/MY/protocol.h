
#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/
#include "stm32f4xx.h"
#include "myPID.h"

#ifdef _cplusplus
extern "C" {
#endif   

/* 数据接收缓冲区大小 */
#define PROT_FRAME_LEN_RECV  128

/* 校验数据的长度 */
#define PROT_FRAME_LEN_CHECKSUM    1

/* 数据头结构体 */
typedef __packed struct
{ 
  uint16_t head;    // 包头
	uint8_t head_t;
  uint8_t ch;       // 通道
  uint8_t len;     // 包长度
  uint8_t cmd;      // 命令
  
}packet_head_t;

/* 联合体（方便数据转换） */
typedef union
{
  float f;
  int i;
}type_cast_t;

#define FRAME_HEADER     0x4C4246    // 帧头

/* 通道宏定义 */
#define CURVES_CH0      0x00
#define CURVES_CH1      0x01
#define CURVES_CH2      0x02
#define CURVES_CH3      0x03
#define CURVES_CH4      0x04
#define CURVES_CH5      0x05
#define CURVES_CH6      0x06
#define CURVES_CH7      0x07
#define CURVES_CH8      0x08

/* 指令(下位机 -> 上位机) */
#define SEND_TARGET_CMD      0x01     // 发送上位机通道的目标值
#define SEND_FACT_CMD        0x02     // 发送通道实际值
#define SEND_P_I_D_CMD       0x03     // 发送 PID 值（同步上位机显示的值）
#define SEND_START_CMD       0x04     // 发送启动指令（同步上位机按钮状态）
#define SEND_STOP_CMD        0x05     // 发送停止指令（同步上位机按钮状态）
#define SEND_PERIOD_CMD      0x06     // 发送周期（同步上位机显示的值）

/* 指令(上位机 -> 下位机) */
#define SET_P_I_D_CMD        0x10     // 设置 PID 值
#define SET_TARGET_CMD       0x11     // 设置目标值
#define START_CMD            0x12     // 启动指令
#define STOP_CMD             0x13     // 停止指令
#define RESET_CMD            0x14     // 复位指令
#define SET_AIRWAY_CMD       0x15     // 设置气道
#define SEND_PRESSURE_VALUE  0x16     // 发送气压

/* 空指令 */
#define CMD_NONE             0xFF     // 空指令

/* 索引值宏定义 */
//#define HEAD_INDEX_VAL       0x2u     // 包头索引值（3字节）
#define CHX_INDEX_VAL        0x3u     // 通道索引值（1字节）
#define LEN_INDEX_VAL        0x4u     // 包长索引值（1字节）
#define CMD_INDEX_VAL        0x5u     // 命令索引值（1字节）

#define EXCHANGE_H_L_BIT(data)      ((((data) << 24) & 0xFF000000) |\
                                     (((data) <<  8) & 0x00FF0000) |\
                                     (((data) >>  8) & 0x0000FF00) |\
                                     (((data) >> 24) & 0x000000FF))     // 交换高低字节

#define COMPOUND_32BIT(data)        (((*(data-0) << 24) & 0xFF000000) |\
                                     ((*(data-1) << 16) & 0x00FF0000) |\
                                     ((*(data-2) <<  8) & 0x0000FF00) |\
                                     ((*(data-3) <<  0) & 0x000000FF))      // 合成为一个字
  

#define COMPOUND_16BIT(data)        (((*(data-0) << 8) & 0xFF00) |\
                                     ((*(data-1) << 0) & 0x00FF))
/**
 * @brief   接收数据处理
 * @param   *data:  要计算的数据的数组.
 * @param   data_len: 数据的大小
 * @return  void.
 */
void protocol_data_recv(uint8_t *data, uint16_t data_len);

/**
 * @brief   初始化接收协议
 * @param   void
 * @return  初始化结果.
 */
int32_t protocol_init(void);

/**
 * @brief   接收的数据处理
 * @param   void
 * @return  -1：没有找到一个正确的命令.
 */
int8_t receiving_process(void);

/**
  * @brief 设置上位机的值
  * @param cmd：命令
  * @param ch: 曲线通道
  * @param data：参数指针
  * @param num：参数个数
  * @retval 无
  */
void set_computer_value(uint8_t cmd, uint8_t ch, void *data, uint8_t num);
void send_computer_pressure_value(uint8_t cmd, uint8_t ch, void *pressure, uint8_t num);
void pressure_test(	uint8_t data[8], int i, int j, char pressure_id_1, char pressure_id_2);//参数：气压数组data，循环次数i,循环次数j,肌肉号码id1,肌肉号码id2
void set_p_i_d(PID_TypeDef *pid, float p, float i, float d);
//void set_pid_target(uint8_t armnum);
#ifdef _cplusplus
}
#endif   

#endif
