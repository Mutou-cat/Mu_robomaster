/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"
#include "remote_control.h"

#include "detect_task.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
fp32 yaw_relative_angel;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }


void CAN_receive_gimbal_data(uint8_t*rx_data)
{
  *(uint32_t*)&yaw_relative_angel = (rx_data[3] << 24) | (rx_data[2] << 16) 
                                     | (rx_data[1] << 8) | rx_data[0];
}
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
�������, 0:���̵��1 3508���,  1:���̵��2 3508���,2:���̵��3 3508���,3:���̵��4 3508���

û�������ĸ�
4:yaw��̨��� 6020���; 5:pitch��̨��� 6020���; 6:������� 2006���*/
static motor_measure_t motor_chassis[7];

static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  rc_tx_message_1;
static uint8_t              rc_can_send_data_1[8];
static CAN_TxHeaderTypeDef  rc_tx_message_2;
static uint8_t              rc_can_send_data_2[8];
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    //can1������̨������
	  if(hcan==&hcan1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

        switch (rx_header.StdId)
        {
            case CAN_GIMBAL_BOARD_ID:
            {
                //������̨���ݺ���
                CAN_receive_gimbal_data(rx_data);
                break;
            }

            default:
            {
                break;
            }

        }
    }
    //can2���Ե��̵��
    else if(hcan==&hcan2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

        switch (rx_header.StdId)
        {
            case CAN_3508_M1_ID:
            case CAN_3508_M2_ID:
            case CAN_3508_M3_ID:
            case CAN_3508_M4_ID:
            {
                static uint8_t i = 0;
                //get motor id
                i = rx_header.StdId - CAN_3508_M1_ID;
                get_motor_measure(&motor_chassis[i], rx_data);
                detect_hook(CHASSIS_MOTOR1_TOE + i);
                break;
            }
    

            default:
            {
                break;
            }
        }


    }
    
}

//ң����dbusԭʼ����18�ֽڣ���Ч�ֽ�Ϊǰ16�ֽڣ��ֳ�����8�ֽڷ���
void CAN_transmit_rc_data_1()
{
    uint32_t send_mail_box;
    rc_tx_message_1.StdId = CAN_CHASSIS_BOARD_ID;
    rc_tx_message_1.IDE = CAN_ID_STD;
    rc_tx_message_1.RTR = CAN_RTR_DATA;
    rc_tx_message_1.DLC = 0x08;
    rc_can_send_data_1[0]=sbus_rx_data[0];
    rc_can_send_data_1[1]=sbus_rx_data[1];
    rc_can_send_data_1[2]=sbus_rx_data[2];
    rc_can_send_data_1[3]=sbus_rx_data[3];
    rc_can_send_data_1[4]=sbus_rx_data[4];
    rc_can_send_data_1[5]=sbus_rx_data[5];
    rc_can_send_data_1[6]=sbus_rx_data[6];
    rc_can_send_data_1[7]=sbus_rx_data[7];

    HAL_CAN_AddTxMessage(&CHASSIS_BOARD_2_GIMBAL_BOARD, &rc_tx_message_1, rc_can_send_data_1, &send_mail_box);

}
void CAN_transmit_rc_data_2()
{
    uint32_t send_mail_box;
    rc_tx_message_2.StdId = CAN_CHASSIS_BOARD_ID+0x0F0;
    rc_tx_message_2.IDE = CAN_ID_STD;
    rc_tx_message_2.RTR = CAN_RTR_DATA;
    rc_tx_message_2.DLC = 0x08;
    rc_can_send_data_2[0]=sbus_rx_data[8];
    rc_can_send_data_2[1]=sbus_rx_data[9];
    rc_can_send_data_2[2]=sbus_rx_data[10];
    rc_can_send_data_2[3]=sbus_rx_data[11];
    rc_can_send_data_2[4]=sbus_rx_data[12];
    rc_can_send_data_2[5]=sbus_rx_data[13];
    rc_can_send_data_2[6]=sbus_rx_data[14];
    rc_can_send_data_2[7]=sbus_rx_data[15];

    HAL_CAN_AddTxMessage(&CHASSIS_BOARD_2_GIMBAL_BOARD, &rc_tx_message_2, rc_can_send_data_2, &send_mail_box);

}


/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}



/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

const fp32* get_yaw_relative_angle_point(void)
{
    return &yaw_relative_angel;
}