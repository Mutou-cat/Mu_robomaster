#include "CAN_transmit_task.h"
#include "cmsis_os.h"
#include "CAN_receive.h"


void CAN_transmit_task()
{
    osDelay(400);
    while(1)
    {
        CAN_transmit_yaw_relative_data();
        osDelay(10);
    }


}