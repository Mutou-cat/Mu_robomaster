#include "CAN_transmit_task.h"
#include "cmsis_os.h"
#include "CAN_receive.h"


void CAN_transmit_task()
{
    osDelay(400);
    while(1)
    {
        CAN_transmit_rc_data_1();
        CAN_transmit_rc_data_2();
        osDelay(10);
    }
}
