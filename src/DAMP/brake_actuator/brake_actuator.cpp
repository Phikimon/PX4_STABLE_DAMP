//STD
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
//GPIO
#include <drivers/boards/px4fmu-v2/board_config.h>
#include <drivers/drv_gpio.h>
#include "stm32.h"
#include "board_config.h"
#include <arch/board/board.h>
//Topics
#include <uORB/uORB.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/output_pwm.h>
//DAMP
#include <DAMP/lib/common.h>
#include <DAMP/lib/switch.h>

#define MY_NDEBUG

extern "C"
{
    __EXPORT int brake_actuator_main(int argc, char *argv[]);
}

//This is compile-time constant
const int BRAKE_SWITCH_CHANNEL_NUMBER = 5; //< value = real number - 1
                                           //< (because numeraion is from 0)
const int BRAKE_MOTOR_GPIO_CW  = GPIO_GPIO3_OUTPUT;
const int BRAKE_MOTOR_GPIO_CCW = GPIO_GPIO4_OUTPUT;

int brake_actuator_main(int argc, char *argv[])
{
    //Subscribe on rc input data
    px4_pollfd_struct_t input_rc_pollfd = {};
    int                 input_rc_sub_fd = orb_subscribe(ORB_ID(input_rc));
    input_rc_pollfd.fd     = input_rc_sub_fd;
    input_rc_pollfd.events = POLLIN;
    orb_set_interval(input_rc_sub_fd, 100);

    //Init GPIO pin
    stm32_configgpio(BRAKE_MOTOR_GPIO_CW);
    stm32_configgpio(BRAKE_MOTOR_GPIO_CCW);

    int error_counter = 0;
    while (true)
    {
        //Get data from rc and adc
        int poll_ret = px4_poll(&input_rc_pollfd, 1, 1000);

        if (poll_ret > 0)
        {
            if (input_rc_pollfd.revents & POLLIN)
            {
                //If both polls are successful
                //Put data from topics into structures
                input_rc_s raw_data = {};
                orb_copy(ORB_ID(input_rc), input_rc_sub_fd, &raw_data);

                switch_state_t state = SS_OFF;
                state = get_switch_state(raw_data.values[BRAKE_SWITCH_CHANNEL_NUMBER],
                                         BRAKE_SWITCH_CHANNEL_NUMBER);
#ifndef NDEBUG
                PX4_INFO("Switch state = %s", (state ==  SS_ON) ? "SS_ON" : "SS_OFF");
#endif
                if (state == SS_ON)
                {
                    stm32_gpiowrite(BRAKE_MOTOR_GPIO_CW,  false);
                    stm32_gpiowrite(BRAKE_MOTOR_GPIO_CCW, true);
                } else
                {
                    stm32_gpiowrite(BRAKE_MOTOR_GPIO_CW,  true);
                    stm32_gpiowrite(BRAKE_MOTOR_GPIO_CCW, false);
                };
            }
        } else if (poll_ret == 0)
        {
            PX4_ERR("Got no data within a second");
        } else
        { // poll_ret < 0
            if (error_counter < 10 || error_counter % 50 == 0)
            {
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }
            error_counter++;
        };
    }
    return OK;
}
