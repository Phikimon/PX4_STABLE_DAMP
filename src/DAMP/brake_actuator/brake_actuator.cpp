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
//PWM
#include <drivers/drv_pwm_output.h>
//Topics
#include <uORB/uORB.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/output_pwm.h>
//DAMP
#include <DAMP/common.h>

#define MY_NDEBUG

extern "C"
{
    __EXPORT int brake_acturator_main(int argc, char *argv[]);
}

//This is compile-time constant
const int BRAKE_SWITCH_CHANNEL_NUMBER = 5; //< value = real number - 1
                                           //< (because numeraion is from 0)

int brake_acturator_main(int argc, char *argv[])
{
    return 0;
}
