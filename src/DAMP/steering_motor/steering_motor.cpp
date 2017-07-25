#include <drivers/drv_adc.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <DAMP/common.h>

#include <uORB/uORB.h>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/output_pwm.h>
#include <drivers/boards/px4fmu-v2/board_config.h>
#include <drivers/drv_gpio.h>
#include <errno.h>
#include <stm32_adc.h>

#include "stm32.h"
#include "board_config.h"
#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>

#include <DAMP/pid_cont/pid_cont.h>

#undef NDEBUG

//Mathew, pay attention!
#define TODOTODO 1

//Fundamental constants
const int YAW_CHANNEL_NUMBER = 3;
const int AUX_CHANNEL_NUMBER = 4;

//TODO: move to params
const int STEER_PWM_CH = 1;

enum MotorDirection
{
    MD_POSITIVE = 0, //Nevermind clockwise-ness
    MD_NEGATIVE = 1,
    MD_DEFAULT
};

enum MotorMovement
{
    MM_STOP = 0,
    MM_MOVE = 1,
    MM_DEFAULT
};

extern "C"
{
    __EXPORT int steering_motor_main(int argc, char *argv[]);
}

//Inline is used for the purpose of
//lessen number of calls
inline void get_motor_movement_and_direction(int32_t actual_angle_perc,
                                             int32_t desired_angle_perc,
                                             int32_t dead_zone,
                                             MotorMovement*  motor_movement,
                                             MotorDirection* motor_direction,
                                             float* motor_power_perc,
                                             pid_cont_t* pid_var);

inline void transmit_to_driver(MotorMovement  motor_movement,
                               MotorDirection motor_direction,
                               float motor_power_perc,
                               int output_pwm_fd);

int steering_motor_main(int argc, char *argv[])
{
    //Subscribe on rc input data
    int input_rc_sub_fd = orb_subscribe(ORB_ID(input_rc));
    orb_set_interval(input_rc_sub_fd, 100);
    px4_pollfd_struct_t input_rc_pollfd = {};
    input_rc_pollfd.fd     = input_rc_sub_fd,
    input_rc_pollfd.events = POLLIN;

    //Init GPIO pins
    stm32_configgpio(GPIO_GPIO0_OUTPUT); //< Motor IN CW
    stm32_configgpio(GPIO_GPIO1_OUTPUT); //< Motor IN CCW

    int adc_fd = open(ADC0_DEVICE_PATH, O_RDONLY);
    if (adc_fd < 0)
    {
        PX4_ERR("Can't open ADC device");
        exit(1);
    }

    const char *output_pwm_dev = PWM_OUTPUT0_DEVICE_PATH;
    int output_pwm_fd = px4_open(output_pwm_dev, 0);
    if (output_pwm_fd < 0)
    {
        PX4_ERR("Can't open %s", output_pwm_dev);
        exit(1);
    }

    //pid init
    pid_cont_t pid = {0, 7, 2, 0, 0.2f};

    //Main cycle
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
                struct input_rc_s input_rc_data = {};
                orb_copy(ORB_ID(input_rc), input_rc_sub_fd, &input_rc_data);

                //Potentiometer angle
                int current_angle_perc = adc_to_percents(get_6v6_adc_value(adc_fd));
                int desired_angle_perc = yaw_to_percents(input_rc_data.values[YAW_CHANNEL_NUMBER]);
                int dead_zone          = rc5_to_percents(input_rc_data.values[AUX_CHANNEL_NUMBER]);

#ifndef NDEBUG
                PX4_INFO("Current angle = %d%%",     current_angle_perc);
                PX4_INFO("Desired angle = %d%%",     desired_angle_perc);
                PX4_INFO("Dead zone = %d%%",         dead_zone);
#endif
                //Decide if motor has to move and in which direction
                enum MotorMovement  motor_movement  = MM_DEFAULT;
                enum MotorDirection motor_direction = MD_DEFAULT;
                float motor_power_perc;
                get_motor_movement_and_direction(current_angle_perc,
                                                 desired_angle_perc,
                                                 dead_zone,
                                                 &motor_movement,
                                                 &motor_direction,
                                                 &motor_power_perc,
                                                 &pid);
#ifndef NDEBUG
                switch (motor_movement)
                {
                    case MM_STOP:
                    {
                        PX4_INFO("Motor movement = MM_STOP");
                    }; break;
                    case MM_MOVE:
                    {
                        PX4_INFO("Motor movement = MM_MOVE");
                    }; break;
                    case MM_DEFAULT:
                    {
                        PX4_INFO("Motor movement = MM_DEFAULT");
                    }; break;
                }
                switch (motor_direction)
                {
                    case MD_POSITIVE:
                    {
                        PX4_INFO("Motor direction = MD_POSITIVE\n");
                    }; break;
                    case MD_NEGATIVE:
                    {
                        PX4_INFO("Motor direction = MD_NEGATIVE\n");
                    }; break;
                    case MD_DEFAULT:
                    {
                        PX4_INFO("Motor direction = MD_DEFAULT\n");
                    }; break;
                };
#endif
                //Transmit this informaion to steering motor driver
                //Motor PWM pin is considered to be connected to the +5V
                transmit_to_driver(motor_movement,
                                   motor_direction,
                                   TODOTODO,
                                   output_pwm_fd);
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

void get_motor_movement_and_direction(int32_t actual_angle_perc,
                                      int32_t desired_angle_perc,
                                      int32_t dead_zone,
                                      MotorMovement*  motor_movement,
                                      MotorDirection* motor_direction,
                                      float* motor_power_perc,
                                      pid_cont_t* pid_var)
{
     *motor_power_perc = abs(pid_var->next(desired_angle_perc - actual_angle_perc));
     *motor_power_perc = (*motor_power_perc > 100) ? 100 : *motor_power_perc;

    if (abs(actual_angle_perc - desired_angle_perc) < dead_zone)
    {
        *motor_movement  = MM_STOP;
        *motor_direction = MD_DEFAULT;
    } else
    {
        *motor_movement   = MM_MOVE;
        *motor_direction = (actual_angle_perc - desired_angle_perc > 0) ?
                           MD_POSITIVE :
                           MD_NEGATIVE;
    }
}

void transmit_to_driver(MotorMovement  motor_movement,
                        MotorDirection motor_direction,
                        float motor_power_perc,
                        int output_pwm_fd)
{
    switch (motor_movement)
    {
        case MM_STOP:
        {
            stm32_gpiowrite(GPIO_GPIO0_OUTPUT, false);
            stm32_gpiowrite(GPIO_GPIO1_OUTPUT, false);
        }; break;
        case MM_MOVE:
        {
            switch (motor_direction)
            {
                case MD_POSITIVE:
                {
                    stm32_gpiowrite(GPIO_GPIO0_OUTPUT, false);
                    stm32_gpiowrite(GPIO_GPIO1_OUTPUT, true);
                }; break;
                case MD_NEGATIVE:
                {
                    stm32_gpiowrite(GPIO_GPIO0_OUTPUT, true);
                    stm32_gpiowrite(GPIO_GPIO1_OUTPUT, false);
                }; break;
                case MD_DEFAULT:
                {
                    PX4_ERR("Motor direction equals MD_DEFAULT,"
                            " while motor movement equals MM_MOVE."
                            " Aborting");
                    exit(1);
                }
            }
        }; break;
        case MM_DEFAULT:
        {
            PX4_ERR("Motor movement equals MM_DEFAULT. Aborting");
            exit(1);
        }; break;
    }
    int ret = px4_ioctl(output_pwm_fd,
                        PWM_SERVO_SET(STEER_PWM_CH),
                        perc_to_pwm_val(motor_power_perc));
    if (ret != OK)
    {
        PX4_ERR("PWM_SET(%d)", STEER_PWM_CH);
    }
}
