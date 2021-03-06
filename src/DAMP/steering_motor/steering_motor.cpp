//STD
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <errno.h>
//DAMP
#include <DAMP/lib/common.h>
#include <DAMP/pid_cont/pid_cont.h>
//GPIO
#include <drivers/boards/px4fmu-v2/board_config.h>
#include <drivers/drv_gpio.h>
#include "stm32.h"
#include "board_config.h"
#include <arch/board/board.h>
//TOPICS(ADC+RC+PWM)
#include <uORB/uORB.h>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/output_pwm.h>
#include <drivers/drv_adc.h>
#include <stm32_adc.h>
#include <drivers/drv_pwm_output.h>

#undef NDEBUG
//#define SAFE_STEERING

const int YAW_CHANNEL_NUMBER = 3;
const int MOTOR_IN_PWM_PIN = 1;
const int MOTOR_IN_GPIO_CW   = GPIO_GPIO0_OUTPUT;
const int MOTOR_IN_GPIO_CCW  = GPIO_GPIO1_OUTPUT;
//PID constants
const float SENS_BIAS     = 0;
const float BIAS          = 0;
const float TIMESTAMP_AVG = 0.2f;

enum motor_direction_t
{
    MD_POSITIVE = 0, //Nevermind clockwise-ness
    MD_NEGATIVE = 1,
    MD_DEFAULT
};

enum motor_movement_t
{
    MM_STOP = 0,
    MM_MOVE = 1,
    MM_DEFAULT
};

extern "C"
{
    __EXPORT int steering_motor_main(int argc, char *argv[]);
}

inline void get_motor_movement_and_direction(int32_t actual_angle_perc,
                                             int32_t desired_angle_perc,
                                             int32_t dead_zone_perc,
                                             motor_movement_t*  motor_movement,
                                             motor_direction_t* motor_direction,
                                             float* motor_power_perc,
                                             pid_cont_t* pid_var);

inline void transmit_to_driver(motor_movement_t  motor_movement,
                               motor_direction_t motor_direction,
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
    stm32_configgpio(MOTOR_IN_GPIO_CW);
    stm32_configgpio(MOTOR_IN_GPIO_CCW);

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
    pid_cont_t pid = {SENS_BIAS,
                      get_float_param("PROP_SENS"),
                      get_float_param("DIF_SENS"),
                      BIAS,
                      TIMESTAMP_AVG};

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
                float current_adc_value = get_6v6_adc_value(adc_fd);
                int current_angle_perc = float_value_to_percents(current_adc_value,
                                                                 "POT_MIN",
                                                                 "POT_MAX",
                                                                 "POT_TRIM");


                float yaw_value = input_rc_data.values[YAW_CHANNEL_NUMBER];
                static char yaw_min_param_name[sizeof("RCx_MIN")] = {};
                static char yaw_max_param_name[sizeof("RCx_MAX")] = {};
                static char yaw_ref_param_name[sizeof("RCx_TRIM")] = {};
                static bool flag = true;
                if (flag)
                {                                         // Numeration from 1  V
                    sprintf(yaw_min_param_name, "RC%d_MIN",  YAW_CHANNEL_NUMBER + 1);
                    sprintf(yaw_max_param_name, "RC%d_MAX",  YAW_CHANNEL_NUMBER + 1);
                    sprintf(yaw_ref_param_name, "RC%d_TRIM", YAW_CHANNEL_NUMBER + 1);
                    flag = false;
                }
                int desired_angle_perc = float_value_to_percents(yaw_value,
                                                                 yaw_min_param_name,
                                                                 yaw_max_param_name,
                                                                 yaw_ref_param_name);
                int32_t dead_zone_perc = (int32_t)get_float_param("DEAD_ZONE");
#ifndef NDEBUG
                pid.set_sens_prop(get_float_param("PROP_SENS"));
                pid.set_sens_dif(get_float_param("DIF_SENS"));

                PX4_INFO("Current angle = %d%%",   current_angle_perc);
                PX4_INFO("Desired angle = %d%%",   desired_angle_perc);
                PX4_INFO("Angle difference= %d%%", desired_angle_perc - current_angle_perc);
                PX4_INFO("Dead zone = %d%%",       dead_zone_perc);
                PX4_INFO("PID_prop = %d",          pid.get_sens_prop());
                PX4_INFO("PID_diff = %d",          pid.get_sens_dif());
#endif
                //Decide if motor has to move and in which direction
                enum motor_movement_t  motor_movement   = MM_DEFAULT;
                enum motor_direction_t motor_direction  = MD_DEFAULT;
                float               motor_power_perc = 0;
                get_motor_movement_and_direction(current_angle_perc,
                                                 desired_angle_perc,
                                                 dead_zone_perc,
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
#ifdef SAFE_STEERING
                { //< Brackets to incapsulate
                    float pot_max = get_float_param("POT_MAX");
                    float pot_min = get_float_param("POT_MIN");
                    float pot_rng = abs(pot_max - pot_min);
                    const pot_gap = pot_rng * 0.05;
                    if ( (abs(current_adc_value - pot_max) < pot_gap) &&
                         (motor_direction == MD_POSITIVE)             &&
                         (motor_movement  == MM_MOVE)                 )
                    {
                        PX4_INFO("MAX LIMIT REACHED! STOP.");
                        motor_movement  = MM_STOP;
                        motor_direction = MD_DEFAULT;
                    }

                    if ( (abs(current_adc_value - pot_min) < pot_gap) &&
                         (motor_direction == MD_NEGATIVE)             &&
                         (motor_movement  == MM_MOVE)                 )
                    {
                        PX4_INFO("MIN LIMIT REACHED! STOP.");
                        motor_movement  = MM_STOP;
                        motor_direction = MD_DEFAULT;
                    }
                }
#endif //SAFE_STEERING
                //Transmit this informaion to steering motor driver
                //Motor PWM pin is considered to be connected to the +5V
                transmit_to_driver(motor_movement,
                                   motor_direction,
                                   motor_power_perc,
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
                                      int32_t dead_zone_perc,
                                      motor_movement_t*  motor_movement,
                                      motor_direction_t* motor_direction,
                                      float* motor_power_perc,
                                      pid_cont_t* pid_var)
{
    const int PERCENTAGE_MAX  = 100;
    int angle_error_perc = desired_angle_perc - actual_angle_perc;
    *motor_power_perc    = abs(pid_var->next(angle_error_perc));
#ifndef NDEBUG
    PX4_INFO("Motor power = %f", (double)*motor_power_perc);
#endif //~NDEBUG
    *motor_power_perc    = (*motor_power_perc > PERCENTAGE_MAX) ?
                           PERCENTAGE_MAX :
                           *motor_power_perc;

    if (abs(angle_error_perc) < dead_zone_perc)
    {
        *motor_movement  = MM_STOP;
        *motor_direction = MD_DEFAULT;
    } else
    {
        *motor_movement   = MM_MOVE;
        *motor_direction = (angle_error_perc > 0) ?
                           MD_POSITIVE :
                           MD_NEGATIVE;
    }
}

void transmit_to_driver(motor_movement_t  motor_movement,
                        motor_direction_t motor_direction,
                        float motor_power_perc,
                        int output_pwm_fd)
{
    switch (motor_movement)
    {
        case MM_STOP:
        {
            stm32_gpiowrite(MOTOR_IN_GPIO_CW,  true);
            stm32_gpiowrite(MOTOR_IN_GPIO_CCW, true);
        }; break;
        case MM_MOVE:
        {
            switch (motor_direction)
            {
                case MD_POSITIVE:
                {
                    stm32_gpiowrite(MOTOR_IN_GPIO_CW, false);
                    stm32_gpiowrite(MOTOR_IN_GPIO_CCW, true);
                }; break;
                case MD_NEGATIVE:
                {
                    stm32_gpiowrite(MOTOR_IN_GPIO_CW, true);
                    stm32_gpiowrite(MOTOR_IN_GPIO_CCW, false);
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
                        PWM_SERVO_SET(MOTOR_IN_PWM_PIN),
                        perc_to_pwm_val(motor_power_perc));
    if (ret != OK)
    {
        PX4_ERR("PWM_SET(%d)", MOTOR_IN_PWM_PIN);
    }
}
