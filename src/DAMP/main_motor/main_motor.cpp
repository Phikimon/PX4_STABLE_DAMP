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
#include <DAMP/lib/common.h>

#define MY_NDEBUG

extern "C"
{
    __EXPORT int main_motor_main(int argc, char *argv[]);
}

enum main_motor_direction_t
{
    MMD_FORWARD = 0,
    MMD_BACK    = 1,
    MMD_DEFAULT
};

inline uint16_t throttle_input_rc_to_pwm(int input_rc_val);
inline void     publish_pwm             (int pwm_fd, int pin_number, int value);

const int         THROTTLE_CHANNEL_NUMBER = 2; //< value = real number - 1
const int DIRECTION_SWITCH_CHANNEL_NUMBER = 4; //< (because numeraion is from 0)
const int         THROTTLE_PWM_PIN_NUMBER = 0;
const int MOTOR_DIRECTION_GPIO_PIN_NUMBER = GPIO_GPIO2_OUTPUT;

int main_motor_main(int argc, char *argv[])
{
    static uint32_t pwm_disarmed = get_uint32_t_param("PWM_DISARMED");
    PX4_INFO("PWM_DISARMED = %d", pwm_disarmed);

    //Init GPIO pin
    stm32_configgpio(MOTOR_DIRECTION_GPIO_PIN_NUMBER);

    //Subscribe on 'input_rc' topic
    int                 input_rc_sub_fd = orb_subscribe(ORB_ID(input_rc));
    px4_pollfd_struct_t input_rc_pollfd = {};
    input_rc_pollfd.fd     = input_rc_sub_fd,
    input_rc_pollfd.events = POLLIN;
    orb_set_interval(input_rc_sub_fd, 200);

    //Open PWM device
    const char *output_pwm_dev = PWM_OUTPUT0_DEVICE_PATH;
    int output_pwm_fd = px4_open(output_pwm_dev, 0);
    if (output_pwm_fd < 0)
    {
        PX4_ERR("Can't open %s", output_pwm_dev);
        return 1;
    }

    //Publish safe PWM value
    publish_pwm(output_pwm_fd,
                THROTTLE_PWM_PIN_NUMBER,
                pwm_disarmed);

    int error_counter = 0;
    while (true)
    {
        //Get data from topic
        int input_rc_poll_ret = px4_poll(&input_rc_pollfd, 1, 1000);

        if (input_rc_poll_ret > 0)
        {
            if (input_rc_pollfd.revents & POLLIN)
            {
                //Get data from input
                struct input_rc_s raw = {};
                orb_copy(ORB_ID(input_rc), input_rc_sub_fd, &raw);

                static float rc6_trim = get_float_param("RC6_TRIM");
                main_motor_direction_t direction = (raw.values[DIRECTION_SWITCH_CHANNEL_NUMBER] > rc6_trim ?
                                                    MMD_FORWARD :
                                                    MMD_BACK);
                uint16_t pwm_value = throttle_input_rc_to_pwm(raw.values[THROTTLE_CHANNEL_NUMBER]);
#ifndef MY_NDEBUG
                PX4_INFO("Throttle:\t%d", raw.values[THROTTLE_CHANNEL_NUMBER]);
                PX4_INFO("Direction switch:\t%s", direction == MMD_FORWARD ?
                                                  "MMD_FORWARD"            :
                                                  "MMD_BACK"               );
                PX4_INFO("Published PWM value =\t%d\n", pwm_value);
#endif //~MY_NDEBUG
                stm32_gpiowrite(MOTOR_DIRECTION_GPIO_PIN_NUMBER, direction);
                publish_pwm(output_pwm_fd,
                            THROTTLE_PWM_PIN_NUMBER,
                            pwm_value);
            }
        } else
        {
            //On error occured
            //Publish safe PWM value
            publish_pwm(output_pwm_fd,
                        THROTTLE_PWM_PIN_NUMBER,
                        pwm_disarmed);
            // Handle errors
            if (input_rc_poll_ret == 0)
            {
                PX4_ERR("Got no data within a second");
            } else
            if (input_rc_poll_ret < 0)
            {
                // This is seriously bad - should be an emergency
                if (error_counter < 10 || error_counter % 50 == 0)
                {
                    PX4_ERR("ERROR return value from poll(): %d", input_rc_poll_ret);
                }
                error_counter++;
            }
        }
    }
    return OK;
}

void publish_pwm(int pwm_fd, int pin_number, int value)
{
    int ret = px4_ioctl(pwm_fd,
                        PWM_SERVO_SET(pin_number),
                        value);
    if (ret != OK)
    {
        PX4_ERR("PWM_SERVO_SET(%d), abort.", pin_number);
        exit(1);
    }
}

uint16_t throttle_input_rc_to_pwm(int throttle_val)
{
    static char rc_min_param_name[sizeof("RCx_MIN")] = {};
    static char rc_max_param_name[sizeof("RCx_MAX")] = {};
    static bool flag = true;
    if (flag)
    {                                              // Numeration from 1  V
        sprintf(rc_min_param_name, "RC%d_MIN", THROTTLE_CHANNEL_NUMBER + 1);
        sprintf(rc_max_param_name, "RC%d_MAX", THROTTLE_CHANNEL_NUMBER + 1);
        flag = false;
    }

    int duty_perc = float_value_to_percents(throttle_val,
                                            rc_min_param_name,
                                            rc_max_param_name,
                                            rc_min_param_name);

    return perc_to_pwm_val(duty_perc);
}
