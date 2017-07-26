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

//This is compile-time constant
const int         THROTTLE_CHANNEL_NUMBER = 2;
const int DIRECTION_SWITCH_CHANNEL_NUMBER = 6;
const int         THROTTLE_PWM_PIN_NUMBER = 0;
const int MOTOR_DIRECTION_GPIO_PIN_NUMBER = GPIO_GPIO2_OUTPUT;

int main_motor_main(int argc, char *argv[])
{
    static uint32_t pwm_disarmed = get_pwm_disarmed_param();
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

                static float rc6_trim = get_rc6_mid_param();
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

uint16_t throttle_input_rc_to_pwm(int input_rc_val)
{
    static uint32_t pwm_min      = get_pwm_min_param();
    static uint32_t pwm_max      = get_pwm_max_param();
    static float    throttle_min = get_throttle_min_param();
    static float    throttle_max = get_throttle_max_param();
    static bool     firstCall    = true;
    if (firstCall)
    {
        PX4_INFO("PWM_MIN = %d",      pwm_min);
        PX4_INFO("PWM_MAX = %d",      pwm_max);
        PX4_INFO("THROTTLE_MIN = %d", (uint32_t)throttle_min);
        PX4_INFO("THROTTLE_MAX = %d", (uint32_t)throttle_max);
        firstCall = false;
    }

    //Rescale THROTTLE_MIN:THROTTLE_MAX -> 0:1
    float pwm_duty = (input_rc_val - throttle_min) /
                     (throttle_max - throttle_min);

    // Rescale 0:1 -> PWM_MIN:PWM_MAX
    uint16_t pwm_value = pwm_duty * (pwm_max - pwm_min) + pwm_min;
    // Handle if duty is greater than 1 or less than 0
    pwm_value = (pwm_value > pwm_max) ? pwm_max : pwm_value;
    pwm_value = (pwm_value < pwm_min) ? pwm_min : pwm_value;
    return pwm_value;
}
