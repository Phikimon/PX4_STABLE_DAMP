//STD
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
//TOPICS
#include <uORB/uORB.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/output_pwm.h>
//DAMP
#include "switch.h"
#include "common.h"

switch_state_t get_inverted_switch_state(switch_state_t s)
{
    return ((s == SS_ON) ? SS_OFF : SS_ON);
}

switch_state_t poll_switch_state(int input_rc_sub_fd, int channel_num)
{
    px4_pollfd_struct_t input_rc_pollfd = {};
    input_rc_pollfd.fd     = input_rc_sub_fd,
    input_rc_pollfd.events = POLLIN;
    input_rc_s raw         = {};
    int input_rc_poll_ret = 0;

    //Until data read successfully
    while (input_rc_poll_ret == 0)
    {
        input_rc_poll_ret = px4_poll(&input_rc_pollfd, 1, 1000);
        if (input_rc_poll_ret > 0)
        {
            if (input_rc_pollfd.revents & POLLIN)
            {
                orb_copy(ORB_ID(input_rc), input_rc_sub_fd, &raw);
            }
        } else
        {
            // Handle errors
            if (input_rc_poll_ret == 0)
            {
                PX4_ERR("Got no data within a second");
            } else
            if (input_rc_poll_ret < 0)
            {
                PX4_ERR("ERROR return value from poll(): %d", input_rc_poll_ret);
                exit(1);
            }
        }
    }
    return get_switch_state(raw.values[channel_num], channel_num);
}

switch_state_t get_switch_state(int rc_data, int channel_num)
{
    char param_name[sizeof("RCx_TRIM")] = {};
    sprintf(param_name, "RC%d_TRIM", channel_num);
    static float rc_trim = get_float_param(param_name);

    return (rc_data > rc_trim) ?
           SS_ON               :
           SS_OFF;
}
