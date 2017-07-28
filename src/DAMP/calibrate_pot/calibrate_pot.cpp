//STD
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <stdio.h>
#include "systemlib/err.h"
//PARAM
#include "systemlib/param/param.h"
//DAMP
#include "DAMP/lib/common.h"
#include "DAMP/lib/switch.h"
//TOPICS
#include <poll.h>
#include <uORB/uORB.h>
#include <uORB/topics/adc_report.h>
#include <drivers/drv_adc.h>
#include <stm32_adc.h>
#include <uORB/topics/input_rc.h>

const int AUX_CHANNEL_NUMBER = 5;

extern "C"
{
    __EXPORT int calibrate_pot_main(int argc, char *argv[]);
}

int calibrate_pot_main(int argc, char *argv[])
{
    //Parameters' structure
    struct {
        param_t pot_max_;
        param_t pot_min_;
        param_t pot_trim_;
    } params;

    //Check if parameters exist
#define FIND_PARAM(PARAM_NAME, PARAM_FANCY_NAME)          \
    PARAM_NAME = param_find(#PARAM_FANCY_NAME);           \
    if (PARAM_NAME == PARAM_INVALID) {                    \
        PX4_ERR(#PARAM_FANCY_NAME" parameter not found"); \
        return 1;                                         \
    }

    FIND_PARAM(params.pot_max_,  POT_MAX)
    FIND_PARAM(params.pot_min_,  POT_MIN)
    FIND_PARAM(params.pot_trim_, POT_TRIM)

#undef FIND_PARAM

    //Subscribe on rc data
    int fd = orb_subscribe(ORB_ID(input_rc));
    orb_set_interval(fd, 200);

    //Subscribe on adc data
    int adc_fd = open(ADC0_DEVICE_PATH, O_RDONLY);
    if (adc_fd < 0) {
        PX4_ERR("Can't open ADC device");
        exit(1);
    }

    //Calibrate
    enum switch_state_t current_state = poll_switch_state(fd, AUX_CHANNEL_NUMBER);
    PX4_INFO("Change SWD switch state to start POT calibration");
    while (poll_switch_state(fd, AUX_CHANNEL_NUMBER) != get_inverted_switch_state(current_state));
    current_state = get_inverted_switch_state(current_state);

    PX4_INFO("Turn left and change switch state");
    while (poll_switch_state(fd, AUX_CHANNEL_NUMBER) != get_inverted_switch_state(current_state));
    current_state = get_inverted_switch_state(current_state);
    float pot_limit_1 = get_6v6_adc_value(adc_fd);
    PX4_INFO("First limit value = %f", (double)pot_limit_1);

    PX4_INFO("Turn right and change switch state");
    while (poll_switch_state(fd, AUX_CHANNEL_NUMBER) != get_inverted_switch_state(current_state));
    current_state = get_inverted_switch_state(current_state);
    float pot_limit_2 = get_6v6_adc_value(adc_fd);
    PX4_INFO("Second limit value = %f", (double)pot_limit_2);

    PX4_INFO("Put wheels straight and change switch state");
    while (poll_switch_state(fd, AUX_CHANNEL_NUMBER) != get_inverted_switch_state(current_state));
    current_state = get_inverted_switch_state(current_state);
    float pot_trim = get_6v6_adc_value(adc_fd);
    PX4_INFO("Trim value = %f", (double)pot_trim);

    PX4_INFO("Calibration finished!");

    float pot_max = (pot_limit_1 > pot_limit_2) ? pot_limit_1 : pot_limit_2;
    float pot_min = (pot_limit_1 > pot_limit_2) ? pot_limit_2 : pot_limit_1;

    //Set parameters
#define SET_PARAM(PARAM_NAME, VALUE_NAME)                 \
    if (param_set(PARAM_NAME, &VALUE_NAME) != OK) {       \
        PX4_ERR("Failed to set "#VALUE_NAME" parameter"); \
        return 1;                                         \
    }

    SET_PARAM(params.pot_max_,  pot_max)
    SET_PARAM(params.pot_min_,  pot_min)
    SET_PARAM(params.pot_trim_, pot_trim)

#undef SET_PARAM

    return OK;
}
