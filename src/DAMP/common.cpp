#include <px4_posix.h>
#include <DAMP/common.h>
#include <uORB/uORB.h>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/input_rc.h>
#include <stm32_adc.h>

//#include <drivers/drv_pwm_output.h>
//#include <uORB/uORB.h>

#define GET_PARAM(PARAM_NAME, FUNC_NAME, TYPE)       \
TYPE get_##FUNC_NAME##_param(void)                   \
{                                                    \
    TYPE param = 0;                                  \
    param_get(param_find(#PARAM_NAME), &param);      \
    return param;                                    \
}

#include "param_list.txt"

#undef GET_PARAM

#define VALUE_TO_PERCENTS(VALUE_TYPE, VALUE_NAME, FUNC_NAME, REF_FUNC_NAME)     \
int FUNC_NAME##_to_percents(VALUE_TYPE value)                                   \
{                                                                               \
    const int PERCENTAGE_FACTOR  = 100;                                         \
    static VALUE_TYPE value_min = get_##VALUE_NAME##_min_param();               \
    static VALUE_TYPE value_max = get_##VALUE_NAME##_max_param();               \
    static VALUE_TYPE value_ref = get_##VALUE_NAME##_##REF_FUNC_NAME##_param(); \
    static VALUE_TYPE value_rng = (value_max - value_min);                      \
    static bool firstCall = true;                                               \
    if (firstCall)                                                              \
    {                                                                           \
        PX4_INFO(#VALUE_NAME"_min = %d", (uint32_t)value_min);                  \
        PX4_INFO(#VALUE_NAME"_max = %d", (uint32_t)value_max);                  \
        PX4_INFO(#VALUE_NAME"_ref = %d", (uint32_t)value_ref);                  \
        firstCall = false;                                                      \
    }                                                                           \
    /*                                                                          \
     E.g.                                                                       \
     yaw       = 1400 | => yaw_deg = ((1400-1500) / 1000) * 360 =               \
     yaw_ref   = 1500 |            = ((-100) / 1000) * 100 =                    \
     yaw_rng   = 1000 |            = -10%                                       \
     steer_rng = 30   |                                                         \
    */                                                                          \
    return (value - value_ref) / value_rng * PERCENTAGE_FACTOR;                 \
}

#include "value_to_percents_list.txt"

#undef VALUE_TO_PERCENTS

float get_6v6_adc_value(int adc_fd)
{
    const int ADC_6V6_CHANNEL_NUMBER = 8;
    const int MAX_PIXHAWK_ADC_NUM = 12;

    adc_msg_s data[MAX_PIXHAWK_ADC_NUM] = {};
    ssize_t count = read(adc_fd, data, sizeof(data));

    if (count < 0) {
        PX4_ERR("ADC read error");
        exit(1);
    }

    return data[ADC_6V6_CHANNEL_NUMBER].am_data;
}

uint16_t perc_to_pwm_val(float perc)
{
		static uint32_t pwm_min      = get_pwm_min_param();
	    static uint32_t pwm_max      = get_pwm_max_param();
	    const int PERC_FACT = 100;
	    /*static bool     firstCall    = true;
	    if (firstCall)
	    {
	        PX4_INFO("PWM_MIN = %d",      pwm_min);
	        PX4_INFO("PWM_MAX = %d",      pwm_max);

	        firstCall = false;
	    }*/

	    //Rescale 0:100 -> PWM_MIN:PWM_MAX

	    // Rescale 0:1 -> PWM_MIN:PWM_MAX
	    uint16_t pwm_value = perc / PERC_FACT * (pwm_max - pwm_min) + pwm_min;
	    // Handle if duty is greater than 1 or less than 0
	    pwm_value = (pwm_value > pwm_max) ? pwm_max : pwm_value;
	    pwm_value = (pwm_value < pwm_min) ? pwm_min : pwm_value;
	    return pwm_value;
}
