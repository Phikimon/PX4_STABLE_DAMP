//STD
#include <px4_posix.h>
//DAMP
#include <DAMP/lib/common.h>
//Topics
#include <uORB/uORB.h>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/input_rc.h>
#include <stm32_adc.h>

//-----------------------------------------------------------------------------------

#define PX4_DEBUG_INFO(...) PX4_INFO(__VA_ARGS__)

//-----------------------------------------------------------------------------------

#define assert_ptr(PTR_NAME)                            \
{                                                       \
    if (PTR_NAME == NULL)                               \
    {                                                   \
        PX4_ERR("Nullptr "#PTR_NAME" in %s", __func__); \
    }                                                   \
}

//-----------------------------------------------------------------------------------

#define GET_PARAM_FUNC(TYPE)                             \
TYPE get_##TYPE##_param(const char* param_name)          \
{                                                        \
    assert_ptr(param_name);                              \
                                                         \
    param_t param = param_find(param_name);              \
    if (param == PARAM_INVALID)                          \
    {                                                    \
        PX4_ERR("'%s' parameter not found", param_name); \
    }                                                    \
                                                         \
    TYPE param_value = 0;                                \
    param_get(param_find(param_name), &param_value);     \
    return param_value;                                  \
}

GET_PARAM_FUNC(float)
GET_PARAM_FUNC(uint32_t)

#undef GET_PARAM_FUNC

//-----------------------------------------------------------------------------------

#define VALUE_TO_PERCENTS(VALUE_TYPE)                                           \
int VALUE_TYPE##_value_to_percents(VALUE_TYPE  value,                           \
                                   const char* min_param_name,                  \
                                   const char* max_param_name,                  \
                                   const char* ref_param_name)                  \
{                                                                               \
    assert_ptr(min_param_name);                                                 \
    assert_ptr(max_param_name);                                                 \
    assert_ptr(ref_param_name);                                                 \
                                                                                \
    const int PERCENTAGE_FACTOR  = 100;                                         \
    VALUE_TYPE value_min = get_##VALUE_TYPE##_param(min_param_name);            \
    VALUE_TYPE value_max = get_##VALUE_TYPE##_param(max_param_name);            \
    VALUE_TYPE value_ref = get_##VALUE_TYPE##_param(ref_param_name);            \
    VALUE_TYPE value_rng = (value_max - value_min);                             \
                                                                                \
    PX4_DEBUG_INFO("%s = %d", min_param_name, (uint32_t)value_min);             \
    PX4_DEBUG_INFO("%s = %d", max_param_name, (uint32_t)value_min);             \
    PX4_DEBUG_INFO("%s = %d", ref_param_name, (uint32_t)value_min);             \
    /*                                                                          \
     E.g.                                                                       \
     yaw       = 1400 | => yaw_deg = ((1400-1500) / 1000) * 360 =               \
     yaw_ref   = 1500 |            = ((-100) / 1000) * 100 =                    \
     yaw_rng   = 1000 |            = -10%                                       \
     steer_rng = 30   |                                                         \
    */                                                                          \
    return (value - value_ref) / value_rng * PERCENTAGE_FACTOR;                 \
}

VALUE_TO_PERCENTS(float)
VALUE_TO_PERCENTS(uint32_t)

#undef VALUE_TO_PERCENTS

//-----------------------------------------------------------------------------------

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
        static bool     is_first_call    = true;

        static uint32_t pwm_min      = get_uint32_t_param("PWM_MIN");
        static uint32_t pwm_max      = get_uint32_t_param("PWM_MAX");
        const int       PERCENTAGE_FACTOR  = 100;

#ifndef MY_NDEBUG
        if (is_first_call)
        {
            PX4_INFO("PWM_MIN = %d",      pwm_min);
            PX4_INFO("PWM_MAX = %d",      pwm_max);
        }
#endif //~MY_NDEBUG

        // Rescale 0:100 -> PWM_MIN:PWM_MAX
        uint16_t pwm_value = (perc / PERCENTAGE_FACTOR) * (pwm_max - pwm_min) +
                             pwm_min;
        // Handle if duty is greater than 1 or less than 0
        pwm_value = (pwm_value > pwm_max) ? pwm_max : pwm_value;
        pwm_value = (pwm_value < pwm_min) ? pwm_min : pwm_value;

        is_first_call = false;
        return pwm_value;
}
