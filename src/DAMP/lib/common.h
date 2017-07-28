#ifndef COMMON_H_INCLUDED
#define COMMON_H_INCLUDED

#include <uORB/topics/adc_report.h>

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

int float_value_to_percents(float       value,
                            const char* min_param_name,
                            const char* max_param_name,
                            const char* ref_param_name);

int uint32_t_value_to_percents(uint32_t    value,
                               const char* min_param_name,
                               const char* max_param_name,
                               const char* ref_param_name);

float get_6v6_adc_value(int adc_report_sub_fd);

uint16_t perc_to_pwm_val(float duty);

float    get_float_param   (const char* param_name);
uint32_t get_uint32_t_param(const char* param_name);

#ifdef __cplusplus
}
#endif //__cplusplus

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

#endif //~COMMON_H_INCLUDED
