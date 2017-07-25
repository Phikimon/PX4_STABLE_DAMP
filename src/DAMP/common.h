#ifndef COMMON_H_INCLUDED
#define COMMON_H_INCLUDED
#include <uORB/topics/adc_report.h>

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#define GET_PARAM(PARAM_NAME, FUNC_NAME, TYPE) \
TYPE get_##FUNC_NAME##_param(void);

#include "param_list.txt"

#undef GET_PARAM

#define VALUE_TO_PERCENTS(VALUE_TYPE, VALUE_NAME, FUNC_NAME, REF_FUNC_NAME) \
int FUNC_NAME##_to_percents(VALUE_TYPE value);

#include "value_to_percents_list.txt"

#undef VALUE_TO_PERCENTS

float get_6v6_adc_value(int adc_report_sub_fd);

uint16_t perc_to_pwm_val(float duty);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif
