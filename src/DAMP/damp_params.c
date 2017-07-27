/**
 * Pot min value
 *
 * Min adc value when steering
 *
 * @unit adc_units
 * @group DAMP
 */
PARAM_DEFINE_FLOAT(POT_MIN, 0.0f);

/**
 * Pot max value
 *
 * Max adc value when steering
 *
 * @unit adc_units
 * @group DAMP
 */
PARAM_DEFINE_FLOAT(POT_MAX, 0.0f);

/**
 * Pot mid value
 *
 * Mid adc value when steering
 *
 * @unit adc_units
 * @group DAMP
 */
PARAM_DEFINE_FLOAT(POT_TRIM, 0.0f);

/**
 * pid proportional part sense value
 *
 * @unit steering
 * @group DAMP
 */
PARAM_DEFINE_FLOAT(PROP_SENS, 7.0f);

/**
 * pid differential part sense value
 *
 * @unit steering
 * @group DAMP
 */
PARAM_DEFINE_FLOAT(DIF_SENS, 0.0f);

/**
 * steering PID dead zone value in percs
 *
 * @unit steering
 * @group DAMP
 */
PARAM_DEFINE_FLOAT(DEAD_ZONE, 3.0f);
