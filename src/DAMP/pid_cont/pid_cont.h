#ifndef PID_CONT_H
#define PID_CONT_H
class pid_cont_t
{
public:
    pid_cont_t(double sens_bias  = 0,
               double sens_prop  = 0,
               double sens_dif   = 0,
               double bias       = 0,
               double upd_period = 1);

#define SET_PROP(VAR_TYPE, VAR_NAME) \
void set_##VAR_NAME(VAR_TYPE val)    \
{                                    \
    VAR_NAME##_ = val;               \
}

    SET_PROP(double, sens_bias)
    SET_PROP(double, sens_prop)
    SET_PROP(double, sens_dif)

#undef SET_PROP

    double next(double err_val);

    double next(double err_val,
                double timestamp);

private:
    double bias_;
    double sens_bias_;
    double sens_prop_;
    double sens_dif_;
    double err_val_prev_;
    double upd_period_;
};

#endif // PID_CONT_H
