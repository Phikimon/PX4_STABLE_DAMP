#include "DAMP/pid_cont/pid_cont.h"

pid_cont_t::pid_cont_t(double sens_bias,
                       double sens_prop,
                       double sens_dif,
                       double bias,
                       double upd_period):
    bias_        (bias      ),
    sens_bias_   (sens_bias ),
    sens_prop_   (sens_prop ),
    sens_dif_    (sens_dif  ),
    err_val_prev_(0         ),
    upd_period_  (upd_period)
{}

double pid_cont_t::next(double err_val)
{
    return next(err_val, upd_period_);
}

double pid_cont_t::next(double err_val, double timestamp)
{
    err_val_prev_ = err_val;
    upd_period_ = timestamp;
    bias_ = bias_ + err_val * sens_bias_ / upd_period_;

    return bias_ +
           err_val * sens_prop_ +
          (err_val - err_val_prev_) * sens_dif_ * upd_period_;
}
