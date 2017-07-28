#ifndef SWITCH_H_INCLUDED
#define SWITCH_H_INCLUDED

enum switch_state_t
{
    SS_ON  = 0,
    SS_OFF = 1
};

switch_state_t poll_switch_state(int input_rc_sub_fd, int channel_num);
switch_state_t get_switch_state(int rc_data, int channel_num);
switch_state_t get_inverted_switch_state(switch_state_t s);

#endif //~SWITCH_H_INCLUDED
