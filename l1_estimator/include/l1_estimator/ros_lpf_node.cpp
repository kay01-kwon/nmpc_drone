#include "ros_lpf_node.hpp"

RosLpf::RosLpf(const NodeHandle &nh, const double &tau)
:nh_(nh), lpf_obj(Lpf(tau)), 
signal_filtered_(signal_filtered_.setZero()),
time_curr(0), time_offset(0),
init_time(false)
{
    ros_setup();
}

void RosLpf::ros_setup()
{
    signal_subscriber_ = nh_.subscribe("/input_signal",
    1,
    &RosLpf::callback,
    this);
    lpf_publisher_ = nh_.advertise<Lpf_test>("/filtered_signal", 1);

}

void RosLpf::callback(const Lpf_testConstPtr &signal_msg)
{
    if(init_time == false)
    {
        // Before filtering the signal,
        // get the time offset.
        time_offset = signal_msg->stamp.sec
        + signal_msg->stamp.nsec*10e-9;
        init_time = true;
    }
    else
    {
        time_curr =  signal_msg->stamp.sec
        + signal_msg->stamp.nsec*10e-9
        - time_offset;

        mat31_t v_in;
        
        for(size_t i = 0; i < 3; i++)
        {
            v_in(i) = signal_msg->v[i];
        }

        lpf_obj.set_input(v_in);
        lpf_obj.set_time(time_curr);

        // Integrate and get the filtered signal
        lpf_obj.get_filtered_vector(signal_filtered_);

    }

}

void RosLpf::filtered_signal_publish()
{
}
