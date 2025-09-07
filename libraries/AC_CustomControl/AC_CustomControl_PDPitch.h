#pragma once
#include <AP_Common/AP_Common.h>
#include "AC_CustomControl_Backend.h"
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>



#ifndef CUSTOMCONTROL_PDPITCH_ENABLED
    #define CUSTOMCONTROL_PDPITCH_ENABLED AP_CUSTOMCONTROL_ENABLED
#endif

#if CUSTOMCONTROL_PDPITCH_ENABLED

class AC_CustomControl_PDPitch : public AC_CustomControl_Backend {
public:
    AC_CustomControl_PDPitch(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt);


    Vector3f update(void) override;
    void reset(void) override;

    // set the PID notch sample rates
    void set_notch_sample_rate(float sample_rate) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

#if HAL_LOGGING_ENABLED
    void Write_Log() override;
#endif

private:
    float _dt;

    // angle P controller  PID object
    AC_PID        _p_angle_pitch2;

	// rate PID controller  objects
    AC_PID _pid_atti_rate_roll;
    AC_PID _pid_atti_rate_pitch;
    AC_PID _pid_atti_rate_yaw;

    //Calculates target pitch velocities. PD Pitch Angle controller
    Vector3f update_ang_vel_target_from_att_error(const Vector3f &attitude_error_rot_vec_rad);

    Vector3f run_rate_controller(const Vector3f& target_rate);

    bool _use_sqrt_controller;
};

#endif