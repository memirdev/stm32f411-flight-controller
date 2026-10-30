#include "pid_controller.h"

#include <stddef.h>

static float PID_Clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

void PID_Controller_Init(PID_Controller_t *pid,
                         float kp,
                         float ki,
                         float kd,
                         float integral_limit,
                         float output_limit)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = (integral_limit >= 0.0f) ? integral_limit : -integral_limit;
    pid->output_limit = (output_limit >= 0.0f) ? output_limit : -output_limit;
    pid->integral_enabled = 1U;
    PID_Controller_Reset(pid);
}

void PID_Controller_Reset(PID_Controller_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->setpoint = 0.0f;
    pid->measurement = 0.0f;
    pid->error = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->output = 0.0f;
}

void PID_Controller_SetIntegralEnabled(PID_Controller_t *pid, uint8_t enabled)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral_enabled = (enabled != 0U) ? 1U : 0U;
}
void PID_Controller_ResetIntegral(PID_Controller_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->i_term = 0.0f;
}

float PID_Controller_Update(PID_Controller_t *pid,
                            float setpoint,
                            float measurement,
                            float measurement_rate,
                            float dt_s)
{
    float candidate_integral;
    float candidate_i_term;
    float candidate_output;
    uint8_t saturation_pushes_further;

    if (pid == NULL)
    {
        return 0.0f;
    }

    if ((dt_s <= 0.0f) || (dt_s > 0.1f))
    {
        return pid->output;
    }

    pid->setpoint = setpoint;
    pid->measurement = measurement;
    pid->error = setpoint - measurement;
    pid->p_term = pid->kp * pid->error;

    /* D term on the measured rate: no output kick when the setpoint changes. */
    pid->d_term = -pid->kd * measurement_rate;

    if (pid->ki == 0.0f)
    {
        pid->integral = 0.0f;
        candidate_integral = 0.0f;
    }
    else if (pid->integral_enabled == 0U)
    {
        candidate_integral = pid->integral;
    }
    else
    {
        candidate_integral = pid->integral + (pid->error * dt_s);
        candidate_integral = PID_Clamp(candidate_integral, pid->integral_limit);
    }
    /* The integral is not grown while the output is saturated (anti-windup). */
    candidate_i_term = pid->ki * candidate_integral;
    candidate_output = pid->p_term + candidate_i_term + pid->d_term;

    saturation_pushes_further =
        ((candidate_output > pid->output_limit) && (pid->error > 0.0f)) ||
        ((candidate_output < -pid->output_limit) && (pid->error < 0.0f));

    if (saturation_pushes_further == 0U)
    {
        pid->integral = candidate_integral;
    }

    pid->i_term = pid->ki * pid->integral;
    pid->output = PID_Clamp(pid->p_term + pid->i_term + pid->d_term,
                            pid->output_limit);

    return pid->output;
}
