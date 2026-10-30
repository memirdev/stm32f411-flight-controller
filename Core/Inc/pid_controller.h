#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float integral_limit;
    float output_limit;

    float setpoint;
    float measurement;
    float error;
    float p_term;
    float i_term;
    float d_term;
    float output;

    uint8_t integral_enabled;
} PID_Controller_t;

void PID_Controller_Init(PID_Controller_t *pid,
                         float kp,
                         float ki,
                         float kd,
                         float integral_limit,
                         float output_limit);

void PID_Controller_Reset(PID_Controller_t *pid);

void PID_Controller_ResetIntegral(PID_Controller_t *pid);
void PID_Controller_SetIntegralEnabled(PID_Controller_t *pid, uint8_t enabled);

float PID_Controller_Update(PID_Controller_t *pid,
                            float setpoint,
                            float measurement,
                            float measurement_rate,
                            float dt_s);

#ifdef __cplusplus
}
#endif

#endif
