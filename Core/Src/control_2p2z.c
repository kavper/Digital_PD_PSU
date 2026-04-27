#include "control_2p2z.h"

static float df22_clamp(float value, float min_value, float max_value)
{
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

void DF22_Init(DF22_Controller_t *ctrl,
               float b0,
               float b1,
               float b2,
               float a1,
               float a2,
               float out_min,
               float out_max)
{
  if (ctrl == 0) {
    return;
  }

  ctrl->b0 = b0;
  ctrl->b1 = b1;
  ctrl->b2 = b2;
  ctrl->a1 = a1;
  ctrl->a2 = a2;
  ctrl->x1 = 0.0f;
  ctrl->x2 = 0.0f;
  ctrl->out_min = out_min;
  ctrl->out_max = out_max;
}

void DF22_Reset(DF22_Controller_t *ctrl)
{
  if (ctrl == 0) {
    return;
  }

  ctrl->x1 = 0.0f;
  ctrl->x2 = 0.0f;
}

void DF22_SetOutputLimits(DF22_Controller_t *ctrl, float out_min, float out_max)
{
  if (ctrl == 0) {
    return;
  }

  ctrl->out_min = out_min;
  ctrl->out_max = out_max;
}

void DF22_PresetOutput(DF22_Controller_t *ctrl, float output)
{
  if (ctrl == 0) {
    return;
  }

  const float clamped = df22_clamp(output, ctrl->out_min, ctrl->out_max);

  /*
   * DF22 direct-form state for steady output at zero error:
   * out=x1 and x2=-a2*out for the current denominator.
   */
  ctrl->x1 = clamped;
  ctrl->x2 = -ctrl->a2 * clamped;
}

float DF22_Update(DF22_Controller_t *ctrl, float error)
{
  if (ctrl == 0) {
    return 0.0f;
  }

  float out = ctrl->b0 * error + ctrl->x1;
  out = df22_clamp(out, ctrl->out_min, ctrl->out_max);

  const float x1_new = ctrl->b1 * error + ctrl->x2 - ctrl->a1 * out;
  const float x2_new = ctrl->b2 * error - ctrl->a2 * out;

  ctrl->x1 = x1_new;
  ctrl->x2 = x2_new;

  return out;
}
