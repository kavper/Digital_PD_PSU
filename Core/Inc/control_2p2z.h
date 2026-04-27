#ifndef CONTROL_2P2Z_H
#define CONTROL_2P2Z_H

typedef struct
{
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
  float x1;
  float x2;
  float out_min;
  float out_max;
} DF22_Controller_t;

void DF22_Init(DF22_Controller_t *ctrl,
               float b0,
               float b1,
               float b2,
               float a1,
               float a2,
               float out_min,
               float out_max);
void DF22_Reset(DF22_Controller_t *ctrl);
void DF22_SetOutputLimits(DF22_Controller_t *ctrl, float out_min, float out_max);
float DF22_Update(DF22_Controller_t *ctrl, float error);

#endif /* CONTROL_2P2Z_H */
