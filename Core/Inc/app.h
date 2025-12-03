#ifndef __APP_H
#define __APP_H

#define B0 (+1.470512132237)
#define B1 (-1.361262909728)
#define B2 (-1.469501507551)
#define B3 (+1.362273534414)
#define A1 (+0.666857948186)
#define A2 (+0.308471947292)
#define A3 (+0.024670104523)
#define K (+2295.178521854403)
#define REF (178)
#define DUTY_TICKS_MIN (0)
#define DUTY_TICKS_MAX (24480)

float APP_GetCurrentSetpoint();
float APP_GetVIn();
float APP_GetVOut();
float APP_GetIOut();
float APP_GetVBoost();
float APP_GetIIn();
float APP_GetPWM();

void APP_HandleADCInterrupt();

void APP_Init();
void APP_Run();

#endif /* __APP_H */