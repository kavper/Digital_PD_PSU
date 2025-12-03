#ifndef __APP_H
#define __APP_H

// --- KONFIGURACJA HRTIM ---
#define HRTIM_PERIOD 54400
#define PWM_MIN 2720  // 5% (Bezpieczny margines dla Bootstrapu)
#define PWM_MAX 51680 // 95%

// --- NASTAWY ZASILACZA ---
#define V_TARGET_VOLTAGE 12.0f // [V] Docelowe napięcie wyjściowe
#define SOFT_START_STEP 0.05f  // [V] Krok narastania napięcia na pętlę

// --- ZABEZPIECZENIA ---
#define OVP_LIMIT 14.5f // [V] Over Voltage Protection (Wyłączenie)
#define OCP_LIMIT 10.0f // [A] Over Current Protection (Wyłączenie)
#define UVLO_LIMIT                                                             \
  15.0f // [V] Minimalne napięcie zasilania (V_BOOST) żeby w ogóle startować

// --- NASTAWY REGULATORA PID ---
// Wartości trzeba dobrać eksperymentalnie, te są "bezpieczne" na start
#define PID_KP 200.0f             // Wzmocnienie proporcjonalne
#define PID_KI 10.0f              // Wzmocnienie całkujące
#define PID_INTEGRAL_MAX 10000.0f // Limit członu całkującego (Anti-windup)

// --- KALIBRACJA ADC ---
#define COEFF_VOLTAGE 0.008682f
#define COEFF_CURRENT 0.002442f

float APP_GetCurrentSetpoint();
float APP_GetVIn();
float APP_GetVOut();
float APP_GetIOut();
float APP_GetVBoost();
float APP_GetIIn();

void APP_Init();
void APP_Run();

#endif /* __APP_H */