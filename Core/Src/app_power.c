#include "app_power.h"
#include "control_2p2z.h"
#include "hrtim_pwm.h"
#include "main.h"
#include "pid.h"
#include "power_measurement.h"
#include "stm32g4xx_hal_adc.h"
#include <stdint.h>

#define PSU_VOLTAGE_TARGET_MAX_V (30.0f)
#define PSU_CONTROL_FREQ_HZ      ((float)(HRTIM_PWM_FSW_HZ / HRTIM_ADC_DIV))
#define PSU_DUTY_MAX_START       (0.60f)
#define PSU_DUTY_MAX_NORMAL      (0.80f)
#define PSU_ZERO_SETPOINT_V      (0.01f)
#define PSU_STAGE_DISABLE_V      (0.005f)
/* 10 V step: ~200 ms up/down at the 50 kHz control rate. */
#define PSU_SLEW_UP_V_PER_S      (50.0f)
#define PSU_SLEW_DOWN_V_PER_S    (50.0f)

/*
 * Ostrozny startowy 2P2Z dla fcontrol = 50 kHz.
 *
 * Poprzednie wspolczynniki byly policzone dla fcontrol = 200 kHz i
 * crossover ok. 8 kHz. Po przejsciu na 50 kHz dawaly za agresywna petle
 * i potrafily wzbudzac bucka w okolicy kilku kHz pod obciazeniem.
 *
 * Ten wariant zostawia:
 * - fz1 = fz2 ~= 2.32 kHz, okolica rezonansu LC,
 * - fp1 ~= 22.7 kHz, okolica zera ESR,
 * - niski gain K = 0.5, czyli konserwatywny crossover rzedu setek Hz.
 *
 * Po pomiarze odpowiedzi skokowej mozna stopniowo zwiekszac K, np.
 * 0.65 albo 0.75, ale dopiero gdy na oscyloskopie nie ma dzwonienia.
 */
#define PSU_2P2Z_B0 (0.50000000f)
#define PSU_2P2Z_B1 (-0.74711228f)
#define PSU_2P2Z_B2 (0.27908838f)
#define PSU_2P2Z_A1 (-1.05769608f)
#define PSU_2P2Z_A2 (0.05769608f)

extern ADC_HandleTypeDef hadc1;
extern HRTIM_HandleTypeDef hhrtim1;

uint16_t psu_adc_dma_buffer[PSU_ADC_CHANNEL_COUNT] = {0};

static DF22_Controller_t psu_cv_controller;
static PowerMeasurements_t psu_measurements;
static float psu_target_voltage_v = 0.0f;
static float psu_slewed_voltage_v = 0.0f;
static float psu_ref_adc_ramp_v = 0.0f;
static uint8_t psu_initialized = 0U;
static volatile uint8_t psu_running = 0U;
static volatile uint8_t psu_control_active = 0U;
static volatile uint8_t psu_shutdown_pending = 0U;
static uint8_t psu_adc_dma_running = 0U;
static uint8_t psu_power_stage_enabled = 0U;

static float psu_clamp(float value, float min_value, float max_value)
{
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static void psu_reset_control(float duty_max)
{
  psu_slewed_voltage_v = 0.0f;
  psu_ref_adc_ramp_v = 0.0f;
  DF22_Reset(&psu_cv_controller);
  DF22_SetOutputLimits(&psu_cv_controller, HRTIM_PWM_DUTY_MIN, duty_max);
  HRTIM_PWM_SetDuty(HRTIM_PWM_DUTY_MIN);
}

static void psu_reset_controller_only(float duty_max)
{
  DF22_Reset(&psu_cv_controller);
  DF22_SetOutputLimits(&psu_cv_controller, HRTIM_PWM_DUTY_MIN, duty_max);
  HRTIM_PWM_SetDuty(HRTIM_PWM_DUTY_MIN);
}

static void psu_set_power_stage(uint8_t enabled)
{
  if (enabled == 0U) {
    /*
     * Do not stop HRTIM outputs here. Output start/stop can expose an HRTIM
     * latch in the middle of a PWM period; keep PWM running and only gate the
     * external driver. The timer is stopped later, after DRV_EN is already low.
     */
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
    psu_power_stage_enabled = 0U;
    return;
  }

  if (psu_power_stage_enabled != 0U) {
    return;
  }

  HRTIM_PWM_ForceUpdate();
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);
  psu_power_stage_enabled = 1U;
}

static void psu_start_adc_dma(void)
{
  if (psu_adc_dma_running != 0U) {
    return;
  }

  if (HAL_ADC_Start_DMA(&hadc1,
                        (uint32_t *)psu_adc_dma_buffer,
                        PSU_ADC_CHANNEL_COUNT) != HAL_OK) {
    Error_Handler();
  }

  /*
   * HAL_ADC_Start_DMA enables half-transfer interrupts too. With a 3-sample
   * circular buffer and HRTIM trigger this creates extra IRQ load without
   * adding useful control data.
   */
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);

  psu_adc_dma_running = 1U;
}

static void psu_stop_adc_dma(void)
{
  if (psu_adc_dma_running == 0U) {
    return;
  }

  (void)HAL_ADC_Stop_DMA(&hadc1);
  psu_adc_dma_running = 0U;
}

static float psu_step_towards(float value, float target, float step)
{
  if (value < target) {
    value += step;
    if (value > target) {
      value = target;
    }
  } else if (value > target) {
    value -= step;
    if (value < target) {
      value = target;
    }
  }

  return value;
}

static void psu_update_reference_slew(float target_voltage_v)
{
  const float rate_v_per_s =
      (target_voltage_v >= psu_slewed_voltage_v) ? PSU_SLEW_UP_V_PER_S
                                                 : PSU_SLEW_DOWN_V_PER_S;
  const float step_v = rate_v_per_s / PSU_CONTROL_FREQ_HZ;

  psu_slewed_voltage_v =
      psu_step_towards(psu_slewed_voltage_v, target_voltage_v, step_v);

  if ((target_voltage_v <= PSU_ZERO_SETPOINT_V) &&
      (psu_slewed_voltage_v <= PSU_STAGE_DISABLE_V)) {
    psu_slewed_voltage_v = 0.0f;
  }

  psu_ref_adc_ramp_v = psu_slewed_voltage_v * POWER_VOUT_GAIN;
}

static void psu_control_shutdown_from_isr(void)
{
  psu_reset_control(PSU_DUTY_MAX_START);
  HRTIM_PWM_ForceUpdate();
  psu_set_power_stage(0U);
  psu_control_active = 0U;
  psu_shutdown_pending = 1U;
}

void PSU_Init(void)
{
  if (psu_initialized != 0U) {
    return;
  }

  psu_target_voltage_v = 0.0f;
  psu_slewed_voltage_v = 0.0f;
  psu_ref_adc_ramp_v = 0.0f;
  psu_running = 0U;
  psu_control_active = 0U;
  psu_shutdown_pending = 0U;
  psu_adc_dma_running = 0U;
  psu_power_stage_enabled = 0U;

  DF22_Init(&psu_cv_controller,
            PSU_2P2Z_B0,
            PSU_2P2Z_B1,
            PSU_2P2Z_B2,
            PSU_2P2Z_A1,
            PSU_2P2Z_A2,
            HRTIM_PWM_DUTY_MIN,
            PSU_DUTY_MAX_START);

  HRTIM_PWM_Init(&hhrtim1);
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler();
  }

  psu_initialized = 1U;
}

void PSU_Start(void)
{
  if (psu_initialized == 0U) {
    PSU_Init();
  }

  if (psu_control_active == 0U) {
    psu_reset_control(PSU_DUTY_MAX_START);
    psu_set_power_stage(0U);
    psu_start_adc_dma();
    HRTIM_PWM_ForceUpdate();
    HRTIM_PWM_StartCounter();
    HRTIM_PWM_EnableOutputs();

    if (psu_target_voltage_v > PSU_ZERO_SETPOINT_V) {
      psu_set_power_stage(1U);
    }

    psu_control_active = 1U;
  }

  psu_shutdown_pending = 0U;
  psu_running = 1U;
}

void PSU_Stop(void)
{
  psu_target_voltage_v = 0.0f;
  psu_running = 0U;

  if (psu_control_active != 0U) {
    return;
  }

  psu_set_power_stage(0U);
  psu_reset_control(PSU_DUTY_MAX_START);
  HRTIM_PWM_Stop();
  psu_stop_adc_dma();
}

void PSU_SetVoltage(float voltage_v)
{
  psu_target_voltage_v = psu_clamp(voltage_v, 0.0f, PSU_VOLTAGE_TARGET_MAX_V);
}

void PSU_ControlLoopFromAdc(const uint16_t adc_samples[PSU_ADC_CHANNEL_COUNT])
{
  if (adc_samples == 0) {
    return;
  }

  PowerMeasurement_Update(&psu_measurements,
                          adc_samples[PSU_ADC_INDEX_VOUT],
                          adc_samples[PSU_ADC_INDEX_IOUT],
                          adc_samples[PSU_ADC_INDEX_VBOOST]);

  if (psu_running == 0U) {
    if (psu_control_active == 0U) {
      psu_set_power_stage(0U);
      psu_reset_control(PSU_DUTY_MAX_START);
      return;
    }
  }

  const float target_voltage_v = (psu_running != 0U) ? psu_target_voltage_v : 0.0f;
  psu_update_reference_slew(target_voltage_v);

  if ((psu_running == 0U) &&
      (target_voltage_v <= PSU_ZERO_SETPOINT_V) &&
      (psu_slewed_voltage_v <= PSU_STAGE_DISABLE_V)) {
    psu_control_shutdown_from_isr();
    return;
  }

  if ((psu_running != 0U) &&
      (target_voltage_v <= PSU_ZERO_SETPOINT_V) &&
      (psu_slewed_voltage_v <= PSU_STAGE_DISABLE_V)) {
    psu_reset_controller_only(PSU_DUTY_MAX_START);
    HRTIM_PWM_ForceUpdate();
    psu_set_power_stage(0U);
    return;
  }

  if (psu_slewed_voltage_v <= PSU_STAGE_DISABLE_V) {
    psu_reset_controller_only(PSU_DUTY_MAX_START);
    HRTIM_PWM_ForceUpdate();

    if ((psu_running != 0U) && (target_voltage_v > PSU_ZERO_SETPOINT_V)) {
      psu_set_power_stage(1U);
    } else {
      psu_set_power_stage(0U);
    }

    return;
  }

  const float error_adc_v = psu_ref_adc_ramp_v - psu_measurements.vout_adc_v;

  if (psu_power_stage_enabled == 0U) {
    psu_reset_controller_only(PSU_DUTY_MAX_START);
  }

  float duty_max = PSU_DUTY_MAX_START;
  if ((psu_running != 0U) &&
      (psu_target_voltage_v > PSU_ZERO_SETPOINT_V) &&
      (psu_slewed_voltage_v >= psu_target_voltage_v)) {
    duty_max = PSU_DUTY_MAX_NORMAL;
  }
  DF22_SetOutputLimits(&psu_cv_controller, HRTIM_PWM_DUTY_MIN, duty_max);

  const float duty = DF22_Update(&psu_cv_controller, error_adc_v);

  HRTIM_PWM_SetDuty(duty);

  if (psu_power_stage_enabled == 0U) {
    psu_set_power_stage(1U);
  }
}

void PSU_Service(void)
{
  if (psu_shutdown_pending == 0U) {
    return;
  }

  psu_shutdown_pending = 0U;
  HRTIM_PWM_Stop();
  psu_stop_adc_dma();
}

float PSU_GetMeasuredVoltage(void)
{
  return psu_measurements.vout_v;
}

float PSU_GetMeasuredCurrent(void)
{
  return psu_measurements.iout_a;
}

float PSU_GetMeasuredInputVoltage(void)
{
  return psu_measurements.vboost_v;
}

float PSU_GetDuty(void)
{
  return HRTIM_PWM_GetDuty();
}

float PSU_GetTargetVoltage(void)
{
  return psu_target_voltage_v;
}

float PSU_GetSoftStartVoltage(void)
{
  return psu_slewed_voltage_v;
}

void PSU_AdcDmaIrqHandler(void)
{
  DEBUG_ADC_ISR_HIGH();
  PSU_ControlLoopFromAdc(psu_adc_dma_buffer);
  DEBUG_ADC_ISR_LOW();
}

uint8_t PSU_IsRunning(void)
{
  return psu_running;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance != ADC1) {
    return;
  }

  PSU_AdcDmaIrqHandler();
}
