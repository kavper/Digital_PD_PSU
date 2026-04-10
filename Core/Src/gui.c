#include "gui.h"
#include "app.h"
#include "logo.h"
#include "main.h"
#include "pid.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#define GUI_FILTER_ALPHA 0.5f
#define BTN_DEBOUNCE_MS 50u
#define GUI_V_SNAP_TO_SET 0.01f
#define GUI_V_DEADBAND 0.02f
#define GUI_MODE_AUTOFOLLOW_MS 1200u

static float v_out = 0.0f;
static float i_out = 0.0f;
static float p_out = 0.0f;
static float v_in = 0.0f;

static float v_target_local = 0.0f;

volatile uint8_t enc_click  = 0;
volatile uint8_t aux1_click = 0;
volatile uint8_t aux2_click = 0;

static volatile uint32_t enc_last_ms  = 0;
static volatile uint32_t aux1_last_ms = 0;
static volatile uint32_t aux2_last_ms = 0;
static uint32_t last_manual_focus_ms = 0;
static uint8_t pid_focus_pending = 0;
static PID_ControlMode_t last_pid_mode = PID_CONTROL_MODE_CV;

static inline void debounce_set_flag(volatile uint8_t *flag,
                                     volatile uint32_t *last_ms)
{
  uint32_t now = HAL_GetTick();

  // HAL_GetTick() działa w SysTick (ms). W ISR ok, byle bez długich akcji.
  if ((uint32_t)(now - *last_ms) >= BTN_DEBOUNCE_MS) {
    *last_ms = now;
    *flag = 1;
  }
}

static inline float GUI_FilterVout(float prev, float in) {
  float cand = in;

  if (fabsf(cand - v_target_local) <= GUI_V_SNAP_TO_SET) {
    return v_target_local;
  }

  if (fabsf(cand - prev) < GUI_V_DEADBAND) {
    return prev;
  }

  return prev + GUI_FILTER_ALPHA * (cand - prev);
}

static PID_ControlMode_t GUI_GetDisplayControlMode(void) {
  PID_ControlMode_t mode = PID_GetControlMode();

  if (mode == PID_CONTROL_MODE_CC) {
    return PID_CONTROL_MODE_CC;
  }

  return PID_CONTROL_MODE_CV;
}

static uint8_t enc_synced = 0;

/* ================== KONFIGURACJA ================== */
static const float EDIT_STEPS[] = {10.0f, 1.0f, 0.1f, 0.01f};
#define EDIT_STEP_COUNT (sizeof(EDIT_STEPS) / sizeof(EDIT_STEPS[0]))

/* ================== MAKRA POMOCNICZE ================== */
#define INT_P(x) ((int)(x))
#define FRAC_P(x) ((int)(((x) - (int)(x)) * 100))

/* ================== ZMIENNE STANU ================== */
typedef enum { GUI_VIEW = 0, GUI_EDIT_V, GUI_EDIT_I } gui_mode_t;

static gui_mode_t gui_mode = GUI_VIEW;
static int32_t enc_last = 0;

static bool output_enabled = false;

static float i_target_local = 1.0f;

/* 0–3: V (10,1,0.1,0.01), 4–7: I (10,1,0.1,0.01) */
static uint8_t edit_pos = 0;

static uint8_t GUI_GetUnitsCursorForMode(PID_ControlMode_t mode) {
  return (mode == PID_CONTROL_MODE_CC) ? 5u : 1u;
}

static void GUI_HoldManualFocus(void) {
  last_manual_focus_ms = HAL_GetTick();
}

static void GUI_SetEditSelection(uint8_t new_edit_pos) {
  edit_pos = new_edit_pos % 8u;
  gui_mode = (edit_pos < 4u) ? GUI_EDIT_V : GUI_EDIT_I;
}

static void GUI_ApplyAutoFocus(PID_ControlMode_t mode) {
  GUI_SetEditSelection(GUI_GetUnitsCursorForMode(mode));
  enc_last = TIM1->CNT / 4;
}

/* ================== PROSTE BUTTON HANDLING ================== */
typedef struct {
  uint8_t stable; /* 1=puszczony, 0=wciśnięty */
  uint8_t last_raw;
  uint32_t last_change_ms;
} btn_t;

static void BTN_Init(btn_t *b, uint8_t initial_raw) {
  b->stable = initial_raw;
  b->last_raw = initial_raw;
  b->last_change_ms = HAL_GetTick();
}

void BTN_enc_handle() {
  debounce_set_flag(&enc_click, &enc_last_ms);
}

void BTN_aux1_handle() {
  debounce_set_flag(&aux1_click, &aux1_last_ms);
}

void BTN_aux2_handle() {
  debounce_set_flag(&aux2_click, &aux2_last_ms);
}

void BTN_reset() {
  enc_click = 0;
  aux1_click = 0;
  aux2_click = 0;
}

__unused static uint8_t __BTN_CLICK(btn_t *b, uint8_t raw) {
  uint32_t now = HAL_GetTick();

  if (raw != b->last_raw) {
    b->last_raw = raw;
    b->last_change_ms = now;
  }

  if ((now - b->last_change_ms) < BTN_DEBOUNCE_MS) {
    return 0;
  }

  if (raw != b->stable) {
    b->stable = raw;
    if (b->stable == 0) {
      return 1; /* click = zbocze opadające */
    }
  }

  return 0;
}

/* ================== FUNKCJE GRAFICZNE ================== */
void GUI_FillRect(int x, int y, int w, int h, SSD1306_COLOR color) {
  for (int i = 0; i < h; i++) {
    ssd1306_Line(x, y + i, x + w, y + i, color);
  }
}

void GUI_DrawProgressBar(int x, int y, int w, int h, int percent) {
  if (percent > 100)
    percent = 100;
  if (percent < 0)
    percent = 0;

  ssd1306_DrawRectangle(x, y, x + w, y + h, White);

  int fill_w = (w * percent) / 100;
  if (fill_w > 0) {
    GUI_FillRect(x + 1, y + 2, fill_w, h - 3, White);
  }
}

static void GUI_DrawDigitUnderline(int base_x, int y_line, uint8_t step_idx,
                                   SSD1306_COLOR color) {
  if (step_idx > 3)
    step_idx = 3;

  static const uint8_t digit_x_offset[4] = {0, 10, 10 + 10 + 5,
                                            10 + 10 + 5 + 10};

  int x0 = base_x + digit_x_offset[step_idx];
  int x1 = x0 + 7;

  ssd1306_Line(x0, y_line, x1, y_line, color);
}

/* ================== OUTPUT ENABLE (HW) ================== */
static void GUI_SetOutputEnabled(bool en) {
  if (!en) {
    /* OFF: najpierw zjedź napięciem (lub reset) */
    PID_SetTargetVoltage(0.0f);
    PID_SetTargetCurrent(i_target_local); /* limit prądu zostaje */
    PID_Reset();

    /* potem wyłącz driver */
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);

    output_enabled = false;
    return;
  }

  /* ON: najpierw PID_Reset(), potem targety, na końcu enable pin */
  PID_Reset();
  PID_SetTargetVoltage(v_target_local);
  PID_SetTargetCurrent(i_target_local);

  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);

  output_enabled = true;
}

/* ================== LOGIKA ENKODERA ================== */
static void GUI_HandleEncoder(void) {
  int32_t cnt = TIM1->CNT / 4;
  int32_t delta = cnt - enc_last;

  if (delta == 0)
    return;
  enc_last = cnt;

  if (gui_mode != GUI_VIEW) {
    GUI_HoldManualFocus();
  }

  if (edit_pos < 4) {
    float step = EDIT_STEPS[edit_pos];
    v_target_local += (float)delta * step;
    if (v_target_local < 0.0f)
      v_target_local = 0.0f;
    if (v_target_local > 30.0f)
      v_target_local = 30.0f;

    /* zawsze aktualizuj PID target */
    PID_SetTargetVoltage(v_target_local);

  } else if (edit_pos < 8) {
    float step = EDIT_STEPS[edit_pos - 4];
    i_target_local += (float)delta * step;

    if (i_target_local < 0.0f)
      i_target_local = 0.0f;
    if (i_target_local > 10.0f)
      i_target_local = 10.0f;

    PID_SetTargetCurrent(i_target_local);
  }
}

/* ================== GŁÓWNY EKRAN (128x128) ================== */
/* UWAGA: wygląda tak jak u Ciebie, nie ruszam układu/fontów */
static void GUI_DrawMain(void) {
  char buf[32];
  ssd1306_Fill(Black);
  PID_ControlMode_t pid_mode = GUI_GetDisplayControlMode();

  float v_out_raw = PID_GetVOut();
  float i_out_raw = PID_GetIOut();
  float v_in_raw = PID_GetVIn();

  // Napięcie na ekranie jest lekko wygładzone dla czytelności.
  v_out = GUI_FilterVout(v_out, v_out_raw);
  i_out = i_out_raw;
  v_in = v_in_raw;
  p_out = v_out * i_out;

  float v_display = v_target_local;
  float i_display = i_target_local;

  int pwm_percent = (int)(PID_GetPWM() * 100.0f);

  ssd1306_SetCursor(2, 2);
  snprintf(buf, sizeof(buf), "IN: %d.%dV", INT_P(v_in), FRAC_P(v_in) / 10);
  ssd1306_WriteString(buf, Font_6x8, White);

  ssd1306_SetCursor(2, 13);
  uint8_t p_avail = APP_GetPower();
  snprintf(buf, sizeof(buf), "MAX POWER: %uW", p_avail);
  ssd1306_WriteString(buf, Font_6x8, White);

  ssd1306_SetCursor(70, 3);
  ssd1306_WriteString("PWM", Font_6x8, White);
  GUI_DrawProgressBar(90, 3, 36, 6, pwm_percent);

  ssd1306_Line(0, 23, 127, 23, White);

  int x_center = (128 - (6 * 16)) / 2;

  if (!output_enabled) {
    int x_off = (128 - (3 * 16)) / 2;
    ssd1306_SetCursor(x_off, 35);
    ssd1306_WriteString("OFF", Font_16x24, White);
  } else {
    snprintf(buf, sizeof(buf), "%02d.%02dV", INT_P(v_out), FRAC_P(v_out));
    ssd1306_SetCursor(x_center, 35);
    ssd1306_WriteString(buf, Font_16x24, White);
  }

  ssd1306_SetCursor(8, 65);
  snprintf(buf, sizeof(buf), "%d.%02d A", INT_P(i_out), FRAC_P(i_out));
  ssd1306_WriteString(buf, Font_16x15, White);

  ssd1306_SetCursor(70, 65);
  snprintf(buf, sizeof(buf), "%2d.%01d W", INT_P(p_out), FRAC_P(p_out) / 10);
  ssd1306_WriteString(buf, Font_16x15, White);

  ssd1306_Line(0, 85, 127, 85, White);
  ssd1306_Line(64, 85, 64, 128, White);

  const bool active_v_box =
      (gui_mode == GUI_VIEW) ? (pid_mode != PID_CONTROL_MODE_CC)
                             : (gui_mode == GUI_EDIT_V);
  const bool active_i_box =
      (gui_mode == GUI_VIEW) ? (pid_mode == PID_CONTROL_MODE_CC)
                             : (gui_mode == GUI_EDIT_I);

  SSD1306_COLOR  txt_v;
  SSD1306_COLOR  txt_i;

  if (active_v_box) {
    txt_v = Black;
    GUI_FillRect(0, 86, 63, 42, White);
  } else {
    
    txt_v = White;
  }

  if (active_i_box) {
    txt_i = Black;
    GUI_FillRect(65, 86, 63, 42, White);
  } else {
    txt_i = White;
  }

  ssd1306_SetCursor(8, 91);
  ssd1306_WriteString("SET [V]", Font_7x10, txt_v);

  ssd1306_SetCursor(8, 108);
  snprintf(buf, sizeof(buf), "%02d.%02d", INT_P(v_display), FRAC_P(v_display));
  ssd1306_WriteString(buf, Font_16x15, txt_v);

  if (active_v_box) {
    uint8_t v_cursor =
        (gui_mode == GUI_EDIT_V) ? edit_pos : GUI_GetUnitsCursorForMode(pid_mode);
    GUI_DrawDigitUnderline(10, 123, v_cursor, txt_v);
  }

  ssd1306_SetCursor(72, 91);
  ssd1306_WriteString("SET [A]", Font_7x10, txt_i);

  ssd1306_SetCursor(72, 108);
  snprintf(buf, sizeof(buf), "%02d.%02d", INT_P(i_display), FRAC_P(i_display));
  ssd1306_WriteString(buf, Font_16x15, txt_i);

  if (active_i_box) {
    uint8_t i_cursor = (gui_mode == GUI_EDIT_I)
                           ? (edit_pos - 4u)
                           : (GUI_GetUnitsCursorForMode(pid_mode) - 4u);
    GUI_DrawDigitUnderline(74, 123, i_cursor, txt_i);
  }

  ssd1306_UpdateScreen();
}

/* ================== API GLOWNE ================== */
void GUI_Init(void) {
  enc_synced = 0;
  enc_last = TIM1->CNT / 4;

  ssd1306_Init();
  ssd1306_Fill(Black);

  ssd1306_DrawBitmap(0, 0, logo, 128, 128, White);
  ssd1306_UpdateScreen();

  enc_last = TIM1->CNT / 4;
  HAL_Delay(100);

  /* PIN ENABLE: twardo OFF na starcie */
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);

  /* startowe nastawy */
  v_target_local = 0.0f;
  i_target_local = 1.0f;

  PID_SetTargetVoltage(v_target_local);
  PID_SetTargetCurrent(i_target_local);

  gui_mode = GUI_VIEW;
  edit_pos = GUI_GetUnitsCursorForMode(GUI_GetDisplayControlMode());
  last_manual_focus_ms = 0u;
  pid_focus_pending = 0u;
  last_pid_mode = GUI_GetDisplayControlMode();

  /* HARD OFF na starcie */
  GUI_SetOutputEnabled(false);
}

void GUI_Process(void) {
  static uint32_t last_draw_ms = 0;
  uint32_t now = HAL_GetTick();
  PID_ControlMode_t pid_mode = GUI_GetDisplayControlMode();

  static btn_t btn_enc;
  static btn_t btn_aux1;
  static btn_t btn_aux2;
  static uint8_t inited = 0;

  if (!inited) {
    BTN_Init(&btn_enc, HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin));
    BTN_Init(&btn_aux1, HAL_GPIO_ReadPin(BTN_AUX1_GPIO_Port, BTN_AUX1_Pin));
    BTN_Init(&btn_aux2, HAL_GPIO_ReadPin(BTN_AUX2_GPIO_Port, BTN_AUX2_Pin));
    inited = 1;
  }

    /* Pierwszy cykl: zsynchronizuj enkoder i nie licz delty, bo potrafi strzelić */
  if (!enc_synced) {
    enc_last = TIM1->CNT / 4;
    enc_synced = 1;

    /* Dla pewności: startowe nastawy, żeby GUI nie pokazało śmieci */
    v_target_local = 0.0f;
    i_target_local = 1.0f;
    PID_SetTargetVoltage(v_target_local);
    PID_SetTargetCurrent(i_target_local);

    /* Twardo OFF */
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
    output_enabled = false;
    last_manual_focus_ms = now;
    pid_focus_pending = 0u;
    last_pid_mode = pid_mode;
    edit_pos = GUI_GetUnitsCursorForMode(pid_mode);

    /* I tyle. Wracamy, żeby nic nie ruszyć w tym cyklu. */
    return;
  }

  GUI_HandleEncoder();

  /* ENC: toggle output + hardware pin */
  if (enc_click) {
    GUI_SetOutputEnabled(!output_enabled);
    enc_click = 0;
  }

  /* Edycja pozycji cyfry */
  if (gui_mode == GUI_VIEW) {
    edit_pos = GUI_GetUnitsCursorForMode(pid_mode);
    pid_focus_pending = 0u;

    if (aux1_click || aux2_click) {
      /* UWAGA: nie bierz PID_GetCurrentSetpoint() bo to robi “SET 30V” */
      v_target_local = PID_GetTargetVoltage();
      i_target_local = PID_GetTargetCurrent();

      GUI_ApplyAutoFocus(pid_mode);
      GUI_HoldManualFocus();
      aux1_click = 0;
      aux2_click = 0;
    }
  } else {
    if (aux1_click) {
      GUI_SetEditSelection((uint8_t)(edit_pos + 1u));
      GUI_HoldManualFocus();
      aux1_click = 0;
    }
    if (aux2_click) {
      GUI_SetEditSelection((uint8_t)(edit_pos + 7u));
      GUI_HoldManualFocus();
      aux2_click = 0;
    }

    if (pid_mode != last_pid_mode) {
      pid_focus_pending = 1u;
    }

    if (pid_focus_pending &&
        ((uint32_t)(now - last_manual_focus_ms) >= GUI_MODE_AUTOFOLLOW_MS)) {
      GUI_ApplyAutoFocus(pid_mode);
      pid_focus_pending = 0u;
    }
  }

  last_pid_mode = pid_mode;

  if ((HAL_GetTick() - last_draw_ms) < 30u) {
    return;
  }
  last_draw_ms = HAL_GetTick();

  GUI_DrawMain();
}
