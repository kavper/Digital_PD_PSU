#include "gui.h"
#include "app.h"
#include "logo.h"
#include "main.h"
#include "pid.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>
#include <stdbool.h>

#define GUI_FILTER_ALPHA 0.8f

static float v_out = 0.0f;
static float i_out = 0.0f;
static float p_out = 0.0f;
static float v_in  = 0.0f;

static inline float GUI_Filter(float prev, float in) {
  return prev + GUI_FILTER_ALPHA * (in - prev);
}

/* ================== KONFIGURACJA ================== */
#define BTN_DEBOUNCE_MS      50u

static const float EDIT_STEPS[] = { 10.0f, 1.0f, 0.1f, 0.01f };
#define EDIT_STEP_COUNT (sizeof(EDIT_STEPS)/sizeof(EDIT_STEPS[0]))

/* ================== MAKRA POMOCNICZE ================== */
#define INT_P(x)  ((int)(x))
#define FRAC_P(x) ((int)(((x) - (int)(x)) * 100))

/* ================== ZMIENNE STANU ================== */
typedef enum { GUI_VIEW = 0, GUI_EDIT_V, GUI_EDIT_I } gui_mode_t;

static gui_mode_t gui_mode = GUI_VIEW;
static int32_t enc_last = 0;

static bool output_enabled = false;

static float v_target_local = 0.0f;
static float i_target_local = 1.0f;

/* 0–3: V (10,1,0.1,0.01), 4–7: I (10,1,0.1,0.01) */
static uint8_t edit_pos = 0;

/* ================== PROSTE BUTTON HANDLING ================== */
typedef struct {
  uint8_t  stable;        /* 1=puszczony, 0=wciśnięty */
  uint8_t  last_raw;
  uint32_t last_change_ms;
} btn_t;

static void BTN_Init(btn_t *b, uint8_t initial_raw) {
  b->stable = initial_raw;
  b->last_raw = initial_raw;
  b->last_change_ms = HAL_GetTick();
}

static uint8_t BTN_Click(btn_t *b, uint8_t raw) {
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
  if (percent > 100) percent = 100;
  if (percent < 0)   percent = 0;

  ssd1306_DrawRectangle(x, y, x + w, y + h, White);

  int fill_w = (w * percent) / 100;
  if (fill_w > 0) {
    GUI_FillRect(x + 1, y + 2, fill_w, h - 3, White);
  }
}

static void GUI_DrawDigitUnderline(int base_x, int y_line,
                                   uint8_t step_idx,
                                   SSD1306_COLOR color)
{
  if (step_idx > 3) step_idx = 3;

  static const uint8_t digit_x_offset[4] = {
    0,
    10,
    10 + 10 + 5,
    10 + 10 + 5 + 10
  };

  int x0 = base_x + digit_x_offset[step_idx];
  int x1 = x0 + 7;

  ssd1306_Line(x0, y_line, x1, y_line, color);
}

/* ================== OUTPUT ENABLE (HW) ================== */
static void GUI_SetOutputEnabled(bool en)
{
  if (!en) {
    /* OFF: najpierw uspokój PID i napięcie */
    PID_SetTargetVoltage(0.0f);
    PID_SetTargetCurrent(i_target_local); /* limit prądu zostaje */
    PID_Reset();

    /* potem wyłącz driver */
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);

    output_enabled = false;
    return;
  }

  /* ON: najpierw reset PID i nastawy, potem enable driver */
  PID_Reset();
  PID_SetTargetVoltage(v_target_local);
  PID_SetTargetCurrent(i_target_local);

  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);

  output_enabled = true;
}

/* ================== LOGIKA ENKODERA ================== */
void GUI_HandleEncoder(void) {
  int32_t cnt = TIM1->CNT / 4;
  int32_t delta = cnt - enc_last;

  if (delta == 0) return;
  enc_last = cnt;

  if (edit_pos < 4) {
    float step = EDIT_STEPS[edit_pos];
    v_target_local += (float)delta * step;
    if (v_target_local < 0.0f)  v_target_local = 0.0f;
    if (v_target_local > 30.0f) v_target_local = 30.0f;

    /* zawsze aktualizuj PID setpoint, ale HW driver i tak jest OFF gdy trzeba */
    PID_SetTargetVoltage(v_target_local);

  } else if (edit_pos < 8) {
    float step = EDIT_STEPS[edit_pos - 4];
    i_target_local += (float)delta * step;

    /* bez “debilnego” 100mA minimum */
    if (i_target_local < 0.0f)  i_target_local = 0.0f;
    if (i_target_local > 20.0f) i_target_local = 20.0f;

    PID_SetTargetCurrent(i_target_local);
  }
}

/* ================== GŁÓWNY EKRAN (128x128) ================== */
/* UWAGA: wygląda tak jak u Ciebie, nie ruszam układu/fontów */
static void GUI_DrawMain(void) {
  char buf[32];
  ssd1306_Fill(Black);

  float v_out_raw = PID_GetVOut();
  float i_out_raw = PID_GetIOut();
  float v_in_raw  = PID_GetVIn();

  v_out = GUI_Filter(v_out, v_out_raw);
  i_out = GUI_Filter(i_out, i_out_raw);
  v_in  = GUI_Filter(v_in,  v_in_raw);
  p_out = GUI_Filter(p_out, v_out * i_out);

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

  SSD1306_COLOR bg_v, txt_v;
  SSD1306_COLOR bg_i, txt_i;

  if (gui_mode == GUI_EDIT_V) {
    bg_v = White; txt_v = Black;
    GUI_FillRect(0, 86, 63, 42, White);
  } else {
    bg_v = Black; txt_v = White;
  }

  if (gui_mode == GUI_EDIT_I) {
    bg_i = White; txt_i = Black;
    GUI_FillRect(65, 86, 63, 42, White);
  } else {
    bg_i = Black; txt_i = White;
  }

  ssd1306_SetCursor(8, 91);
  ssd1306_WriteString("SET [V]", Font_7x10, txt_v);

  ssd1306_SetCursor(8, 108);
  snprintf(buf, sizeof(buf), "%02d.%02d", INT_P(v_display), FRAC_P(v_display));
  ssd1306_WriteString(buf, Font_16x15, txt_v);

  if (gui_mode == GUI_EDIT_V) {
    GUI_DrawDigitUnderline(10, 123, edit_pos, txt_v);
  }

  ssd1306_SetCursor(72, 91);
  ssd1306_WriteString("SET [A]", Font_7x10, txt_i);

  ssd1306_SetCursor(72, 108);
  snprintf(buf, sizeof(buf), "%02d.%02d", INT_P(i_display), FRAC_P(i_display));
  ssd1306_WriteString(buf, Font_16x15, txt_i);

  if (gui_mode == GUI_EDIT_I) {
    GUI_DrawDigitUnderline(74, 123, edit_pos - 4, txt_i);
  }

  ssd1306_UpdateScreen();
}

/* ================== API GLOWNE ================== */
void GUI_Init(void) {
  ssd1306_Init();
  ssd1306_Fill(Black);

  ssd1306_DrawBitmap(0, 0, logo, 128, 128, White);
  ssd1306_UpdateScreen();

  enc_last = TIM1->CNT / 4;
  HAL_Delay(100);

  /* startowe nastawy */
  v_target_local = 0.0f;
  i_target_local = 1.0f;

  PID_SetTargetVoltage(v_target_local);
  PID_SetTargetCurrent(i_target_local);

  gui_mode = GUI_VIEW;
  edit_pos = 0;

  /* HARD OFF na starcie */
  GUI_SetOutputEnabled(false);
}

void GUI_Process(void) {
  static uint32_t last_draw_ms = 0;

  static btn_t btn_enc;
  static btn_t btn_aux1;
  static btn_t btn_aux2;
  static uint8_t inited = 0;

  if (!inited) {
    BTN_Init(&btn_enc,  HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin));
    BTN_Init(&btn_aux1, HAL_GPIO_ReadPin(BTN_AUX1_GPIO_Port, BTN_AUX1_Pin));
    BTN_Init(&btn_aux2, HAL_GPIO_ReadPin(BTN_AUX2_GPIO_Port, BTN_AUX2_Pin));
    inited = 1;
  }

  GUI_HandleEncoder();

  uint8_t enc_raw  = HAL_GPIO_ReadPin(ENC_SW_GPIO_Port,  ENC_SW_Pin);
  uint8_t aux1_raw = HAL_GPIO_ReadPin(BTN_AUX1_GPIO_Port, BTN_AUX1_Pin);
  uint8_t aux2_raw = HAL_GPIO_ReadPin(BTN_AUX2_GPIO_Port, BTN_AUX2_Pin);

  uint8_t enc_click  = BTN_Click(&btn_enc,  enc_raw);
  uint8_t aux1_click = BTN_Click(&btn_aux1, aux1_raw);
  uint8_t aux2_click = BTN_Click(&btn_aux2, aux2_raw);

  /* ENC: toggle output + hardware pin */
  if (enc_click) {
    GUI_SetOutputEnabled(!output_enabled);
  }

  /* Edycja pozycji cyfry */
  if (gui_mode == GUI_VIEW) {
    if (aux1_click || aux2_click) {
      edit_pos = 0;
      gui_mode = GUI_EDIT_V;

      /* UWAGA: nie bierz PID_GetCurrentSetpoint() bo to robi “SET 30V” */
      v_target_local = PID_GetTargetVoltage();
      i_target_local = PID_GetTargetCurrent();

      enc_last = TIM1->CNT / 4;
    }
  } else {
    if (aux1_click) {
      edit_pos = (edit_pos + 1) % 8;
    }
    if (aux2_click) {
      edit_pos = (edit_pos + 7) % 8;
    }

    gui_mode = (edit_pos < 4) ? GUI_EDIT_V : GUI_EDIT_I;
  }

  if ((HAL_GetTick() - last_draw_ms) < 30u) {
    return;
  }
  last_draw_ms = HAL_GetTick();

  GUI_DrawMain();
}