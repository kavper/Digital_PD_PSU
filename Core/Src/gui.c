#include "gui.h"
#include "app.h"
#include "logo.h"
#include "main.h"
#include "pid.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>

#define GUI_FILTER_ALPHA 0.8f

static float v_out = 0.0f;
static float i_out = 0.0f;
static float p_out = 0.0f;
static float v_in = 0.0f;

static inline float GUI_Filter(float prev, float in) {
  return prev + GUI_FILTER_ALPHA * (in - prev);
}

/* ================== KONFIGURACJA ================== */
#define V_STEP 0.5f
#define I_STEP 0.1f

/* ================== MAKRA POMOCNICZE ================== */
#define INT_P(x) ((int)(x))
#define FRAC_P(x) ((int)(((x) - (int)(x)) * 100))

/* ================== ZMIENNE STANU ================== */
typedef enum { GUI_VIEW = 0, GUI_EDIT_V, GUI_EDIT_I } gui_mode_t;

static gui_mode_t gui_mode = GUI_VIEW;
static int32_t enc_last = 0;

// Lokalne zmienne do edycji
static float v_target_local = 5.0f;
static float i_target_local = 1.0f;

/* ================== FUNKCJE GRAFICZNE ================== */

// Pomocnicza funkcja do wypełniania prostokąta kolorem
void GUI_FillRect(int x, int y, int w, int h, SSD1306_COLOR color) {
  for (int i = 0; i < h; i++) {
    ssd1306_Line(x, y + i, x + w, y + i, color);
  }
}

// Rysuje pasek postępu
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

/* ================== LOGIKA ENKODERA ================== */
void GUI_HandleEncoder(void) {
  int32_t cnt = TIM1->CNT / 4;
  int32_t delta = cnt - enc_last;

  if (delta == 0)
    return;
  enc_last = cnt;

  if (gui_mode == GUI_EDIT_V) {
    v_target_local += delta * V_STEP;
    if (v_target_local < 0.0f)
      v_target_local = 0.0f;
    if (v_target_local > 30.0f)
      v_target_local = 30.0f;
    PID_SetTargetVoltage(v_target_local);
  } else if (gui_mode == GUI_EDIT_I) {
    i_target_local += delta * I_STEP;
    if (i_target_local < 0.1f)
      i_target_local = 0.1f;
    if (i_target_local > 20.0f)
      i_target_local = 20.0f;
    PID_SetTargetCurrent(i_target_local);
  }
}

/* ================== GŁÓWNY EKRAN (128x128) ================== */
static void GUI_DrawMain(void) {
  char buf[32];
  ssd1306_Fill(Black);

  // --- POBRANIE DANYCH Z PID ---
  float v_out_raw = PID_GetVOut();
  float i_out_raw = PID_GetIOut();
  float v_in_raw = PID_GetVIn();

  v_out = GUI_Filter(v_out, v_out_raw);
  i_out = GUI_Filter(i_out, i_out_raw);
  v_in = GUI_Filter(v_in, v_in_raw);
  p_out = GUI_Filter(p_out, v_out * i_out);

  // Setpointy
  float v_display =
      (gui_mode == GUI_EDIT_V) ? v_target_local : PID_GetCurrentSetpoint();
  float i_display = i_target_local;

  int pwm_percent = (int)(PID_GetPWM() * 100.0f);

  // --- 1. GÓRNY PASEK (Status) ---
  ssd1306_SetCursor(2, 2);
  snprintf(buf, sizeof(buf), "IN: %d.%dV", INT_P(v_in), FRAC_P(v_in) / 10);
  ssd1306_WriteString(buf, Font_6x8, White);

  ssd1306_SetCursor(2, 13);
  uint8_t p_avail = APP_GetPower();
  snprintf(buf, sizeof(buf), "MAX POWER: %uW", p_avail);
  ssd1306_WriteString(buf, Font_6x8, White);

  // Pasek PWM
  ssd1306_SetCursor(70, 3);
  ssd1306_WriteString("PWM", Font_6x8, White);
  GUI_DrawProgressBar(90, 3, 36, 6, pwm_percent);

  ssd1306_Line(0, 23, 127, 23, White);

  // --- 2. SEKCJA GŁÓWNA (Wyjście) ---
  // Y: 25 - 80
  // Generujemy tekst najpierw, aby wiedzieć co wyświetlamy
  snprintf(buf, sizeof(buf), "%02d.%02dV", INT_P(v_out), FRAC_P(v_out));

  // Centrowanie:
  // Tekst "XX.XXV" ma zawsze 6 znaków.
  // Szerokość czcionki Font_16x26 to 16 pikseli.
  // (128 - (6 * 16)) / 2  => (128 - 96) / 2 => 16
  int x_center = (128 - (6 * 16)) / 2;

  ssd1306_SetCursor(x_center, 35);
  ssd1306_WriteString(buf, Font_16x24, White);

  ssd1306_SetCursor(8, 65);
  snprintf(buf, sizeof(buf), "%d.%02d A", INT_P(i_out), FRAC_P(i_out));
  ssd1306_WriteString(buf, Font_16x15, White);

  ssd1306_SetCursor(70, 65);
  snprintf(buf, sizeof(buf), "%2d.%01d W", INT_P(p_out), FRAC_P(p_out) / 10);
  ssd1306_WriteString(buf, Font_16x15, White);

  // Linie podziału dolnego
  ssd1306_Line(0, 85, 127, 85, White);  // Pozioma
  ssd1306_Line(64, 85, 64, 128, White); // Pionowa

  // --- 3. SEKCJA EDYCJI (Inwersja kolorów przy wyborze) ---
  // Y: 86 - 128

  // Zmienne pomocnicze kolorów
  SSD1306_COLOR bg_v, txt_v;
  SSD1306_COLOR bg_i, txt_i;

  // Logika kolorów dla LEWEJ STRONY (Voltage)
  if (gui_mode == GUI_EDIT_V) {
    bg_v = White;
    txt_v = Black;
    // Wypełniamy tło lewego dolnego rogu (0-63)
    GUI_FillRect(0, 86, 63, 42, White);
  } else {
    bg_v = Black;
    txt_v = White;
  }

  // Logika kolorów dla PRAWEJ STRONY (Current)
  if (gui_mode == GUI_EDIT_I) {
    bg_i = White;
    txt_i = Black;
    // Wypełniamy tło prawego dolnego rogu (65-127)
    GUI_FillRect(65, 86, 63, 42, White);
  } else {
    bg_i = Black;
    txt_i = White;
  }

  // -- RYSOWANIE TEKSTU LEWA STRONA (V) --
  ssd1306_SetCursor(8, 91);
  ssd1306_WriteString("SET [V]", Font_7x10, txt_v);

  ssd1306_SetCursor(8, 108);
  snprintf(buf, sizeof(buf), "%02d.%02d", INT_P(v_display), FRAC_P(v_display));
  ssd1306_WriteString(buf, Font_16x15, txt_v);

  // -- RYSOWANIE TEKSTU PRAWA STRONA (I) --
  ssd1306_SetCursor(72, 91);
  ssd1306_WriteString("SET [A]", Font_7x10, txt_i);

  ssd1306_SetCursor(72, 108);
  snprintf(buf, sizeof(buf), "%02d.%02d", INT_P(i_display), FRAC_P(i_display));
  ssd1306_WriteString(buf, Font_16x15, txt_i);

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

  v_target_local = PID_GetCurrentSetpoint();
  i_target_local = 1.5f;
}

void GUI_Process(void) {
  static uint32_t last_tick = 0;
  static uint8_t last_btn = 1;

  GUI_HandleEncoder();

  if (HAL_GetTick() - last_tick < 50)
    return;
  last_tick = HAL_GetTick();

  uint8_t btn = HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin);
  if (last_btn && !btn) {
    if (gui_mode == GUI_VIEW) {
      gui_mode = GUI_EDIT_V;
      v_target_local = PID_GetCurrentSetpoint();
    } else if (gui_mode == GUI_EDIT_V) {
      gui_mode = GUI_EDIT_I;
    } else {
      gui_mode = GUI_VIEW;
    }
  }
  last_btn = btn;

  GUI_DrawMain();
}