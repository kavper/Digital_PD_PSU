
#include "gui.h"
#include "app.h"
#include "kne.h"
#include "pid.h"

#include "ssd1306.h"
#include "ssd1306_fonts.h"

#include <stdio.h>
#include <string.h>

void GUI_Init() {

  ssd1306_Init();

  ssd1306_Fill(White);
  ssd1306_UpdateScreen();

  ssd1306_Fill(Black);
  ssd1306_DrawBitmap(0, 0, logo, 128, 128, White);
  ssd1306_UpdateScreen();
  ssd1306_Fill(Black);
}


void GUI_Process() {
  static uint32_t last_tick = 0;
  uint32_t now = HAL_GetTick(); // czas w ms od startu MCU

  if (now - last_tick >= 100) { // 500ms
    last_tick = now;
    GUI_Update();
    // UpdateGraph();
  }
}

void GUI_Update() {

  char buf[32];

  // Nagłówek
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("USB-PD PSU", Font_11x18, White);

  // --- Dane wejściowe ---
  float v_in = PID_GetVIn();
  float i_in = PID_GetIIn();
  float p_in = v_in * i_in;

  // Vin i Iin w jednej linii
  ssd1306_SetCursor(0, 25);
  snprintf(buf, sizeof(buf), "Vi:%02d.%02dV", (int)v_in,
           (int)((v_in - (int)v_in) * 100));
  ssd1306_WriteString(buf, Font_6x8, White);

  // przesunięcie X
  int x_offset = strlen(buf) * 6 + 5;
  ssd1306_SetCursor(x_offset, 25);
  snprintf(buf, sizeof(buf), "Ii:%d.%02dA", (int)i_in,
           (int)((i_in - (int)i_in) * 100));
  ssd1306_WriteString(buf, Font_6x8, White);

  // Pin
  ssd1306_SetCursor(0, 40);
  snprintf(buf, sizeof(buf), "Pi:%02d.%02dW", (int)p_in,
           (int)((p_in - (int)p_in) * 100));
  ssd1306_WriteString(buf, Font_6x8, White);

  // --- Dane wyjściowe ---
  float v_out = PID_GetVOut();
  float i_out = PID_GetIOut();
  float p_out = v_out * i_out;

  // Vout + Iout
  ssd1306_SetCursor(0, 55);
  snprintf(buf, sizeof(buf), "Vo:%02d.%02dV", (int)v_out,
           (int)((v_out - (int)v_out) * 100));
  ssd1306_WriteString(buf, Font_6x8, White);

  x_offset = strlen(buf) * 6 + 5;
  ssd1306_SetCursor(x_offset, 55);
  snprintf(buf, sizeof(buf), "Io:%02d.%02dA", (int)i_out,
           (int)((i_out - (int)i_out) * 100));
  ssd1306_WriteString(buf, Font_6x8, White);

  // Pout
  ssd1306_SetCursor(0, 70);
  snprintf(buf, sizeof(buf), "Po:%02d.%02dW", (int)p_out,
           (int)((p_out - (int)p_out) * 100));
  ssd1306_WriteString(buf, Font_6x8, White);

  // --- Setpoint ---
  float current_setpoint = PID_GetCurrentSetpoint();
  ssd1306_SetCursor(0, 85);
  snprintf(buf, sizeof(buf), "Vset:%02d.%02dV", (int)current_setpoint,
           (int)((current_setpoint - (int)current_setpoint) * 100));
  ssd1306_WriteString(buf, Font_6x8, White);

  // --- Vboost ---
  float v_boost = PID_GetVBoost();
  ssd1306_SetCursor(0, 100);
  snprintf(buf, sizeof(buf), "Vboost:%02d.%02dV", (int)v_boost,
           (int)((v_boost - (int)v_boost) * 100));
  ssd1306_WriteString(buf, Font_6x8, White);

  // --- PWM (DODANE) ---
  uint32_t pwm = PID_GetPWM() * 100.0f;
  ssd1306_SetCursor(0, 115);
  snprintf(buf, sizeof(buf), "PWM:%u%%", pwm);
  ssd1306_WriteString(buf, Font_6x8, White);

  ssd1306_UpdateScreen();
}

#define GRAPH_WIDTH 128
#define GRAPH_HEIGHT 128
#define GRAPH_X 0
#define GRAPH_Y 0

// Zakres napięcia (dostosuj do swojej ładowarki)
#define V_MIN 0.0f
#define V_MAX 25.0f

static float graph_buffer[GRAPH_WIDTH];
static uint8_t graph_index = 0;

static uint8_t Graph_MapVoltageToY(float v) {
  if (v < V_MIN)
    v = V_MIN;
  if (v > V_MAX)
    v = V_MAX;

  float norm = (v - V_MIN) / (V_MAX - V_MIN);
  uint8_t y = GRAPH_Y + GRAPH_HEIGHT - 1 - (uint8_t)(norm * (GRAPH_HEIGHT - 1));
  return y;
}

void Graph_AddSample(float v) {
  graph_buffer[graph_index] = v;
  graph_index = (graph_index + 1) % GRAPH_WIDTH;
}

void Graph_Draw(void) {
    ssd1306_Fill(Black);
  // 1. Oś Y i skala
  for (int i = 0; i <= 5; i++) {
    float v = V_MIN + i * (V_MAX - V_MIN) / 5.0f;

    uint8_t y = Graph_MapVoltageToY(v);
    char buf[10];
    snprintf(buf, sizeof(buf), "%2dV", (int)v);

    ssd1306_SetCursor(GRAPH_X, y - 4);
    ssd1306_WriteString(buf, Font_6x8, White);

    // pozioma kreska skali
  }

  // 2. Wykres liniowy
  uint8_t last_x = 0;
  uint8_t last_y = 0;

  for (int i = 0; i < GRAPH_WIDTH; i++) {
    int idx = (graph_index + i) % GRAPH_WIDTH;

    float v = graph_buffer[idx];
    uint8_t y = Graph_MapVoltageToY(v);

    uint8_t x = GRAPH_X + i;

    if (i > 0)
      ssd1306_Line(last_x, last_y, x, y, White);

    last_x = x;
    last_y = y;
  }

  // 3. Ramka wykresu
  ssd1306_DrawRectangle(GRAPH_X + 25, GRAPH_Y, GRAPH_X + 127,
                        GRAPH_Y + GRAPH_HEIGHT - 1, White);
}

void UpdateGraph() {
  char buf[32];
  float v_out = PID_GetVOut();
  ssd1306_WriteString(buf, Font_6x8, White);

  // --- Dodaj próbkę i narysuj wykres ---
  Graph_AddSample(v_out);
  Graph_Draw();

  ssd1306_UpdateScreen();
}