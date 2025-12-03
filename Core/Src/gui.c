
#include "app.h"
#include "kne.h"
#include "gui.h"

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
}

void GUI_Process()
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick(); // czas w ms od startu MCU

    if (now - last_tick >= 500) { // 500ms
        last_tick = now;
        GUI_Update();
    }
}

void GUI_Update() {
    ssd1306_Fill(Black);

    char buf[32];

    // Nagłówek
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("USB-PD PSU", Font_11x18, White);

    // --- Dane wejściowe ---
    float v_in = APP_GetVIn();
    float i_in = APP_GetIIn();
    float p_in = v_in * i_in;

    // Vin i Iin w jednej linii
    ssd1306_SetCursor(0, 25);
    snprintf(buf, sizeof(buf), "Vi:%d.%02dV", 
             (int)v_in, (int)((v_in - (int)v_in) * 100));
    ssd1306_WriteString(buf, Font_6x8, White);

    // przesunięcie X na koniec Vin + odstęp 5px
    int x_offset = strlen(buf) * 6 + 5; // Font_6x8 szerokość znaku ~6px
    ssd1306_SetCursor(x_offset, 25);
    snprintf(buf, sizeof(buf), "Ii:%d.%02dA",
             (int)i_in, (int)((i_in - (int)i_in) * 100));
    ssd1306_WriteString(buf, Font_6x8, White);

    // Pin pod spodem
    ssd1306_SetCursor(0, 40);
    snprintf(buf, sizeof(buf), "Pi:%d.%02dW",
             (int)p_in, (int)((p_in - (int)p_in) * 100));
    ssd1306_WriteString(buf, Font_6x8, White);

    // --- Dane wyjściowe ---
    float v_out = APP_GetVOut();
    float i_out = APP_GetIOut();
    float p_out = v_out * i_out;

    // Vout i Iout w jednej linii
    ssd1306_SetCursor(0, 55);
    snprintf(buf, sizeof(buf), "Vo:%d.%02dV",
             (int)v_out, (int)((v_out - (int)v_out) * 100));
    ssd1306_WriteString(buf, Font_6x8, White);

    x_offset = strlen(buf) * 6 + 5;
    ssd1306_SetCursor(x_offset, 55);
    snprintf(buf, sizeof(buf), "Io:%d.%02dA",
             (int)i_out, (int)((i_out - (int)i_out) * 100));
    ssd1306_WriteString(buf, Font_6x8, White);

    // Pout pod spodem
    ssd1306_SetCursor(0, 70);
    snprintf(buf, sizeof(buf), "Po:%d.%02dW",
             (int)p_out, (int)((p_out - (int)p_out) * 100));
    ssd1306_WriteString(buf, Font_6x8, White);

    // --- Setpoint ---
    float current_setpoint = APP_GetCurrentSetpoint();
    ssd1306_SetCursor(0, 85);
    snprintf(buf, sizeof(buf), "Set:%d.%02dA",
             (int)current_setpoint,
             (int)((current_setpoint - (int)current_setpoint) * 100));
    ssd1306_WriteString(buf, Font_6x8, White);

    // --- Vboost ---
    float v_boost = APP_GetVBoost();
    ssd1306_SetCursor(0, 100);
    snprintf(buf, sizeof(buf), "Vboost:%d.%02dV",
             (int)v_boost,
             (int)((v_boost - (int)v_boost) * 100));
    ssd1306_WriteString(buf, Font_6x8, White);

    ssd1306_UpdateScreen();
}







