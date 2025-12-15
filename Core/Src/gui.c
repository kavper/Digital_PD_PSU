#include "gui.h"
#include "pid.h"
#include "main.h" 
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>

#define GUI_FILTER_ALPHA  0.8f

static float v_out = 0.0f;
static float i_out = 0.0f;
static float p_out = 0.0f;
static float v_in  = 0.0f;

static inline float GUI_Filter(float prev, float in)
{
    return prev + GUI_FILTER_ALPHA * (in - prev);
}

/* ================== KONFIGURACJA ================== */
#define V_STEP  0.5f
#define I_STEP  0.1f

/* ================== MAKRA POMOCNICZE ================== */
#define INT_P(x)  ((int)(x))
#define FRAC_P(x) ((int)(((x) - (int)(x)) * 100))

/* ================== ZMIENNE STANU ================== */
typedef enum {
    GUI_VIEW = 0,
    GUI_EDIT_V,
    GUI_EDIT_I
} gui_mode_t;

static gui_mode_t gui_mode = GUI_VIEW;
static int32_t enc_last = 0;

// Lokalne zmienne do edycji - inicjalizowane bezpiecznymi wartościami
static float v_target_local = 5.0f; 
static float i_target_local = 1.0f;

/* ================== FUNKCJE GRAFICZNE ================== */

// Rysuje pasek postępu (np. dla PWM)
void GUI_DrawProgressBar(int x, int y, int w, int h, int percent) {
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;

    ssd1306_DrawRectangle(x, y, x + w, y + h, White);
    
    int fill_w = (w * percent) / 100;
    if (fill_w > 0) {
        // Rysujemy linię pośrodku jako wypełnienie
        for(int i=2; i<h-1; i++) {
             ssd1306_Line(x+1, y+i, x+fill_w, y+i, White);
        }
    }
}

/* ================== LOGIKA ENKODERA ================== */
void GUI_HandleEncoder(void) {
    int32_t cnt = TIM1->CNT / 4;
    int32_t delta = cnt - enc_last;
    
    if (delta == 0) return;
    enc_last = cnt;

    // Obsługa zmiany wartości tylko w trybach edycji
    if (gui_mode == GUI_EDIT_V) {
        v_target_local += delta * V_STEP;
        
        // Limity (0-30V)
        if (v_target_local < 0.0f) v_target_local = 0.0f;
        if (v_target_local > 30.0f) v_target_local = 30.0f;

        // Wyślij do PID
        PID_SetTargetVoltage(v_target_local);
    }
    else if (gui_mode == GUI_EDIT_I) {
        i_target_local += delta * I_STEP;
        
        // Limity (0-20A)
        if (i_target_local < 0.1f) i_target_local = 0.1f;
        if (i_target_local > 20.0f) i_target_local = 20.0f;

        // Wyślij do PID
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
float v_in_raw  = PID_GetVIn();

v_out = GUI_Filter(v_out, v_out_raw);
i_out = GUI_Filter(i_out, i_out_raw);
v_in  = GUI_Filter(v_in,  v_in_raw);
p_out = GUI_Filter(p_out, v_out * i_out);
    
    // Setpointy
    float v_display = (gui_mode == GUI_EDIT_V) ? v_target_local : PID_GetCurrentSetpoint();
    float i_display = i_target_local;

    int pwm_percent = (int)(PID_GetPWM() * 100.0f);


    // --- 1. GÓRNY PASEK (Status) ---
    // Y: 0 - 15
    ssd1306_SetCursor(2, 2);
    snprintf(buf, sizeof(buf), "IN: %d.%dV", INT_P(v_in), FRAC_P(v_in)/10);
    ssd1306_WriteString(buf, Font_7x10, White);

    // Pasek PWM (Większy i wyraźniejszy)
    ssd1306_SetCursor(75, 2);
    ssd1306_WriteString("PWM", Font_6x8, White);
    GUI_DrawProgressBar(75, 12, 45, 6, pwm_percent);

    ssd1306_Line(0, 22, 127, 22, White); // Linia oddzielająca górę


    // --- 2. SEKCJA GŁÓWNA (Wyjście - WIELKIE) ---
    // Y: 25 - 80
    
    // NAPIĘCIE (Środek ekranu)
    int x_center = (128 - (7 * 11)) / 2; 
    ssd1306_SetCursor(x_center, 35);
    snprintf(buf, sizeof(buf), "%02d.%02d V", INT_P(v_out), FRAC_P(v_out));
    ssd1306_WriteString(buf, Font_11x18, White);

    // PRĄD I MOC (Pod napięciem, z odstępami)
    ssd1306_SetCursor(10, 65);
    snprintf(buf, sizeof(buf), "%d.%02d A", INT_P(i_out), FRAC_P(i_out));
    ssd1306_WriteString(buf, Font_7x10, White);

    ssd1306_SetCursor(70, 65);
    snprintf(buf, sizeof(buf), "%2d.%01d W", INT_P(p_out), FRAC_P(p_out)/10);
    ssd1306_WriteString(buf, Font_7x10, White);

    ssd1306_Line(0, 85, 127, 85, White); // Linia oddzielająca dół


    // --- 3. SEKCJA EDYCJI (Na samym dole - rozdzielona) ---
    // Y: 90 - 128
    
    // -- LEWA: NAPIĘCIE (Duży box) --
    if (gui_mode == GUI_EDIT_V) {
        ssd1306_DrawRectangle(2, 90, 62, 124, White); // Ramka aktywna
        ssd1306_DrawRectangle(3, 91, 61, 123, White); // Pogrubienie
        ssd1306_SetCursor(10, 95);
        ssd1306_WriteString("SET V", Font_6x8, White);
        
        ssd1306_SetCursor(8, 108);
        snprintf(buf, sizeof(buf), "%02d.%02d", INT_P(v_display), FRAC_P(v_display));
        ssd1306_WriteString(buf, Font_7x10, White);
    } else {
        // Nieaktywne
        ssd1306_SetCursor(10, 95);
        ssd1306_WriteString("SET V", Font_6x8, White);
        ssd1306_SetCursor(8, 108);
        snprintf(buf, sizeof(buf), "%02d.%02d", INT_P(v_display), FRAC_P(v_display));
        ssd1306_WriteString(buf, Font_7x10, White);
    }

    // -- PRAWA: LIMIT PRĄDU (Duży box) --
    if (gui_mode == GUI_EDIT_I) {
        ssd1306_DrawRectangle(66, 90, 126, 124, White); // Ramka aktywna
        ssd1306_DrawRectangle(67, 91, 125, 123, White); // Pogrubienie
        ssd1306_SetCursor(74, 95);
        ssd1306_WriteString("LIMIT", Font_6x8, White);
        
        ssd1306_SetCursor(72, 108);
        snprintf(buf, sizeof(buf), "%d.%02dA", INT_P(i_display), FRAC_P(i_display));
        ssd1306_WriteString(buf, Font_7x10, White);
    } else {
        // Nieaktywne
        ssd1306_SetCursor(74, 95);
        ssd1306_WriteString("LIMIT", Font_6x8, White);
        ssd1306_SetCursor(72, 108);
        snprintf(buf, sizeof(buf), "%d.%02dA", INT_P(i_display), FRAC_P(i_display));
        ssd1306_WriteString(buf, Font_7x10, White);
    }
    
    // Pionowa kreska na dole
    ssd1306_Line(64, 85, 64, 128, White);

    ssd1306_UpdateScreen();
}

/* ================== API GLOWNE ================== */
void GUI_Init(void) {
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
    
    enc_last = TIM1->CNT / 4;
    
    HAL_Delay(100); 
    
    // Synchronizacja na start
    v_target_local = PID_GetCurrentSetpoint();
    i_target_local = 1.5f; 
}

void GUI_Process(void) {
    static uint32_t last_tick = 0;
    static uint8_t last_btn = 1;

    GUI_HandleEncoder();

    if (HAL_GetTick() - last_tick < 50) return;
    last_tick = HAL_GetTick();

    uint8_t btn = HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin);
    if (last_btn && !btn) {
        if (gui_mode == GUI_VIEW) {
            gui_mode = GUI_EDIT_V;
            v_target_local = PID_GetCurrentSetpoint(); // Sync
        } else if (gui_mode == GUI_EDIT_V) {
            gui_mode = GUI_EDIT_I;
        } else {
            gui_mode = GUI_VIEW;
        }
    }
    last_btn = btn;

    GUI_DrawMain();
}