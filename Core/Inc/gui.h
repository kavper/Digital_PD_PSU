#ifndef __GUI_H
#define __GUI_H

void GUI_Init();
void GUI_Update(); // natychmiastowa aktualizacja ekranu
void UpdateGraph(); // aktualizacja wykresu
void GUI_Process(); // aktualizacja ekranu w petli glownej co 500ms

#endif /* __GUI_H */